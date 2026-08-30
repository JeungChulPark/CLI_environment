#!/usr/bin/env python3
"""compare_prompt_boxes.py — 현행 프롬프트 vs CLIP-v3 프롬프트의 실제 bbox 비교 렌더 (READ-ONLY).

각 GT 프레임에 대해 두 프롬프트 세트로 YOLO(per-prompt, imgsz960, conf0.02, top_k3)를 돌리고,
객체별 선택 후보를 후단 게이트(semantic/appe/HSV, Phase 1C 불변)에 통과시켜
  [원본 | A 현행 | B v3]  3분할 이미지를 만든다.

박스 표시: 실선+굵게 = ACCEPT(최종 검출) · 점선풍(얇게)+어두움 = 게이트 탈락
라벨은 클래스명만. 프레임 정보는 파일명에 있음.

산출: outputs/phase27_clip_prompt/boxviz/{gt,}/<ds>_<frame>.png + box_compare_manifest.csv + metrics
"""
import argparse, csv, glob, json, os, sys
from collections import defaultdict
import numpy as np
import cv2, torch, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, os.path.join(RSRCH, "phase21_fn_audit")); sys.path.insert(0, REPO)
import phase21_common as p21
import yolo_ism as yi, yolo_ism_object_n as o_n, ism_hsv

OUT = os.path.join(REPO, "outputs", "phase27_clip_prompt")
VIZ = os.path.join(OUT, "boxviz")
FRAMES = os.path.join(RSRCH, "gt_input", "frames")
V3CSV = os.path.join(OUT, "csv", "prompts_v3_selected.csv")
T = p21.T; TOPK = 3; CONF = 0.02; IMGSZ = 960
COL = {"Bear": (60,180,250), "Rabbit": (245,245,245), "Dinosaur": (90,220,90),
       "milk": (240,200,120), "choco_hazelnut_high": (80,90,190), "Febreze_high": (250,170,80),
       "Mugcup_high": (200,130,250), "saffron": (170,230,250), "Sauce_high": (60,140,240),
       "Sikhye_high": (70,220,240)}


def build():
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device","cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(objs, model, dev, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights","mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    for o in objs:
        o["_tcls"]=o["tcls"].to(dev); o["_flat11"]=torch.cat(o["tappe_blocks"][11],0).to(dev)
        o["_seg"]=torch.cat([torch.full((t.shape[0],),i,dtype=torch.long)
                             for i,t in enumerate(o["tappe_blocks"][11])]).to(dev)
        o["_nview"]=len(o["tappe_blocks"][11])
        o["_hsv"]=ism_hsv.load_cache(os.path.join(os.path.dirname(o["cls_cache"]),
                                     f"{o['name']}_hsv.npz"), o["template_dir"])[0]
    return defaults, objs, model, seg, pool, dev


def score_frame(bgr, prompt_of, objs, model, seg, pool, dev, ycache, W):
    """프레임 1장 → {obj: dict(box,conf,sem,appe,hsv,accept)}"""
    from ultralytics import YOLOWorld
    rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB); norm=yi.normalize_rgb(rgb); H,Wd=bgr.shape[:2]
    items=[]
    for o in objs:
        pr=prompt_of[o["name"]]
        if pr not in ycache:
            y=YOLOWorld(W); y.set_classes([pr]); ycache[pr]=y
        r=ycache[pr].predict(bgr,conf=CONF,imgsz=IMGSZ,verbose=False,device=dev)
        bx=[]
        if len(r) and r[0].boxes is not None and len(r[0].boxes):
            b=r[0].boxes
            for j in range(len(b)):
                xy=b.xyxy[j].tolist()
                x1=max(0,min(int(xy[0]),Wd-1)); y1=max(0,min(int(xy[1]),H-1))
                x2=max(x1+1,min(int(xy[2]),Wd)); y2=max(y1+1,min(int(xy[3]),H))
                bx.append(((x1,y1,x2,y2),float(b.conf[j])))
        bx.sort(key=lambda t:-t[1])
        for box,cf in bx[:TOPK]: items.append((o["name"],box,cf))
    if not items: return {}
    crops,keep=[],[]
    for i,(nm,box,cf) in enumerate(items):
        c=yi.crop_resize_pad(norm,list(box))
        if c is not None: crops.append(c); keep.append(i)
    if not crops: return {}
    cls_all,patch_all=o_n.dinov2_blocks_forward(model,crops,dev,[11])
    masks=yi.segment_boxes(seg,bgr,[list(items[i][1]) for i in keep],dev)
    by={o["name"]:o for o in objs}; best={}
    for ci,i in enumerate(keep):
        nm,box,cf=items[i]; o=by[nm]
        sims=(o["_tcls"]@cls_all[ci].to(dev)).cpu()
        sem=float(torch.topk(sims,min(5,sims.numel())).values.mean()); t1=int(torch.argmax(sims))
        x1,y1,x2,y2=box; mask=masks[ci]; qp=patch_all[11][ci]
        fg=yi.masked_query_patches(qp.cpu(),mask,list(box),pool)[0].to(dev) if mask is not None else qp
        if fg.shape[0]==0: appe=0.0
        else:
            sim=fg@o["_flat11"].T
            per=torch.full((fg.shape[0],o["_nview"]),-1.0,device=dev,dtype=sim.dtype)
            per.scatter_reduce_(1,o["_seg"].unsqueeze(0).expand(fg.shape[0],-1),sim,reduce="amax")
            appe=float(per.mean(0).clamp(0,1)[t1])
        mc=mask[y1:y2,x1:x2] if mask is not None else None
        hsv=1.0
        if o["_hsv"] is not None and mc is not None and mc.sum()>=1:
            try: hsv=float(ism_hsv.similarity(ism_hsv.query_hist(bgr[y1:y2,x1:x2],mc.astype(np.uint8)),o["_hsv"]))
            except Exception: hsv=1.0
        if nm not in best or sem>best[nm]["sem"]:
            acc = sem>=float(o["similarity_threshold"]) and appe>=float(o["appe_gate"]) and hsv>=T
            best[nm]=dict(box=box,conf=cf,sem=sem,appe=appe,hsv=hsv,accept=acc)
    return best


def draw(bgr,best):
    img=bgr.copy()
    for nm,b in sorted(best.items()):
        x1,y1,x2,y2=b["box"]; col=COL.get(nm,(200,200,200))
        if b["accept"]:
            cv2.rectangle(img,(x1,y1),(x2,y2),col,3)
            cv2.rectangle(img,(x1,max(0,y1-20)),(x1+9*len(nm.replace('_high','')),y1),col,-1)
            cv2.putText(img,nm.replace('_high',''),(x1+2,max(13,y1-5)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.42,(0,0,0),1)
        else:
            d=tuple(int(c*0.45) for c in col)
            cv2.rectangle(img,(x1,y1),(x2,y2),d,1)
    return img


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--limit",type=int,default=0); a=ap.parse_args()
    os.makedirs(VIZ,exist_ok=True); os.makedirs(os.path.join(OUT,"csv"),exist_ok=True)
    gt=p21.load_gt(); defaults,objs,model,seg,pool,dev=build()
    W=defaults.get("weights","yolov8m-worldv2.pt")
    cur={o["name"]:o["yolo_prompt"] for o in objs}
    v3=dict(cur)
    for r in csv.DictReader(open(V3CSV)):
        if r["v3_selected"]: v3[r["object"]]=r["v3_selected"]
    print("현행:",cur); print("v3  :",v3)
    ycA,ycB={},{}
    frames=sorted(gt.keys())
    if a.limit: frames=frames[:a.limit]
    man=[]; agg={"A":[0,0,0],"B":[0,0,0]}
    for n,(ds,fr) in enumerate(frames,1):
        bgr=cv2.imread(os.path.join(FRAMES,ds,f"frame_{fr:06d}.png"))
        if bgr is None: continue
        bA=score_frame(bgr,cur,objs,model,seg,pool,dev,ycA,W)
        bB=score_frame(bgr,v3,objs,model,seg,pool,dev,ycB,W)
        vis=gt[(ds,fr)]
        for tag,bb in (("A",bA),("B",bB)):
            for o in p21.OBJECTS:
                acc=o in bb and bb[o]["accept"]; v=o in vis
                if v and acc: agg[tag][0]+=1
                elif acc: agg[tag][1]+=1
                elif v: agg[tag][2]+=1
        combo=np.hstack([bgr,draw(bgr,bA),draw(bgr,bB)])
        fn=f"{ds}_{fr:06d}.png"; cv2.imwrite(os.path.join(VIZ,fn),combo)
        accA=sorted(o for o in bA if bA[o]["accept"]); accB=sorted(o for o in bB if bB[o]["accept"])
        man.append({"name":f"{ds}_{fr:06d}","bag":ds,"frame":fr,
                    "gt_visible":";".join(sorted(vis)),
                    "A_accept":";".join(accA),"B_accept":";".join(accB),
                    "changed":int(accA!=accB),"image":fn})
        if n%40==0: print(f"  {n}/{len(frames)}")
    with open(os.path.join(OUT,"csv","box_compare_manifest.csv"),"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(man[0].keys())); w.writeheader(); w.writerows(man)
    res={}
    for t in ("A","B"):
        TP,FP,FN=agg[t]; P=TP/max(TP+FP,1); R=TP/max(TP+FN,1)
        res[t]=dict(TP=TP,FP=FP,FN=FN,precision=round(P,4),recall=round(R,4),
                    f1=round(2*P*R/max(P+R,1e-9),4))
    json.dump(res,open(os.path.join(OUT,"metrics","prompt_v3_ab.json"),"w"),indent=2)
    print("\n=== A(현행) vs B(CLIP v3) ===")
    for t,m in res.items(): print(f"  {t}: TP {m['TP']} FP {m['FP']} FN {m['FN']} F1 {m['f1']}")
    dT=res["B"]["TP"]-res["A"]["TP"]; dF=res["B"]["FP"]-res["A"]["FP"]
    print(f"  Δ(B-A) TP {dT:+d} FP {dF:+d} F1 {res['B']['f1']-res['A']['f1']:+.4f}")
    print(f"  판정(FP<=86 & TP↑): {'PASS' if res['B']['FP']<=86 and dT>0 else 'FAIL'}")
    print(f"  결과 달라진 프레임: {sum(m['changed'] for m in man)}/{len(man)}")
    print(f"-> {VIZ}")


if __name__=="__main__":
    main()
