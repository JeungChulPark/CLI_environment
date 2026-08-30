#!/usr/bin/env python3
"""eval_recovery.py — '현행이 놓친 위치를 새 설정이 찾는가' 를 순환 없이 측정한다 (READ-ONLY).

문제: 현행이 못 찾은 위치에는 라벨이 없다(라벨은 전부 현행 박스에 붙었다).
해법: **인접 프레임 기대 위치**. 직전 연구 E3 과 같은 논리다.
      (ds, f, obj) 가 '정답 위치 후보 없음' 으로 판정됐고, f±W 안에 그 객체가 **운영에서
      수락된** 프레임이 있으면 그 박스를 기대 위치로 삼는다. 카메라가 움직이므로 완벽하지
      않지만, 어떤 설정이 그 근처에 박스를 만드는지는 설정 간 **공정 비교**가 된다.
      현행도 같은 기준으로 채점되므로 순환이 아니다.

산출: results/recovery_experiment.csv, recommended_cases.csv
"""
import csv, os
from collections import defaultdict
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
RSRCH=os.path.dirname(ROOT); OBS=os.path.join(RSRCH,"ism_accuracy_observation")
RESID=os.path.join(RSRCH,"ism_residual_accuracy_research"); GT=os.path.join(RSRCH,"gt_input")
DUMP=os.path.join(ROOT,"candidate_dumps"); RES=os.path.join(ROOT,"results")
DATASETS=["sam_105018","sam_105314","sam_105652","sam_110104","sam_110532","sam_110633"]
W=5; IOU_REC=0.3; CONF=0.02
def iou(a,b):
    ix1,iy1=max(a[0],b[0]),max(a[1],b[1]); ix2,iy2=min(a[2],b[2]),min(a[3],b[3])
    iw,ih=max(0,ix2-ix1),max(0,iy2-iy1); inter=iw*ih
    u=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/u if u>0 else 0.0
coord={}; cand=defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(f"{OBS}/frame_dumps/{ds}_boxes.csv")):
        coord[r["uid"]]=(int(r["x1"]),int(r["y1"]),int(r["x2"]),int(r["y2"]))
    for r in csv.DictReader(open(f"{OBS}/frame_dumps/{ds}_pairs.csv")):
        if int(r["is_candidate"])==1:
            cand[(ds,int(r["frame_id"]),r["object"])].append(
                {"uid":r["uid"],"sem":float(r["sem_top5"]),"appe":float(r["appe11_clstop1"]),
                 "st":float(r["sim_thr"]),"ag":float(r["appe_gate"])})
acc=defaultdict(dict)
for k,v in cand.items():
    b=max(v,key=lambda c:c["sem"])
    if b["sem"]>=b["st"] and b["appe"]>=b["ag"]: acc[(k[0],k[2])][k[1]]=b["uid"]
TGT=[]
for r in csv.DictReader(open(f"{RESID}/results/fn345_stage_breakdown.csv")):
    if r["stage"] not in ("F2","F3","F4","F5","F6"): continue
    ds,fr,ob=r["dataset"],int(r["frame_id"]),r["object"]
    ref=acc.get((ds,ob),{})
    near=sorted([(abs(f2-fr),f2) for f2 in ref if 0<abs(f2-fr)<=W])
    if near: TGT.append((ds,fr,ob,coord[ref[near[0][1]]],near[0][0],r["stage"]))
print(f"기대 위치를 세울 수 있는 미검출 사례 {len(TGT)}건 (전체 F2~F6 대비)")
def load(cfg):
    d=os.path.join(DUMP,cfg)
    if not os.path.isdir(d): return None
    box=defaultdict(list)
    for ds in DATASETS:
        p=os.path.join(d,f"{ds}.csv")
        if not os.path.isfile(p): continue
        for r in csv.DictReader(open(p)):
            if float(r["conf"])<CONF: continue
            box[(ds,int(r["frame_id"]))].append((r["target"],r["source"],
                (int(r["x1"]),int(r["y1"]),int(r["x2"]),int(r["y2"])),float(r["conf"])))
    return box
CFG=["cur","cur_1280","cur_1920","ml960","ml960_agn","generic","ext","tile2x2","yoloe_txt","yoloe_pf"]
BOX={c:load(c) for c in CFG}; BOX={k:v for k,v in BOX.items() if v}
for cs,n in [(["cur","ext"],"union_cur_ext"),(["cur","generic"],"union_cur_generic"),
             (["cur","tile2x2"],"union_cur_tile"),(["cur","yoloe_txt"],"union_cur_yoloe")]:
    if all(c in BOX for c in cs):
        m=defaultdict(list)
        for c in cs:
            for k,v in BOX[c].items(): m[k].extend(v)
        BOX[n]=m
rows=[]; cases=[]
for cfg,box in BOX.items():
    own=any_=0
    for ds,fr,ob,eb,dist,st in TGT:
        bs=box.get((ds,fr),[])
        o=[x for x in bs if x[0]==ob]
        oh=any(iou(x[2],eb)>=IOU_REC for x in o)
        ah=any(iou(x[2],eb)>=IOU_REC for x in bs)
        own+=oh; any_+=ah
        if oh and cfg!="cur":
            best=max((x for x in o if iou(x[2],eb)>=IOU_REC),key=lambda x:iou(x[2],eb))
            cases.append({"config":cfg,"dataset":ds,"frame_id":fr,"object":ob,"orig_stage":st,
                "expected_from_frame_dist":dist,"expected_bbox":";".join(map(str,eb)),
                "found_bbox":";".join(map(str,best[2])),"source_prompt":best[1],
                "conf":round(best[3],4),"iou":round(iou(best[2],eb),3)})
    rows.append({"config":cfg,"n_target":len(TGT),"recover_own":own,
        "recover_own_rate":round(own/max(len(TGT),1),4),"recover_any":any_,
        "recover_any_rate":round(any_/max(len(TGT),1),4)})
rows.sort(key=lambda r:-r["recover_own"])
with open(os.path.join(RES,"recovery_experiment.csv"),"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
if cases:
    with open(os.path.join(RES,"recommended_cases.csv"),"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(cases[0].keys())); w.writeheader(); w.writerows(cases)
print(f"\n=== 미검출 사례 복구율 (인접 프레임 기대 위치, IoU>={IOU_REC})")
print(f"{'설정':22s} {'own복구':>8} {'own율':>7} {'any복구':>8} {'any율':>7}")
for r in rows:
    print(f"{r['config']:22s} {r['recover_own']:>8} {r['recover_own_rate']:>7.3f} "
          f"{r['recover_any']:>8} {r['recover_any_rate']:>7.3f}")
print(f"\n검수 후보(현행 외 설정이 복구한 사례) {len(cases)}건 -> recommended_cases.csv")
