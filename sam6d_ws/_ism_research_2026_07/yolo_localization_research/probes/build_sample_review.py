#!/usr/bin/env python3
"""build_sample_review.py — 복구 사례 30건 표본 검수 큐 (READ-ONLY, 전수 라벨링 아님).

묻는 것 하나: "초록 상자 안에 그 객체가 실제로 있는가?"
  주황 = 인접 프레임에서 가져온 **기대 위치**(근사)
  초록 = 새 설정이 만든 **복구 박스**
  빨강 = 현행이 그 객체 후보로 만든 박스 (없을 수 있음)
BBox 를 새로 그리게 하지 않는다.
"""
import csv, os, random
from collections import defaultdict
import cv2, numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
RSRCH=os.path.dirname(ROOT); GT=os.path.join(RSRCH,"gt_input")
RES=os.path.join(ROOT,"results"); HR=os.path.join(ROOT,"human_review")
DUMP=os.path.join(ROOT,"candidate_dumps")
os.makedirs(os.path.join(HR,"crops"),exist_ok=True)
N=30; rng=random.Random(0)
R=list(csv.DictReader(open(os.path.join(RES,"recommended_cases.csv"))))
# (ds,frame,object) 중복 제거 — 대표 config 는 권장안 우선
PRI={"yoloe_txt":0,"union_cur_yoloe":0,"ext":1,"union_cur_ext":1,"tile2x2":2,
     "union_cur_tile":2,"cur_1280":3,"cur_1920":3,"ml960":4,"ml960_agn":4,"union_cur_generic":4}
best={}
for r in R:
    k=(r["dataset"],r["frame_id"],r["object"])
    if k not in best or PRI.get(r["config"],9)<PRI.get(best[k]["config"],9): best[k]=r
U=list(best.values())
print(f"중복 제거 후 고유 사례 {len(U)}건")
# 층화: 권장안(YOLOE) 우선 + 객체 다양성
byo=defaultdict(list)
for r in U: byo[r["object"]].append(r)
sel=[]
for o in sorted(byo,key=lambda o:-len(byo[o])):
    pool=sorted(byo[o],key=lambda r:PRI.get(r["config"],9))
    take=max(2,round(N*len(byo[o])/len(U)))
    sel+= rng.sample(pool[:max(take*3,take)],min(take,len(pool)))
sel=sel[:N]
def frame(ds,fr):
    p=os.path.join(GT,"frames",ds,f"frame_{int(fr):06d}.png")
    return cv2.imread(p) if os.path.isfile(p) else None
def curbox(ds,fr,ob):
    p=os.path.join(DUMP,"cur",f"{ds}.csv"); out=[]
    if os.path.isfile(p):
        for r in csv.DictReader(open(p)):
            if int(r["frame_id"])==int(fr) and r["target"]==ob and float(r["conf"])>=0.02:
                out.append((int(r["x1"]),int(r["y1"]),int(r["x2"]),int(r["y2"]),float(r["conf"])))
    return out
rows=[]
for i,r in enumerate(sorted(sel,key=lambda x:(x["dataset"],int(x["frame_id"]))),1):
    im=frame(r["dataset"],r["frame_id"])
    if im is None: continue
    cid=f"S{i:03d}"
    eb=tuple(map(int,r["expected_bbox"].split(";")))
    fb=tuple(map(int,r["found_bbox"].split(";")))
    cv2.rectangle(im,(eb[0],eb[1]),(eb[2],eb[3]),(0,165,255),2)
    for (x1,y1,x2,y2,c) in curbox(r["dataset"],r["frame_id"],r["object"]):
        cv2.rectangle(im,(x1,y1),(x2,y2),(0,0,220),2)
    cv2.rectangle(im,(fb[0],fb[1]),(fb[2],fb[3]),(0,230,0),3)
    cv2.rectangle(im,(0,0),(im.shape[1],46),(0,0,0),-1)
    cv2.putText(im,f"{cid}  {r['dataset']} f{r['frame_id']}   TARGET = {r['object']}",(6,18),
        cv2.FONT_HERSHEY_SIMPLEX,0.55,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(im,"green=recovered box   orange=expected(from adjacent frame)   red=current",(6,38),
        cv2.FONT_HERSHEY_SIMPLEX,0.45,(180,180,180),1,cv2.LINE_AA)
    p=os.path.join(HR,"crops",f"{cid}.png"); cv2.imwrite(p,im)
    rows.append({"case_id":cid,"dataset":r["dataset"],"frame_id":r["frame_id"],
        "object":r["object"],"config":r["config"],"source_prompt":r["source_prompt"],
        "conf":r["conf"],"iou_with_expected":r["iou"],
        "expected_from_frame_dist":r["expected_from_frame_dist"],
        "found_bbox":r["found_bbox"],"image_path":p,
        "review_label":"","reviewed":"no","note":""})
with open(os.path.join(HR,"sample_review_queue.csv"),"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
import collections
print(f"표본 {len(rows)}건 -> human_review/sample_review_queue.csv")
print("  설정별:",collections.Counter(r["config"] for r in rows).most_common())
print("  객체별:",collections.Counter(r["object"] for r in rows).most_common())
