#!/usr/bin/env python3
"""make_vis.py — 대표 사례 시각화 (READ-ONLY)."""
import csv, os
from collections import defaultdict
from pathlib import Path
import cv2, numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
RSRCH=os.path.dirname(ROOT); GT=os.path.join(RSRCH,"gt_input")
OBS=os.path.join(RSRCH,"ism_accuracy_observation"); RES=os.path.join(ROOT,"results")
VIS=os.path.join(ROOT,"visualizations"); os.makedirs(VIS,exist_ok=True)
DUMP=os.path.join(ROOT,"candidate_dumps")
def frame(ds,fr):
    p=os.path.join(GT,"frames",ds,f"frame_{fr:06d}.png")
    return cv2.imread(p) if os.path.isfile(p) else None
def boxes(cfg,ds,fr,obj=None):
    p=os.path.join(DUMP,cfg,f"{ds}.csv")
    if not os.path.isfile(p): return []
    out=[]
    for r in csv.DictReader(open(p)):
        if int(r["frame_id"])!=fr or float(r["conf"])<0.02: continue
        if obj and r["target"]!=obj: continue
        out.append(((int(r["x1"]),int(r["y1"]),int(r["x2"]),int(r["y2"])),float(r["conf"]),r["source"]))
    return out
def draw(im,bs,color,label=""):
    for (b,c,s) in bs:
        cv2.rectangle(im,(b[0],b[1]),(b[2],b[3]),color,2)
        cv2.putText(im,f"{s[:14]} {c:.2f}",(b[0],max(12,b[1]-4)),
            cv2.FONT_HERSHEY_SIMPLEX,0.4,color,1,cv2.LINE_AA)
    return im
# 1) 복구 사례: 현행은 못 찾고 ext/tile 은 찾은 것
R=list(csv.DictReader(open(os.path.join(RES,"recommended_cases.csv"))))
seen=set(); n=0; panels=[]
for r in R:
    k=(r["dataset"],r["frame_id"],r["object"])
    if k in seen or r["config"] not in ("ext","tile2x2"): continue
    seen.add(k)
    im=frame(r["dataset"],int(r["frame_id"]))
    if im is None: continue
    a=im.copy(); b=im.copy()
    draw(a,boxes("cur",r["dataset"],int(r["frame_id"]),r["object"]),(0,0,255))
    draw(b,boxes(r["config"],r["dataset"],int(r["frame_id"]),r["object"]),(0,220,0))
    eb=tuple(map(int,r["expected_bbox"].split(";")))
    for im2 in (a,b): cv2.rectangle(im2,(eb[0],eb[1]),(eb[2],eb[3]),(0,200,255),2)
    for im2,t in ((a,f"cur (miss)"),(b,f"{r['config']} (recovered)")):
        cv2.rectangle(im2,(0,0),(im2.shape[1],24),(0,0,0),-1)
        cv2.putText(im2,f"{t}  {r['dataset']} f{r['frame_id']} {r['object']}",(5,17),
            cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
    panels.append(np.hstack([a,b])); n+=1
    if n>=6: break
if panels:
    h=min(p.shape[0] for p in panels); w=min(p.shape[1] for p in panels)
    cv2.imwrite(os.path.join(VIS,"recovered_by_ext_tile.jpg"),
        np.vstack([cv2.resize(p,(w,h)) for p in panels]),[cv2.IMWRITE_JPEG_QUALITY,90])
    print(f"recovered_by_ext_tile.jpg ({n} 사례)")
# 2) wrong-location 사례 (Sauce/choco)
T=[r for r in csv.DictReader(open(os.path.join(RES,"wrong_location_taxonomy.csv")))
   if r["L_type"] in ("L2","L3") and r["note"].strip()]
panels=[]; n=0
for r in T:
    im=frame(r["dataset"],int(r["frame_id"]))
    if im is None: continue
    draw(im,boxes("cur",r["dataset"],int(r["frame_id"]),r["object"]),(0,0,255))
    cv2.rectangle(im,(0,0),(im.shape[1],24),(0,0,0),-1)
    cv2.putText(im,f"{r['L_type']} {r['dataset']} f{r['frame_id']} target={r['object']}",(5,17),
        cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1,cv2.LINE_AA)
    panels.append(im); n+=1
    if n>=6: break
if panels:
    h=min(p.shape[0] for p in panels); w=min(p.shape[1] for p in panels)
    rs=[cv2.resize(p,(w,h)) for p in panels]
    cv2.imwrite(os.path.join(VIS,"wrong_location_examples.jpg"),
        np.vstack([np.hstack(rs[i:i+2]) for i in range(0,len(rs)-1,2)]),
        [cv2.IMWRITE_JPEG_QUALITY,90])
    print(f"wrong_location_examples.jpg ({n} 사례)")
