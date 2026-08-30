#!/usr/bin/env python3
"""dump_yoloe.py — YOLOE(같은 ultralytics 패키지)로 동일 프레임의 proposal 을 덤프한다 (READ-ONLY).

YOLO-World 를 교체하는 것이 아니라 **localization 이 실제로 더 좋아지는지 실측**하기 위한 비교군이다.
설정
  yoloe_txt   현행과 동일한 10개 프롬프트, text prompt 모드, imgsz 960
  yoloe_pf    prompt-free 모드 (내장 어휘) — class-agnostic proposal 근사
운영 코드·config 는 건드리지 않는다.
"""
import argparse, csv, os, sys, time
from collections import defaultdict
from pathlib import Path
import cv2, numpy as np, torch

HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
RSRCH=os.path.dirname(ROOT); REPO=os.path.dirname(RSRCH)
sys.path.insert(0,REPO)
import yolo_ism_object_n as o_n
GT=os.path.join(RSRCH,"gt_input"); OUT=os.path.join(ROOT,"candidate_dumps")
CONV=os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
DATASETS=["sam_105018","sam_105314","sam_105652","sam_110104","sam_110532","sam_110633"]
CONF=0.005

def bag_frames(ds,want):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts=get_typestore(Stores.ROS2_HUMBLE); hi=max(want)
    with AnyReader([Path(os.path.join(CONV,ds))],default_typestore=ts) as rd:
        conns=[c for c in rd.connections if c.topic=="/camera/camera/color/image_raw"]; i=-1
        for conn,t,raw in rd.messages(connections=conns):
            i+=1
            if i>hi: break
            if i not in want: continue
            m=rd.deserialize(raw,conn.msgtype)
            b=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.width,3)
            yield i,(cv2.cvtColor(b,cv2.COLOR_RGB2BGR) if m.encoding.lower()=="rgb8" else b.copy())

ap=argparse.ArgumentParser(); ap.add_argument("--mode",default="txt"); a=ap.parse_args()
want=defaultdict(set)
for r in csv.DictReader(open(os.path.join(GT,"user_visibility_gt.csv"),encoding="utf-8")):
    if r["user_reviewed"]=="yes": want[r["dataset_name"]].add(int(r["frame_id"]))
defaults,objs=o_n.load_config(o_n.DEFAULT_CONFIG)
dev=defaults.get("device","cuda:0") if torch.cuda.is_available() else "cpu"
CURP=[o["yolo_prompt"] for o in objs]; P2O={o["yolo_prompt"]:o["name"] for o in objs}
from ultralytics import YOLOE
cfg=f"yoloe_{a.mode}"
if a.mode=="txt":
    m=YOLOE("yoloe-11l-seg.pt"); m.set_classes(CURP, m.get_text_pe(CURP)); names=CURP
else:
    m=YOLOE("yoloe-11l-seg-pf.pt"); names=None
os.makedirs(os.path.join(OUT,cfg),exist_ok=True)
t0=time.time(); tot=0
for ds in DATASETS:
    rows=[]
    for fi,bgr in bag_frames(ds,want[ds]):
        r=m.predict(bgr,conf=CONF,imgsz=960,max_det=300,verbose=False,device=dev)
        if len(r) and r[0].boxes is not None and len(r[0].boxes):
            b=r[0].boxes; nm=r[0].names
            for j in range(len(b)):
                xy=b.xyxy[j].tolist(); ci=int(b.cls[j])
                src=names[ci] if names else str(nm.get(ci,ci))
                rows.append({"frame_id":fi,"source":src,"target":P2O.get(src,""),
                    "x1":max(0,int(xy[0])),"y1":max(0,int(xy[1])),
                    "x2":int(xy[2]),"y2":int(xy[3]),"conf":round(float(b.conf[j]),4)})
    with open(os.path.join(OUT,cfg,f"{ds}.csv"),"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=["frame_id","source","target","x1","y1","x2","y2","conf"])
        w.writeheader(); w.writerows(rows)
    tot+=len(rows)
print(f"{cfg}: 박스 {tot:,}  ({time.time()-t0:.0f}s)")
