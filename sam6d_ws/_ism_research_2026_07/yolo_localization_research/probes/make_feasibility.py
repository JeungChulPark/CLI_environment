#!/usr/bin/env python3
"""make_feasibility.py — temporal/depth 타당성과 detector 비교표를 실측 기반으로 작성 (READ-ONLY)."""
import csv, os, glob
from collections import defaultdict
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(HERE)
RSRCH=os.path.dirname(ROOT); RES=os.path.join(ROOT,"results")
OBS=os.path.join(RSRCH,"ism_accuracy_observation")
# --- depth 가용성 실측
CONV=os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
import subprocess
topics=set()
try:
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts=get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(os.path.join(CONV,"sam_105314"))],default_typestore=ts) as rd:
        for c in rd.connections: topics.add((c.topic,c.msgtype))
except Exception as e: topics.add(("(읽기 실패)",str(e)))
depth=[t for t,_ in topics if "depth" in t.lower()]
print("bag 토픽:", sorted(t for t,_ in topics))
print("depth 토픽:", depth or "없음")

TD=[
 {"방법":"F 이전 프레임 BBox 유지","구현 가능":"가능","필요 자원":"없음(기존 후보 재사용)",
  "실측 근거":"인접 프레임 수락박스 IoU 중앙 0.452, IoU>0.7 32.3% (직전 연구)",
  "예상 효과":"낮음 — 3fps 사무실 스윕이라 단순 복사는 위치가 어긋난다",
  "위험":"오답 BBox 시간축 전파","판정":"단독 기각"},
 {"방법":"F optical flow 전파","구현 가능":"가능(opencv 내장)","필요 자원":"CPU, 프레임당 수 ms",
  "실측 근거":"미측정 — flow 정확도를 이 데이터로 재지 않았다",
  "예상 효과":"현재 산출물로는 확인할 수 없음","위험":"큰 카메라 이동에서 누적 오차","판정":"보류"},
 {"방법":"F lightweight tracker (CSRT/KCF)","구현 가능":"가능(opencv-contrib 필요)",
  "필요 자원":"객체당 CPU 트래커","실측 근거":"미측정",
  "예상 효과":"현재 산출물로는 확인할 수 없음","위험":"드리프트·재초기화 정책 필요","판정":"보류"},
 {"방법":"F 후보 급소실 시에만 fallback","구현 가능":"가능","필요 자원":"없음",
  "실측 근거":"고립 dropout 410건 중 raw 부재 0.2%(직전 관찰) — 급소실 자체가 드물다",
  "예상 효과":"낮음 — 대상 사례가 적다","위험":"낮음","판정":"우선순위 낮음"},
 {"방법":"H depth foreground 분리","구현 가능":"조건부",
  "필요 자원":"정렬된 depth 프레임","실측 근거":f"현재 GT bag 토픽: {', '.join(sorted(t for t,_ in topics))[:120]}",
  "예상 효과":"BBox 가 독립 객체를 가리키는지 검증에는 유효 가능",
  "위험":"작은 객체·반사·depth hole","판정":"depth 토픽 확인 필요"},
 {"방법":"H depth cluster 로 신규 후보 생성","구현 가능":"조건부","필요 자원":"정렬 depth + 클러스터링",
  "실측 근거":"미측정","예상 효과":"현재 산출물로는 확인할 수 없음",
  "위험":"책상 위 밀집 객체가 하나로 뭉침","판정":"보류"},
]
with open(os.path.join(RES,"temporal_depth_feasibility.csv"),"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(TD[0].keys())); w.writeheader(); w.writerows(TD)

# --- detector 비교 (실측 + 조사)
M=list(csv.DictReader(open(os.path.join(RES,"method_comparison.csv"))))
R=list(csv.DictReader(open(os.path.join(RES,"recovery_experiment.csv"))))
def get(c,rows,k): 
    for r in rows:
        if r["config"]==c: return r[k]
    return ""
AD=[
 {"detector":"YOLO-World v2-m (현행)","설치":"이미 설치","크기":"57 MB","추론(339프레임)":"38 s",
  "박스/프레임":get("cur",M,"n_box_per_frame"),"미검출 복구율(own)":get("cur",R,"recover_own_rate"),
  "미검출 복구율(any)":get("cur",R,"recover_any_rate"),"open-vocab":"예",
  "통합 난이도":"현행","근거":"실측"},
 {"detector":"YOLOE-11l-seg (text prompt)","설치":"**같은 ultralytics 패키지**(8.4.64) — 신규 의존성 0",
  "크기":"71 MB (+ MobileCLIP 572 MB, 최초 1회)","추론(339프레임)":"5 s",
  "박스/프레임":get("yoloe_txt",M,"n_box_per_frame"),
  "미검출 복구율(own)":get("yoloe_txt",R,"recover_own_rate"),
  "미검출 복구율(any)":get("yoloe_txt",R,"recover_any_rate"),"open-vocab":"예(text/visual/prompt-free)",
  "통합 난이도":"낮음 — YOLOWorld→YOLOE 교체 + set_classes 시그니처만 다름","근거":"실측"},
 {"detector":"Grounding DINO","설치":"별도 설치 필요(transformers 또는 원저장소)","크기":"~700 MB(Swin-T)",
  "추론(339프레임)":"미측정","박스/프레임":"미측정","미검출 복구율(own)":"미측정",
  "미검출 복구율(any)":"미측정","open-vocab":"예(phrase grounding)",
  "통합 난이도":"중간 — 별도 파이프라인","근거":"조사만 (설치·실행 안 함)"},
 {"detector":"OWLv2","설치":"transformers 로 가능","크기":"~600 MB",
  "추론(339프레임)":"미측정","박스/프레임":"미측정","미검출 복구율(own)":"미측정",
  "미검출 복구율(any)":"미측정","open-vocab":"예","통합 난이도":"중간","근거":"조사만"},
 {"detector":"SAM automatic mask generator","설치":"이미 설치(mobile_sam/sam_b)","크기":"41/375 MB",
  "추론(339프레임)":"미측정(전수 마스크는 비쌈)","박스/프레임":"미측정",
  "미검출 복구율(own)":"해당없음(클래스 없음)","미검출 복구율(any)":"미측정",
  "open-vocab":"아니오(class-agnostic)","통합 난이도":"낮음 — 이미 MobileSAM 사용 중",
  "근거":"조사만 — generic 프롬프트가 any 0.487 을 낸 것과 비교 필요"},
]
with open(os.path.join(RES,"alternative_detector_comparison.csv"),"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(AD[0].keys())); w.writeheader(); w.writerows(AD)
print("\n-> temporal_depth_feasibility.csv, alternative_detector_comparison.csv")
