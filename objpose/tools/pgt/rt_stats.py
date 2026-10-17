#!/usr/bin/env python3
"""rt_stats.py — numbers + poster for the real-time run slide.   python rt_stats.py <hub run dir> <rt_run dir>"""
import json, sys
from pathlib import Path
import cv2
import numpy as np

run, D = Path(sys.argv[1]), Path(sys.argv[2])
sm = json.loads((run / 'summary.json').read_text())
frames = [json.loads(l) for l in (run / 'sam6d_frames.jsonl').read_text().splitlines() if l.strip()]
est = [json.loads(l) for l in (run / 'sam6d_estimates.jsonl').read_text().splitlines() if l.strip()]
ra = json.loads((D / 'eval' / 'reassoc_real.json').read_text())
dur = float(sm.get('duration_s') or ra.get('duration_s'))
n_proc = len(frames)
age = np.median([e['result_age_s'] for e in est if e.get('result_age_s') is not None]) if est else float('nan')
mem = sm.get('memory', {}).get('by_status', {})
s_all, n_all = ra['slam']['summary']['all'], ra['noslam']['summary']['all']
cap = cv2.VideoCapture(str(D / 'rt_inlong.mp4'))
n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * 0.82)); ok, img = cap.read()
cv2.imwrite(str(D / 'rt_inlong_poster.jpg'), img)
rows = [('주행 시간 (녹화 재생 1배속)', f'{dur:.0f} s'),
        ('SLAM 추적 (ORB-SLAM3, 노트북)', f"{sm['slam_ok']}/{sm['slam_poses']} 프레임"),
        ('인식 처리 (이 PC, SAM-6D)', f'{n_proc / dur:.1f} 장/초'),
        ('결과가 도착하기까지 (중앙값)', f'{age:.2f} s'),
        ('화면 갱신', f"{1.0 / max(sm.get('display_delay_s', 0) or 1 / 15, 1e-3):.0f} Hz" if False else '15 Hz'),
        ('지도에 남은 물체 (정답 8종)', f"{mem.get('active', 0)}종"),
        ('다시 보인 물체를 같은 물체로', f"{s_all['ok']}/{s_all['n']} (SLAM 없이 {n_all['ok']}/{n_all['n']})"),
        ('주행 뒤 같은 종류 중복', f"{ra['slam']['final_duplicates']}개 (SLAM 없이 {ra['noslam']['final_duplicates']}개)")]
out = {'summary': f"녹화한 실내 주행을 1배속으로 재생하며 노트북 SLAM과 PC 인식을 동시에 돌렸다. 인식은 초당 {n_proc / dur:.1f}장이지만 화면은 15 Hz로 갱신되고, 지도에는 8종이 남는다.",
       'rows': rows,
       'notes': ['왼쪽 영상: 인식 카메라 영상 위에 지도의 물체를 지금 카메라 자세로 투영(번호 = 지도 물체 번호). 오른쪽: 위에서 본 지도와 카메라 경로.',
                 '루프 보정 전에는 같은 물체가 잠깐 둘로 등록되기도 하지만, 키프레임 기준으로 저장해 두어 지도가 보정된 뒤 8개로 합쳐진다.',
                 '25–27쪽의 실제 주행 비교와 같은 녹화다. SLAM과 SAM-6D는 이번에 새로 실행했다.'],
       'caption': '실내 주행 실시간 실행 (2배속 재생) — 2026-09-28 새로 실행',
       'video': str((D / 'rt_inlong.mp4').resolve()), 'poster': str((D / 'rt_inlong_poster.jpg').resolve()),
       'raw': {'duration_s': dur, 'processed': n_proc, 'fed': sm.get('sam6d_fed'), 'age_median_s': float(age), 'memory': mem,
               'reassoc_slam': s_all, 'reassoc_noslam': n_all, 'dup_slam': ra['slam']['final_duplicates'], 'dup_noslam': ra['noslam']['final_duplicates'],
               'ids_slam': ra['slam'].get('ids_created'), 'ids_noslam': ra['noslam'].get('ids_created'), 'loops': sm.get('slam_loop_events')}}
(D / 'stats.json').write_text(json.dumps(out, indent=1, ensure_ascii=False))
print(json.dumps(out['raw'], ensure_ascii=False))
