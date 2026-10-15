# 작업기록 장비구성 재현

> 작업 기록 · 장비 구성 · 재현 안내 — 재시작 메모(08), 원고 v0.3 · 상태 전이 수식 · 저장소 사고(19), 다른 장비(4090 · 5090 · .41)에서 다시 돌리기와 4090 재현 결과(26), .41 한 대 구성 · FM-Fusion 재검출 · .192 드라이버 복구(28). 개별 문서 4개를 그대로 이어 붙인 묶음입니다(제목 단계만 한 칸 내림). 맨 앞의 '종합' 절은 구성 문서를 읽고 정리한 요약(`묶음요약/G_작업기록_장비구성_재현.md`)이고, 원문은 `개별문서/` 폴더에 있습니다. 생성 2026-10-15.

**이 묶음의 장**

- **08_재시작_메모** — 08. 재시작 메모 (2026-10-06 새벽, 사용자 요청으로 중단)
- **19_작업기록_원고v0.3_3080반복_261010** — 19. 작업 기록 — 원고 v0.3, 상태 전이 수식, RTX 3080 반복 측정 (2026-10-10)
- **26_다른장비_재현_안내_4090_5090_261011** — 26. 다른 장비에서 다시 돌리기 — 주말(10-10 · 10-11) 노트북 실험을 RTX 4090 · RTX 5090(Blackwell)으로 (2026-10-11)
- **28_작업기록_41단독_FMFusion재검출_192복구_261014** — 28. 작업 기록 — .41 한 대 구성, FM-Fusion 검출 재실행(.41 · .192), .192 드라이버 복구 (2026-10-14)

## 종합 — 이 묶음이 말하는 것 (2026-10-15 정리)

실험 결과가 아니라 **언제 무엇을 어느 장비에서 했고, 무엇이 막혔고 어떻게 풀었는지**를 적은 작업 기록과 재현 안내 네 개입니다. 원래 A 묶음(논문 기획)에 섞여 있던 것을 2026-10-15 에 따로 묶었습니다. 다른 장비에서 실험을 다시 돌리거나, 어떤 수치가 어느 장비 · 어느 날 것인지 확인할 때 봅니다.

| 문서 | 날짜 | 장비 | 내용 |
|---|---|---|---|
| 08 재시작 메모 | 10-06 새벽 | 노트북 + Mac | 사용자 요청으로 중단한 시점의 상태, 남은 일 순서, 결과 위치 |
| 19 작업 기록 | 10-10 | 노트북 | 저장소 동기화 사고(자기 참조 링크 3개, `template/` 삭제 → T7 에서 복구), **원고 v0.3**(인식기를 개선안 3 + 군집 먼저로), 5.1절 상태 전이 수식 (5)~(12), 3080 반복 측정 중단, 오후 작업(21 · 22 · 23 문서) |
| 26 다른 장비 재현 안내 | 10-11 → 10-14 | 4090 · 5090 · .41 | 주말 노트북 실험(19~25)을 다른 장비에서 돌리는 순서: 저장소 밖 자산(기준 방법 코드, 데이터, `summary.json`), 장비마다 고칠 곳 세 군데; §7 4090 에서 실제로 걸린 것, **§8 08 세트 4090 재현 결과**, §9 sudo 없는 .41 에서 ORB-SLAM3 빌드 |
| 28 작업 기록 | 10-14 | .41 · .192 | .41 한 대 구성(hub + ORB-SLAM3), FM-Fusion 검출 재실행(.41 · .192, 설정 파일 누락 사고 1건), .192 드라이버 575 → 580 으로 멈춤 해결 |

**장비와 역할 (문서 전체에서 쓰는 이름)**

| 이름 | 장비 | 역할 | 쓰인 문서 |
|---|---|---|---|
| 노트북 | RTX 3080 Ti Laptop (i9-12900HK) + Mac M4 Pro(ORB-SLAM3) | 10-04 ~ 10-11 확인용 실험 전부 | 04~08, 11, 13, 19~25 |
| 이 PC / .192 (`slam192`) | RTX 4090 ×4, 10.254.176.192 | 10-06 ~ 10-12 재측정, 로컬 ORB-SLAM3; 10-08 부터 .41 의 SLAM 클라이언트 | 09, 10, 12, 15, 16, 17, 18 A·B, 26, 27 |
| .41 | RTX PRO 6000 Blackwell ×4, 10.254.176.41, sudo 없음 | 10-08 부터 인식 서버(hub), Blackwell 재실행, 10-14 부터 ORB-SLAM3 도 한 머신 | 06 §9, 11 §4, 12 §5, 16 §4, 18 C·E, 15 §7, 20 §7, 21 §4, 23 §7, 27, 28 |
| .3 | RTX 4090 | FoundationPose · GigaPose 같은 GPU 비교 | 18 D, 17 표 1 |
| 5090 | RTX 5090 (예정) | 논문 최종 수치 3회 반복 | 03, 17 §5, 26 |

**막혔던 것과 해결(다음에 또 만날 때)**

- **.192 드라이버 멈춤**(10-07 · 08 · 13 · 14, 드라이버 575.57.08): GroundingDINO 검출(컴파일된 deformable-attention 커널)을 띄우면 1~3분 안에 nvidia-smi 무응답. 10-14 사용자가 **580.178.04** 로 올린 뒤 44분 실행 무사(28 §4, 27 객체지도 §9.1). 디스플레이 GPU(nvidia-smi 3번)는 피해서 씀.
- **.41 에 sudo 없음**: ORB-SLAM3 의존 라이브러리(libsqlite3-dev, nlohmann-json3-dev)를 `deps/install` 에 손으로 두고 `build_host.sh` 가 apt 를 건너뛰게 함(26 §9). bop_toolkit 의 VSD 렌더러는 EGL 벤더를 NVIDIA 로 강제(06 §9).
- **.41 DualMap 실패 10회**: conda 환경에 `mobileclip` 이 없어 검출기 초기화가 조용히 빠져나감(`'Detector' object has no attribute 'yolo'`) + "검출 0개 프레임 건너뛰기" 패치 누락. .192 의 패키지 · 파일을 복사해 해결(23 §7.4, 28 §1).
- **FM-Fusion 새 작업 폴더에 설정 파일 누락**: `config_*.yaml` · `likelihood_*.txt` · `splits/` 를 복사하지 않으면 융합이 빈 지도를 내고 채점기가 0점을 찍음(28 §3).
- **저장소 사고**(10-10): 서버 사본이 만든 자기 참조 심링크가 추적되어 노트북 `template/`(2 GB) 삭제 → T7 이동 디스크에서 복구(19 §1).
- **ORB-SLAM3 잘못된 루프 보정**(3 m 급): 해당 실행은 버리고 다시 돌리는 규칙(22 §0, 15 §2).

**재현 순서(26 §4):** 짧은 것(실데이터 5개 실시간, 약 30분)부터 돌려 환경을 확인하고, 긴 것(YCB-V 반복, 기준 방법)은 뒤에. 장비마다 고칠 곳은 `objpose/pc/catalog.py` 의 `DATASET_ROOT`, 스크립트의 장비 태그(`l3080` 등), `realdata/logs_live_261011/*.sh` 의 `REPO=` 경로.

**남은 일(28 §6):** 26 §7 · 27(서버·클라이언트) §4 의 ".192 멈춤 → .41 단독" 조치 해제 표기, 5090 에서 세 방법 최종 실행, `run_fmf_detect_41.sh` / `_192.sh` 를 호스트 변수 하나로 합치기.

[TOC]


---

<a id="08_재시작_메모"></a>

## 08. 재시작 메모 (2026-10-06 새벽, 사용자 요청으로 중단)

### 중단 시점의 상태
- 모든 실행을 멈췄습니다. GPU는 비어 있습니다.
- 끝난 결과는 `07_전체_비교표.md`(3.2절 YCB 36장, 3절 YCB 실시간, 3.3절 실제 데이터)에 있습니다.

### 진행 기록
- 2026-10-06 06시: 실제 데이터 6개 설정 완료. 결과는 `07_전체_비교표.md` 3.3절에 있습니다.
  - hybrid 실패 원인(불러오는 순서) 수정
  - Mac은 실행 중 2시간 caffeinate로 잠자기 방지

### 남은 일 (다시 시작할 때 순서)
1. ~~실제 데이터의 "원본 판정 + 우리 자세 검증"이 지도를 비우는 원인 찾기~~ (완료: 불러오는 순서 문제)
   - 증상: 모든 후보가 `candidate_verification_failed`, 일부는 PEM 입력 깊이 부족(`no valid PEM instances`)입니다.
   - 확인 스크립트: `src/realdata/debug_hybrid.py hybrid` (실제 프레임 한 장으로 재현)
   - 코드: `src/realdata/live_cores.py` (hybrid 모드)
2. ~~Mac 연결 복구 후 실제 데이터 남은 실행~~ (완료). 참고:: `src/realdata/run_real.sh` 안의 hybL05, hybS05(수정 후 재실행), hybS06
   - Mac IP가 바뀌었을 수 있습니다(192.168.219.111 응답 없음).
3. **자세 검증 경량화 실험** (시작 전에 중단): `src/ycbv/det_study/run_verify_cost.sh`
   - 검증 후보 100/50/30/10, 간격 3/10, fp16/bf16, 조합. 대기 조건 줄(`until grep ... real.log`)은 지우고 실행합니다.
4. 경량화 결과가 좋으면 우리 개선안을 그 설정으로 YCB와 실제 데이터 실시간 재측정
5. 최종 수치는 RTX 5090에서 장면별 3회 반복

### 실행 결과 위치
- YCB 36장: `src/ycbv/out/quick/` (요약 `ladder/quick_combo.py`)
- YCB 실시간: `objpose/output/ycbv_live_*`, 채점 `src/ycbv_live/results_*.json`, `results_penalized.json`
- 실제 데이터: `objpose/output/real_260901_*`, 채점 `src/realdata/results_real.json`

---

<a id="19_작업기록_원고v0.3_3080반복_261010"></a>

## 19. 작업 기록 — 원고 v0.3, 상태 전이 수식, RTX 3080 반복 측정 (2026-10-10)

> 노트북(RTX 3080 Ti Laptop)에서 한 작업입니다. 원고 수치는 그대로 RTX 4090 기준이고, 3080 결과는 확인용입니다. 4090·Blackwell 서버 실험(18 문서)은 서버에서 따로 진행합니다.

### 0. 요약

| 항목 | 상태 | 결과 위치 |
|---|---|---|
| 노트북 저장소를 원격 최신(52aa1a4)으로 맞춤 | 완료 (사고 1건, 1절) | `objpose/memory-dup-suppression` |
| 원고 v0.3 (영문·국문) — 인식기를 개선안 3 + 군집 먼저로 교체 | 완료 | `etrij/ETRIJ_manuscript_v0.3.*`, `ko/논문_국문_v0.3.*` |
| 원고 5.1 — 10-09 에 전개한 상태 전이 수식 반영 | 완료 | 식 (5)~(12) |
| YCB-V 12영상 3회 반복 (원본 0.5 / 개선안 3 + 군집 먼저), 3080 | **중단** (07:22, 4절) | `src/ycbv_live/results_3080/` |
| 공간 객체 상태 지도 비교 (제안 / FM-Fusion / DualMap) | 2단계 완료 | 20 문서 |
| 260901 실제 영상 3회 반복, 3080 | 대기 — `sam6d_realtime/template/` 복구 필요 | |

### 1. 저장소 동기화와 사고

- 노트북 브랜치가 원격보다 4커밋 뒤처져 있었습니다(80ef541 YCB-V 3회 반복, 0c539f3 군집 먼저 코드, 1fea233 17 문서, 52aa1a4 18 문서). 그래서 10-10 아침 첫 보고의 "YCB-V 반복 측정이 남음"은 틀린 내용이었습니다. 4090 에서는 10-07 에 이미 끝나 있었습니다(15 §4-4).
- 받아올 때 git 이 덮어쓰려던 추적 안 된 파일 17개를 먼저 비교했습니다.
  - 그림 12개는 원격과 내용이 같아 지웠습니다.
  - 결과 JSON 5개는 노트북 실행 값이라 원격과 달랐습니다. `src/ycbv_live/laptop_3080ti/` 로 옮겨 보관했습니다(g13p, g13p_nover, hybS05, origS_th05, orig_th05).
- **사고:** 0c539f3 이 서버 사본에서 만든 자기 참조 링크 3개를 추적하고 있었습니다.
  - `sam6d_realtime/template`
  - `sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints`
  - `sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints`

  세 폴더는 `.gitignore` 대상이라, git 은 받아올 때 경고 없이 로컬 폴더를 지우고 링크로 바꿨습니다.
  - **가중치는 다시 받아 복구했습니다.**

    | 파일 | 받은 곳 |
    |---|---|
    | DINOv2 ViT-S/14, ViT-L/14 | dl.fbaipublicfiles.com |
    | `sam-6d-pem-base.pth` | Google Drive `1joW9IvwsaRJYxoUmGo68dBVg-HcFNyI7` (HuggingFace 는 401) |
    | `FastSAM-x.pt` | `ycbv_work` 에서 복사 |
    | `mae_pretrain_vit_base.pth` | `sam6d_realtime/checkpoints` 에서 복사 |

  - **`template/`(실제 영상 물체 템플릿)는 복구하지 못했습니다.** 이 노트북에는 사본이 없습니다. T7 디스크에서 다시 복사하거나 CAD 에서 다시 렌더해야 합니다. YCB-V 측정은 `ycbv_work/templates` 를 쓰므로 영향이 없습니다.
  - 재발 방지: 커밋 `cd017a7` 로 링크 3개를 추적에서 뺐습니다. 4090 서버에서 받을 때 같은 일이 생기지 않도록, 서버에서는 받기 전에 세 폴더를 다른 곳에 옮겨 두는 것이 안전합니다.

### 2. 원고 v0.3

빌드 스크립트는 `src/build_v03.py`(영문), `src/build_v03_ko.py`(국문)입니다. 영문 8쪽, 국문 10쪽으로 투고 한도 10쪽 안입니다. 키프레임 고정 관련 절(4.1·4.2·4.4·4.5, 5.2, 7.1·7.2)과 제목은 v0.2 와 같습니다.

| 절 | v0.2 | v0.3 |
|---|---|---|
| 초록 | 재연결 72/74 | 검증으로 장당 0.24초에 틀린 답 1/6 (재연결은 7.2절에 유지) |
| 기여 3 | 텍스트 프롬프트 + 관문 인식기, 약 2 Hz | 답하지 않을 수 있는 인식기(가설 집합 검증 + 군집 먼저) |
| 4.3 인식 | YOLO-World + MobileSAM + 관문 4개, 예시 프레임 | SAM-6D 앞단·PEM 그대로 + 윤곽 0.421·무늬 0.450·합의(o = 0.3 / 0.5) + 군집 먼저(68군집, 143 → 54 ms). 7.1·7.2 실행은 텍스트 프롬프트 변형으로 했다고 밝힘 |
| 5.1 객체 상태 지도 | 문장으로 설명 | **식 (5)~(12)로 전개 (3절)** |
| 6 실험 환경 | 한 장 비교 RTX 5090 [확인 필요] | 7.3·7.4 는 RTX 4090 재생. 데이터 표에 LM-O·TUD-L·IC-BIN 추가 |
| 7.3 인식 | 표 4: SAM-6D 대 텍스트·정답 상자 (YCB-V, YCB-M, 실내) | 표 4: 900장, 같은 GPU 의 GigaPose·FoundationPose 행 추가(16·17 문서). 표 5: BOP 데이터셋 3개(18 §C). 합의 문턱 곡선, 기권 사유 |
| 7.4 실시간 | 표 5: YCB-V 실시간 한 번 | 표 6: 3회 반복 — 260901 8/0 ×3 대 6.7/2.3, YCB-V 잘못 43.7 대 46.7 (15 §4-3, §4-4) |
| 7.5 한계 | 가림, 무작위 가설 | 무늬 없는 물체의 재현율 손실(LM-O·TUD-L −23~27%p), YCB-V 실시간 오답 |
| 그림 | 그림 4(인식기 구조), 그림 6(관문 절제) | 둘 다 뺌. **개선안 3 구조 그림을 새로 그려야 함** |

빌드 도구 수정: `src/build_docx.py` 의 `runs()` 가 한 겹 중첩 아래첨자(`T_{K_{i}←O_{i}}`)를 처리합니다. v0.2 까지는 식 (4) 등에서 `_{i}` 가 그대로 찍혔습니다.

### 3. 원고 5.1 에 넣은 상태 전이 수식

출처는 10-09 세션의 "상태 전이를 수식으로 전개" 답변입니다. 구현(`objectmemory_ws/.../core/existence.py`, `visibility.py`, `object_memory_runner.py`)과 허브 기본값(`--pd-base 0.08 --clutter-ratio 0.01 --pd-max-range 1.5 --dup-ratio 5`)을 그대로 따릅니다.

| 식 | 내용 |
|---|---|
| (5) | 검출 확률 P_D,k = P_D · 1[0 < z ≤ 1.5 m] · 1[(u, v) ∈ Ω], Ω 는 가장자리 5% 제외 |
| (6), (7) | 베이즈 갱신 (검출 / 미검출) |
| (8), (9) | 로그 오즈 덧셈: 검출 +ln(P_D/c) = +2.079, 보이는데 미검출 +ln(1 − P_D) = −0.083, 상한 ln 99 |
| (10) | 유지 조건 q* = −ln(1 − P_D) / ln[P_D / (c(1 − P_D))] = 3.86% |
| 본문 | 상태 T/A/L/M/D 와 전이 조건, 연속 미검출 46·46·66·26프레임, 재검출 3번에 상한, 두 검출 사이 30프레임 |
| (11) | 연결 기준 ‖t − t_z‖ ≤ 0.15 m · m(s), 후보는 3배 |
| (12) | 중복 억제 ρ · n_i ≥ max n_j, ρ = 5 |

**확인 필요(원고에 [verify] 로 표시):**
- `--pd-base 0.08 / --clutter-ratio 0.01` 은 2026-09-19 커밋(1127926, b85464c)부터의 기본값입니다. 그 전 값은 0.15 / 0.03 입니다. 7.2절 재연결 실험(밝은 홀 151초, 91초 주행)이 어느 값으로 돌았는지는 실행 폴더에 기록이 없어 확인하지 못했습니다.
- q* 3.86% 와 비교할 "측정 검출률 6~20%"의 출처를 다시 확인해야 합니다.
- `objpose/README.md` 의 "치운 물체가 lost 되기까지 약 14 → 27프레임"은 위 계산(46~66프레임)과 다르므로 인용하지 않았습니다.

### 4. RTX 3080 반복 측정 (중단)

> 07:22 중단. 4090 에서 이미 끝난 측정(15 §4-4)의 반복이라 새 정보가 없었습니다. "3080 으로 측정"은 10-09 에 시작한 공간 객체 상태 지도 비교 실험을 뜻했으며, 그 결과는 20 문서에 있습니다. 원본 0.5 1회차만 끝났습니다(인식 0.56 Hz, 처리 388 프레임, 등록 55/55, 잘못 올라간 항목 35, 지도 ADD-S 87.8).

- 스크립트는 `src/ycbv_live/run_repeat_3080.sh` 입니다. 15 §4-4 와 같은 규약이고, 3회 × 2구성 × 12영상입니다.
  - 원본 0.5: `orig_live_core.py`, 판정 0.5, ViT-L. 태그 `l3080_orig05_r<k>`.
  - 개선안 3 + 군집 먼저: `live_cores.py` hybrid, ViT-S, 0.5, `--verify-cluster-first`, `--cluster-occupancy 0.3`. 태그 `l3080_hyb3cf_r<k>`.
- 06:46 에 시작했고 장면당 약 3분 걸려 10:30 쯤 끝날 예정입니다. 로그는 `repeat_3080.log` 입니다.
- 시험 실행(장면 48, 개선안 3 + 군집 먼저)은 정상이었습니다. SLAM 이 같은 노트북에서 돌아 프레임 일부를 버립니다(1736 처리 / 498 버림). 10-06 노트북 실행과 같은 수준이고 두 구성이 같은 조건입니다.
- 결과가 나오면 15 §4-4(4090)와 나란히 이 문서 4절에 붙입니다.

### 5. 커밋 (원격에는 올리지 않음)

| 커밋 | 내용 |
|---|---|
| `cd017a7` | 자기 참조 링크 3개 추적 해제 |
| `4a8972c` | 원고 v0.3 (영문·국문), v0.1·v0.2 원고, 그림 원본, 결과 요약 JSON, 3080 반복 스크립트, `objpose/docs/rtab_backends_260924` |
| (이 문서와 함께) | 5.1 상태 전이 수식, `runs()` 중첩 아래첨자 수정 |

### 6. 남은 일

1. 공간 객체 상태 지도 비교의 남은 일(20 문서 5절).
2. `template/` 복구: T7 디스크(`sudo mount -t drvfs D: /mnt/d`)에서 복사한 뒤 260901 3080 반복 측정.
3. 원고 [verify] 항목: 녹화 실행의 GPU, 7.2절 존재확률 값, 측정 검출률.
4. 개선안 3 인식기 구조 그림 (v0.2 그림 4 자리).
5. 최종 수치: RTX 5090 또는 Blackwell 서버에서 3회 반복(서버 실험은 18 문서).

### 7. 오후 작업 (2026-10-10, 추가)

| 항목 | 상태 | 문서 |
|---|---|---|
| 공간 객체 상태 지도 비교 3단계 — ObChange-Kitchen (제안 / FM-Fusion / DualMap, 방문별·연속 지도) | 완료 | 21 문서 |
| 260901 실 데이터 반복 측정, 3080 (원본 0.5 / 개선안 3 + 군집 먼저, 각 3회) | 완료 | 22 문서 |
| 공간 객체 상태 지도 비교 4단계 — 실 데이터 녹화 5개 (세 방법, 같은 Mac ORB-SLAM3 궤적) | 완료 | 23 문서 |
| `sam6d_realtime/template/` 복구 | 완료 — T7 `D:\old\CLI_environment\sam6d_realtime\template` 에서 robocopy (2 GB) | 1절 |

- **ObChange:** 데이터는 Kitchen(6.5 GB)만 받았고 다른 TU Wien 데이터는 쓰지 않았습니다(사용자 결정). 정답 생성(`obchange_gt.py`, 25개, ICP 자세), rosbag 변환(`obchange_prep.py`, 8,974프레임, `/tf` 카메라 궤적), 세 방법 실행기의 `--dataset` 지원, 허브 `--memory-snapshots`(방문 끝 메모리 상태 저장), 채점기의 채점 제외 영역(프라이팬·접시, 사용자 결정), 재방문 채점(`obchange_score.py`)을 만들었습니다.
- **FM-Fusion 수정:** 지도 생성 프로그램의 "4,000프레임 초과 시 종료" 상한을 20,000 으로 올려 다시 빌드(`IntegrateInstanceMap.cpp`, 원본 `.orig_4000cap`).
- **실수 기록:** 대기 조건의 문자열 오류로 기존 방법 실행이 42분 늦게 시작했고, `pgrep -f` 패턴이 제 셸 명령과 겹쳐 셸이 두 번 종료됐습니다(다시 실행해 확인).
- **260901:** 개선안 3 의 2회차는 ORB-SLAM3 의 3.10 m 잘못된 루프 보정으로 버리고 다시 돌렸습니다.
- **실 데이터 비교(저녁):** 녹화 2~5 는 Mac 과 함께 제안 방법을 새로 실시간 실행해 궤적을 얻었고(4회, 각 8/0·7/1), 녹화 1 은 22 문서의 2b회차 궤적을 썼습니다. 처음 만든 궤적은 지워진 키프레임을 기준으로 한 자세에 보정을 안 줬는데(녹화 1 의 26%, 근처 보정량 최대 11 cm), 기준 방법 실행 전에 가장 가까운 보정을 쓰도록 고쳐 다시 만들었습니다. 결과(40개 합계): 제안 39/1, FM-Fusion 33/174, DualMap 14/12 · 19/18.

---

<a id="26_다른장비_재현_안내_4090_5090_261011"></a>

## 26. 다른 장비에서 다시 돌리기 — 주말(10-10 · 10-11) 노트북 실험을 RTX 4090 · RTX 5090(Blackwell)으로 (2026-10-11)

> 이번 주말 노트북(RTX 3080 Ti Laptop)에서 한 실험을 4090 과 5090 장비에서 그대로 다시 돌리기 위한 안내입니다. 결과와 해석은 19~25 문서에 있고, 이 문서는 **무엇을 옮기고, 어디를 고치고, 어떤 순서로 돌리는지**만 다룹니다. 논문 최종 수치는 5090 으로 냅니다.

### 0. 요약

- 주말 실험은 다섯 가지입니다. YCB-V 실시간 3회 반복(19), 실데이터 260901 3회 반복(22), 객체 지도 비교 YCB-V(20) · ObChange-Kitchen(21) · 실데이터 5개(23·24).
- 코드는 저장소 브랜치 `objpose/memory-dup-suppression`(0f32112 이후)에 모두 있습니다. 다만 다음은 저장소에 없어 따로 옮겨야 합니다.
  - 기준 방법 코드: FM-Fusion, DualMap(로컬 수정 포함)
  - 데이터: 녹화, YCB-V, ObChange
  - 채점 기준 실행의 `summary.json`
- 장비마다 고칠 곳은 세 군데입니다. `objpose/pc/catalog.py` 의 `DATASET_ROOT`, 스크립트 안의 `l3080` 태그, `realdata/logs_live_261011/*.sh` 의 `REPO=` 절대 경로입니다.
- 실데이터 실시간 실행에는 Mac(ORB-SLAM3)이 필요합니다. YCB-V 실시간은 그 장비의 로컬 ORB-SLAM3 로 돕니다(15 문서와 같은 방식).
- 권하는 순서는 4절과 같습니다. 짧은 것(실데이터 5개 실시간, 약 30분)부터 돌려 환경을 확인하고, 긴 것(YCB-V 반복, 기준 방법)은 뒤에 둡니다.

### 1. 주말 실험과 문서

| 실험 | 문서 | 실행 스크립트 (`paper_etrij_261003/src/` 기준) | 결과 파일 | 노트북 소요 |
|---|---|---|---|---|
| YCB-V 12영상 실시간, 원본 0.5 대 개선안 3 + 군집 먼저, 3회 | 19 §4 · §7 | `ycbv_live/run_repeat_3080.sh` | `ycbv_live/results_3080/` | 회당 두 구성 × 12영상(각 약 11분 분량 재생) |
| 260901 실데이터, 같은 두 구성, 3회 | 22 | `realdata/run_real_repeat_3080.sh` (+ `_redo.sh`) | `realdata/results_real5_repeats_3080.json` | 실행당 약 6분 × 6 |
| 객체 지도 비교 ① YCB-V | 20 | `objmap/ycbv_gt.py`, `run_ours.py`, `fmfusion_*.py`, `dualmap_run.sh`, `evaluate.py` | `objmap/out/` | — |
| 객체 지도 비교 ② ObChange-Kitchen | 21 | `objmap/obchange_gt.py`, `obchange_prep.py`, `seq_bags.py`, `run_obchange.sh`, `run_obchange_baselines.sh` | `objmap/out/obchange_scores.json` | — |
| 객체 지도 비교 ③ 실데이터 5개 | 23 | `objmap/run_real_compare.sh` | `objmap/out/real_scores.json` | — |
| ③ 재실행: 제안 실시간 5개 + 기준 방법 | 24 | `realdata/logs_live_261011/run_live5_261011.sh`, `objmap/run_real_compare_261011.sh` | `realdata/results_real5_live_261011.json`, `objmap/out_261011/real_scores.json` | 실시간 약 27분(녹화 1 재실행 포함), 기준 방법 약 40분 |
| 요약 | 25 | — | — | — |

- 소요 시간은 24 문서 실행 기록(`progress.log`, `out_261011/real_logs/`)에서 가져왔습니다. 24 의 기준 방법 40분은 23 의 FM-Fusion 검출 결과를 재사용한 시간입니다. 새 장비에서는 검출을 처음부터 하므로 더 걸립니다(3080 기준 검출한 프레임당 약 1.2초, 20 문서).
- 4090 에서 전에 돌린 기록은 09(한 장 평가, 경로) · 15(실시간 여섯 구성 · 반복, 체인 스크립트 `~/Dataset/bop/ycbv_work/run_combo_4090.sh`)에 있습니다.

### 2. 준비

#### 2.1 코드

```bash
cd <저장소>              # 4090 은 전에 CLI_env_paper(15 문서), old/CLI_environment(09 문서)를 썼음
git fetch origin
git checkout objpose/memory-dup-suppression
git pull                 # 0f32112 이후
```

- **받기 전에 확인:** 10-10 에 4090 사본(CLI_env_paper)에서 pull 하다가, 커밋 0c539f3 의 자기 참조 링크 3개(`sam6d_realtime/template`, SAM-6D 두 `checkpoints`)가 git 에서 무시하던 로컬 폴더를 지운 일이 있습니다. 지금 브랜치 끝에서는 이 링크가 추적되지 않지만, 받기 전에 `git diff --stat HEAD..origin/objpose/memory-dup-suppression` 으로 링크·폴더 경로를 보고 세 폴더는 먼저 다른 곳에 복사해 둡니다.
- `sam6d_realtime/`(검증 · 배정 대안 · 크기 관문 수정 포함)은 저장소에 들어 있습니다. 09 문서 4절에서 "노트북에만 있다"고 했던 `sam6d_core.py`, `yolo_ism_object_n.py`, `verify_config.py`, `Pose_Estimation_Model/utils/model_utils.py` 도 지금은 커밋돼 있어 따로 복사하지 않아도 됩니다.

#### 2.2 장비마다 고칠 곳

| 파일 | 지금 값 | 바꿀 것 |
|---|---|---|
| `objpose/pc/catalog.py` 28행 | `DATASET_ROOT = Path("/home/jucpark/DeepLearning/Dataset")` | 그 장비의 녹화 폴더(4090 은 `~/Dataset` 였음, 09 문서) |
| `realdata/logs_live_261011/run_live5_261011.sh`, `run_rerun1.sh` | `REPO=/home/jucpark/DeepLearning/CLI_environment` | 그 장비의 저장소 경로 |
| 위 두 스크립트, `ycbv_live/run_repeat_3080.sh`, `realdata/run_real_repeat_3080*.sh` | 실행 이름의 `l3080` | `l4090` / `l5090`. 파일을 `_4090.sh` · `_5090.sh` 로 복사해 고치고, 결과 폴더(`results_3080`)와 결과 JSON 이름도 같이 바꿉니다. 노트북 결과를 덮어쓰지 않기 위해서입니다 |
| `realdata/run_real_repeat_3080.sh` 11행 | ObChange 기준 방법의 로그에 `BASELINES_DONE` 이 나올 때까지 기다림 | 혼자 돌릴 때는 그 `until` 줄을 지웁니다 |

나머지 스크립트는 `$HOME/anaconda3/envs/<이름>/bin/python`, `$HOME/DeepLearning/Dataset/bop/ycbv_work`, `$HOME/DeepLearning/baselines` 를 씁니다. 장비의 폴더가 다르면 같은 이름으로 심볼릭 링크를 만들어 두는 편이 스크립트를 고치는 것보다 간단합니다. 예: `ln -s ~/Dataset ~/DeepLearning/Dataset`.

#### 2.3 실행 환경 (conda)

| 환경 | 쓰는 곳 | 비고 |
|---|---|---|
| `sam6d` | 허브, SAM-6D, 채점, 변환기 전부 | 노트북과 같은 패키지. 4090 에는 이미 있음(15 문서) |
| `fmfusion` | `fmfusion_detect.py`(GroundingDINO + SAM) | 노트북 설치 기록 `~/DeepLearning/baselines/fmfusion_env.log` |
| `dualmap` | `dualmap_run.sh` | DualMap 저장소 안내대로 설치 |

**5090(Blackwell) 주의:**
- PyTorch 는 CUDA 12.8 이상 빌드가 필요합니다. 그보다 낮은 빌드에는 Blackwell 용 커널이 없습니다.
- 직접 빌드하는 CUDA 확장은 그 장비에서 다시 빌드해야 합니다.
  - GroundingDINO 연산(20 문서 §4에 적어 둠)
  - SAM-6D 자세 추정의 pointnet2 연산
- 세 환경 모두 `python -c "import torch; print(torch.cuda.get_device_capability())"` 가 (12, 0)을 내는지 먼저 봅니다.
- 5090 에서는 아직 한 번도 돌려 보지 않았습니다. 위 목록은 예상입니다.

#### 2.4 기준 방법 코드 (저장소 밖)

`~/DeepLearning/baselines/` 아래에 둡니다. 노트북에서 폴더째 복사해도 되고, 새로 받아 아래 수정만 넣어도 됩니다.

| 폴더 | 원 저장소 · 커밋 | 로컬 수정 |
|---|---|---|
| `FM-Fusion` | HKUST-Aerial-Robotics/FM-Fusion, b82d64c | `src/IntegrateInstanceMap.cpp` 71행 프레임 상한 4000 → 20000(ObChange 연속 입력 8,974프레임 때문). 수정 후 다시 빌드. Open3D 0.18 devel 패키지(`open3d-devel-linux-x86_64-cxx11-abi-0.18.0`)를 씀 |
| `gsam_fmfusion` | glennliu/Grounded-Segment-Anything, 9cab32b | 없음 |
| `DualMap` | Eku127/DualMap, 157235e | `utils/object_detector.py`: ① `DUALMAP_ULTRALYTICS_BGR=1` 일 때 ultralytics 에 BGR 사본을 넘김(BGR 수정 판), ② YOLO 검출이 없는 프레임을 건너뜀(없으면 멈춤). 노트북에서 `git -C ~/DeepLearning/baselines/DualMap diff > dualmap.patch` 로 떠서 `git apply` |
| `weights` | — | `groundingdino_swint_ogc.pth`, `sam_vit_h_4b8939.pth` (3.1 GB) |

#### 2.5 데이터

| 데이터 | 노트북 위치 | 크기 | 쓰는 실험 |
|---|---|---|---|
| 실데이터 녹화 5개 | `~/DeepLearning/Dataset/<녹화>` (260826_etri_eightcircle_dark, 260901_cbnu_bigeightcircle, 260901_cbnu_eightcircle, 260910_object, 260915_eightcircle) | 24 + 25 + 12 + 15 + 15 GB | 22, 23, 24 |
| 실데이터 인식 카메라 프레임 | `~/DeepLearning/Dataset/real_work` | 15 GB | 23, 24 (`real_prep.py` 로 다시 만들 수 있음) |
| YCB-V 재생용 녹화 | `~/DeepLearning/Dataset/bop/ycbv_bags` | 30 GB | 19 |
| YCB-V 원본 · 자산 | `~/DeepLearning/Dataset/bop/ycbv`, `ycbv_work/{templates, features, core_repo, ycbv_objects.yaml, predefined_poses, FastSAM-x.pt}` | 자산 약 2 GB | 19, 20, 전부(인식기) |
| ObChange-Kitchen | `~/DeepLearning/Dataset/obchange` (Kitchen, bags, work) | 49 GB (zip 포함) | 21 |
| 기준 방법 작업 폴더 | `ycbv_work/fmfusion`, `ycbv_work/dualmap` | 1.2 GB, 7.6 GB | 준비 스크립트(`*_prep.py`)가 다시 만듦. 옮기지 않아도 됨 |

- **채점 기준 위치:** 실데이터 채점(`eval_real.py`, `real_score.py`)은 물체 8개의 기준 위치를 이전 실행들의 `summary.json` 중앙값으로 잡습니다. 그 실행 폴더는 git 에 없으므로 `summary.json` 만 옮깁니다(합계 0.5 MB 이하).

  ```bash
  cd ~/DeepLearning/CLI_environment
  tar czf ref_summaries.tgz $(for r in live_260901_cbnu_bigeightcircle_orbslam3 live_260901_cbnu_bigeightcircle_orbslam3_f2000_dev01 \
      live_260901_cbnu_bigeightcircle_orbslam3_f2000_dup5 live_260915_eightcircle_orbslam3 live_260826_etri_dark_orbslam3_f2000 \
      live_260826_etri_dark_orbslam3_f2000_dup live_260826_etri_dark_orbslam3_f2000_dup5 live_260901_cbnu_eightcircle_orbslam3 \
      ref_260910_object_ours; do echo objpose/output/$r/summary.json; done)
  # 새 장비의 저장소 최상위에서: tar xzf ref_summaries.tgz
  ```

  - 260901_cbnu_bigeightcircle 은 기준 실행이 없어도 `results_real_laptop.json`(커밋됨)의 기준 위치를 씁니다.
  - 나머지 4개 녹화는 이 파일이 없으면 채점이 멈춥니다.
- **노트북 결과와 같은 자리에서 비교하려면** 23 · 24 의 제안 방법 실행 폴더도 옮깁니다. 대상은 `objpose/output/real_*_l3080_hyb3cf`, `real5_260901_l3080_hyb3cf_r2b`, `live_261011_*`, 합계 210 MB 입니다. 수치만 비교한다면 커밋된 결과 JSON 으로 충분합니다.

#### 2.6 Mac 연결 (실데이터 실시간)

- 새 장비에서 `ssh mac` 이 비밀번호 없이 돼야 합니다. 스크립트는 `BatchMode=yes` 로 붙고 Mac 에 `caffeinate` 를 겁니다.
  - 그 장비의 공개키를 Mac `~/.ssh/authorized_keys` 에 넣습니다.
  - `~/.ssh/config` 에 `Host mac` 을 적습니다. Mac 주소는 10-11 기준 192.168.219.108 이고 자주 바뀝니다.
- Mac 은 `~/Documents/DefenseMeta/Dataset` 의 같은 녹화를 재생합니다(`catalog.MAC_ROOT`). Mac 쪽은 바꿀 것이 없습니다.
- 허브가 Mac 에 SSH 역방향 터널(`-R 17001:127.0.0.1:17001`)을 열고 SLAM 결과를 받습니다(실행 폴더의 `mac_command.txt`). SSH 만 되면 따로 열 포트는 없습니다.
- 한 번에 한 장비만 Mac 을 씁니다. 4090 과 5090 이 같은 Mac 에 동시에 붙으면 SLAM 재생이 섞입니다.

### 3. 실행 전 확인 (10분)

```bash
cd <저장소>/paper_etrij_261003/src
PY=$HOME/anaconda3/envs/sam6d/bin/python
$PY -c "import torch;print(torch.__version__, torch.cuda.get_device_name(0), torch.cuda.get_device_capability())"
(cd ../../objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('260915_eightcircle','orbslam3',2000)))")   # 경로가 그 장비 것인지
timeout 8 ssh -o BatchMode=yes mac true && echo MAC_OK
```

가장 짧은 녹화(260915_eightcircle, 128초)를 하나 돌려 보고 `summary written` 이 나오는지 확인한 뒤 4절로 넘어갑니다.

### 4. 실행 순서

| 순서 | 실험 | 명령 (고친 사본 기준) | 결과를 비교할 노트북 값 |
|---|---|---|---|
| 1 | 실데이터 5개 실시간 (24 와 같은 설정) | `bash realdata/logs_live_261011/run_live5_<장비>.sh` | 24 §2: 40 / 0 (23 은 39 / 1) |
| 2 | 위 궤적으로 기준 방법 재실행 | `run_real_compare_261011.sh` 사본(태그·`OBJMAP_REAL_RUNS` 를 새 실행 이름으로) | 24 §4: FMF 33 / 176, DM 13 / 15, DM BGR 20 / 21 |
| 3 | 260901 3회 반복 | `bash realdata/run_real_repeat_<장비>.sh` | 22, 4090 값은 15 §4-3 |
| 4 | YCB-V 12영상 3회 반복 | `bash ycbv_live/run_repeat_<장비>.sh 1 2 3` | 19 §4 · §7, 4090 값은 15 §4-4 |
| 5 | 객체 지도 비교 YCB-V | 20 문서 1절 순서(`ycbv_gt.py` → `run_ours.py` → FM-Fusion → DualMap → `evaluate.py`) | 20 §2 |
| 6 | 객체 지도 비교 ObChange-Kitchen | `obchange_gt.py` → `obchange_prep.py` → `seq_bags.py` → `run_obchange.sh` → `run_obchange_baselines.sh` | 21 §3 |

- **순서 1 과 2:** 1 이 끝나야 2 를 돌릴 수 있습니다(2 가 1 의 궤적을 씀).
  - 2 의 스크립트는 23 의 FM-Fusion 검출 결과를 하드 링크해 재사용합니다. 새 장비에는 그 폴더가 없으므로 링크 없이 전부 다시 검출합니다(스크립트는 그대로 돎).
  - 앞부분의 `cp -n $W/dualmap/real_prompts*_colors.json` 은 새 장비에서 "파일 없음"으로 실패하지만 무시해도 됩니다. DualMap 이 첫 실행에서 클래스별 표시 색 파일을 새로 만들고, 결과에는 영향이 없습니다.
- **GPU 를 같이 쓰지 않기:**
  - 제안 방법은 실시간 재생이라 GPU 를 같이 쓰면 인식 빈도가 떨어지고 결과가 바뀝니다.
  - 실시간 실험(1, 3, 4)은 혼자 돌립니다.
  - FM-Fusion 과 DualMap 은 오프라인이라 서로 겹쳐도 됩니다(스크립트가 둘을 동시에 돌림).
- **루프 오결합:**
  - 실데이터에서 ORB-SLAM3 가 수 m 짜리 잘못된 루프 보정을 낼 때가 있습니다. 노트북에서는 22 의 3.10 m, 24 녹화 1 의 296.7 cm 가 그랬습니다.
  - 각 실행 끝 줄의 `max correction so far` 를 봅니다. 지금까지 3 m 안팎의 보정이 나온 실행은 버리고 다시 돌렸습니다(15 §2 의 3.6 m, 22 의 3.10 m, 24 의 296.7 cm). 다른 실행은 6~53 cm 였습니다.
  - 녹화 4(260910_object)는 2 m 넘는 보정이 두 번 모두 나왔고(23 문서 235.5 cm, 24 문서 216.0 cm), 버리지 않고 그대로 썼습니다. 같은 기준을 따릅니다.

### 5. 결과 정리

- 실행 이름에 장비 태그(`l4090`, `l5090`)를 넣었으면 `eval_real.py --runs ...`, `real_score.py`, `eval_live.py`, `eval_penalized.py` 는 그대로 씁니다.
- 노트북 결과와 나란히 놓을 때는 24 의 `realdata/compare_live_261011.py` 를 복사해 실행 목록만 바꾸면 됩니다.
- 문서는 장비별로 새 번호(27~)를 붙여 `개별문서/` 에 쓰고, `build_bundles.py` 의 해당 묶음에 넣은 뒤 `python3 build_bundles.py` 로 묶음을 다시 만듭니다.
  - 실시간 → C
  - 객체 지도 비교 → F
  - 장비 확인 실행 → B 또는 C

### 6. 장비 사이 차이로 결과가 달라질 수 있는 곳

| 항목 | 영향 |
|---|---|
| 인식 빈도 | 실시간 재생이라 GPU 가 빠르면 처리하는 프레임이 늘어납니다. 22 §3 에서 개선안 3 + 군집 먼저는 3080(3.48 Hz)과 4090(7.54 Hz) 모두 6번 다 8 / 0 이었고, 원본 0.5 는 0.87 대 3.63 Hz 로 차이가 커서 결과도 흔들렸습니다. 5090 에서는 원본 쪽 수치가 가장 많이 바뀔 것으로 봅니다 |
| ORB-SLAM3 | Mac 쪽은 같습니다. YCB-V 실시간의 로컬 ORB-SLAM3(`~/objpose_pc/run_slam.sh`)는 장비마다 빌드가 따로라 궤적이 조금 다를 수 있습니다 |
| 기준 방법 | 오프라인이라 장비와 무관해야 합니다. 다만 GroundingDINO · SAM 의 반정밀도 연산 차이로 상자가 약간 달라질 수 있습니다 |

### 7. 4090 에서 실제로 걸린 것 (2026-10-12 추가)

이 PC(RTX 4090 ×4, 드라이버 575 → CUDA 12.9 까지)에서 2절 안내대로 옮겨 돌리다가 걸린 것들입니다. 5090/.41 에서도 같은 일이 생길 수 있어 적어 둡니다.

| 항목 | 증상 | 원인과 조치 |
|---|---|---|
| FM-Fusion 빌드 | 링크 오류 `undefined reference to std::__1::basic_iostream` | Open3D 0.18 devel 패키지가 libc++ 로 만들어져 있음. `-DCMAKE_EXE_LINKER_FLAGS="-lc++ -lc++abi"` 로 링크하고 libc++.so.1 · libc++abi.so.1 (conda `libcxx`, `libcxxabi` 패키지)을 Open3D `lib/` 옆에 두면 됨 (`patchelf --set-rpath '$ORIGIN' libOpen3D.so`) |
| FM-Fusion 실행 | `Read 0 frames`, 지도 없음 | 노트북 `ycbv_work/fmfusion/scans/<장면>/{color,depth}` 가 `Dataset/bop/ycbv/full/test/<장면>/{rgb,depth}` 를 가리키는 링크인데 T7 사본에 rgb 가 없었음(depth 만). `ycbv_test_all.zip` 에서 48~59 장면의 rgb·depth 를 풀면 됨. Open3D 는 대상이 없는 링크를 조용히 건너뛰어 프레임이 0 이 됨 |
| DualMap 환경 | `No module named mobileclip`, `Detector has no attribute yolo` | ① `3rdparty/mobileclip` 서브모듈이 사본에 비어 있음 → `pip install git+https://github.com/apple/ml-mobileclip` ② `model/` 가중치가 사본에 없음 → `mobile_sam.pt`(sam6d_realtime 것), `FastSAM-s.pt`(SAM-6D ISM checkpoints 것) 복사, `yolov8l-world.pt` 는 ultralytics 가 내려받음(96 MB) ③ `scikit-learn` 필요 |
| conda 환경 복사 | .41 의 `dualmap` 은 torch cu130 | 드라이버 580 미만에서는 못 씀. 이 PC 는 `sam6d` 환경을 복제(`conda create --clone sam6d --offline`)한 뒤 DualMap `environment.yml` 의 pip 목록을 얹음. `fmfusion` 도 같은 방식 + `pip install -e gsam_fmfusion/GroundingDINO`(CUDA 연산 빌드: `CUDA_HOME=/usr/local/cuda-12.9`, `TORCH_CUDA_ARCH_LIST=8.9`, `CPATH=<env>/targets/x86_64-linux/include` — cusparse.h 가 거기에만 있음) |
| 포트 17001 | 실시간 체인이 서로 SLAM 을 죽임 | .41 의 실시간 08 체인이 이 PC 의 ORB-SLAM3 를 ssh 역터널(17001)로 씀. 같은 PC 의 로컬 체인은 `SLAM_PORT=17002 HTTP_PORT=8766`(`ycbv_live/run_live.sh`, `run_4090/*.sh`)으로 돌리고 `pkill slam_stream` 은 자기 포트 것만 |
| GPU 겹침 | 10-07, 10-08 드라이버 멈춤 | 둘 다 GPU0 실시간 + 다른 GPU 배치가 겹쳤을 때. 이 PC 에서는 GPU 작업을 하나씩만 순서대로 |
| DualMap 멈춤 | YCB-V 장면 49 에서 25분 제한까지 진행 없음 | YOLO-World 가 상자를 하나도 내지 않은 프레임에서 `curr_detections` 가 이전 값으로 남아 빈 마스크를 필터링하다 멈춤. `utils/object_detector.py`: 빈 결과면 `curr_detections=None`, `process_detections` 는 그때 `curr_results={}` 로 반환. 장면 49 는 `dualmap_fixup_4090.sh` 로 재실행 |
| 실제 영상 채점 기준 행 | `eval_real.py` 가 `live_<녹화>_orbslam3/slam_poses.jsonl` 없음으로 실패 | 노트북 배포 실행 폴더는 요약(`summary.json`)만 복사돼 있고 궤적이 없음. 기준 행을 이 PC 에서 만든 `ref_<녹화>_ours_r1`(기준 위치는 r1 · r2 중앙값, `EVAL_REAL_REFS`)로 바꿔 채점 |
| SLAM 스트림 멈춤 | 260826 기준 r2 가 27초 뒤 자세 없음(오류 없음) | 1회성. 폴더를 `_slamstall_261012` 로 두고 재실행(27 문서 4절 장면 57 과 같은 증상) |
| 실데이터 기준 방법 준비 | `real_prep.py` 가 `StopIteration`(bag 없음) / 장면 1 실행 이름 없음 | ① `objpose/pc/catalog.py` 의 `DATASET_ROOT`(`~/DeepLearning/Dataset/<녹화>`)에 이 PC 는 260826 만 있음 → 나머지 4 녹화는 NAS `data_slam_converted/<녹화>` 로 심볼릭 링크 ② `fmfusion_4090/scans` 가 노트북 `fmfusion/scans` 전체를 가리키는 링크라 실데이터 준비가 노트북 입력을 덮어쓸 뻔함 → 장면별 링크 폴더로 바꿈 ③ 프레임 번호가 궤적에 따라 달라지므로 `real_prep.py` 출력 폴더를 `OBJMAP_REAL_WORK`(태그별 `real_work_<TAG>`)로 분리 ④ 260901 큰 8자의 4090 실행 이름은 `real5_260901_hybS05`(군집 먼저 없음) |
| GroundingDINO 적재 | `'BertModel' object has no attribute 'get_extended_attention_mask'` | `sam6d` 를 복제한 `fmfusion` 환경의 transformers 가 5.19. GroundingDINO 의 `bertwarper.py` 는 4.x API 를 씀 → `pip install transformers==4.44.2` (fmfusion 환경에만). YCB-V · ObChange 는 노트북 검출을 재사용해서 이 오류를 만나지 않았고, 실데이터는 궤적이 달라 검출부터 다시 돌려야 해서 드러남 |

같은 날 폴더도 정리했습니다. 코드는 `~/DeepLearning/server/CLI_environment` 하나(옛 `CLI_env_paper` 는 삭제, 이 문서의 그 경로는 지금의 server/CLI_environment 를 뜻함), 4090 체인 스크립트는 `src/run_4090/`, 기준 방법 코드는 `~/DeepLearning/baselines`, 데이터는 NAS `/mnt/synology/Dataset/DeepLearning_dataset/`(옛 `~/DeepLearning/Dataset` 경로는 링크로 유지)입니다. 환경 설치 기록은 `src/objmap/setup_baselines_4090.sh` 입니다.

### 8. 4090 재현 결과 — 08 문서 세트 (2026-10-12)

`src/run_4090/run_08_4090.sh`(16:06~19:29): 260901 큰 8자 6구성(R1), 실제 영상 4개 × 4구성(R2), YCB-V 12영상 × 6구성. SLAM 은 이 PC 의 ORB-SLAM3(f2000). 결과 `src/realdata/results_real5_4090.json`, `results_real_<녹화>_4090.json`, `src/ycbv_live/results_penalized_4090.json`. 노트북 값은 08 문서(1회 측정, Mac SLAM).

**R1. 260901 큰 8자 (평균 점수 % / 인식 빈도)**

| 구성 | 노트북 3080 | 이 PC 4090 |
|---|---|---|
| 원본 0.5 | 53.6 / 0.89 Hz | 57.9 / 3.62 Hz |
| 개선안 (g13) | 79.2 / 3.68 Hz | 77.0 / 4.77 Hz |
| 개선안 2 (원본 S 0.5) | – | 28.7 / 6.89 Hz |
| 개선안 3 | 77.3 / 2.77 Hz | 78.6 / 6.17 Hz |
| 개선안 3 검증 없음 | – | 29.3 / 7.80 Hz |
| A안 | – | 79.6 / 5.16 Hz |

**R2. 실제 영상 4개 (평균 점수 %, 노트북 → 4090; 4090 의 기준 행은 체인 A 의 배포 실행 r1)**

| 녹화 | 기준(배포, 4090) | 원본 0.5 | 개선안 | 개선안 3 | A안 |
|---|---|---|---|---|---|
| 260915 8자 | 72.2 | 59.3 → 58.1 | 60.7 → 65.9 | 67.8 → 70.7 | 66.9 → 70.2 |
| 260901 cbnu 8자 | 83.9 | 62.3 → 65.1 | 80.1 → 81.3 | 69.6 → 76.8 | 77.7 → 81.7 |
| 260910 물체 | 59.1 | 61.1 → 45.9 | 72.9 → 57.5 | 71.0 → 59.7 | 56.0 → 68.5 |
| 260826 어두운 8자 | 94.4 | 63.5 → 68.8 | 88.0 → 91.2 | 88.2 → 90.8 | 86.6 → 91.5 |

- 인식 빈도는 원본 0.6~0.9 → 2.1~3.3 Hz, 개선안 3 0.9~2.5 → 4.2~6.0 Hz. 순위(검증 있는 세 구성 > 원본)는 네 녹화 모두 유지됩니다.
- 260910 은 원본 0.5 와 개선안 3 이 두 번 모두 루프 보정 1 m 를 넘겨 마지막 시도를 채점했습니다(18 문서 A 절과 같은 이유). 4090 값이 노트북보다 낮은 것은 그 보정 때문입니다.
- 기준 행이 노트북(Mac SLAM 배포 실행)과 다르므로 절대값은 참고만 하고, 구성 간 차이로 읽습니다.

**YCB-V 12영상 (프레임 평균 점수 %, 벌점 포함)**

| 구성 | 노트북 3080 | 이 PC 4090 |
|---|---|---|
| 원본 0.5 | 64.6 | 57.5 |
| 개선안 (g13p) | 60.4 | 62.4 |
| 개선안 2 (원본 S 0.5) | 37.0 | 31.6 |
| 개선안 3 | 70.1 | 63.3 |
| 개선안 3 검증 없음 | – | 27.9 |
| A안 | – | 50.9 |

- 4090 에서는 원본 0.5 와 개선안 3 이 노트북보다 6~7 점 낮고 개선안은 2 점 높습니다. 27 문서와 같은 원인(인식 프레임이 2~3 배라 없는 물체의 답이 더 많이 통과)으로 보이며, 검증 없음(27.9)과의 차이는 그대로입니다.
- 실행 시간: 실제 영상 1회 5~12분(NAS 에서 녹화 읽기 2분 포함), YCB-V 1장면 1~2분, 세트 전체 3시간 23분.

### 9. sudo 없는 호스트(.41)에서 ORB-SLAM3 빌드 (2026-10-14 추가)

`objpose/linux_slam/build_host.sh` 는 부족한 apt 패키지를 sudo 로 설치하는데, .41 처럼 sudo 가 없는 계정이면 `libsqlite3-dev`·`nlohmann-json3-dev` 두 개에서 멈춥니다. 다른 머신의 `/usr/include/sqlite3.h`·`sqlite3ext.h`·`nlohmann/` 을 `~/objpose/deps/install/include/` 에 복사하고, `deps/install/lib/libsqlite3.so` 를 시스템 `libsqlite3.so.0` 으로 링크하고, `deps/install/lib/cmake/nlohmann_json/nlohmann_jsonConfig.cmake` 에 헤더 전용 INTERFACE 타깃을 적어 두면 스크립트가 apt 를 건너뛰고 빌드합니다(커밋 32dbc801). Pangolin 소스와 `Vocabulary/ORBvoc.txt` 는 이미 빌드된 머신의 `~/objpose` 에서 복사하면 됩니다. hub 와 SLAM 을 한 머신에서 돌리는 방법은 27 문서(서버·클라이언트) 4절에 있습니다.

---

<a id="28_작업기록_41단독_FMFusion재검출_192복구_261014"></a>

## 28. 작업 기록 — .41 한 대 구성, FM-Fusion 검출 재실행(.41 · .192), .192 드라이버 복구 (2026-10-14)

> .41(RTX PRO 6000 Blackwell ×4, hub·SAM-6D 서버)에서 한 하루 작업 기록입니다. 아침까지는 .192(RTX 4090 ×4, ORB-SLAM3 클라이언트)가 GroundingDINO 검출만 띄우면 드라이버가 멈추는 상태였고(10-13·10-14 세 번), 낮에 사용자가 .192 드라이버를 올려 복구했습니다. 세부 표는 각 실험을 적은 문서 절에 있고, 이 문서는 하루의 흐름·판단·사고를 한 곳에 모은 것입니다. 커밋은 모두 `paper/etrij-blackwell-261008` 브랜치입니다.

### 0. 요약

| 시각 | 한 일 | 결과 | 자세한 곳 | 커밋 |
|---|---|---|---|---|
| ~09:00 | 실데이터 5녹화 객체 지도 비교를 .41 궤적으로 세 번째 실행(FM-Fusion·DualMap 을 .41 에서) | 제안 90.5% 대 DualMap-BGR 31.2% 대 DualMap 24.1% 대 FM-Fusion 15.0%, 순위 재현 | 23 §7, 24 §7 | 09db2769 |
| 09:46~09:53 | .192 브랜치 병합, **ORB-SLAM3 를 .41 에 빌드(sudo 없이)**, hub + SLAM 한 머신 점검(260901 eightcircle) | 8/1 · 6.79 Hz — 서버·클라이언트 구성과 같음 | 27(서버·클라이언트) §4, 26 §9 | d9c0528e, 32dbc801, b8da1bd7 |
| 09:47~10:30 | **FM-Fusion 검출(GroundingDINO + SAM)부터 .41 에서 재실행** (사고 1건, 3절) | 노트북 검출과 같음 — YCB-V F1 0.296, ObChange 방문별 0.248 | 27(객체지도 4090) §9 | c8e8e4e5 |
| 09:54 | 사용자: .192 드라이버 575.57.08 → **580.178.04**, 재부팅 | | | |
| 11:01~11:45 | **FM-Fusion 검출을 드라이버 올린 .192 에서 원래 구성으로 재실행** | 44분 멈춤 없음, 결과 노트북·.41 과 같음 | 27(객체지도 4090) §9.1 | a5c0a8ae |

**하루의 결론**

1. FM-Fusion 의 기준 값(YCB-V F1 0.296, ObChange 방문별 F1 0.248)은 검출·융합 장비를 노트북(3080 Ti Laptop) · .41(RTX PRO 6000) · .192(RTX 4090) 어느 것으로 바꿔도 TP · FP · FN 까지 같습니다. 20 · 21 문서의 FM-Fusion 행은 그대로 씁니다. 장비에 따라 바뀌는 기준 방법은 DualMap BGR 수정뿐입니다(27 §7).
2. 제안 방법의 실시간 구성은 "hub + ORB-SLAM3 한 머신(.41)" 과 "hub(.41) + ORB-SLAM3(.192)" 가 같은 결과를 냅니다. ORB-SLAM3 는 CPU 작업이라 GPU 서버와 한 머신에 두어도 손해가 없습니다.
3. .192 의 멈춤은 580 드라이버로 사라졌습니다. 10-14 아침에 넣어 둔 회피책(deformable attention 을 PyTorch 경로로, 실험을 .41 로 몰기)은 더 필요 없고, 실시간 서버·클라이언트 구성으로 돌아갈 수 있습니다.

### 1. 아침 — 실데이터 비교 세 번째 실행 (23 · 24 §7)

- 10-13 저녁부터 돌린 체인(`src/objmap/run_objmap_real_41.sh`, `run_objmap_real_41_dm.sh`): .41 Blackwell + .192 ORB-SLAM3 로 돌린 제안 방법 실행(15 문서 §7, `real_<녹화>_hybS05`)의 궤적 위에서 FM-Fusion·DualMap 을 .41 에서 다시 돌렸습니다.
- DualMap 이 .41 에서 10회 전부 실패했던 원인 두 가지를 10-14 에 고쳤습니다. dualmap conda 환경에 `mobileclip` 이 없어 검출기 초기화가 조용히 빠져나간 것(.192 의 패키지를 복사)과, `utils/object_detector.py` 에 .192 의 "검출 0개 프레임 건너뛰기" 패치가 없어 첫 프레임에서 예외 뒤 25분 제한까지 멈춘 것(.192 파일을 그대로 복사해 두 머신 코드를 같게 맞춤)입니다. 증상이 "yolo 속성 오류"와 0% 멈춤으로만 보여 원인과 멀리 떨어져 있었습니다.
- 결과: 합계 40개 중 제안 38/2(90.5%), FM-Fusion 34/186(15.0%), DualMap 14/18(24.1%), DualMap-BGR 20/24(31.2%). 세 번의 실행(노트북 10-10, 노트북 10-11, .41)에서 순위와 차이가 그대로입니다. 제안의 .41 인식 빈도는 평균 6.70 Hz(노트북 2.7 Hz).

### 2. .41 한 대 구성 — ORB-SLAM3 를 서버에 (27 서버·클라이언트 §4, 26 §9)

**왜:** .192 가 GroundingDINO 검출만 띄우면 1~3분 안에 멈춰(10-13 10:08, 10-14 두 번) 데스크톱까지 굳었고, 커널 로그는 sudo 가 없어 볼 수 없었습니다. 사용자 지시로 ".192 를 고칠 때까지 모든 실험을 .41 에서" 하기로 했습니다.

**만든 것**

- `.41:~/objpose` 에 ORB-SLAM3 호스트(`build/slam_stream`, `run_slam.sh`)를 저장소의 `objpose/linux_slam/build_host.sh` 로 빌드했습니다(48코어 약 10분). Pangolin 소스와 ORB 어휘 파일은 .192 에서 복사.
- .41 은 sudo 가 없어 `libsqlite3-dev` · `nlohmann-json3-dev` 를 apt 로 못 깝니다. .192 의 헤더를 `deps/install/include/` 에 두고, `libsqlite3.so` 는 시스템 `.so.0` 링크, nlohmann 은 헤더 전용 cmake 설정 파일을 손으로 써서 `build_host.sh` 가 apt 를 건너뛰고 `CMAKE_PREFIX_PATH` 로 찾도록 고쳤습니다(32dbc801).
- hub 는 SLAM 을 `ssh -R` 역터널로 띄우므로 `ssh localhost` 키 로그인만 열면 코드 수정 없이 `--mac-host localhost` 로 됩니다. 실행 스크립트 `src/realdata/run_live_41solo.sh`.

**점검 — 260901_cbnu_eightcircle, 개선안 3 + 군집 먼저**

| 구성 | 인식 빈도 | 최종 지도 맞음 / 잘못 | 실행 중 평균 점수 | 지도 등록 중앙값 |
|---|---|---|---|---|
| .41 hub + .192 ORB-SLAM3 (10-12) | 6.93 Hz | 7 / 1 | 76.6% | 38.1 s |
| .41 hub + .41 ORB-SLAM3 (10-14 09:48) | 6.79 Hz | 8 / 1 | 77.1% | 38.0 s |

SLAM hello 까지 18초, 4,536 프레임 추적(드롭 0), 1,024 프레임 인식, 151초 녹화를 1배속 완주, 루프 보정 최대 7.8 cm. 종료 시 hub 가 `KeyboardInterrupt` 트레이스백을 남겼지만 summary 를 쓴 뒤의 shutdown 단계라 결과에는 영향이 없습니다.

### 3. FM-Fusion 검출부터 .41 에서 (27 객체지도 4090 §9)

**왜:** 27 §7 의 FM-Fusion 4090 행은 융합만 다시 돌리고 검출은 노트북 것을 재사용한 것이라 "같은 GPU 재실행" 단서가 붙어 있었습니다. 검출을 .192 에서 돌리려다 세 번 멈췄으므로 .41 에서 돌렸습니다. 스크립트 `src/objmap/run_fmf_detect_41.sh`, 작업 폴더 `ycbv_work/fmfusion_41det/`(color/depth/pose 는 노트북 scans 링크, 검출 폴더만 새로), 결과 `src/objmap/out_41det/`.

**실행:** 09:47 검출 시작(GPU 2, 10프레임마다) → 10:04 검출 끝(YCB-V 2,067장 11.3분, ObChange 898장 4.9분, 장당 0.32 s) → 융합·채점 10:13~10:30.

**사고 — 융합 전부 실패(rc=1), 10:04.** 스크립트가 4090 체인(`chain_ours_4090.sh`)과 달리 `config_*.yaml` · `likelihood_*.txt` · `splits/` 를 새 작업 폴더에 복사하지 않아 `IntegrateInstanceMap` 이 설정을 못 읽고 빈 지도를 냈고, 그 상태로 채점기가 돌아 FM-Fusion 0 점을 찍었습니다. 설정을 복사하고 검출 단계를 뺀 융합·채점만 다시 돌렸으며(`ycbv_work/run_fmf_fuse_41.sh`), 스크립트에 복사 줄을 넣었습니다. 새 작업 폴더를 만드는 스크립트는 FM-Fusion 이 읽는 보조 파일 세 가지를 꼭 함께 복사해야 합니다.

**결과:** YCB-V 28/106/27(F1 0.296, mIoU 0.371), 중심 5 cm 29/105/26(0.307), ObChange 방문별 13/67/12(0.248) · IoU 3/77/22(0.057) — 노트북 검출 기반 값과 TP · FP · FN 까지 같습니다. 연속 지도만 FP 가 6개 늘었습니다(100 → 106). 검출 자체는 2D 상자 12,780 → 12,778 개, 좌표는 소수점 3~4째 자리, 점수는 0.001 안에서만 다른 부동소수 차이입니다.

### 4. .192 복구와 같은 검출 재실행 (27 객체지도 4090 §9.1)

- 09:54 사용자가 .192 드라이버를 575.57.08 → 580.178.04 로 올리고 재부팅했습니다. 디스플레이는 nvidia-smi 3번(bus E1) GPU 에 붙어 있습니다.
- .192 에는 별도 Claude 세션이 떠 있었고, 그 세션이 09:59~10:38 에 `chain_ours_4090.sh fmf_detect`(`--msda pytorch`)를 돌렸습니다. 그런데 YCB-V 검출 폴더에 옛 드라이버 때 멈추다 쌓인 결과가 남아 있어 12장면을 모두 건너뛰었고(로그 "0 frames detected"), ObChange 898장만 새로 검출했습니다. 섞인 실행이라 깨끗한 재실행으로 치지 않았습니다.
- 11:01 새 작업 폴더 `ycbv_work/fmfusion_4090det2/`, 결과 `src/objmap/out_4090det2/` 로 `src/objmap/run_fmf_detect_192.sh` 를 띄웠습니다. 구성은 노트북·.41 과 같은 **원래 구성**(컴파일된 `_C` deformable-attention 커널, `--msda cuda`)이고, 디스플레이 GPU 를 피해 bus 01(0번)을 썼습니다. 다른 세션의 폴더(`out_4090`, `fmfusion_4090`)는 건드리지 않았습니다.
- 2분마다 로그 · 검출 파일 수 · nvidia-smi 응답 · ssh 를 확인했고, 검출 16.7분 + 융합 26분 + 채점 1분, **44분 동안 이상이 없었습니다**(11:45 끝). 옛 드라이버에서는 같은 검출이 1~3분 안에 멈췼습니다.

| 평가 | 노트북 | .41 | .192 (580) |
|---|---|---|---|
| YCB-V IoU 0.25 TP / FP / FN, F1 | 28 / 106 / 27, 0.296 | 28 / 106 / 27, 0.296 | 28 / 106 / 27, 0.296 |
| ObChange 방문별 중심 5 cm | 13 / 67 / 12, 0.248 | 13 / 67 / 12, 0.248 | 13 / 67 / 12, 0.248 |
| ObChange 연속 중심 5 cm | 11 / 100 / 14, 0.162 | 11 / 106 / 14, 0.155 | 11 / 105 / 14, 0.156 |
| 2D 상자 YCB-V / ObChange | 12,780 / 3,995 | 12,778 / 3,997 | 12,778 / 3,996 |
| 검출 장당 시간 | 1.18 s | 0.32 s | 0.33 s |

### 5. 머신 운용에서 배운 것

- **.192 멈춤의 원인은 575 드라이버(와 GroundingDINO `_C` 커널 조합)로 보는 것이 맞습니다.** 디스플레이 GPU 공유를 의심했지만, 580 에서는 같은 커널로 17분 검출이 무사했습니다. Xid 코드는 끝까지 확인하지 못했습니다(adm 그룹이 아니라 dmesg 를 못 봄). 그래도 .192 실험은 디스플레이 GPU(부팅마다 바뀔 수 있음, 지금은 3번)를 피하는 습관은 유지합니다.
- **같은 저장소 체크아웃이라도 머신마다 로컬 패치와 pip 패키지가 다를 수 있습니다.** DualMap 의 mobileclip · 검출 0개 패치가 그 예였습니다. 기준 방법이 .41 에서 이상하면 먼저 .192 와 `git diff` · md5 · 환경 패키지를 비교합니다.
- **새 작업 폴더로 기준 방법을 돌릴 때는 보조 파일을 함께 복사합니다.** FM-Fusion 은 `config_*.yaml` · `likelihood_*.txt` · `splits/`, DualMap 은 `model/` · mobileclip.
- **한 머신에 Claude 세션이 둘 이상 떠 있을 수 있습니다.** 같은 출력 폴더를 쓰면 서로 결과를 덮어쓰므로, 다른 세션이 만든 폴더는 두고 새 폴더에 씁니다(오늘 `out_4090det2`).
- **ssh 로 원격 실행을 감시할 때는** 완료 표시뿐 아니라 실패 표시(ABORT, rc≠0)와 머신 멈춤 표시(nvidia-smi 무응답, ssh 실패)를 모두 잡는 감시로 걸어야 멈춤이 "조용한 진행 중" 으로 보이지 않습니다.

### 6. 남은 일

- 26 §7 과 27(서버·클라이언트) §4 에 적은 ".192 멈춤 → .41 단독" 조치 해제. 서버·클라이언트 구성(.41 hub + .192 ORB-SLAM3)으로 260901 eightcircle 실시간 점검을 한 번 더 돌려 10-12 · 10-14 결과와 나란히 두면 됩니다.
- 5090 에서 세 방법을 모두 돌려 논문 본문의 최종 수치를 정하는 일(26 문서 순서).
- .41 의 `run_fmf_detect_41.sh` 와 .192 의 `run_fmf_detect_192.sh` 는 경로만 다른 같은 스크립트입니다. 다음 장비에서는 하나로 합쳐 `OBJMAP_HOST` 변수로 경로를 고르게 하면 됩니다.