# _perf_multiinstance_probe_2026_08_30 — 멀티 인스턴스 타당성 실측

실시간 SAM-6D 가시화 모듈 설계에 앞서, "같은 GPU에 SAM-6D 인스턴스를 N개 띄우면
처리량이 늘어나는가"를 실측한 일회성 probe. 원본 코드는 수정하지 않았고
**이 폴더만 지우면 원상 복구**된다.

- 환경: RTX 5090 Laptop 24GB · conda env `sam6d` · bag `data/260826_eightcircle_dark` (640×480)
- 방법: bag 프레임 60장(stride 15)을 메모리에 선적재 후 `Sam6DCore.process()` 연속 호출.
  GPU 사용률은 pynvml 50ms 샘플링. 다중 인스턴스는 barrier 파일로 시작 시점 정렬.
- 실행: `bash run_multi.sh <N> <tag>` (내부에서 conda/ROS/MPS env 설정)

## 결과 (2026-08-30)

| 구성 | 합산 FPS | 스케일링 | 프레임당 지연(mean/p90) | PEM(mean) | GPU util(mean) | VRAM/proc |
|---|---|---|---|---|---|---|
| 1개 (기준) | **2.83** | 1.00× | 353 / 615 ms | 277 ms | 66% | 6.0 GB |
| 2개, MPS 없음 | 2.44 | **0.86×** (오히려 감소) | 819 / 1500 ms | 691 ms | 89%* | 6.0 GB |
| 2개, MPS | **3.52** | 1.24× | 568 / 1035 ms | 432 ms | 82% | 6.0 GB |
| 3개, MPS | 3.56 | 1.26× (포화) | 844 / 1530 ms | 600 ms | 86% | 6.0 GB |

\* MPS 없는 89%는 두 컨텍스트가 time-slice로 번갈아 도는 시간의 합산이라 실제 처리량과 무관.

단계별 (단일 기준): yolo 7ms / ism 87ms / **pem 277ms** / total 353ms.
검출 물체가 많은 프레임은 PEM이 500~650ms까지 늘어남 (p90 기준) — "0.5초"의 정체.

## 결론

1. **MPS 없이 다중 프로세스는 역효과** — CUDA 컨텍스트가 직렬화되어 합산 FPS가 오히려 준다.
2. **MPS를 켜야 이득이 생기지만 +24~26%에서 포화** (2.83 → 3.5 FPS). 병목인 PEM의
   GPU 커널이 이미 GPU를 상당히 채우고 있어(단일 util 66%) 겹칠 여지가 크지 않다.
3. 인스턴스를 늘릴수록 **개별 프레임 지연은 증가** (353ms → 568ms @2개). 가시화 관점에서
   갱신 주기(FPS)는 늘고 표시되는 포즈의 신선도는 나빠지는 트레이드오프.
4. VRAM은 문제 아님 (프로세스당 6.0GB, `expandable_segments:True` 기준 3개=18.4GB).
5. 권장: **2개 워커 + MPS**까지만 (3개는 이득 0). 그 이상은 멀티 인스턴스가 아니라
   PEM 자체(후보 300개 파이프라인) 경량화가 필요.

## MPS 데몬 운용

```bash
# 시작 (기본 경로 사용 — 경로가 길면 unix socket 108자 제한으로 조용히 실패한다!)
export CUDA_MPS_PIPE_DIRECTORY=/tmp/nvidia-mps CUDA_MPS_LOG_DIRECTORY=/tmp/nvidia-mps-log
mkdir -p $CUDA_MPS_PIPE_DIRECTORY $CUDA_MPS_LOG_DIRECTORY
nvidia-cuda-mps-control -d
# 클라이언트(추론 프로세스)도 같은 CUDA_MPS_PIPE_DIRECTORY 를 export 해야 붙는다
# 종료
echo quit | nvidia-cuda-mps-control
```

## 산출물

- `outputs/<tag>_i<N>.json` — 프레임별 단계 ms + GPU 샘플 전체 (원자료)
- `outputs/<tag>_i<N>.log` — 실행 로그와 요약
- `bench_continuous.py` — 측정 스크립트 (bag 선적재 → 연속 추론 → pynvml 샘플링)
- `run_multi.sh` — N개 동시 실행 런처 (barrier 정렬)
