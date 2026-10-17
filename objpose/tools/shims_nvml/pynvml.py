"""pynvml 대역 — NVIDIA 라이브러리와 커널 모듈 버전이 어긋나 있는 동안(자동 업데이트 뒤 재부팅 전) 만 쓴다.

그 상태에서는 nvmlInit() 가 NVMLError_LibRmVersionMismatch 로 실패하고, gorilla(utils/gpu.py)가 import 때 nvmlInit() 을 불러
SAM-6D 의 자세 추정 모델을 불러오지 못한다. CUDA 연산 자체는 된다. 이 폴더를 PYTHONPATH 앞에 두면 nvmlInit() 이 아무 일도 하지 않는다.
재부팅해서 버전이 맞으면 이 폴더를 PYTHONPATH 에서 빼면 된다.
"""


class NVMLError(Exception):
    pass


def nvmlInit(*a, **k): return None
def nvmlShutdown(*a, **k): return None
def nvmlDeviceGetCount(*a, **k): return 0


def __getattr__(name):          # 그 밖의 함수는 부르면 오류 (조용히 틀린 값을 주지 않는다)
    def _missing(*a, **k): raise NVMLError(f"pynvml shim: {name} unavailable (driver/library version mismatch; reboot to fix)")
    return _missing
