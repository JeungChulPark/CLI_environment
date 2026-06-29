import blenderproc as bproc
import bpy
# BlenderProc GPU 인식 여부 확인 스크립트
# 실행: blenderproc run check_gpu.py

bproc.init()

prefs = bpy.context.preferences.addons['cycles'].preferences
prefs.refresh_devices()

print("\n===== Blender GPU 인식 결과 =====")

found_gpu = False
for device_type in ['CUDA', 'OPTIX', 'METAL', 'HIP']:
    try:
        prefs.compute_device_type = device_type
        devices = prefs.get_devices_for_type(device_type)
        if devices:
            for d in devices:
                status = "사용 가능" if d.use else "비활성"
                print(f"  [{device_type}] {d.name} — {status}")
                found_gpu = True
    except Exception:
        pass

if found_gpu:
    print("\n결과: GPU 인식 성공 → GPU 렌더링 가능")
else:
    print("\n결과: GPU 미인식 → CPU 렌더링만 가능")

print("=================================\n")
