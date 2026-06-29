@echo off
REM SAM-6D Pipeline Configuration
REM Usage (Windows CMD):
REM   call config.bat

set CAD_PATH=D:\LDH_ws\sam_6d\SAM-6D\Data\Example\obj_000006.ply
set RGB_PATH=D:\LDH_ws\sam_6d\SAM-6D\Data\Example\rgb.png
set DEPTH_PATH=D:\LDH_ws\sam_6d\SAM-6D\Data\Example\depth.png
set CAMERA_PATH=D:\LDH_ws\sam_6d\SAM-6D\Data\Example\camera.json
set OUTPUT_DIR=D:\LDH_ws\sam_6d\SAM-6D\Data\Example\outputs_obj6
set SEGMENTOR_MODEL=sam
set SEG_PATH=%OUTPUT_DIR%\sam6d_results\detection_ism.json

echo [config.bat] Variables loaded:
echo   CAD_PATH     = %CAD_PATH%
echo   RGB_PATH     = %RGB_PATH%
echo   DEPTH_PATH   = %DEPTH_PATH%
echo   CAMERA_PATH  = %CAMERA_PATH%
echo   OUTPUT_DIR   = %OUTPUT_DIR%
echo   SEGMENTOR    = %SEGMENTOR_MODEL%
