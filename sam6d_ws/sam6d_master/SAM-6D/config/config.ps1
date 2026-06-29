# SAM-6D Pipeline Configuration
# Usage (PowerShell):
#   . .\config.ps1   (dot-sourcing 필수)

$env:CAD_PATH      = "D:\LDH_ws\sam_6d\SAM-6D\Data\Example\obj_000005.ply"
$env:RGB_PATH      = "D:\LDH_ws\sam_6d\SAM-6D\Data\Example\rgb.png"
$env:DEPTH_PATH    = "D:\LDH_ws\sam_6d\SAM-6D\Data\Example\depth.png"
$env:CAMERA_PATH   = "D:\LDH_ws\sam_6d\SAM-6D\Data\Example\camera.json"
$env:OUTPUT_DIR    = "D:\LDH_ws\sam_6d\SAM-6D\Data\Example\outputs"
$env:SEGMENTOR_MODEL = "sam"
$env:SEG_PATH      = "$env:OUTPUT_DIR\sam6d_results\detection_ism.json"

Write-Host "[config.ps1] Variables loaded:"
Write-Host "  CAD_PATH     = $env:CAD_PATH"
Write-Host "  RGB_PATH     = $env:RGB_PATH"
Write-Host "  DEPTH_PATH   = $env:DEPTH_PATH"
Write-Host "  CAMERA_PATH  = $env:CAMERA_PATH"
Write-Host "  OUTPUT_DIR   = $env:OUTPUT_DIR"
Write-Host "  SEGMENTOR    = $env:SEGMENTOR_MODEL"
