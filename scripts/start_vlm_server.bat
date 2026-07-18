@echo off
REM Chay tu thu muc goc project (script nam trong scripts/)
cd /d "%~dp0.."
echo ================================================
echo   LLAMA.CPP SERVER  --  Qwen2.5-VL-3B-Instruct
echo ================================================
echo.

if not exist "layer2_brain/models/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf" (
    echo [LOI] Khong tim thay: layer2_brain/models/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf
    echo       Chay: python layer2_brain/setup_vlm.py
    pause & exit /b 1
)

set SERVER=llama.cpp\bin\llama-server.exe
if not exist "%SERVER%" (
    where llama-server >nul 2>&1
    if errorlevel 1 (
        echo [LOI] Khong tim thay llama-server.exe
        echo       Download tai: https://github.com/ggml-org/llama.cpp/releases
        pause & exit /b 1
    )
    set SERVER=llama-server
)

echo Server : %SERVER%
echo Model  : layer2_brain/models/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf
echo MMProj : layer2_brain/models/mmproj-Qwen2.5-VL-3B-Instruct-f16.gguf
echo Host   : 127.0.0.1:8080
echo Context: 4096 tokens  ^|  GPU layers: 0  ^|  Threads: 4
echo.

%SERVER% ^
    --model "layer2_brain/models/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf" ^
    --mmproj "layer2_brain/models/mmproj-Qwen2.5-VL-3B-Instruct-f16.gguf" ^
    --host 127.0.0.1 ^
    --port 8080 ^
    --ctx-size 1024 ^
    --n-gpu-layers 0 ^
    --threads 4 ^
    --flash-attn on ^
    --cache-type-k q8_0 ^
    --cache-type-v q8_0 ^
    --batch-size 512 ^
    --chat-template chatml

pause
