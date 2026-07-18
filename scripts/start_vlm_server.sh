#!/bin/bash
# Start llama-server (CUDA) for Qwen2.5-VL-3B on the Jetson Xavier.
# Usage:
#   ./scripts/start_vlm_server.sh            # n_gpu_layers = 99 (offload everything, default)
#   ./scripts/start_vlm_server.sh 16         # offload 16 layers (Table 4.2 sweep)
#   NGL=0 ./scripts/start_vlm_server.sh      # CPU-only
cd "$(dirname "$0")/.." || exit 1

NGL="${1:-${NGL:-99}}"          # GPU offload layers (arg 1 or the NGL variable)
PORT="${PORT:-8080}"
CTX="${CTX:-2048}"
THREADS="${THREADS:-4}"

SERVER="llama.cpp/build/bin/llama-server"
MODEL="layer2_brain/models/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf"
MMPROJ="layer2_brain/models/mmproj-Qwen2.5-VL-3B-Instruct-f16.gguf"

[ ! -f "$SERVER" ] && { echo "[ERR] Not built: $SERVER"; exit 1; }
[ ! -f "$MODEL" ]  && { echo "[ERR] Missing model: $MODEL (run: python layer2_brain/setup_vlm.py)"; exit 1; }

# CUDA runtime libs (libcublas/libcudart...) - REQUIRED for the CUDA build
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}"

echo "================================================"
echo "  LLAMA.CPP CUDA  --  Qwen2.5-VL-3B-Instruct"
echo "  n_gpu_layers=$NGL | ctx=$CTX | port=$PORT | threads=$THREADS"
echo "================================================"

# Do not force --chat-template: let jinja use the Qwen-VL template bundled in the GGUF
exec "$SERVER" \
    --model "$MODEL" \
    --mmproj "$MMPROJ" \
    --host 127.0.0.1 --port "$PORT" \
    --ctx-size "$CTX" \
    --n-gpu-layers "$NGL" \
    --threads "$THREADS"
