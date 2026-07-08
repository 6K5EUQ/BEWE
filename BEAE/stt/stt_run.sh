#!/bin/sh
# STT 워커 실행 래퍼 — C++ stt_module 이 fork+exec 로 호출.
# faster-whisper(ctranslate2) 가 GPU 쓰려면 venv 의 nvidia cublas/cudnn .so 경로가
# LD_LIBRARY_PATH 에 있어야 한다. python 버전 무관하게 glob 으로 잡는다.
DIR="$(cd "$(dirname "$0")" && pwd)"
NV="$(ls -d "$DIR"/.venv/lib/python*/site-packages/nvidia 2>/dev/null | head -1)"
if [ -n "$NV" ]; then
    export LD_LIBRARY_PATH="$NV/cublas/lib:$NV/cudnn/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec "$DIR/.venv/bin/python" "$DIR/stt_worker.py"
