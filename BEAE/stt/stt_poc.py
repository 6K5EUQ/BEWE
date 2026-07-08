#!/usr/bin/env python3
# BEWE STT PoC — faster-whisper large-v3 로 복조 음성 WAV 를 전사.
# 1단계 검증용: 실시간 데몬/IPC 없이 파일 배치 전사 정확도만 확인.
# 입력 WAV = mono 48kHz int16 (demod.cpp 산출). 16kHz 리샘플 후 whisper.
import sys, time
import numpy as np
import soundfile as sf
from faster_whisper import WhisperModel

MODEL = "large-v3"
DEV, COMPUTE = "cuda", "float16"

def load_16k_mono(path):
    audio, sr = sf.read(path, dtype="float32", always_2d=False)
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    if sr != 16000:                       # 선형보간 리샘플 (PoC 수준으로 충분)
        n = int(round(len(audio) * 16000 / sr))
        audio = np.interp(np.linspace(0, len(audio), n, endpoint=False),
                          np.arange(len(audio)), audio).astype("float32")
    return audio

def main(paths):
    t0 = time.time()
    model = WhisperModel(MODEL, device=DEV, compute_type=COMPUTE)
    print(f"[load] {MODEL} on {DEV}/{COMPUTE}  ({time.time()-t0:.1f}s)\n")
    for p in paths:
        audio = load_16k_mono(p)
        dur = len(audio) / 16000
        t = time.time()
        segments, info = model.transcribe(
            audio, language="ko", vad_filter=True,
            vad_parameters=dict(min_silence_duration_ms=500))
        segs = list(segments)
        rtf = (time.time() - t) / max(dur, 1e-3)
        print(f"=== {p.split('/')[-1]}  ({dur:.1f}s audio, RTF={rtf:.2f}, lang={info.language} p={info.language_probability:.2f})")
        if not segs:
            print("  (무음/발화 없음)")
        for s in segs:
            print(f"  [{s.start:5.1f}-{s.end:5.1f}] {s.text.strip()}")
        print()

if __name__ == "__main__":
    main(sys.argv[1:])
