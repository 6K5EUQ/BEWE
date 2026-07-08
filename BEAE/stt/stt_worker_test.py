#!/usr/bin/env python3
# stt_worker.py 단독 테스트 — WAV 하나를 발화 프레임 1개로 만들어 워커에 stdin 파이프,
# stdout 자막 JSON 확인. (C++ demod 를 흉내내는 하니스)
import sys, struct, subprocess, soundfile as sf, numpy as np, os

MAGIC = 0x31545453
HDR = struct.Struct("<IIII")

def frame(ch, sr, audio):
    a = np.asarray(audio, dtype="<f4")
    return HDR.pack(MAGIC, ch, sr, len(a)) + a.tobytes()

def main():
    wav = sys.argv[1]
    audio, sr = sf.read(wav, dtype="float32", always_2d=False)
    if audio.ndim > 1: audio = audio.mean(axis=1)
    worker = os.path.join(os.path.dirname(__file__), "stt_worker.py")
    py = os.path.join(os.path.dirname(__file__), ".venv/bin/python")
    if not os.path.exists(py): py = sys.executable
    p = subprocess.Popen([py, worker], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         env={**os.environ})
    # 채널 3, 5 두 개로 같은 오디오 보내 채널 태그 확인
    p.stdin.write(frame(3, sr, audio))
    p.stdin.write(frame(5, sr, audio))
    p.stdin.flush()
    p.stdin.close()   # EOF → 워커 종료
    for line in p.stdout:
        print(line.decode("utf-8", "replace").rstrip())
    p.wait()

if __name__ == "__main__":
    main()
