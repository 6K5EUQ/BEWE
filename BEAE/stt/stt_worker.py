#!/usr/bin/env python3
# BEWE STT 공유 워커 — 여러 채널이 whisper 모델 1개를 공유.
# C++ demod 가 squelch open→close 발화 조각(mono float @ AUDIO_SR)을 프레임으로 stdin 에 보냄.
# 워커는 발화를 큐에 넣고 단일 스레드로 순차 전사 → stdout 에 자막 JSON 한 줄씩.
#
# 프레임 프로토콜 (little-endian):
#   stdin:  [u32 magic='STT1'][u32 ch_id][u32 sr][u32 n_samples][f32 audio[n]]
#   stdout: JSON line
#     준비완료:  {"ready":true,"model":"large-v3"}
#     자막:      {"ch":N,"t_ms":<epoch ms>,"dur":<sec>,"text":"..."}
#     에러:      {"error":"..."}
#
# 종료: stdin EOF (C++ 가 마지막 STT 채널 끄면 파이프 닫음) → flush 후 exit.
import sys, os, json, struct, threading, queue, time

MAGIC = 0x31545453  # 'STT1' LE
MODEL = os.environ.get("BEWE_STT_MODEL", "large-v3")
LANG  = os.environ.get("BEWE_STT_LANG", "ko")
HDR   = struct.Struct("<IIII")   # magic, ch, sr, n

def log(obj):
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()

def read_exact(f, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = f.read(n - len(buf))
        if not chunk:
            return None            # EOF
        buf.extend(chunk)
    return bytes(buf)

def main():
    import numpy as np
    from faster_whisper import WhisperModel

    model = WhisperModel(MODEL, device="cuda", compute_type="float16")
    # 워밍업(첫 추론 지연 제거) — 0.5초 무음 1회
    list(model.transcribe(np.zeros(8000, dtype="float32"), language=LANG))
    log({"ready": True, "model": MODEL})

    q = queue.Queue(maxsize=64)

    def transcriber():
        while True:
            item = q.get()
            if item is None:
                return
            ch, sr, audio = item
            # whisper 는 16k 기대 — 선형보간 리샘플
            if sr != 16000:
                n = int(round(len(audio) * 16000 / sr))
                audio = np.interp(np.linspace(0, len(audio), n, endpoint=False),
                                  np.arange(len(audio)), audio).astype("float32")
            dur = len(audio) / 16000.0
            try:
                segs, _ = model.transcribe(audio, language=LANG, vad_filter=False)
                text = " ".join(s.text.strip() for s in segs).strip()
            except Exception as e:
                log({"error": f"transcribe: {e}", "ch": ch})
                sys.stderr.write(f"[stt] transcribe ERROR ch={ch}: {e}\n"); sys.stderr.flush()
                continue
            sys.stderr.write(f"[stt] result ch={ch} dur={dur:.1f}s text={text!r}\n"); sys.stderr.flush()
            if text:
                log({"ch": ch, "t_ms": int(time.time() * 1000), "dur": round(dur, 1), "text": text})

    th = threading.Thread(target=transcriber, daemon=True)
    th.start()

    stdin = sys.stdin.buffer
    while True:
        hdr = read_exact(stdin, HDR.size)
        if hdr is None:
            break                  # EOF → 종료
        magic, ch, sr, n = HDR.unpack(hdr)
        if magic != MAGIC or n == 0 or n > 16_000_000:
            log({"error": f"bad frame magic={magic:#x} n={n}"}); break
        body = read_exact(stdin, n * 4)
        if body is None:
            break
        sys.stderr.write(f"[stt] frame ch={ch} sr={sr} n={n} ({n/max(sr,1):.1f}s)\n"); sys.stderr.flush()
        audio = np.frombuffer(body, dtype="<f4").copy()
        try:
            q.put_nowait((ch, sr, audio))
        except queue.Full:
            log({"error": "queue full — dropping utterance", "ch": ch})

    q.put(None)
    th.join(timeout=30)

if __name__ == "__main__":
    main()
