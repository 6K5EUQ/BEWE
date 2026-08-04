# BEAE/uav — ExpressLRS 2.4 GHz 신호 역산·해독

## 목표

2.4 GHz ExpressLRS 조종 링크를 **바인드 문구 모름 + 조종기 제어 불가** 상태에서
수동 관측만으로 분석한다. 광대역 IQ(61.44 MSPS, ~50 MHz 커버)를 오프라인 처리.

- **1단계**: FHSS 호핑 시퀀스 재구성 → LCG 시드 역산 → UID 확정 (에너지 검출 기반, LoRa 복조 불필요)
- **2단계**: SX1280 LoRa 심볼 복조 → SYNC 패킷에서 UID[4:5] 평문 추출 + 페이로드 디코드

## 근거 (ExpressLRS 소스에서 확정, `~/ExpressLRS`)

- ISM2G4: 2400.4~2479.4 MHz, **80채널, 1.000 MHz 간격**, sync 채널 = ch40 = 2440.4 MHz
- FHSS 시퀀스: LCG `seed=(214013·seed+2531011) mod 2^31`, 출력 `seed>>16`
- 시드 = `OtaGetUidSeed()` = `(UID[2]<<24)|(UID[3]<<16)|(UID[4]<<8)|(UID[5]^4)` — **UID 바이트 2~5만**
- sync 채널은 freq_count(80)당 정확히 1회, 블록 시작마다 등장 → 위상 앵커
- SYNC 패킷이 UID[4], UID[5]를 **평문 송출** (UID5는 model-id 마스크 0x3f로 XOR 가능)
- 암호화 없음. LoRa sync word 없음, 라디오 CRC off. UID가 사실상 유일한 열쇠
- 패킷 레이트별 LoRa 파라미터: SF5~8, BW 812.5 kHz, **LI(long-interleaved) 코딩레이트** — 표준 LoRa 디코더 불가
- CRC: 커스텀 14/16bit, init = `(UID[4]<<8|UID[5]) ^ (4<<8) ^ nonce`

## 디렉터리 구조 (기존 BEAE 관례 따름)

```
BEAE/uav/
  PLAN.md                  ← 이 문서
  README.md
  requirements.txt
  .venv/                   ← gitignore (numpy/scipy, 2단계에서 확장)
  uav_elrs/                ← 파이썬 패키지
    __init__.py
    fhss.py                ← LCG + 시퀀스 생성 (소스 1:1 포팅)
    uid.py                 ← 바인드문구→UID, UID→시드, 시드 역산
    detect.py             ← 1단계: IQ → 채널별 에너지 → 호핑 이벤트 추출
    solve.py               ← 1단계: 호핑 이벤트 → LCG 시드 매칭 → UID
    lora.py                ← 2단계: SX1280 LoRa 디챠프·디인터리브·디코드
    sync_packet.py         ← 2단계: SYNC 패킷 파싱 → UID[4:5]
    crc.py                 ← 커스텀 CRC14/16 (소스 포팅, 검증용)
  tests/
    test_fhss.py           ← 소스 알고리즘과 시퀀스 일치 검증
    test_uid.py            ← 바인드문구→UID→시드→역산 왕복
    test_solve_sim.py      ← 합성 호핑열로 역산 유일성 검증
  data/                    ← gitignore (IQ 녹음, 중간산물)
```

## 실행 순서 (검증 게이트로 끊음)

### Phase A — 알고리즘 검증 (하드웨어·IQ 없이, 순수 계산)
**여기서 역산이 유일해로 수렴 안 하면 2단계는 무의미하므로 먼저 한다.**

1. `fhss.py` — LCG + `FHSSrandomiseFHSSsequenceBuild` 를 소스 그대로 포팅
2. `uid.py` — 바인드문구 MD5→UID, `OtaGetUidSeed`, 시드→UID 역산
3. `test_fhss.py` — 알려진 시드로 생성한 시퀀스가 소스 로직과 바이트 일치
4. `solve.py` — **합성 실험**: 임의 UID로 시퀀스 생성 → 그중 62%만 관측(채널 랜덤 마스킹) →
   sync 채널 위상 앵커 + LCG 시드 브루트포스(2^32, 또는 UID[4:5] 알려지면 2^16) → UID 복원
5. `test_solve_sim.py` — 관측률 40~80% 스윕, 관측 길이별로 UID 유일 확정에 필요한 최소
   호핑 수를 측정. **결과표를 README 에 남긴다** (이게 1단계 실현성의 정량 근거)

→ **게이트: 62% 관측에서 UID 유일 확정되면 Phase B 진행. 아니면 여기서 멈추고 보고.**

### Phase B — 1단계 실측 파이프라인 (IQ 필요, LoRa 복조 없음)
6. `detect.py` — 61.44 MSPS IQ → 폴리페이즈 채널라이저로 채널 분할 → 채널별 에너지 시계열 →
   패킷 버스트 검출 → (시각, 채널번호) 호핑 이벤트 열
7. `solve.py` 실측 연결 — 실 호핑열 → 시드 역산 → UID
8. 검증: 같은 IQ에서 sync 채널(2440.4) 주기성이 freq_count 규칙과 맞는지 교차확인

### Phase C — 2단계 LoRa 복조 (가장 큰 작업)
9. `lora.py` — SX1280 LoRa: 다운챠프 상관 → 심볼 → 그레이·디인터리브(**LI 코딩레이트**)→
   화이트닝 없음 확인 → 페이로드 바이트
10. `sync_packet.py` — SYNC 패킷(type=0b10) 파싱 → UID[4],UID[5] 평문 추출
    (이게 되면 Phase A 의 역산이 2^32→2^16 으로 축소되어 훨씬 쉬워진다)
11. `crc.py` — 커스텀 CRC 로 복조 결과 검증 (UID 확정 후)
12. 페이로드 디코드 — RC 채널값(스로틀/방향) 언팩

→ 각 Phase 끝에 사용자 확인. LoRa LI 디인터리브가 막히면 그 지점에서 보고.

## 구현 상태 (2026-08-03, 실측 IQ 이전 단계 완료)

- **Phase A**: 완료. fhss/uid/crc 소스 1:1 포팅, C 크로스체크 **바이트 동일**.
  solve 유일성 게이트 통과 (UID4/5 known 시 40/80 커버·≤20홉 유일 복원).
- **Phase B**: 완료. STFT 채널라이저 + 버스트 검출 → 호핑 이벤트. 합성 IQ 검증.
- **Phase C**: 심볼 복조(디챠프/FFT/그레이) 정확, os 1~4·IQ반전 왕복 통과.
  프레임 코덱·SYNC→UID 파이프라인 왕복 통과. **바이트 단계(LI 인터리버)만 실측 보정 대기.**
- **적대적 검증** 워크플로로 5건 지적 → 전부 반영: base_chirp 오버샘플 버그(수정),
  find_preamble STO(추가), channel_hz 레지스터 오차(문서화+exact 함수), numeric-UID
  지름길(추가), docstring 라인참조(수정). 테스트 25개 전부 통과.

### 근본 한계 (알고리즘이 드러냄, 실측으로도 안 바뀜)
- UID[0], UID[1]: 전파 미포함 → RF 복원 불가 (링크 추종·복조엔 불필요).
- UID[2] MSB: LoRa 모드에서 자유 비트 (LCG mod 2^31). UID[2]와 UID[2]^0x80 운용 동일.

### 실측 IQ 도착 시 (남은 것)
- 61.44 MSPS 낼 SDR 확정 (BladeRF 유력).
- LoRa 프레임 sync: CFO 디로테이션 + 서브칩 STO 리샘플 (실신호 필요).
- SX1280 LI 인터리버: 실 SYNC 버스트(known-plaintext)로 보정.

## 위험·불확실성 (명시)

- **하드웨어 미확정**: 61.44 MSPS 를 실제로 낼 SDR 이 어느 기지에 있는지 미확인.
  RTL 불가, Pluto USB 병목 의심, BladeRF 유력 — Phase B 전에 실측 확인 필요.
- **LI 코딩레이트**: SX1280 의 long-interleaved LoRa 는 표준 디코더가 처리 못 함.
  Phase C 최대 난관. 소스의 인터리버 구조를 역추적해야 함.
- **50/79 MHz 커버**: 30채널은 못 봄. Phase A 의 합성 실험이 이 손실을 견디는지가 관건.
- **저장량**: 61.44 MSPS 복소 = ~246 MB/s. 오프라인 배치 전제.
- **BEWE 코어와 무관**: 이건 독립 오프라인 도구다. C++ 모듈·IPC·모듈등록 없음.
  기존 BEAE(ais/amc)처럼 BEWE 가 소켓으로 부르는 구조가 아니라 순수 CLI 분석기.

## gitignore 추가 (`~/BEWE/.gitignore`)

```
BEAE/uav/.venv/
BEAE/uav/**/__pycache__/
BEAE/uav/data/
```
