# BEWE: HOST = CLI 전용, JOIN = GUI 전용 분리

> 작성 2026-07-31. 기준 커밋 **`62ac670` (v13.19.0 — DF/KrakenSDR 커밋 완료)**.
> (최초 작성 시 기준은 `6d71b02` v13.18.2 + 미커밋 DF 워킹트리였다. DF 를 먼저
>  커밋해 롤백 지점을 확보했고, 이 문서의 모든 line 번호는 `62ac670` 기준이다.)
> 진행 상태는 문서 끝 "진행 현황" 참조. DF 커밋 후 재검증 결과는 "재검증" 절 참조.

## Context

BEWE 는 지금 **HOST 를 두 가지 방식으로 띄울 수 있다**:

- `build-cli/BEWE` (`-DCLI=ON`, `BEWE_HEADLESS=1`) → `run_cli_host()` (src/cli_host.cpp:674)
- `build/BEWE` (GUI) → `run_streaming_viewer()` (src/ui.cpp:2611) 안의 `mode_sel==1` 경로 (src/ui.cpp:4602-5332)

두 HOST 는 **완전히 별개 구현**이다. `srv->cb.*` 등록(cli_host.cpp:862-1546 vs ui.cpp:4608-5011), Central open_room/mux(cli_host.cpp:1567-1981 vs ui.cpp:5039-5320), `/rx`·`/chassis`, STATUS/HEARTBEAT, SDR 재접속 상태머신, 종료 절차까지 전부 중복이고 **이미 갈라졌다**:

| 항목 | CLI HOST | GUI HOST |
|---|---|---|
| `HostState` 영속화 (cf/sr/gain/채널/DF) | 있음 | **없음** (host_state.cpp 가 CLI 전용) |
| `MissionPush` / `HistCheck` 워커 | 있음 | **없음** |
| `bewe_mod_reconcile` (디코더 자동복구 + 일일 아카이브 push) | 있음 | **없음** |
| `on_start_iq_rec`/`on_db_save`/`on_db_delete`/`on_db_download_req`/`on_mission_list_req` | 등록 | **미등록** |
| `/powercycle` | 동작 | 거부 |
| FFT 정지 워치독 / 4-state `sdr_st` / `broadcast_disk_stat` | 있음 | 없음 |
| CH_SYNC 주기 | 5 Hz | 10 Hz |

GUI-HOST 는 이미 **기능적으로 열등한 두 번째 HOST** 다. 신규 기능마다 두 곳에 반영해야 하고, 어느 쪽이 진짜인지 혼동을 만든다.

**결과 목표**

| 바이너리 | 역할 | 상태 |
|---|---|---|
| `build/BEWE` (GUI) | **JOIN 전용** — Central 경유 원격 클라이언트 | SDR 도 NetServer 도 **링크 단계에서 불가능** |
| `build-cli/BEWE` (CLI) | **HOST 전용** | 현행 + CLI 명령 소폭 추가 + 버그픽스 1건 |
| `central/build/bewe_central` | Central | **무변경** |

LOCAL 모드도 같이 제거한다. LOCAL 은 코드상 `HOST − NetServer − Central` 일 뿐이라(ui.cpp:4534-4728 공유), 남기면 SDR/캡처/복조/녹음 엔진 전체가 GUI 에 계속 링크되어 목적을 달성 못 한다.

## 확정된 결정

1. LOCAL 모드도 제거 — GUI 는 JOIN 만.
2. **빌드 분리까지** — 런타임 UI 숨김이 아니라 GUI 타깃에서 host 소스 링크 해제.
3. CLI 로 포팅: **Notch 필터 / TimeMachine IQ 스냅샷**. HOST 카리별 오디오 녹음은 포팅 안 함(상실 수용).
4. GUI 의 SDR pkg 의존성(libbladeRF/librtlsdr/libiio/libad9361)은 **이번엔 유지** — Phase 5 선택.
5. ~~df_view.cpp 를 GUI 에서 제거~~ → **철회. df_view.cpp 는 GUI 에 남긴다.** 아래 정정 참조.

### 정정 — DF 설정은 이미 원격 동기화돼 있다

계획 수립 중 "DF 설정은 GUI-HOST 로컬 전용" 이라고 잘못 판단했으나, 코드 확인 결과 **이미 완전히 와이어 동기화**돼 있다:

- `df_get_cfg` (kraken_io.cpp:697) — JOIN 은 `net_cli->df_cfg` (HOST 방송 정본) 를 읽는다
- `df_set_cfg` (kraken_io.cpp:713) — JOIN 은 `net_cli->send_df_config(in)` 로 **요청만** 보낸다
- `df_broadcast_cfg` (kraken_io.cpp:732) — HOST 가 `PktDfConfig` 정본을 방송
- 주석 kraken_io.cpp:658-661 이 이 설계를 명시: "JOIN 은 자기 값을 갖지 않는다"

따라서 **DF 설정 패널은 JOIN 의 정상 기능**이고, `/df cfg` CLI 명령은 **불필요** (계획에서 삭제). 대신 `df_join.cpp` 가 DF API 전체의 JOIN arm 을 제공해야 한다.

## 컴파일타임 축

src/config.hpp:7 직후 파생 매크로. **CMake 노브를 새로 만들지 않는다** — 두 마커가 어긋날 수 없다.

```c
#ifdef BEWE_HEADLESS
  #define BEWE_HOST_BUILD 1
#endif
```

기존 `BEWE_HEADLESS` 47개 지점은 **UI 가용성 게이트**로 의미 유지. 새로 넣는 ~12개는 **역할 게이트**. 양쪽 다 컴파일되는 파일에서만 쓴다 — GUI 전용 파일(`ui.cpp`, `*_view.cpp`)에서는 **가드가 아니라 삭제**.

---

# 재검증 — DF 커밋(`62ac670`) 이후 충돌 점검

계획 수립 시점의 워킹트리 DF 를 커밋한 뒤, split 계획과 부딪히는 지점을 코드로 확인한 결과.
**line 번호는 전부 `62ac670` 기준이며 앵커 4개(ui.cpp 2610 `run_streaming_viewer` / 3710 `if(!do_logout)` / 3731 `mode_sel==2 && cli` / 4602 `mode_sel==1`)로 검증했다.**

| # | 발견 | 판정 | 반영 위치 |
|---|---|---|---|
| F1 | `df_view.cpp:12` 가 `net_server.hpp` 를 include 하지만 `net_srv` 를 안 쓴다 (서버 판별은 `:98` `v.net_cli != nullptr`) | **수정 필요** — 헤더 가드를 켜면 GUI 에 남은 유일한 `net_server.hpp` 진입점이 된다 | Phase 3 |
| F2 | `ui.cpp:9178-9196` DF 결과 드레인이 `host_chat_mtx`/`host_chat_log`(§2C 삭제 대상) + `net_srv->broadcast_chat` 사용 | **삭제 아님, 재지정** — JOIN 에서도 `df_post_refusal` 이 `pending_df_result` 를 채우므로 거절 사유의 유일한 표시 경로다 | §2B-3 |
| F3 | `NetClient::ChatMsg`(net_client.hpp:241)에 `is_error` 가 없다 | **필드 추가** — 와이어 아님(net_client.cpp:665 필드별 대입). 없으면 §2B-3 재지정 후 DF 거절·System 메시지의 빨강(ui.cpp:9621)이 죽는다 | §2B-3b |
| F4 | **notch 는 와이어 동기화가 없다** — `net_protocol/net_client/net_server` grep 0건. 순수 뷰어측(생성 ui.cpp:1945-1981 우드래그, 소비 = 스펙트럼 마스킹 + `update_channel_squelch`) | **설계 유효** — split 후 JOIN 은 수신 FFT 에 자기 로컬 notch 를 계속 적용하고, HOST 는 `/notch` + HostState 로 자기 것을 갖는다. 서로 독립이 정상 | 4.1 |
| F5 | `tm_rec_start()` 는 GUI 뷰 상태(`tm_display_fft_idx`/`tm_offset`/`selected_ch`)와 `tm_iq_file_ready` 에 의존 | **4.2 재작성** — 인자 받는 어댑터로 | 4.2 상세 표 |
| F6 | `set_hist_state_fn` 누락 재확인 (cli_host.cpp:2002 `set_state_fn` 만 / ui.cpp:95 에 존재) | 버그 확정 | 4.3 |
| F7 | `df_join.cpp` 가 채워야 할 DF 심볼 = `fft_viewer.hpp:986-1035` 의 **15개**. §0.3 표가 전부 커버함 | **표 유효** | §0.3 |
| F8 | Kraken 진입점(`initialize_kraken` :975, `capture_and_process_kraken` :981, `bewe_spawn_capture` 의 `case HWType::KRAKEN` :1176)은 DF API 가 아니라 **캡처 API** | Phase 3 헤더 가드로 함께 제거 — `df_join.cpp` 대상 아님 | Phase 3 |
| F9 | `src/df/*` 를 include 하는 곳은 `kraken_io.cpp:24-26` 뿐 (`df_view.cpp` 는 `PktDfConfig` 등 plain 타입만 씀) | **GUI 에서 `src/df/*.cpp` 7개 드롭이 안전** — 헤더 전파 없음 | Phase 3 |
| F10 | `--sdr` 화이트리스트에 `kraken` 추가됨 (main.cpp), `initialize_hardware` 에 kraken 선분기 (hw_detect.cpp) | Phase 1 의 `--sdr` 가드가 그대로 커버 | Phase 1 |

## 범위 밖 (보고만)

- **`df::run_selftest` 는 아무도 안 부른다.** `df_selftest.hpp:2` 주석은 "`--df-selftest` 로 돌린다" 고 하는데 그 플래그를 파싱하는 코드가 없다 (`grep selftest` = df/ 밖 0건). 양쪽 빌드에 378줄이 링크만 되고 실행 경로가 없다. 배선하든 빼든 **별도 커밋**.
- **Central 은 여전히 무변경으로 충분하다.** DF 가 넣은 것은 전부 *추가*(`DF_CONFIG=0x5F`, `CmdType 0x24/0x25`, `hw_type` 값 3, `PktHeartbeat` 말미 필드)라 기존 구조체 레이아웃이 안 바뀌고, Central 은 generic 릴레이로 통과시킨다.
  단 **`0x5F` 는 `central_server.cpp:1159` 의 `is_ctrl` 화이트리스트 밖**이라 백프레셔 시 드롭될 수 있다 → DF 설정 방송이 유실되면 다음 방송까지 JOIN 이 stale 값을 본다. 이 split 과 무관한 별건.
- `connect_tier` 가 `s_connect_tier` 에서 한 번도 대입되지 않아 **모든 JOIN 이 Tier 1 로 인증**된다 (ui.cpp:2900 vs 팝업 게이트 :3590-3592).

---

# 실행 순서

## Phase 0 — 축 + 재배치 (동작 변화 0, 양쪽 빌드 green)

| # | 파일 | 작업 |
|---|---|---|
| 0.1 | src/config.hpp:7 | `BEWE_HOST_BUILD` 별칭 추가 |
| 0.2 | src/fft_viewer.hpp:454 | `sched_has_overlap` 를 **클래스 내 인라인 정의**로 승격 (본문은 sched_record.cpp:50-60, 11줄, 순수함). JOIN 이 ui.cpp:8450 에서 무조건 호출하므로 `sched_record.cpp` 드롭의 전제. 원본 삭제 |
| 0.3 | **신규** `src/df_join.cpp` | `kraken_io.cpp` 의 **DF API JOIN arm 전체**를 담는 GUI 전용 구현. 아래 표 참조 |

### 0.3 — `df_join.cpp` 가 정의해야 할 심볼

GUI 에서 `kraken_io.cpp` + `src/df/*` 를 드롭하므로, `fft_viewer.hpp:986-1035` 가 선언한 DF API 를 JOIN 관점으로 전부 재정의한다. df_view.cpp 와 ui.cpp 가 그대로 동작해야 한다.

| 심볼 | JOIN 구현 | 원본 |
|---|---|---|
| `df_get_cfg` | `net_cli->df_cfg` 복사 (없으면 기본값) | kraken_io.cpp:697-711 의 `if(net_cli)` arm |
| `df_set_cfg` | `net_cli->send_df_config(in)` | :714 |
| `df_broadcast_cfg` | no-op (`net_srv` 없음) | :732 |
| `df_snr_threshold` | `df_get_cfg` 경유 (그대로) | :739 |
| `df_link_state` | `net_cli->remote_df_state` 매핑 | :443-455 의 `if(net_cli)` arm |
| `df_request_by_display_num` | 연결·`remote_hw==3`·필터존재 검사 후 `cmd_df_measure(dnum)` | :540-560 |
| `df_post_refusal` / `df_format_line` / `df_short_reason` | 순수 함수 — 그대로 이식 | :488-530 |
| `df_get_live` | 빈 `DFLive{}` (로컬 DAQ 없음 — 현재 JOIN 동작과 동일) | :744 |
| `df_measuring` / `df_engine_ready` | `false` | :648, :438 |
| `df_submit` | `false` | :464 |
| `df_pump` / `df_stop_engine` | no-op | :613, :653 |

## Phase 1 — 공용 파일 가드 (CLI 결과물 불변)

전부 `#ifdef BEWE_HOST_BUILD`. CLI 는 `BEWE_HEADLESS ⇒ BEWE_HOST_BUILD` 이므로 **본문 그대로 포함 = 무변경**.

| 파일 | 지점 | 내용 |
|---|---|---|
| src/main.cpp:38-46 | `--sdr` 파싱 | 가드. `--session-mode` 화이트리스트에서 `"host"` 제거(:49), `--help` 문구 수정 |
| src/timemachine.cpp:226,233 | `net_srv->broadcast_wf_event` | 각 1줄 가드 |
| src/mission.cpp:336 | `MissionPush::scan_mission_dir_enqueue` | 1줄 가드 |
| src/mission.cpp:680-712 | `mission_broadcast_sync` | **본문만** 가드, 심볼은 유지 |
| src/iq_record.cpp:305,481,721 | `MissionPush::enqueue` | 각 1줄 가드 |
| src/module_registry.cpp | `:3` include, `:310-332`/`:335-355`/`:358-370`/`:754-757` 정의 + 호출부 `:715,726,737,766,782,790,797,805` | 정의와 호출부를 **같은 매크로로** 가드. 기존 JOIN early-return 은 그대로 |
| src/modules/dmr/dmr_decode.cpp:74-80 | `net_srv->send_audio` arm | 가드 |
| src/audio.cpp:31,54-68 | 비-remote arm | **가드 아님 — 삭제.** `:13-17` headless early-return 은 손대지 않아 CLI 경로 완전 동일 |

## Phase 2 — `ui.cpp` HOST/LOCAL 철거 (최대 단계, ~2,300줄 제거)

원칙: 모든 지점에서 **`if(net_cli)` arm 만 남기고 `else` / `else if(net_srv)` arm 삭제**.

### 2A. 새 상태머신

```
main.cpp: --session-mode=join 만 허용
 ├─ 인자 없음 → PARENT: 로그인 루프(2820-2846 유지) → globe(JOIN 피커만)
 │     마커 클릭 → POP_JOIN → spawn_session_child("join", …)  [3651 유지]
 │     빈 곳 클릭 → 좌표 표시만 (전 tier)
 │     탈출은 창 닫기뿐 → 3692-3701 return
 └─ --session-mode=join → CHILD: join_room + connect_fd
       성공 → JOIN 셋업 3731-4533 → 메인루프
       실패 → 에러 화면 → join_failed=true → 기존 teardown 으로 낙하
```

설계를 싸게 만든 발견 3가지:
- `do_main_menu` / `do_logout` / `do_chassis_reset` 은 **이미 죽은 경로** — 어디서도 true 가 안 된다. `/reset` 재진입·로그아웃 루프는 실체가 없다.
- LOCAL 버튼(:3449)을 지우면 `mode_done ≡ g_session_args.mode_set` → **3701 이후는 전부 join child 전용**이 증명된다.
- `if(!do_logout){` (:3710) ~ `}` (:12716) 를 `if(!join_failed){` 로 재활용하면 접속 실패 처리가 **플래그 1개 + 기존 teardown** 으로 끝난다. goto·재들여쓰기 불필요.

`mode_sel` 은 **완전 삭제**. `if(mode_sel==2 && cli){` (:3731) → `if(cli){` 로만 바꿔 800줄 JOIN 본문을 재들여쓰기하지 않는다.

### 2B. 삭제/편집 순서 (아래에서 위로, 각 단계 컴파일 가능)

> **line 번호는 지표일 뿐이다. 각 단계는 반드시 인용된 코드 문구로 grep 해서 찾을 것.**
> 실측 결과 일부 번호가 최대 10줄 어긋나 있었다 (아래 표에서 정정). 아래에서 위로 지우는
> 이유가 이것이다 — 하단을 먼저 지우면 아직 손대지 않은 상단 번호가 유효하게 남는다.
>
> 실측 검증된 앵커 (`62ac670`): `4602 if(mode_sel==1){` · **HOST 블록 종료 `5343`** ·
> `5347-5349 status_last/sq_sync_last/heartbeat_last` · **메인루프 `5696 while(!glfwWindowShouldClose(win) && !do_logout && !do_main_menu)`** ·
> `7820 auto ops2=v.net_srv->get_operators()` · `9188-9195 DF 드레인`

| 단계 | 범위 | 작업 |
|---|---|---|
| 1 | `:504-790`, `:1515-1930`, `:6350-6600`, `:6739/6873/6882/6941`, `:7126-7128`, `:7679`, `:7820-7870`, `:8104-8108`, `:8637-8668`, `:9280-9358`, `:12601-12651`, 상단바 `:6954-6968` | 잎사귀 `net_srv` arm 일괄 제거. 밴드바 `:1521-1522/1707-1708/1826-1827`(JOIN 이 `g_cats` 읽음) **유지**, `host_publish_band_plan` 람다 `:1597-1614` 는 호출부 제거 후 삭제 |
| 2 | `:7263-7334` STAT 수신기 선택, `:7607-7654` Operators | `is_host_local` arm / `[LOCAL]` arm / `net_srv->get_operators()` arm 삭제, `net_cli->op_list` arm 을 본문으로 |
| 3 | `:9560-9885` 채팅 + **`:9178-9196` DF 결과 드레인** | 3-arm 전부 `net_cli` 로 붕괴. **`push_local` 을 `net_cli->chat_log` 로 재지정** — `host_chat_log` 가 삭제되므로 필수이며, 동시에 기존 버그(JOIN 시스템 메시지가 안 보임)를 고친다. **DF 드레인도 같은 재지정 대상**: `:9188-9193` 이 `host_chat_mtx`/`host_chat_log` 를, `:9195` 가 `net_srv->broadcast_chat` 을 쓴다. 드레인 자체는 **삭제하면 안 된다** — JOIN 에서도 `df_post_refusal`(kraken_io.cpp:521, df_join.cpp 로 이식)이 `pending_df_result` 를 채우므로 거절 사유가 이 경로로만 사용자에게 보인다. `net_srv` 줄만 제거하고 push 대상을 바꾼다 |
| 3b | src/net_client.hpp:241 | `struct ChatMsg` 에 `bool is_error=false;` 추가. 와이어 구조체가 아니다 (net_client.cpp:665 가 `from`/`msg` 만 필드 대입) — 안전. 없으면 §3 재지정 시 DF 거절·System 메시지의 빨강(ui.cpp:9621)이 죽는다 |
| 4 | `~:5700-6160` | HOST 주기 브로드캐스트(CH_SYNC/FFT_META/STATUS/HEARTBEAT — `sq_sync_last` :5756, `status_last` :5777, DF 상태 하트비트 :5819-5826), `sched_tick`, SDR 런타임 스위치, chassis1/2, `/rx` 대기플래그, 언플러그 재검출 루프(`usb_deep_powercycle`/`usb_reset_vidpid`) 삭제. **메인루프 조건 `:5696`** → `while(!glfwWindowShouldClose(win))` |
| 5 | **원자적**: `:4534-5343` 삭제 → `}` 한 개 + `NetServer* srv`(`:2933`) + `:3731` + `:3714-3729` 재작성 | 하드웨어 초기화·`bewe_spawn_capture`·`LongWaterfall::start_worker`·미션 히스토리 워커·LOCAL 밴드플랜 로드 + `if(mode_sel==1){…}` 전체(서버·전 콜백·IQ 릴레이 청커·Central open_room/mux/재접속·OP_LIST 파서·`net_bcast_worker`) |
| 6 | **원자적**: globe + 상태변수 | LOCAL 버튼 `:3437-3454`, HOST 팝업 `:3528-3586`, `POP_HOST`, 클릭 핸들러 `:3272-3291`, child boot `:3007-3023`, 루프조건 `:3176`, §2C 목록 |
| 7 | 동반 파일 | src/session_spawn.hpp:26-27 + src/session_spawn.cpp:79-92 `reap_finished_children` 에서 `pid_t&` 파라미터 제거 |
| 8 | `:125-529` `update_channel_squelch`, `:534-599` `set_channel_detect` | **스텁으로 축소** (헤더가 선언하고 GUI 유일 정의라 심볼은 필요). CLI 사본(cli_host.cpp:56/429)은 손대지 않는다 |
| 9 | 헤더 include `:7,14,17` | `net_server.hpp` / `host_band_plan.hpp` / `hist_check.hpp` 제거 |

### 2C. 댕글링 정리 (§2B-6 과 함께)

`srv`(2933) · `active_host_pid`(3000) · `chassis_reset_mode`/`do_chassis_reset`(2854-2855) · `pending_chassis1/2_reset`·`pending_rx_stop/start`(2859-2862) · `usb_reset_pending`(2858) · `s_station_name/lat/lon/s_station_set`(2902-2904) · `s_relay_op_list/mtx`(2913-2914, write-only) · `s_host_port`/`host_port` · `s_connect_host/port`(이미 dead) · `mode_err_msg/timer`(이미 dead) · `host_chat_log`/`host_chat_mtx`/`LocalChatMsg`(2864-2866) · `cap` 스레드(2891) · `bewe_mod_set_broadcast` 람다(2889-2890, `module_registry` 가 `g_broadcast` null-guard 하므로 미설정 안전) · `register_host_state_fn`(71-104) · `heartbeat_last`/`status_last`/`sq_sync_last`(**5347-5349**) · `pending_lat/lon`/`new_station_name` · `scan_available_sdrs` 호출부.

**부수 효과**: `v.net_srv`/`srv`/`central_cli` 를 포인터 캡처하던 **detached 스레드 5개**(Central 자동재접속 5289-5314, IQ 릴레이 5754/5834, chassis-2 5896-5929, USB reset 6074-6100, chassis-2 채팅 9738-9765) 가 통째로 사라져 수명 위험이 제거된다.

## Phase 3 — CMake 절단 + 컴파일타임 증명

### GUI 타깃에서 제거 (CMakeLists.txt:136-246)

`net_server.cpp` · `net_stream.cpp` · `hw_detect.cpp` · `bladerf_io.cpp` · `rtlsdr_io.cpp` · `pluto_io.cpp` · `kraken_io.cpp` · `src/df/*.cpp` (7개) · `demod.cpp` · `sched_record.cpp` · `hist_check.cpp` · `mission_push.cpp` · `host_band_plan.cpp` · `region_save.cpp`
**추가**: `src/df_join.cpp`
**유지**: `df_view.cpp` (정정 참조) — 단 **`df_view.cpp:12` 의 `#include "net_server.hpp"` 는 제거**한다. 파일 안에서 `net_srv` 를 한 번도 안 쓴다(유일한 서버 판별은 `:98` 의 `v.net_cli != nullptr`). 아래 헤더 가드를 켜는 순간 이 include 가 유일하게 남은 GUI 측 `net_server.hpp` 진입점이 된다

### GUI 에 남기는 것 (근거)

| 파일 | 이유 |
|---|---|
| `df_view.cpp` | DF 설정은 HOST 소유·와이어 동기화 — JOIN 이 원격 편집하는 정상 기능 |
| `host_band_categories.cpp` | **JOIN 이 `g_cats` 에 미러링**(ui.cpp:3940-3949)하고 밴드바 렌더가 `g_mtx`/`g_cats` 를 무조건 읽는다. 자립적(host 의존 없음) |
| `long_waterfall.cpp` | `long_waterfall_view.cpp` 가 `FileHeader`/`byte_to_db`/`v4ext`/`current_file_path`/`FILE_VERSION*` 사용. writer 워커만 시작 안 함 |
| `iq_record.cpp` | `start/stop_join_audio_rec` + `write_default_info_file`(JOIN region `.info` 생성, ui.cpp:4075/4272) |
| `mission.cpp` · `audio*.cpp` · `timemachine.cpp` · `module_registry.cpp` · `net_client.cpp` · `central_client.cpp` · `login.cpp` · `session_spawn.cpp` · `sa_compute.cpp` · `eid_compute.cpp` · `fft_viewer.cpp` | JOIN 필수 |

### 헤더 가드 (증명 장치)

src/fft_viewer.hpp:4 `#include "net_server.hpp"` 와 :690 `NetServer* net_srv` 를 `#ifdef BEWE_HOST_BUILD` 로 감싼다.

- 안전: `net_server.hpp` 가 주는 `net_protocol.hpp`/`channel.hpp`/`config.hpp` 는 바로 다음 줄 `net_client.hpp` 가 동일하게 공급. 헤더에 비-constexpr static 없음
- **순서 중요**: Phase 2 를 grep 기반으로 먼저 끝낸 뒤 **마지막에** 이 가드를 켠다. 먼저 켜면 12,000줄 파일에 150개 캐스케이딩 에러가 쏟아진다. 켠 뒤 남는 에러 = 놓친 host 호출부 목록 (= 컴파일러가 증명해 준다)
- 같이 가드: `bewe_spawn_capture`(:1171-1186 — `case HWType::KRAKEN` 포함), `initialize_*` + `capture_and_process_*` 선언 블록(:970-981 — `initialize_kraken`/`capture_and_process_kraken` 포함), USB reset 선언, `g_sdr_force`/`scan_*`(:43-45)
- **DF API 선언(:986-1035)은 가드하지 않는다** — GUI 는 `df_join.cpp` 로 같은 심볼을 채운다 (§0.3)

## Phase 4 — CLI 포팅 + 버그픽스

cli_host.cpp stdin 디스패처(`:2952` `/ch` 패턴 재사용) + `/help`(`:3053`) 갱신.

| # | 항목 | 구현 |
|---|---|---|
| 4.1 | `/notch add <lo_MHz> <hi_MHz>` / `/notch list` / `/notch del <n>` | `v.notches` + `notches_mtx`(fft_viewer.hpp:137-143) 직접 조작. 소비측은 이미 headless 에 있다 — `update_channel_squelch`(cli_host.cpp:87-94), detect 빈 스킵(:225-240). **`HostState` 에 필드 추가**(host_state.cpp `fingerprint`/`save`/`apply_*`)해 재시작 후 보존. ~~착수 전 와이어 동기화 재확인~~ → **확인 완료: notch 는 와이어에 없다**(재검증 F4). 설계 그대로 진행 |
| 4.2 | `/tm save <ch> [sec_ago]` | ~~`v.tm_rec_start()` 호출만~~ → **확인 결과 단독 호출로는 항상 false 를 리턴한다**(재검증 F5). 아래 선행 세팅 필요 |
| 4.3 | **버그픽스** `set_hist_state_fn` | cli_host 는 `set_state_fn`(:2002)만 걸고 `set_hist_state_fn`(central_client.hpp:265)을 안 건다. GUI-HOST 만 걸고 있었다(ui.cpp:95-106). 안 옮기면 **Central 상태페이지에서 전 기지의 live-HIST 정보가 사라진다.** ui.cpp 코드 그대로 이식 (~10줄). **확인 완료** (재검증 F6) |

### 4.2 상세 — `tm_rec_start()` 의 실제 전제조건

`timemachine.cpp:322-338` 이 요구하는 것 (하나라도 어긋나면 조용히 `false`):

| 전제 | CLI 현재 상태 | `/tm save` 가 해야 할 일 |
|---|---|---|
| `tm_iq_file_ready && tm_iq_fd>=0` | **기본 OFF** (cli_host.cpp:2037 — 원격 토글 시 lazy open) | 꺼져 있으면 "TM IQ is off" 로 거절. 자동으로 켜지 말 것 (SSD 롤링 쓰기가 시작된다) |
| `iq_row_avail[tm_display_fft_idx % MAX_FFTS_MEMORY]` | `tm_display_fft_idx` 는 GUI 스페이스바(`tm_update_display`)가 세팅. CLI 는 **0 고정** | `tm_freeze_idx = 현재 fft 인덱스` 로 놓고 `sec_ago` → `tm_offset` → `tm_display_fft_idx` 를 직접 계산 |
| `selected_ch >= 0 && channels[fi].filter_active` | CLI 에 "선택 채널" 개념 없음 | `<ch>` 인자(표시번호)를 슬롯 인덱스로 변환해 `selected_ch` 에 대입 |
| `start_rec()` 의 목적지 | 미션 IDLE 이면 녹음 경로가 빈 문자열 | 미션 ACTIVE 아니면 거절 메시지 |

즉 `/tm save` 는 얇은 래퍼가 아니라 **GUI 의 뷰 상태를 인자로 대체하는 어댑터**다. `sec_ago` 생략 시 0(라이브 최신).

## Phase 5 — 선택 (별도 커밋)

- **의존성 슬림화**: `libiio`/`libad9361`/`volk` 는 **헤더 편집 0** 으로 GUI 에서 제거 가능(전부 드롭되는 .cpp 에서만 include). `libbladeRF`/`librtlsdr` 는 fft_viewer.hpp:18-19 + :851-852 가드 필요. CMake 의 `pkg_check_modules(… REQUIRED)`(CMakeLists.txt:41,46,48,49,50)를 `if(CLI)` 안으로 이동 → JOIN PC 에 SDR -dev 패키지 불필요
- `remote_mode` 항상 true 로 죽는 arm 정리 (ui.cpp:8862-8901, 8986-8994, 6718, FFTViewer 메서드 463/684/695/702/736/744) — 순수 dead code
- `iq_record.cpp:244-723`(host 레코더) 가드

---

# 상실 수용 / 사용자 가시 변화

| 항목 | 판정 |
|---|---|
| HOST 카리별 오디오 녹음 (ui.cpp:6460 R 키) | **상실** (결정). 패킷 없음. JOIN 은 자기 PC 로컬 녹음으로 계속 가능 |
| HOST 운용자의 로컬 region IQ 저장 (ui.cpp:6453) | 대체됨 — JOIN 이 `REQUEST_REGION` 요청하면 cli_host 가 이미 서빙(:1034) |
| globe 의 LOCAL 버튼 / HOST 배치 팝업 | 제거. 빈 곳 클릭은 전 tier 좌표 표시로 통일 |
| `/powercycle` **동작 변화(개선)** | 지금은 상대가 GUI-HOST 면 `"station-only"` 로 튕겼다. 이제 모든 상대가 cli_host 라 **실제로 재시작/재부팅된다.** 운용 시 주의 |
| 채팅 시스템 메시지 | 이제 JOIN 채팅창에 보인다 (`push_local` 재지정 부수효과) |
| R 키 | region 미선택 + 복조 채널 미선택 시 무동작 (LOCAL IQ 녹음 fallback 소멸) |

**보고만 하고 고치지 않는 기존 버그** (범위 밖): `connect_tier` 가 `s_connect_tier`(초기값 1)에서 한 번도 대입되지 않아 **모든 JOIN 이 Tier 1 로 인증**된다(ui.cpp:2900 vs 팝업 게이트 :3590-3592).

---

# 검증

```bash
# 0. 구조 변경이므로 캐시 재사용 금지
rm -rf /home/ku/BEWE/build /home/ku/BEWE/build-cli

# 1. GUI (JOIN 전용)
cmake -S /home/ku/BEWE -B /home/ku/BEWE/build && cmake --build /home/ku/BEWE/build -j$(nproc)

# 2. CLI (HOST) — 동작 불변이어야 함
cmake -S /home/ku/BEWE -B /home/ku/BEWE/build-cli -DCLI=ON && cmake --build /home/ku/BEWE/build-cli -j$(nproc)
grep '^CLI:BOOL' /home/ku/BEWE/build-cli/CMakeCache.txt      # ON

# 3. Central (net_protocol.hpp 무변경 — 형식 확인)
cmake -S /home/ku/BEWE/central -B /home/ku/BEWE/central/build && cmake --build /home/ku/BEWE/central/build -j$(nproc)

# 4. 네거티브 증명 (GUI 바이너리)
nm -C build/BEWE | grep -c 'NetServer::'                                   # 0  (현재 110)
nm -C build/BEWE | grep -cE 'FFTViewer::(capture_and_process|initialize|start_dem|region_save|sched_tick)'   # 0
nm -C build/BEWE | grep -c 'df::'                                          # 0  (df/ 엔진 전체 드롭)
nm -C build/BEWE | grep -ci 'heimdall'                                     # 0
# 반대로 df_join.cpp 가 채운 JOIN arm 은 살아 있어야 한다
nm -C build/BEWE | grep -cE 'FFTViewer::df_(get_cfg|set_cfg|link_state|request_by_display_num)'  # 4

# 5. 모듈 제거 가능성 (CLAUDE.md 검증 의무)
mv src/modules /tmp/bewe-modules-stash
rm -rf build build-cli
cmake -S . -B build && cmake --build build -j$(nproc)
cmake -S . -B build-cli -DCLI=ON && cmake --build build-cli -j$(nproc)
strings build/BEWE     | grep -icE 'acars|adsb|btle|dmr|stt|wifi'   # 0
strings build-cli/BEWE | grep -icE 'acars|adsb|btle|dmr|stt|wifi'   # 0
mv /tmp/bewe-modules-stash src/modules
```

**런타임 스모크**
1. CLI HOST (DGS-1 로컬) 기동 → `/status` · `/ch add 125 10 am` · `/notch add 124.9 125.1` · `/notch list` · `/tm save`
2. 재시작 후 notch 가 `host_state_DGS-1.json` 에서 복원되는지
3. GUI 기동 → 로그인 → globe 에 **LOCAL 버튼과 HOST 팝업이 없는지** → 마커 클릭 JOIN
4. JOIN 에서 워터폴·CH_SYNC·Opus 오디오·밴드플랜 편집·HIST(LWF) 목록/다운로드·MSN·SA/EID·**DF 설정 패널 편집이 HOST 에 반영되는지**·`/df N` LED
5. 존재하지 않는 기지로 JOIN 시도 → 에러 화면 후 정상 종료 (LOCAL 로 빠지지 않음)
6. Central 상태페이지에서 해당 기지의 live-HIST 정보가 보이는지 (Phase 4.3)

---

# 버전 / 커밋

- 마지막 커밋 `62ac670` = `v13.19.0` (DF). **커밋 직전 `git log --oneline -1` 재확인 필수** (CLAUDE.md 버전 규칙 — 워킹트리 값을 믿지 말 것).
- **minor → `v13.20.0`.** 와이어 포맷 무변경이라 신규 JOIN GUI 는 기존 CLI HOST 와 그대로 붙는다. major 아님.

```
Rebuild: HOST + JOIN
Commit: BEWE root
Split roles: HOST is CLI-only, GUI is JOIN-only (v13.20.0)
```

문서 갱신 대상: INSTALL.md (`:17`, `:99`, `:162`, `:212`), CENTRAL_SETUP.md (`:30-33`, `:307`) — GUI 로 HOST 를 띄우는 서술이 있으면 수정.

---

# 진행 현황

- [x] **DF/KrakenSDR 선행 커밋** `62ac670` (v13.19.0) — split 의 롤백 지점. AMC python 은 `5ca5d3b` 로 분리
- [x] **DF 커밋 이후 재검증** (F1~F10) — "재검증" 절
- [x] 0.1 `src/config.hpp:13-15` — `BEWE_HOST_BUILD` 별칭 추가
- [x] 0.2 `src/fft_viewer.hpp:456` — `sched_has_overlap` 헤더 인라인화 + `sched_record.cpp` 원본 삭제 (둘 다 완료)
- [x] 0.3 `src/df_join.cpp` 신규 (심볼 15개)
- [x] Phase 1 공용 파일 가드 (main/timemachine/mission/iq_record/module_registry/dmr/audio)
- [x] Phase 2 `ui.cpp` 철거 — 12,797 → 10,534 줄 (**2,263 줄 제거**), `net_srv` 참조 0
- [x] Phase 3 CMake 절단 + 헤더 가드 — GUI 에서 SDR·NetServer·복조·DF 엔진 소스 14개 드롭,
      `fft_viewer.hpp` 의 `net_server.hpp` include 와 `net_srv` 멤버를 `BEWE_HOST_BUILD` 로 가드
- [x] Phase 4.3 `set_hist_state_fn` 버그픽스 (cli_host 로 이식, GUI 사본 삭제)
- [ ] **Phase 4.1 `/notch` CLI (미완)**
- [ ] **Phase 4.2 `/tm save` CLI (미완 — 4.2 상세 표의 어댑터로 구현할 것)**
- [ ] Phase 5 선택 (별도 커밋)

## 검증 결과 (`v13.20.0` 시점)

| 항목 | 결과 |
|---|---|
| GUI / CLI / Central 클린 빌드 | 3/3 green (`rm -rf` 후 재생성) |
| `nm build/BEWE \| grep -c 'NetServer::'` | **0** (기존 110) |
| `capture_and_process\|initialize\|start_dem\|region_save\|sched_tick` | **0** |
| `df::` / `heimdall` | **0** / **0** |
| `df_join.cpp` JOIN arm 심볼 | **4/4** 존재 |
| 모듈 폴더 제거 후 양쪽 빌드 | green (문자열 히트는 밴드플랜 라벨 `WiFi/BT` 와 ImGui/SGP4 심볼 오탐뿐) |
