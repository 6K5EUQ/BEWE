# BEWE CLI Reference

Everything a HOST station (headless `cli_host`) accepts, and how to reach it.
Written for operators and for AI agents driving the fleet through the MCP server
in `tools/mcp/`. Station addresses and credentials are **not** in this repo —
they live in each workstation's `~/.config/bewe-mcp/stations.json`.

## 1. How a station runs

| Item | Value |
|---|---|
| Binary | `~/BEWE/build-cli/BEWE` (CLI build, `-DCLI=ON`) |
| Service | systemd `bewe-station.service` (a drone station may use its own unit name) |
| Command input | FIFO `/run/bewe-<station-lowercase>.cmd` (e.g. `/run/bewe-dgs-2.cmd`) |
| Log | `/var/log/bewe-<station-lowercase>.log` |
| State | `~/BEWE/host_state_<STATION>.json` — CF/SR/gain/channels/DF/notches, restored on restart |
| Satellite rules | `~/BEWE/sat_sched_<STATION>.txt` |

Writing one line into the FIFO runs it immediately — no restart:

```bash
timeout 5 bash -c "echo '/status' > /run/bewe-dgs-2.cmd"
tail -n 20 /var/log/bewe-dgs-2.log
```

If the write blocks, nothing is reading the FIFO: the service is down. Check
`systemctl is-active bewe-station.service`. Old `/tmp/bewe-*.cmd` FIFOs are dead —
writes there vanish silently.

Log noise: a `[HOST] room='...' uptime=...` heartbeat line appears every ~3 s.
`room='<ST>_<ST>' opened` after a restart means the station is back on Central.

Any line not starting with `/` is broadcast as a chat message.

## 2. Units

| Command family | Frequency | Bandwidth / rate | Time |
|---|---|---|---|
| `/freq`, `/ch`, `/notch` | MHz | kHz (`/ch`), MHz (`/notch`) | — |
| `/sr` | — | MSPS | — |
| `/sched`, `/tle` | **Hz** | **Hz** (SR) | KST, seconds |

## 3. Commands

### Status

| Command | Effect |
|---|---|
| `/status` | CF, SR, gain, clients, `SDR=OK/ERROR/STOPPED`, rolling IQ, CPU/RAM, net counters |
| `/clients` | Connected operators |
| `/help` | Built-in list |

### Tuning

| Command | Effect |
|---|---|
| `/freq <MHz>` | Center frequency (0.1–6000) |
| `/sr <MSPS>` | Sample rate (0.1–61.44; RTL-SDR tops out at 3.2) |
| `/autoscale` | Re-fit the waterfall dB window to the current noise floor |
| `/gain [dB]` | KrakenSDR only — tuner gain |
| `/mrc [on\|off]` | KrakenSDR only — combine the 5 channels into the IQ path |

### Channel filters

| Command | Effect |
|---|---|
| `/ch add <CF_MHz> <BW_kHz> [none\|am\|fm]` | New filter in the first free slot. `none` = raw filter for decoder modules. Outside the captured band it is created in *Holding* and resumes when the CF brings it back |
| `/ch list` | Active filters (cf/bw/mode/owner, Holding) |
| `/ch del <n>` | Delete filter n |
| `/notch add <lo_MHz> <hi_MHz>` / `list` / `del <n>` | Mask a band from display and squelch. Persisted |
| `/df <n>` | Direction-find the filter drawn with number n (KrakenSDR) |

### Missions and recordings

| Command | Effect |
|---|---|
| `/mission start` / `end` / `status` | Mission lifecycle. Manual IQ/audio recordings require an ACTIVE mission |
| `/tm save <ch> [sec_ago]` | Cut a channel out of the 60 s rolling IQ buffer (rolling must be on, mission ACTIVE) |
| `/hist check` | Compare retained local `.bewehist` against Central, upload missing rows, delete proven copies. Never runs automatically |

Recording locations:

- Mission ACTIVE: `recordings/missions/<year>/<code>/{iq,audio,hist}/`, pushed to the Central mission archive (the local copy is removed once Central acknowledges it).
- No mission: `recordings/record/{iq,audio}/` (scheduled recordings only).

### Scheduled recording

Scheduled recordings switch the SDR to the requested CF and SR 5 s before the
start, record the **whole band** at that SR (no decimation, file SR = SR), then
restore the previous CF and SR. The whole station is affected while it runs:
waterfall, other channels and HIST follow the retune.

| Command | Effect |
|---|---|
| `/sched add <CF_Hz> <SR_Hz> <YYYY-MM-DD> <HH:MM:SS> <dur_s>` | One-off recording, start time in KST |
| `/sched add sat <NORAD> <CF_Hz> <SR_Hz>` | Standing rule: record **every** pass of that satellite, AOS to LOS at 0 deg elevation. Passes for the next 24 h are expanded into entries and refreshed every 10 min. Persisted across restarts |
| `/sched list` | Rules, then entries `[n] WAIT/ARMED/REC/DONE/FAIL` |
| `/sched del <n>` | Remove entry n (not the one in progress) |
| `/sched del sat <NORAD>` | Remove the rule and its waiting entries |

- Files: `SCHED_IQ_<station>_<code>_<date>_<HHMMSS>-<HHMMSS>_<F.F>MHz.sigmf-data` + `.sigmf-meta`, in the mission `iq/` folder if a mission is ACTIVE, otherwise `record/iq/`. Uploaded to the Central DB when finished.
- Overlapping entries are refused (the pre-arm 5 s counts). Two satellites passing at once: the later one is skipped and logged.
- Max 32 entries per station. Finished satellite entries are dropped an hour after they end.
- Log tags: `[SCHED]` (arm/start/stop/restore), `[SAT]` (pass expansion), `[SCHED-DB]` (upload).

Example:

```
/tle update
/tle pass 58400
/sched add sat 58400 465000000 1000000
/sched list
```

### Orbital elements

| Command | Effect |
|---|---|
| `/tle update` | Fetch today's and yesterday's `leo_`/`all_YYYYMMDD.txt` from Central into `assets/tle/archive/` |
| `/tle status` | Newest local `leo_` and `all_` snapshot |
| `/tle pass <NORAD>` | Name, element age, passes over this station for the next 24 h (KST) |

Lookup order is `leo_` (LEO payloads, no Starlink) then `all_`. Central collects
snapshots daily with `scripts/central-tle-sync.sh`. Elements older than about a
week drift noticeably; refresh before relying on pass times.

### SDR and recovery

Lowest tier first — each step up loses more.

| Command | Effect |
|---|---|
| `/rx stop` | Release the SDR (safe to unplug, station stays online). KrakenSDR: also stops the DAQ |
| `/rx start` | Re-detect the SDR and restore CF/SR/gain |
| `/chassis 1 reset` | USB re-enumerate + SDR reinit. KrakenSDR: restart the heimdall DAQ |
| `/chassis 2 reset` | Network broadcast reset |
| `/powercycle partial` | Above + restart BEWE (keeps the machine) |
| `/powercycle full` | Above + **reboot the machine** |
| `/shutdown` | Clean exit (systemd restarts only on failure) |

## 4. Commands also accepted as chat

JOIN and HOST UI chat can send `/mission ...`, `/hist check`, `/autoscale` and
`/powercycle partial|full`; the station that receives the chat executes them on
itself. Everything else is FIFO (stdin) only.

## 5. Service operations (outside the FIFO)

| Task | Command on the station |
|---|---|
| Restart | `sudo systemctl restart bewe-station.service` |
| Update + rebuild | `cd ~/BEWE && git pull --ff-only && cmake --build build-cli -j$(nproc)` |
| Central restart | `systemctl restart bewe-central.service` |
| Central build | `cmake --build ~/BEWE/central/build -j$(nproc)` |

- A rebuilt binary takes effect only after a restart.
- Wire-format changes: restart Central first, then every HOST.
- A `status=11/SEGV` on stop during restart is a known shutdown issue, not a failed deploy.
