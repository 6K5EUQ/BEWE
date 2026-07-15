# Central Server — Provisioning & Migration Runbook

> **Purpose.** Reproducible, copy-paste runbook to stand up (or migrate) the
> BEWE Central Server so it behaves **exactly** like the current node. Written
> to be read and executed by an operator *or an AI agent* with shell access.
> For the higher-level overview see [`INSTALL.md`](INSTALL.md) §5.
>
> **Context: this is a single-operator development deployment.** One person runs
> the whole fleet over a private Tailscale mesh. So this manual uses the *real*
> hosts, accounts, and IPs (below) — not placeholders — and is meant to be
> immediately runnable. When a machine is rebuilt its account name may change;
> always prefer `~`/`$HOME` and the Tailscale IP over hardcoded absolute paths.
>
> **Current Central (2026-07-15):** host `raspb2`, user `raspb2`, Tailscale
> `100.123.59.3`, Ubuntu 24.04 LTS aarch64 (Raspberry Pi 5). Host-agnostic — an
> x86_64 server works identically.

## Fleet map & SSH strategy (single source: `~/.claude/skills/bewe-fleet/SKILL.md`)

The `bewe-fleet` skill is the canonical, always-synced reference for identity,
paths, and the safe start/stop idioms — read it before fleet actions. The table
below is the working snapshot as of 2026-07-15.

| Node | Role | Tailscale IP | SSH (`user@ip`) | Build dir | Binary |
|---|---|---|---|---|---|
| **Central** | relay + archive (systemd, always on) | `100.123.59.3` | `raspb2@100.123.59.3` | `~/BEWE/central/build` | `bewe_central` |
| **DGS-1** | HOST (dev+HOST, has venv → AI/guard) | `100.99.120.110` | `ku@100.99.120.110` | `~/BEWE/build-cli` | `BEWE` |
| **DGS-2** | HOST | `100.126.69.82` | `dsa@100.126.69.82` | `~/BEWE/build-cli` | `BEWE` |
| **DGS-3** | HOST | `100.126.161.1` | `raspb1@100.126.161.1` | `~/BEWE/build-cli` | `BEWE` |
| **DGS-7** | HOST (temporary, on `gram` laptop, dev+HOST) | `100.66.204.25` | *account varies — ask; gram is usually the machine you run FROM, not into* | `~/BEWE/build-cli` | `BEWE` |
| desk | dev only (not a base) | `100.82.246.35` | — | — | — |

SSH rules that make this "just work":

- **Identity by Tailscale IP, never hostname** (`tailscale ip -4`). If a target's
  IP equals your own machine's, run locally — never self-SSH (it fails).
- **Key auth, no password, port 22.** Quick check per target:
  `timeout 8 ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new <user@ip> 'echo ok'`.
- **If `Permission denied (publickey)`** the current PC's key isn't on the
  target. Register once (you type the target password once):
  `ssh-copy-id -i ~/.ssh/id_ed25519.pub <user@ip>`.
- **Central needs sudo** only for systemd/reboot; the password lives in
  `~/.claude/secrets.local` as `RASPB2_SUDO=...` (chmod 600, never commit/sync).
  HOST deploy/restart needs no sudo.
- **All git remotes are SSH** (`git@github.com:6K5EUQ/BEWE.git`) with each
  machine's key added as a repo Deploy Key, so `git pull` runs non-interactively
  on every node.

Central's connectivity role: every HOST (DGS-1/2/3/7) and every JOIN client
connects **to Central on `100.123.59.3:7700` over Tailscale**. Moving Central
(§7) means re-pointing that address on each client.

Central relays HOST↔JOIN traffic, writes the permanent mission archive
(including HIST waterfall files), and hosts the cross-station emitter database.
It binds a **single TCP port, 7700**. It is stateless apart from `~/BEWE/DataBase/`
(the archive) — migrating Central = install the binary + service, then move (or
re-point) that directory.

---

## 0. What "works like the current Central" means — checklist

A correctly provisioned Central has all of:

- [ ] `bewe_central` built from `~/BEWE/central/` (branch `main`), linked against **libzstd**.
- [ ] Runs under **systemd** as `bewe-central.service`, `enabled` (autostart) + `Restart=always`.
- [ ] Environment `BEWE_HIST_COMPRESS=1` set (HIST on-disk compression, v13.2.0+).
- [ ] Listening on `0.0.0.0:7700`.
- [ ] Reachable from every HOST/JOIN over Tailscale.
- [ ] Archive at `~/BEWE/DataBase/` with adequate free space (production is ~115 GB and growing).
- [ ] Git remote is SSH (`git@github.com:6K5EUQ/BEWE.git`) with a deploy key, so `git pull` works non-interactively.

Section 8 is a verification script that checks every box.

---

## 1. Prerequisites (packages)

Central needs a **subset** of the full BEWE deps — it has no SDR, no audio, no
GUI. Required:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config git \
  libzstd-dev zlib1g-dev
```

| Package | Why |
|---|---|
| `build-essential`, `cmake`, `pkg-config`, `git` | build toolchain |
| `libzstd-dev` | **required** — Central links libzstd for CHANNEL_SYNC wire compression *and* HIST on-disk compression. Without it the build fails at `pkg_check_modules(ZSTD REQUIRED ...)`. |
| `zlib1g-dev` | `find_package(ZLIB REQUIRED)` in the central CMake |

**Not needed on Central:** `libopus-dev`, `libbladerf-dev`, `librtlsdr-dev`,
`libfftw3-dev`, `libasound2-dev`, `libmpg123-dev`, `libvolk-dev`,
OpenGL/GLFW/ImGui. Central does not decode audio, drive an SDR, or render.
(On the production node `libopus-dev` is intentionally absent — that is fine.)

---

## 2. Get the source

```bash
cd ~
git clone https://github.com/6K5EUQ/BEWE.git      # first time
# or, if already cloned:  cd ~/BEWE && git pull --ff-only
cd ~/BEWE
git checkout main
```

> **Non-interactive `git pull` (important for fleet deploy).** Central pulls
> unattended, so HTTPS with no credential helper fails
> (`could not read Username for 'https://github.com'`). Convert the remote to
> SSH and register a deploy key:
>
> ```bash
> ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519    # if none
> cat ~/.ssh/id_ed25519.pub                            # add as a Deploy Key on the GitHub repo (read-only)
> git -C ~/BEWE remote set-url origin git@github.com:6K5EUQ/BEWE.git
> git -C ~/BEWE ls-remote >/dev/null && echo "pull OK"
> ```

---

## 3. Build

```bash
cd ~/BEWE/central
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"
# artifact: ~/BEWE/central/build/bewe_central
```

Sources compiled (from `central/CMakeLists.txt`): `central_main.cpp`,
`central_server.cpp`, `central_mission_archive.cpp`, `emitter_db.cpp`,
`info_parse.cpp`; includes `../src`. Links `pthread ZLIB::ZLIB ${ZSTD_LIBRARIES}`.

Confirm the zstd link (HIST compression + wire compression depend on it):

```bash
ldd ~/BEWE/central/build/bewe_central | grep -i zstd     # must print libzstd.so.1
```

> **Stale-cache trap.** If cmake errors with a path referencing an old source
> tree (e.g. `.../BE_WE/...`), the build dir has a stale `CMakeCache.txt`.
> Fix by regenerating: `rm -rf ~/BEWE/central/build && mkdir ~/BEWE/central/build && cd $_ && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)"`.

---

## 4. systemd service (autostart + restart)

The unit file is **not** in the repo — it is provisioned per node. Install this
exact unit (paths use the target's own `$HOME` / user — edit `raspb2` if the
account differs):

```bash
USER_NAME="$(whoami)"
REPO="$HOME/BEWE"
sudo tee /etc/systemd/system/bewe-central.service >/dev/null <<EOF
[Unit]
Description=BEWE Central relay (port 7700)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER_NAME}
WorkingDirectory=${REPO}/central/build
ExecStart=${REPO}/central/build/bewe_central
Restart=always
RestartSec=5
StandardOutput=append:/var/log/bewe-central.log
StandardError=append:/var/log/bewe-central.log

[Install]
WantedBy=multi-user.target
EOF
```

Key properties (match production):
- **`Restart=always` / `RestartSec=5`** — Central self-heals on crash.
- **`WorkingDirectory` = the build dir** — Central resolves `~/BEWE/DataBase/`
  from `$HOME`, so the archive lands under the service user's home. Keep the
  service `User=` consistent with the account that owns `~/BEWE`.
- Logs append to **`/var/log/bewe-central.log`** (rotate via logrotate if desired).

---

## 5. Enable HIST on-disk compression (v13.2.0+) — do this before first start

HIST compression is **gated by an environment variable**, off by default. Turn
it on with a drop-in so it persists across reboots and survives unit edits:

```bash
sudo mkdir -p /etc/systemd/system/bewe-central.service.d
printf '[Service]\nEnvironment=BEWE_HIST_COMPRESS=1\n' \
  | sudo tee /etc/systemd/system/bewe-central.service.d/hist-compress.conf
```

Effect: when a HIST segment is finalized (hourly rollover, CF/SR change, mission
end), a **background worker** rewrites it losslessly (per-block zstd + a
frequency-delta transform). The live recording hot path and the host→central
wire flow are unchanged; files still recording stay raw until they close. If the
variable is unset, Central leaves finalized files uncompressed (raw v3) — that is
the safe rollback (delete the drop-in, `daemon-reload`, `restart`).

- Optional: `Environment=BEWE_HIST_ZSTD_LEVEL=N` (default 3; higher = smaller but
  slower — avoid ≥9 on a Pi, it takes minutes per file).
- **Lockstep requirement:** every node that *reads* HIST (the GUI viewers / JOIN
  clients) must run a binary that understands the compressed (v4) format
  **before** Central starts writing it. Old viewers reject v4 files cleanly (they
  won't open), so deploy the fleet's reader binaries first, then flip this flag.

---

## 6. Start & enable

```bash
sudo touch /var/log/bewe-central.log && sudo chown "$(whoami)" /var/log/bewe-central.log
sudo systemctl daemon-reload
sudo systemctl enable --now bewe-central.service
sleep 3
systemctl is-active bewe-central.service          # -> active
tail -5 /var/log/bewe-central.log                 # -> [Central] listening on port 7700
```

---

## 7. Migrating from an existing Central (data move)

The archive (`~/BEWE/DataBase/`) is the only stateful part. To move Central from
the old node (e.g. `raspb2`) to a new one:

1. Provision the new node through §1–§6 but **do not** point HOSTs at it yet.
2. Stop writes on the old node so the copy is consistent:
   `sudo systemctl stop bewe-central.service` (on old).
3. Copy the archive preserving structure (large — production is ~115 GB; run
   over Tailscale). From the **new** node, pulling from the old (raspb2):
   ```bash
   rsync -aH --info=progress2 raspb2@100.123.59.3:~/BEWE/DataBase/ ~/BEWE/DataBase/
   ```
4. Start the new node: `sudo systemctl start bewe-central.service` and verify §8.
5. **Re-point every HOST/JOIN to the new Central's Tailscale IP.** The clients
   are DGS-1 (`ku@100.99.120.110`), DGS-2 (`dsa@100.126.69.82`),
   DGS-3 (`raspb1@100.126.161.1`), DGS-7 (`gram`, `100.66.204.25`), plus any
   operator workstation running the GUI. Update wherever each client stores the
   Central address, then restart that client (`/bewe-restart <base>` idioms live
   in the `bewe-fleet` skill). Confirm each reconnects: its globe marker
   repopulates and the new node's log shows the incoming connection.
6. Add the new node to the fleet map above and to
   `~/.claude/skills/bewe-fleet/SKILL.md` (§2 table) so future ops target the
   right host. Decommission the old node once all stations migrated and archive
   counts match.

> **Add the new machine's SSH key as a repo Deploy Key** and set its git remote
> to `git@...` (see §2) before relying on unattended `git pull` for future deploys.

> HIST files already compressed (v4) copy verbatim and remain readable — no
> recompression needed. Mixed v3/v4 files coexist fine.

---

## 8. Verification (run on the target after §6)

```bash
echo "== service ==";      systemctl is-active bewe-central.service; systemctl is-enabled bewe-central.service
echo "== port ==";         ss -tlnp 2>/dev/null | grep 7700 || echo "NOT LISTENING"
echo "== zstd link ==";    ldd ~/BEWE/central/build/bewe_central | grep -i zstd || echo "MISSING libzstd"
echo "== compress env ==";  PID=$(systemctl show bewe-central.service -p MainPID --value); \
  tr '\0' '\n' </proc/$PID/environ 2>/dev/null | grep BEWE_HIST_COMPRESS || echo "compress OFF"
echo "== archive ==";      du -sh ~/BEWE/DataBase 2>/dev/null; ls -d ~/BEWE/DataBase/missions
echo "== git remote ==";   git -C ~/BEWE remote get-url origin; git -C ~/BEWE branch --show-current
echo "== reachable ==";    tailscale ip -4 2>/dev/null | head -1
```

Expected: `active` + `enabled`; port 7700 LISTEN; libzstd linked;
`BEWE_HIST_COMPRESS=1`; archive present; remote is `git@...` on `main`; a
Tailscale IP prints.

Then from an operator workstation: the 3D globe repopulates with live station
markers, and finalized HIST files begin appearing as v4 (compressed) — confirm
with `grep -a "HIST compress OK" /var/log/bewe-central.log`.

---

## 9. Facts an agent must not get wrong

- **Binary name is `bewe_central`** (built in `central/build/`), *not* `BEWE`.
  `BEWE` / `build-cli/BEWE` is the HOST/JOIN client — a different program.
- **libzstd-dev is mandatory** on Central; libopus-dev is not (Central has no audio).
- **Only port 7700** — no other ports to forward. Mesh VPN (Tailscale) carries it.
- **`~/BEWE/DataBase/` is the archive** and resolves from the service `User=`'s
  `$HOME`. Keep the service user consistent with the repo owner.
- HIST compression is **opt-in via `BEWE_HIST_COMPRESS=1`**; unset = raw v3.
  Deploy reader binaries fleet-wide before enabling it (lockstep).
- Central relays FFT payloads **opaquely** and decodes HIST only to write the
  archive; it never decodes audio.
