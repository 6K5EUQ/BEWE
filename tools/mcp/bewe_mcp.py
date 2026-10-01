#!/usr/bin/env python3
"""BEWE fleet MCP server (stdio, standard library only).

Lets an AI agent read the BEWE command reference and drive HOST stations:
inject CLI commands into a station's FIFO, read its log, check status,
restart it. Station addresses live outside the repo (public) in
~/.config/bewe-mcp/stations.json (override: BEWE_MCP_STATIONS). See
stations.example.json next to this file.
"""
import base64
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DOC = os.path.join(REPO, "docs", "CLI.md")
CFG = os.environ.get("BEWE_MCP_STATIONS",
                     os.path.expanduser("~/.config/bewe-mcp/stations.json"))
SECRETS = os.path.expanduser("~/.claude/secrets.local")
PROTOCOL = "2024-11-05"
# Periodic stats lines (station every ~3 s, Central per room) — hidden by default.
HEARTBEAT = re.compile(r"\[HOST\] room='[^']*' uptime=|\[Central\] \[STATS\] room=")

# Commands that take a station (or its machine) down, drop the SDR or end a
# mission. They need confirm=true so a guess never runs them.
DANGEROUS = ("/shutdown", "/powercycle", "/rx stop", "/chassis", "/mission end")

INSTRUCTIONS = (
    "BEWE fleet control. Read bewe_guide once before sending commands: it is the "
    "full command reference (units differ: /freq and /ch use MHz/kHz, /sched and "
    "/tle use Hz). Use bewe_stations to see which stations exist and are up, "
    "bewe_cmd to run a CLI command on a station and read its reply, bewe_log for "
    "history. Commands that stop the SDR, the process or the machine require "
    "confirm=true - ask the user first.")


# ── config / shell ─────────────────────────────────────────────────────────
def load_stations():
    try:
        with open(CFG) as f:
            data = json.load(f)
    except FileNotFoundError:
        raise RuntimeError(f"no station config at {CFG} (copy stations.example.json)")
    return {s["name"].upper(): s for s in data.get("stations", [])}


def station(name):
    st = load_stations().get(str(name).upper())
    if not st:
        raise RuntimeError(f"unknown station '{name}'. Known: {', '.join(load_stations())}")
    return st


_self_ip = None
def self_ip():
    global _self_ip
    if _self_ip is None:
        try:
            out = subprocess.run(["tailscale", "ip", "-4"], capture_output=True,
                                 text=True, timeout=5).stdout.split()
            _self_ip = out[0] if out else ""
        except Exception:
            _self_ip = ""
    return _self_ip


def is_local(st):
    ssh = st.get("ssh", "")
    return ssh == "local" or (self_ip() and ssh.split("@")[-1] == self_ip())


def run(st, script, stdin=None, timeout=30):
    """Run a bash script on the station (locally when it is this machine)."""
    if is_local(st):
        cmd = ["bash", "-c", script]
    else:
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", st["ssh"], script]
    try:
        p = subprocess.run(cmd, input=stdin, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, "", f"timeout after {timeout}s"
    return p.returncode, p.stdout, p.stderr


def sudo_password(st):
    """sudo: 'nopass' | 'none' (root login) | 'env:A,B' (first set variable wins,
    from the environment or ~/.claude/secrets.local)."""
    spec = st.get("sudo", "nopass")
    if not spec.startswith("env:"):
        return None
    names = spec[4:].split(",")
    vals = {}
    try:
        with open(SECRETS) as f:
            for line in f:
                m = re.match(r"\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)=(.*)", line)
                if m:
                    vals[m.group(1)] = m.group(2).strip().strip("'\"")
    except FileNotFoundError:
        pass
    for n in names:
        v = os.environ.get(n) or vals.get(n)
        if v:
            return v
    raise RuntimeError(f"sudo password not found ({', '.join(names)}) in env or {SECRETS}")


def q(s):
    return "'" + str(s).replace("'", "'\\''") + "'"


# ── tools ──────────────────────────────────────────────────────────────────
def t_guide(topic=""):
    with open(DOC) as f:
        text = f.read()
    if not topic:
        return text
    # section whose heading mentions the topic, down to the next heading of same level
    lines, out, level = text.splitlines(), [], None
    for ln in lines:
        m = re.match(r"(#+)\s", ln)
        if m and level is not None and len(m.group(1)) <= level:
            break
        if level is None and m and topic.lower() in ln.lower():
            level = len(m.group(1))
        if level is not None:
            out.append(ln)
    return "\n".join(out) if out else f"no section matching '{topic}'. Headings:\n" + \
        "\n".join(l for l in lines if l.startswith("#"))


def t_stations(check=True):
    rows = []
    for name, st in load_stations().items():
        row = {"name": name, "role": st.get("role", "host"), "local": bool(is_local(st))}
        if st.get("note"):
            row["note"] = st["note"]
        if check:
            repo = st.get("repo", "~/BEWE")
            rc, out, err = run(st, f"systemctl is-active {q(st.get('unit', 'bewe-station.service'))}; "
                                   f"git -C {repo} log --oneline -1", timeout=15)
            if rc == 255 or rc == 124:
                row["reachable"] = False
                row["error"] = (err or out).strip()[-200:]
            else:
                parts = out.strip().splitlines()
                row["reachable"] = True
                row["service"] = parts[0] if parts else "?"
                row["commit"] = parts[1] if len(parts) > 1 else "?"
        rows.append(row)
    return json.dumps(rows, indent=1, ensure_ascii=False)


def log_size(st):
    rc, out, err = run(st, f"stat -c %s {q(st['log'])} 2>/dev/null || echo 0", timeout=15)
    if rc in (124, 255):
        raise RuntimeError(f"station unreachable: {(err or out).strip()[-200:]}")
    try:
        return int(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return 0


def strip_heartbeat(text):
    return "\n".join(l for l in text.splitlines() if not HEARTBEAT.search(l))


def t_cmd(name, command, wait=2.0, confirm=False):
    st = station(name)
    if st.get("role") == "central":
        raise RuntimeError("Central has no command FIFO - use bewe_log/bewe_restart")
    command = command.strip()
    if "\n" in command or not command:
        raise RuntimeError("one single-line command")
    if command.startswith(DANGEROUS) and not confirm:
        raise RuntimeError(f"'{command}' stops the SDR, the process or the machine. "
                           "Ask the user, then call again with confirm=true.")
    before = log_size(st)
    b64 = base64.b64encode((command + "\n").encode()).decode()
    rc, out, err = run(st, f"timeout 5 bash -c \"echo {b64} | base64 -d > {st['fifo']}\" "
                           f"&& echo INJECT_OK || echo INJECT_BLOCKED", timeout=20)
    if "INJECT_OK" not in out:
        raise RuntimeError("FIFO write blocked - service not running? " + (err or out).strip())
    time.sleep(max(0.2, min(float(wait), 30.0)))
    rc, out, _ = run(st, f"tail -c +{before + 1} {q(st['log'])} | head -c 60000", timeout=20)
    reply = strip_heartbeat(out).strip()
    return reply or "(no output yet - raise wait or check bewe_log)"


def t_log(name, lines=60, grep="", heartbeat=False):
    st = station(name)
    n = max(1, min(int(lines), 2000))
    cmd = f"tail -n 20000 {q(st['log'])}"
    if not heartbeat:
        cmd += " | grep -avE \"\\[HOST\\] room='[^']*' uptime=|\\[Central\\] \\[STATS\\] room=\""
    if grep:
        cmd += f" | grep -aiE {q(grep)}"
    cmd += f" | tail -n {n}"
    rc, out, err = run(st, cmd, timeout=30)
    return out.strip() or err.strip() or "(empty)"


def t_restart(name, confirm=False, wait=20):
    if not confirm:
        raise RuntimeError("restart interrupts the station. Ask the user, then call with confirm=true.")
    st = station(name)
    unit = st.get("unit", "bewe-central.service" if st.get("role") == "central" else "bewe-station.service")
    pw = sudo_password(st)
    spec = st.get("sudo", "nopass")
    # 재시작 전 로그 끝 — 복귀 판정은 이 뒤에 새로 찍힌 줄만 본다 (옛 프로세스 줄 오인 방지)
    before = log_size(st) if st.get("role") != "central" else 0
    if spec == "none":
        rc, out, err = run(st, f"systemctl restart {q(unit)}", timeout=60)
    elif pw is None:
        rc, out, err = run(st, f"sudo -n systemctl restart {q(unit)}", timeout=60)
    else:
        rc, out, err = run(st, f"sudo -S -p '' systemctl restart {q(unit)}", stdin=pw + "\n", timeout=60)
    if rc != 0:
        raise RuntimeError(f"restart failed rc={rc}: {(err or out).strip()}")
    if st.get("role") == "central":
        time.sleep(3)
        _, out, _ = run(st, f"systemctl is-active {q(unit)}")
        return f"{unit}: {out.strip()}"
    # wait for the new process to reopen its room on Central
    deadline = time.time() + max(5, min(int(wait), 120))
    while time.time() < deadline:
        time.sleep(3)
        _, out, _ = run(st, f"tail -c +{before + 1} {q(st['log'])} | grep -a \"room '.*' opened\" | tail -1",
                        timeout=15)
        if out.strip():
            return f"{unit} restarted; {out.strip()}"
    return f"{unit} restarted, but no 'room opened' within {wait}s - check bewe_log"


def t_status(name):
    st = station(name)
    unit = st.get("unit", "bewe-station.service")
    repo = st.get("repo", "~/BEWE")
    _, out, err = run(st, f"systemctl is-active {q(unit)}; systemctl show {q(unit)} "
                          f"-p ActiveEnterTimestamp --value; git -C {repo} log --oneline -1", timeout=15)
    head = f"service/started/commit:\n{(out or err).strip()}"
    if st.get("role") == "central":
        return head
    return head + "\n\n" + t_cmd(name, "/status", wait=1.5)


TOOLS = [
    {"name": "bewe_guide", "fn": t_guide,
     "description": "BEWE CLI reference (docs/CLI.md): every station command with units, "
                    "recording/schedule/TLE behaviour and recovery tiers. Read before bewe_cmd. "
                    "Optional topic returns one section (e.g. 'Scheduled', 'Channel', 'recovery').",
     "schema": {"type": "object", "properties": {"topic": {"type": "string"}}}},
    {"name": "bewe_stations", "fn": t_stations,
     "description": "List configured stations (HOSTs and Central) with reachability, service state "
                    "and deployed commit.",
     "schema": {"type": "object", "properties": {
         "check": {"type": "boolean", "description": "probe each station (default true)"}}}},
    {"name": "bewe_cmd", "fn": t_cmd,
     "description": "Run one CLI command on a HOST station (writes it to the station FIFO) and "
                    "return the log lines it produced. Examples: '/status', '/ch list', "
                    "'/sched add sat 58400 465000000', '/tle pass 58400'. Commands that "
                    "stop the SDR/process/machine or end a mission need confirm=true.",
     "schema": {"type": "object", "required": ["station", "command"], "properties": {
         "station": {"type": "string", "description": "e.g. DGS-2"},
         "command": {"type": "string"},
         "wait": {"type": "number", "description": "seconds to collect output (default 2, max 30)"},
         "confirm": {"type": "boolean"}}}},
    {"name": "bewe_log", "fn": t_log,
     "description": "Tail a station log (Central included). Heartbeat lines are hidden unless "
                    "heartbeat=true. grep is a case-insensitive extended regex, e.g. 'SCHED|SAT'.",
     "schema": {"type": "object", "required": ["station"], "properties": {
         "station": {"type": "string"},
         "lines": {"type": "integer", "description": "default 60, max 2000"},
         "grep": {"type": "string"},
         "heartbeat": {"type": "boolean"}}}},
    {"name": "bewe_status", "fn": t_status,
     "description": "Service state, start time, deployed commit and the station's own /status.",
     "schema": {"type": "object", "required": ["station"], "properties": {
         "station": {"type": "string"}}}},
    {"name": "bewe_restart", "fn": t_restart,
     "description": "Restart a station's systemd service (state is restored from host_state). "
                    "Requires confirm=true. Waits for the Central room to reopen.",
     "schema": {"type": "object", "required": ["station", "confirm"], "properties": {
         "station": {"type": "string"},
         "confirm": {"type": "boolean"},
         "wait": {"type": "integer", "description": "seconds to wait for room reopen (default 20)"}}}},
]
ARGMAP = {"bewe_cmd": {"station": "name"}, "bewe_log": {"station": "name"},
          "bewe_status": {"station": "name"}, "bewe_restart": {"station": "name"}}


# ── JSON-RPC over stdio ────────────────────────────────────────────────────
def reply(msg_id, result=None, error=None):
    m = {"jsonrpc": "2.0", "id": msg_id}
    if error is not None:
        m["error"] = error
    else:
        m["result"] = result
    sys.stdout.write(json.dumps(m, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def handle(req):
    method, mid, params = req.get("method"), req.get("id"), req.get("params") or {}
    if mid is None:
        return  # notification
    if method == "initialize":
        reply(mid, {"protocolVersion": params.get("protocolVersion", PROTOCOL),
                    "capabilities": {"tools": {}, "resources": {}},
                    "serverInfo": {"name": "bewe", "version": "1.0.0"},
                    "instructions": INSTRUCTIONS})
    elif method == "ping":
        reply(mid, {})
    elif method == "tools/list":
        reply(mid, {"tools": [{"name": t["name"], "description": t["description"],
                               "inputSchema": t["schema"]} for t in TOOLS]})
    elif method == "tools/call":
        name, args = params.get("name"), dict(params.get("arguments") or {})
        tool = next((t for t in TOOLS if t["name"] == name), None)
        if not tool:
            reply(mid, error={"code": -32602, "message": f"unknown tool {name}"})
            return
        for k, v in ARGMAP.get(name, {}).items():
            if k in args:
                args[v] = args.pop(k)
        try:
            text, err = tool["fn"](**args), False
        except Exception as e:  # reported to the agent, not a protocol error
            text, err = f"error: {e}", True
        reply(mid, {"content": [{"type": "text", "text": text}], "isError": err})
    elif method == "resources/list":
        reply(mid, {"resources": [{"uri": "bewe://docs/cli", "name": "BEWE CLI reference",
                                   "mimeType": "text/markdown"}]})
    elif method == "resources/read":
        if params.get("uri") != "bewe://docs/cli":
            reply(mid, error={"code": -32602, "message": "unknown resource"})
            return
        reply(mid, {"contents": [{"uri": "bewe://docs/cli", "mimeType": "text/markdown",
                                  "text": t_guide()}]})
    else:
        reply(mid, error={"code": -32601, "message": f"method not found: {method}"})


def main():
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            handle(json.loads(line))
        except Exception as e:
            sys.stderr.write(f"bewe-mcp: {e}\n")


if __name__ == "__main__":
    main()
