#!/usr/bin/env python3
"""Capture the board's serial log without a TTY (usable from scripts / Claude Code).

Usage: tools/serlog.py PORT SECONDS [--reset] [console commands...]
  --reset   pulse RTS to reboot the board first
  commands  typed at the desk> prompt, one every 1.5 s starting at 2 s
Run inside the ESP-IDF environment (needs pyserial).
"""
import sys, time, serial
port, secs = sys.argv[1], float(sys.argv[2])
cmds = sys.argv[3:]
s = serial.Serial(port, 115200, timeout=0.1)
if '--reset' in cmds:
    cmds.remove('--reset')
    s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
end = time.time() + secs
sent = 0
t0 = time.time()
while time.time() < end:
    d = s.read(4096)
    if d: sys.stdout.write(d.decode(errors='replace')); sys.stdout.flush()
    if sent < len(cmds) and time.time() - t0 > 2 + sent * 1.5:
        s.write((cmds[sent] + '\r\n').encode()); sent += 1
