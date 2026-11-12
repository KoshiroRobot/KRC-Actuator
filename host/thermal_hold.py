#!/usr/bin/env python3
import math
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.expanduser("~"))
import krcbus

T_WIND_MAX = 85.0
T_BOARD_MAX = 60.0
ALPHA = 0.00393
HALF = 2.0
HI = 1.15
COOL = (800, 2400)
R_GUESS = 1.71
V_MAX_MV = 12500
OUT_DIR = os.path.expanduser("~/thermal")
LOG_HEAD = "t,phase,v_cmd_mv,i_ma,ia_ma,ib_ma,ic_ma,id_ma,iq_ma,board_c,therm_pct,trip,R_ohm,Tw_c\n"


def usage():
    return "使い方: python3 thermal_hold.py <node> <電流 A> [保つ秒=1500] [冷ます秒=600]   （node は 1〜%d・電流は 0.5〜4.0 A）" % krcbus.NODE_MAX


def parse_args(argv):
    a = list(argv[1:])
    try:
        I = float(a[1]) if len(a) > 1 else None
        hold_s = float(a[2]) if len(a) > 2 else 1500.0
        cool_s = float(a[3]) if len(a) > 3 else 600.0
    except ValueError:
        I = None
    if (len(a) < 2 or len(a) > 4 or not krcbus.valid_node(a[0]) or I is None
            or not all(math.isfinite(x) for x in (I, hold_s, cool_s)) or not (hold_s > 0 and cool_s >= 0)):
        print(usage())
        raise SystemExit(1)
    if not (0.5 <= I <= 4.0):
        print("電流は 0.5〜4.0 A")
        raise SystemExit(1)
    return int(a[0]), I, hold_s, cool_s


def main(argv):
    node, I, hold_s, cool_s = parse_args(argv)
    i_set = (I * HI, I * (2.0 - HI * HI) ** 0.5)
    print("軸 %d を整列モードで %.2f A（%.2f / %.2f A を 2 秒ごと）に保って温めます（保つ %.0f 秒・冷ます %.0f 秒）"
          % (node, I, i_set[0], i_set[1], hold_s, cool_s), flush=True)
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind(("can0",))
    s.setblocking(False)

    def send(cid, data):
        s.send(struct.pack("=IB3x8s", cid, len(data), bytes(data).ljust(8, b"\x00")))

    def cfg(sub, p1=0, p2=0):
        p1 &= 0xFFFF
        p2 &= 0xFFFF
        send(0x120 + node, [0xA5, sub, p1 >> 8, p1 & 0xFF, p2 >> 8, p2 & 0xFF, 0, 0])

    def s16(d, i):
        return struct.unpack(">h", bytes(d[i:i + 2]))[0]

    def board_temp(d):
        if d is None or len(d) < 6:
            return None
        raw = struct.unpack(">h", bytes(d[4:6]))[0]
        return None if raw == krcbus.TEMP_INVALID_RAW else raw / 10.0

    tlm, tlm_t = {}, {}

    def drain():
        while True:
            try:
                fr = s.recv(16)
            except BlockingIOError:
                return
            cid, dlc, data = struct.unpack("=IB3x8s", fr)
            cid &= 0x1FFFFFFF
            base = 0x300 + node * 0x10
            if base <= cid < base + 0x10:
                tlm[cid - base] = data[:dlc]
                tlm_t[cid - base] = time.time()

    time.sleep(0.3)
    drain()
    T0 = board_temp(tlm.get(0))
    if T0 is None:
        print("基板温度が読めないので流しません")
        return 1

    def stop():
        for _ in range(3):
            cfg(0)
            time.sleep(0.05)

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, time.strftime("hold_n%d_%%Y%%m%%d_%%H%%M%%S.csv" % node))
    f = open(path, "w", buffering=1)
    f.write(LOG_HEAD)
    print("記録:", path, "  始めの基板温度 %.1f ℃" % T0, flush=True)

    t0 = time.time()
    phase, t_phase = "heat", t0
    v = [i_set[0] * R_GUESS * 1000.0, i_set[1] * R_GUESS * 1000.0]
    lvl, t_half = 0, t0
    half_buf, last_half = [], None
    R0 = R = Tw = None
    nh, i0s, rise = [0, 0], [[], []], [None, None]
    bc, t_bc = T0, tlm_t[0]
    reason, rc = "終わり", 0
    nxt = 0.0
    try:
        while True:
            now = time.time()
            drain()
            if tlm_t.get(0, 0.0) > t_bc:
                t_bc = tlm_t[0]
                b_ = board_temp(tlm[0])
                if b_ is None:
                    reason, rc = "基板温度が読めなくなった", 1
                    break
                bc = b_
                if bc >= T_BOARD_MAX:
                    reason = "基板が %.1f ℃" % bc
                    break
            vcmd = int(min(V_MAX_MV, v[lvl])) if phase == "heat" else COOL[lvl]
            if now >= nxt:
                cfg(1, vcmd, 0)
                nxt = now + 0.1
            if now - t_half >= HALF:
                if half_buf:
                    vd = sum(x[0] for x in half_buf) / len(half_buf)
                    im = sum(x[1] for x in half_buf) / len(half_buf)
                    if phase == "heat" and im > 100:
                        nh[lvl] += 1
                        if 2 <= nh[lvl] <= 4:
                            i0s[lvl].append(im)
                        elif nh[lvl] > 4:
                            I0 = sum(i0s[lvl]) / len(i0s[lvl])
                            rise[lvl] = (I0 / im - 1.0) / ALPHA
                            if rise[0] is not None and rise[1] is not None:
                                Tw = T0 + (rise[0] + rise[1]) / 2.0
                                R = R_GUESS * (1.0 + ALPHA * (Tw - T0))
                    last_half = (vd, im, phase)
                half_buf = []
                lvl ^= 1
                t_half = now
            if 3 in tlm and 5 in tlm and now - tlm_t.get(3, 0) < 0.15:
                ia, ib, ic = s16(tlm[3], 0), s16(tlm[3], 2), s16(tlm[3], 4)
                imag = ((2.0 / 3.0) * (ia * ia + ib * ib + ic * ic)) ** 0.5
                iq, idd = s16(tlm[5], 4), s16(tlm[5], 6)
                th = tlm[4][0] if 4 in tlm else -1
                tr = tlm[1][7] if 1 in tlm else -1
                if now - t_half >= HALF - 1.0:
                    half_buf.append((vcmd, imag))
                f.write("%.2f,%s,%d,%.0f,%d,%d,%d,%d,%d,%.1f,%d,%d,%s,%s\n" % (
                    now - t0, phase, vcmd, imag, ia, ib, ic, idd, iq, bc, th, tr,
                    "" if R is None else "%.5f" % R, "" if Tw is None else "%.2f" % Tw))
                tlm_t[3] = 0.0
                if tr not in (0, -1):
                    reason = "遮断した（理由 %d）" % tr
                    break
            if now - tlm_t.get(5, 0) > 2.0 and now - t0 > 2.0:
                reason = "テレメトリが途切れた"
                break
            if phase == "heat" and ((Tw is not None and Tw >= T_WIND_MAX) or now - t_phase >= hold_s):
                print("冷ます段へ（%.0f 秒・巻線 %s ℃）" % (now - t0, "?" if Tw is None else "%.1f" % Tw), flush=True)
                phase, t_phase = "cool", now
            if phase == "cool" and now - t_phase >= cool_s:
                break
            if int(now - t0) % 60 == 0 and now - t0 > 1 and getattr(sys, "_last", -1) != int(now - t0):
                sys._last = int(now - t0)
                print("%5.0f s %s R=%s 巻線=%s ℃ 基板=%.1f ℃ 電圧 %.0f/%.0f mV" % (
                    now - t0, phase, "?" if R is None else "%.4f" % R, "?" if Tw is None else "%.1f" % Tw,
                    bc, v[0], v[1]), flush=True)
            time.sleep(0.005)
    except KeyboardInterrupt:
        reason = "Ctrl-C"
    finally:
        stop()
        f.close()
        print("止めました:", reason, "記録:", path, flush=True)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv))
