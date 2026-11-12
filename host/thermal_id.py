#!/usr/bin/env python3
import math
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import krcbus

T_WIND_MAX = 70.0
T_BOARD_MAX = 60.0
ALPHA = 0.00393
HALF = 2.0
HEAT = (4200, 5800)
COOL = (800, 2400)
OUT_DIR = os.path.expanduser("~/thermal")
LOG_HEAD = "t,phase,v_cmd_mv,i_ma,ia_ma,ib_ma,ic_ma,id_ma,iq_ma,board_c,therm_pct,trip,R_ohm,Tw_c\n"


def usage():
    return "使い方: python3 thermal_id.py <node> [温める秒の上限=900] [冷ます秒=600]   （node は 1〜%d）" % krcbus.NODE_MAX


def parse_args(argv):
    a = list(argv[1:])
    try:
        heat = float(a[1]) if len(a) > 1 else 900.0
        cool = float(a[2]) if len(a) > 2 else 600.0
    except ValueError:
        heat = cool = None
    if (not a or len(a) > 3 or not krcbus.valid_node(a[0]) or heat is None
            or not (math.isfinite(heat) and math.isfinite(cool)) or not (heat > 0 and cool >= 0)):
        print(usage())
        raise SystemExit(1)
    return int(a[0]), heat, cool


def board_temp(d):
    if d is None or len(d) < 6:
        return None
    raw = struct.unpack(">h", bytes(d[4:6]))[0]
    return None if raw == krcbus.TEMP_INVALID_RAW else raw / 10.0


def open_log(path):
    f = open(path, "w", buffering=1)
    f.write(LOG_HEAD)
    return f


def main(argv, sock=None):
    node, heat_max_s, cool_s = parse_args(argv)
    print("軸 %d を整列モードで温めて、巻線の熱を測ります（温める上限 %.0f 秒・冷ます %.0f 秒）" % (node, heat_max_s, cool_s),
          flush=True)
    if sock is None:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind(("can0",))
    sock.setblocking(False)
    s = sock

    def send(cid, data):
        s.send(struct.pack("=IB3x8s", cid, len(data), bytes(data).ljust(8, b"\x00")))

    def cfg(sub, p1=0, p2=0):
        p1 &= 0xFFFF
        p2 &= 0xFFFF
        send(0x120 + node, [0xA5, sub, p1 >> 8, p1 & 0xFF, p2 >> 8, p2 & 0xFF, 0, 0])

    def s16(d, i):
        return struct.unpack(">h", bytes(d[i:i + 2]))[0]

    tlm = {}
    tlm_t = {}

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
    if 0 not in tlm:
        print("テレメトリが来ません", flush=True)
        return 1
    T0 = board_temp(tlm[0])
    if T0 is None:
        print("基板温度が読めません（塊 0 が「測れない」印：サーミスタの断線・短絡・ADC）。巻線の温度が作れず、"
              "「巻線 ≥ 70 ℃」「基板 ≥ 60 ℃」の停止が効かないので、電流を流さずにやめます", flush=True)
        return 1

    def stop():
        for _ in range(3):
            cfg(0)
            time.sleep(0.05)

    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, time.strftime("thermal_n%d_%%Y%%m%%d_%%H%%M%%S.csv" % node))
    f = open_log(path)
    print("記録:", path, flush=True)

    t0 = time.time()
    phase = "heat"
    t_phase = t0
    cur = HEAT
    lvl = 0
    t_half = t0
    half_buf = []
    last_half = None
    R0 = None
    R = None
    Tw = None
    bc = T0
    t_bc = tlm_t[0]
    reason = "終わり"
    rc = 0
    try:
        print("始めの基板温度 %.1f ℃" % T0, flush=True)
        nxt = 0.0
        while True:
            now = time.time()
            drain()
            if tlm_t.get(0, 0.0) > t_bc:
                t_bc = tlm_t[0]
                b_ = board_temp(tlm[0])
                if b_ is None:
                    reason = "基板温度が読めなくなった（塊 0 が「測れない」印）"
                    rc = 1
                    break
                bc = b_
                if bc >= T_BOARD_MAX:
                    reason = "基板が %.1f ℃" % bc
                    break
            if now >= nxt:
                cfg(1, cur[lvl], 0)
                nxt = now + 0.1
            if now - t_half >= HALF:
                if half_buf:
                    vd = sum(x[0] for x in half_buf) / len(half_buf)
                    idm = sum(x[1] for x in half_buf) / len(half_buf)
                    if last_half is not None and last_half[2] == phase and abs(idm - last_half[1]) > 300 and abs(vd - last_half[0]) > 100:
                        r = (vd - last_half[0]) / (idm - last_half[1])
                        R = r if R is None else 0.7 * R + 0.3 * r
                        if R0 is None:
                            R0 = R
                        Tw = T0 + (R / R0 - 1.0) / ALPHA
                    last_half = (vd, idm, phase)
                half_buf = []
                lvl ^= 1
                t_half = now
            if 3 in tlm and 5 in tlm and now - tlm_t.get(3, 0) < 0.15:
                ia = s16(tlm[3], 0)
                ib = s16(tlm[3], 2)
                ic = s16(tlm[3], 4)
                imag = ((2.0 / 3.0) * (ia * ia + ib * ib + ic * ic)) ** 0.5
                iq = s16(tlm[5], 4)
                idd = s16(tlm[5], 6)
                vd = cur[lvl]
                idm = imag
                th = tlm[4][0] if 4 in tlm else -1
                tr = tlm[1][7] if 1 in tlm else -1
                if now - t_half >= HALF - 1.0:
                    half_buf.append((vd, idm))
                f.write("%.2f,%s,%d,%.0f,%d,%d,%d,%d,%d,%.1f,%d,%d,%s,%s\n" % (
                    now - t0, phase, cur[lvl], imag, ia, ib, ic, idd, iq, bc, th, tr,
                    "" if R is None else "%.5f" % R, "" if Tw is None else "%.2f" % Tw))
                tlm_t[3] = 0.0
                if tr not in (0, -1):
                    reason = "遮断した（理由 %d）" % tr
                    break
            if now - tlm_t.get(5, 0) > 2.0 and now - t0 > 2.0:
                reason = "テレメトリが途切れた"
                break
            if phase == "heat" and ((Tw is not None and Tw >= T_WIND_MAX) or now - t_phase >= heat_max_s):
                print("冷ます段へ（%.0f 秒・巻線 %s ℃）" % (now - t0, "?" if Tw is None else "%.1f" % Tw), flush=True)
                phase = "cool"
                t_phase = now
                cur = COOL
            if phase == "cool" and now - t_phase >= cool_s:
                break
            if int(now - t0) % 30 == 0 and now - t0 > 1 and getattr(sys, "_last", -1) != int(now - t0):
                sys._last = int(now - t0)
                print("%4.0f s %s R=%s 巻線=%s ℃ 基板=%.1f ℃ 見積もり=%s%%" % (
                    now - t0, phase, "?" if R is None else "%.4f" % R, "?" if Tw is None else "%.1f" % Tw,
                    bc, tlm[4][0] if 4 in tlm else "?"), flush=True)
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
