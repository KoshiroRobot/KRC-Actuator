#!/usr/bin/env python3
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import krcbus

STEPS_MV = (6000, 10000, 12000, 13000, 13500)


def usage():
    return "使い方: python3 vcheck.py <node>   （node は 1〜%d）" % krcbus.NODE_MAX


def parse_args(argv):
    a = list(argv[1:])
    if len(a) != 1 or not krcbus.valid_node(a[0]):
        print(usage())
        raise SystemExit(1)
    return int(a[0])


def step_why(acc, tr):
    if not acc:
        return "電流が読めません（この段で塊 3・5 が 1 本も来ない：tlm 0 か間引きが大きい）。上げずに止めます"
    if tr is None:
        return "遮断が読めません（この段で塊 1 が来ない）。上げずに止めます"
    if tr != 0:
        return "遮断しました（理由 %d）。止めます" % tr
    return None


def main(argv, sock=None):
    node = parse_args(argv)
    print("軸 %d に整列モードで %s V を 1.2 秒ずつかけます（間に 12 秒冷ます）"
          % (node, " / ".join("%g" % (v / 1000.0) for v in STEPS_MV)), flush=True)
    if sock is None:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind(("can0",))
    sock.setblocking(False)
    s = sock

    def send(cid, d):
        s.send(struct.pack("=IB3x8s", cid, len(d), bytes(d).ljust(8, b"\x00")))

    def cfg(sub, p1=0, p2=0):
        p1 &= 0xFFFF
        p2 &= 0xFFFF
        send(0x120 + node, [0xA5, sub, p1 >> 8, p1 & 0xFF, p2 >> 8, p2 & 0xFF, 0, 0])

    def s16(d, i):
        return struct.unpack(">h", bytes(d[i:i + 2]))[0]

    tlm = {}
    tt = {}

    def drain():
        while True:
            try:
                fr = s.recv(16)
            except BlockingIOError:
                return
            cid, dlc, data = struct.unpack("=IB3x8s", fr)
            cid &= 0x1FFFFFFF
            b = 0x300 + node * 0x10
            if b <= cid < b + 0x10:
                tlm[cid - b] = data[:dlc]
                tt[cid - b] = time.time()

    rc = 0
    try:
        for v in STEPS_MV:
            t0 = time.time()
            acc = []
            while time.time() - t0 < 1.2:
                cfg(1, v, 0)
                time.sleep(0.05)
                drain()
                if time.time() - t0 > 0.6 and tt.get(3, 0.0) >= t0 and tt.get(5, 0.0) >= t0:
                    ia, ib, ic = s16(tlm[3], 0), s16(tlm[3], 2), s16(tlm[3], 4)
                    acc.append((((2 / 3) * (ia * ia + ib * ib + ic * ic)) ** 0.5, s16(tlm[5], 6), -(ib + ic), ia + ib + ic))
            tr = tlm[1][7] if (1 in tlm and len(tlm[1]) >= 8 and tt.get(1, 0.0) >= t0) else None
            if acc:
                n = len(acc)
                idm = sum(x[1] for x in acc) / n
                a2 = sum(x[2] for x in acc) / n
                sm = sum(x[3] for x in acc) / n
                print("V=%5.1f V  制御が使う id=%5.2f A（V/id=%.3f Ω）  -(b+c)=%5.2f A  生の3相の和=%+.2f A  遮断=%s（%d 点）" % (
                    v / 1000, idm / 1000, v / idm if idm else float("nan"), a2 / 1000, sm / 1000,
                    "–" if tr is None else "%d" % tr, n), flush=True)
            cfg(0)
            why = step_why(acc, tr)
            if why is not None:
                print("V=%5.1f V: %s" % (v / 1000, why), flush=True)
                rc = 1
                break
            time.sleep(12)
    finally:
        for _ in range(3):
            cfg(0)
            time.sleep(0.05)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv))
