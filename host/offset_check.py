#!/usr/bin/env python3
import math
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(1, os.path.expanduser("~"))
import krcbus

KD = 0.5
SPEED_MIN_FRAC = 0.5


def usage():
    return ("使い方: python3 offset_check.py <速さ rad/s> <node>   （速さは 0〜%g・node は 1〜%d。"
            "ほかの道具と違って速さが先）" % (krcbus.V_MAX, krcbus.NODE_MAX))


def parse_args(argv):
    a = list(argv[1:])
    try:
        spd = float(a[0]) if len(a) == 2 else None
    except ValueError:
        spd = None
    if spd is None or not math.isfinite(spd) or not (0.0 < spd <= krcbus.V_MAX) or not krcbus.valid_node(a[1]):
        print(usage())
        raise SystemExit(1)
    return spd, int(a[1])


def mean_of(acc):
    if not acc:
        return None
    return [sum(x[k] for x in acc) / len(acc) for k in range(5)]


def judge(res, spd, tr):
    bad = []
    if tr is None:
        bad.append("遮断が読めない（塊 1 が来ない）")
    elif tr != 0:
        bad.append("遮断した（理由 %d）" % tr)
    for v in (spd, -spd):
        m = res.get(v)
        if m is None:
            bad.append("v=%+g で点が 0 個（塊 5・7・10 が来ない：tlm 0 か間引きが大きい）" % v)
        elif m[4] * (1 if v > 0 else -1) < SPEED_MIN_FRAC * spd:
            bad.append("v=%+g で実速度 %+.2f rad/s（指令の %.0f %% に届かない：回っていない）" % (v, m[4], SPEED_MIN_FRAC * 100))
    return bad


class Link(object):
    def __init__(self, node, sock=None):
        self.node = node
        if sock is None:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind(("can0",))
        sock.setblocking(False)
        self.s = sock
        self.tlm = {}
        self.tt = {}
        self.sc = None
        self.scale = krcbus.ScaleRead()
        self.n4 = 0

    def send(self, cid, d):
        self.s.send(struct.pack("=IB3x8s", cid, len(d), bytes(d).ljust(8, b"\x00")))

    def special(self, code):
        self.send(0x100 + self.node, [0xFF] * 7 + [code])

    def mit(self, v):
        sc = self.sc
        p = krcbus._pack(0.0, -sc["p"], sc["p"], 16)
        vv = krcbus._pack(v, -sc["v"], sc["v"], 12)
        kp = krcbus._pack(0.0, 0.0, sc["kp"], 12)
        kd = krcbus._pack(KD, 0.0, sc["kd"], 12)
        t = krcbus._pack(0.0, -sc["t"], sc["t"], 12)
        self.send(0x100 + self.node, [p >> 8, p & 0xFF, vv >> 4, ((vv & 0xF) << 4) | (kp >> 8), kp & 0xFF,
                                      kd >> 4, ((kd & 0xF) << 4) | (t >> 8), t & 0xFF])

    def drain(self):
        while True:
            try:
                fr = self.s.recv(16)
            except BlockingIOError:
                return
            cid, dlc, data = struct.unpack("=IB3x8s", fr)
            cid &= 0x1FFFFFFF
            b = 0x300 + self.node * 0x10
            if b <= cid < b + 0x10:
                self.tlm[cid - b] = data[:dlc]
                self.tt[cid - b] = time.time()
                if cid - b == 13:
                    self.scale.feed(data[:dlc])
                elif cid - b == 4:
                    self.n4 += 1

    def read_board(self):
        t0 = time.time()
        t_end = t0 + krcbus.SCALE_WAIT_S
        while time.time() < t_end:
            self.drain()
            mode = krcbus.board_mode_why(self.tlm.get(4), self.n4, final=False)
            if self.scale.result() is not None and mode != krcbus.BOARD_MODE_WAIT:
                break
            d12 = self.tlm.get(12)
            if d12 is not None and len(d12) >= 5 and d12[4] > 0:
                t_end = max(t_end, t0 + krcbus.tlm13_wait_s(d12[4]))
            time.sleep(0.005)
        r = self.scale.result()
        if r is None:
            return "基板の目盛りが読めない（塊 13 が来ない。tlm 0 になっていないか）"
        self.sc = r[1]
        why = krcbus.board_mode_why(self.tlm.get(4), self.n4, final=True)
        if why is not None:
            return "ノード%d の%s" % (self.node, why)
        return None


def s16(d, i):
    return struct.unpack(">h", bytes(d[i:i + 2]))[0]


def main(argv, sock=None):
    spd, node = parse_args(argv)
    print("軸 %d を ±%g rad/s で回して、電圧から電気角のずれを見ます" % (node, spd), flush=True)
    L = Link(node, sock)
    why = L.read_board()
    if why is not None:
        print("回しません: %s" % why)
        return 1
    res = {}
    t_start = time.time()
    try:
        L.special(0xFC)
        time.sleep(0.15)
        for v in (spd, -spd):
            t0 = time.time()
            acc = []
            last7 = 0
            while time.time() - t0 < 3.0:
                L.mit(v)
                time.sleep(0.005)
                L.drain()
                if time.time() - t0 > 1.5 and 7 in L.tlm and L.tt[7] != last7 and 5 in L.tlm and 10 in L.tlm:
                    last7 = L.tt[7]
                    acc.append((s16(L.tlm[7], 0), s16(L.tlm[7], 2), s16(L.tlm[5], 6), s16(L.tlm[5], 4),
                                s16(L.tlm[10], 2) / 10.0))
            m = mean_of(acc)
            res[v] = m
            if m is None:
                print("v=%+5.1f rad/s: 点が 0 個" % v, flush=True)
            else:
                print("v=%+5.1f rad/s: 実速度 %+6.2f  vd %+6d mV  vq %+6d mV  id %+5d mA  iq %+5d mA（%d 点）"
                      % (v, m[4], m[0], m[1], m[2], m[3], len(acc)), flush=True)
            for _ in range(60):
                L.mit(0.0)
                time.sleep(0.005)
    finally:
        L.special(0xFD)
    L.drain()
    d1 = L.tlm.get(1)
    tr = d1[7] if (d1 is not None and len(d1) >= 8 and L.tt.get(1, 0.0) >= t_start) else None
    bad = judge(res, spd, tr)
    if bad:
        print("**回っていないので測れません**（電気角のずれは出しません）: %s" % "／".join(bad))
        return 1
    p, q = res[spd], res[-spd]
    vd_a = (p[0] - q[0]) / 2
    vq_a = (p[1] - q[1]) / 2
    eps = math.degrees(math.atan2(vd_a, vq_a))
    print("反対称: vd %.0f mV・vq %.0f mV → 電気角のずれ ε ≈ %+.1f°（電気角）" % (vd_a, vq_a, eps))
    print("参考: 電気角 1° ＝ %.1f カウント（16384 / 21 極対 / 360°）" % (16384 / 21 / 360))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
