#!/usr/bin/env python3
import math
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import krcbus

V_MV = 4000
REV_PER_S = 0.5
REVS = 3
FIRST_S = 1.2
MIN_PTS = 20
SD_MAX_DEG = 30.0
FOLLOW_LO, FOLLOW_HI = 0.8, 1.2
CMD_MIN_REV = 1.0
LAG_MIN_DEG = -3.0


def usage():
    return "使い方: python3 offset_cal.py <node> [apply]   （node は 1〜%d。apply でパラメータ 16 に入れる・保存はしない）" % krcbus.NODE_MAX


def parse_args(argv):
    a = list(argv[1:])
    if not a or not krcbus.valid_node(a[0]) or len(a) > 2 or (len(a) == 2 and a[1] != "apply"):
        print(usage())
        raise SystemExit(1)
    return int(a[0]), len(a) == 2


def wrap(x):
    return ((x + 32768) % 65536) - 32768


def u16(d, i):
    return (d[i] << 8) | d[i + 1]


def deg(x):
    return x * 360.0 / 65536.0


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
        self.prm = {}
        self.prm_n = {}
        self.n13 = 0

    def send(self, cid, d):
        self.s.send(struct.pack("=IB3x8s", cid, len(d), bytes(d).ljust(8, b"\x00")))

    def cfg(self, sub, p1=0, p2=0):
        p1 &= 0xFFFF
        p2 &= 0xFFFF
        self.send(0x120 + self.node, [0xA5, sub, p1 >> 8, p1 & 0xFF, p2 >> 8, p2 & 0xFF, 0, 0])

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
                k = cid - b
                self.tlm[k] = data[:dlc]
                self.tt[k] = time.time()
                if k == 13:
                    self.n13 += 1
                    for i in (0, 3):
                        if data[i]:
                            self.prm[data[i]] = struct.unpack(">h", data[i + 1:i + 3])[0]
                            self.prm_n[data[i]] = self.n13

    def wait(self, pred, first_s=FIRST_S):
        end = time.time() + first_s
        grown = False
        while True:
            self.drain()
            if pred():
                return True
            if not grown and 12 in self.tlm and len(self.tlm[12]) >= 5 and self.tlm[12][4] > 0:
                grown = True
                div = self.tlm[12][4]
                end = max(end, time.time() + (krcbus.PARAM_PAGES * 2 + 1) * div * krcbus.TLM_LOOP_S)
            if time.time() >= end:
                return False
            time.sleep(0.01)

    def trip(self, since=0.0):
        if 1 not in self.tlm or self.tt.get(1, 0.0) < since or len(self.tlm[1]) < 8:
            return None
        return self.tlm[1][7]


def read_state(L):
    need = ((8, "塊 8（極対数・向き）"), (5, "塊 5（電気角）"), (1, "塊 1（遮断）"))
    ok = L.wait(lambda: all(k in L.tlm for k, _ in need) and 16 in L.prm)
    if not ok:
        miss = [w for k, w in need if k not in L.tlm] + ([] if 16 in L.prm else ["パラメータ 16（塊 13）"])
        return None, ("読めません（%s が来ない：tlm 0 か、間引き tlm N が大きい）" % "・".join(miss))
    pp = L.tlm[8][2] & 0x7F
    dr = -1 if L.tlm[8][3] == 0xFF else 1
    return (pp, dr, L.prm[16], u16(L.tlm[5], 2), L.trip()), None


def sweep_stats(sweeps):
    pts = [x for sw in sweeps for x in sw]
    n = len(pts)
    if n == 0:
        return dict(n=0, mean=None, sd=None, cmd=None, enc=None)
    diffs = [wrap(c - e) for c, e in pts]
    m = sum(diffs) / float(n)
    sd = (sum((x - m) ** 2 for x in diffs) / n) ** 0.5
    cm = sum(wrap(sw[i][0] - sw[i - 1][0]) for sw in sweeps for i in range(1, len(sw)))
    em = sum(wrap(sw[i][1] - sw[i - 1][1]) for sw in sweeps for i in range(1, len(sw)))
    return dict(n=n, mean=m, sd=sd, cmd=cm, enc=em)


def judge(st, tr0, tr1):
    bad = []
    for when, tr in (("測る前", tr0), ("測ったあと", tr1)):
        if tr is None:
            bad.append("%sの遮断が読めない（塊 1 が来ない）" % when)
        elif tr != 0:
            bad.append("%sに遮断していた（理由 %d）" % (when, tr))
    for sgn, name in ((+1, "正転"), (-1, "逆転")):
        s = st.get(sgn) or sweep_stats([])
        if s["n"] < MIN_PTS:
            bad.append("%sの点が %d 個（%d 個に足りない：tlm の間引きが大きい）" % (name, s["n"], MIN_PTS))
            continue
        if sgn * s["cmd"] < CMD_MIN_REV * 65536:
            bad.append("%sで指令の電気角が進んでいない（%.2f 電気回転：基板が整列モードに入っていない）"
                       % (name, s["cmd"] / 65536.0))
            continue
        r = s["enc"] / float(s["cmd"])
        if r < FOLLOW_LO:
            bad.append("%sでエンコーダが指令に付いてこない（指令の %.0f %% しか動いていない：ロータが回らない・固着）"
                       % (name, r * 100.0))
        elif r > FOLLOW_HI:
            bad.append("%sでエンコーダが指令より多く進んだ（指令の %.0f %%：引き回しでは起きない。極対数などの食い違いを疑う）"
                       % (name, r * 100.0))
        if deg(s["sd"]) > SD_MAX_DEG:
            bad.append("%sのばらつき %.1f° が大きい（%.0f° まで）" % (name, deg(s["sd"]), SD_MAX_DEG))
    if not bad:
        lag = (st[+1]["mean"] - st[-1]["mean"]) / 2.0
        if deg(lag) < LAG_MIN_DEG:
            bad.append("摩擦の遅れが負（%.1f°：指令より先に進むことはない。引き回せていない）" % deg(lag))
    return bad


def measure(L, e0, revs=REVS, rev_per_s=REV_PER_S):
    samples = {+1: [], -1: []}
    th = float(e0)
    step = 65536.0 * rev_per_s * 0.01
    try:
        t1 = time.time()
        while time.time() - t1 < 1.0:
            L.cfg(1, V_MV, int(th) & 0xFFFF)
            time.sleep(0.01)
            L.drain()
        for sgn in (+1, -1, +1, -1):
            n = int(revs / rev_per_s / 0.01)
            last = L.tt.get(5)
            cur = []
            samples[sgn].append(cur)
            for k in range(n):
                th += sgn * step
                L.cfg(1, V_MV, int(th) & 0xFFFF)
                time.sleep(0.01)
                L.drain()
                if k > n // 6 and 5 in L.tlm and L.tt[5] != last:
                    last = L.tt[5]
                    cur.append((u16(L.tlm[5], 0), u16(L.tlm[5], 2)))
    finally:
        for _ in range(3):
            L.cfg(0)
            time.sleep(0.05)
    return samples


def apply_and_readback(L, new):
    L.drain()
    n0 = L.n13
    L.cfg(22, 16, new)
    ok = L.wait(lambda: L.prm_n.get(16, -1) >= n0 + 2)
    if not ok:
        print("パラメータ 16 に %d を送った → **読み返せません**（塊 13 が来ない：tlm 0 か間引きが大きい）。"
              "krc.py param で確かめてから保存してください" % new)
        return False
    got = L.prm[16]
    if got != new:
        print("パラメータ 16 に %d を送った → 基板の読み返し %d　**食い違い**（遮断中は受けない）" % (new, got))
        return False
    print("パラメータ 16 に入れた → 基板の読み返し %d（保存はまだ）" % got)
    return True


def main(argv, sock=None):
    node, apply_ = parse_args(argv)
    print("軸 %d の電気角オフセットを測ります%s" % (node, "（apply：測った値をパラメータ 16 に入れる・保存はしない）"
                                                  if apply_ else ""), flush=True)
    L = Link(node, sock)
    stt, why = read_state(L)
    if why is not None:
        print(why)
        return 1
    pp, dr, off, e0, tr0 = stt
    print("極対数 %d・向き %+d・いまのオフセット %d・いまの電気角 %d" % (pp, dr, off, e0), flush=True)
    if tr0 != 0:
        print("遮断中（理由 %d）なので測れません（「遮断を解除」してから）" % tr0)
        return 1
    t_start = time.time()
    samples = measure(L, e0)
    L.wait(lambda: L.trip(t_start) is not None)
    tr1 = L.trip(t_start)
    st = dict((k, sweep_stats(v)) for k, v in samples.items())
    for sgn, name in ((+1, "正転"), (-1, "逆転")):
        s = st[sgn]
        if s["n"]:
            print("%s: 指令−エンコーダ %+.1f°（ばらつき %.1f°・%d 点・エンコーダが動いた量は指令の %.0f %%）"
                  % (name, deg(s["mean"]), deg(s["sd"]), s["n"],
                     (100.0 * s["enc"] / s["cmd"]) if s["cmd"] else float("nan")))
        else:
            print("%s: 点が 0 個" % name)
    bad = judge(st, tr0, tr1)
    if bad:
        print("**測れていないので結論を出しません**%s: %s" % ("（apply もしない）" if apply_ else "", "／".join(bad)))
        return 1
    delta = (st[+1]["mean"] + st[-1]["mean"]) / 2.0
    lag = (st[+1]["mean"] - st[-1]["mean"]) / 2.0
    print("→ オフセットのずれ %+.1f°（電気角）・摩擦の遅れ %.1f°" % (deg(delta), deg(lag)))
    d_off = -delta / (dr * pp * 4.0)
    new = int(round(off + d_off)) % 16384
    print("オフセット %d → %d（%+.1f カウント）" % (off, new, d_off), flush=True)
    if apply_:
        return 0 if apply_and_readback(L, new) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
