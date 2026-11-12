#!/usr/bin/env python3
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import krcbus
import offset_cal as oc

REV_PER_S = oc.REV_PER_S
REVS_EXTRA = 2
SETTLE_S = 1.0
N_BINS = 8
BIN_MIN = 3
RES_RMS_MAX_DEG = 8.0


def usage():
    return ("使い方: python3 enc_cal.py <node> [apply]   （node は 1〜%d。apply でパラメータ 16・39〜42 に入れる・保存はしない）"
            % krcbus.NODE_MAX)


def parse_args(argv):
    a = list(argv[1:])
    if not a or not krcbus.valid_node(a[0]) or len(a) > 2 or (len(a) == 2 and a[1] != "apply"):
        print(usage())
        raise SystemExit(1)
    return int(a[0]), len(a) == 2


def read_corr(L):
    ids = krcbus.ENC_CORR_PARAMS
    if not L.wait(lambda: all(p in L.prm for p in ids)):
        return None
    return [L.prm[p] for p in ids]


def measure(L, e0, pp, rev_per_s=REV_PER_S, order=(+1, -1)):
    samples = {+1: [], -1: []}
    revs = pp + REVS_EXTRA
    th = float(e0)
    step = 65536.0 * rev_per_s * 0.01
    skip = int(SETTLE_S / 0.01)
    try:
        t1 = time.time()
        while time.time() - t1 < 1.0:
            L.cfg(1, oc.V_MV, int(th) & 0xFFFF)
            time.sleep(0.01)
            L.drain()
        for sgn in order:
            n = int(revs / rev_per_s / 0.01)
            last5, last1 = L.tt.get(5), L.tt.get(1)
            for k in range(n):
                th += sgn * step
                L.cfg(1, oc.V_MV, int(th) & 0xFFFF)
                time.sleep(0.01)
                L.drain()
                if (k >= skip and 5 in L.tlm and 1 in L.tlm and len(L.tlm[1]) >= 6
                        and L.tt[5] != last5 and L.tt[1] != last1):
                    last5, last1 = L.tt[5], L.tt[1]
                    samples[sgn].append((oc.u16(L.tlm[5], 0), oc.u16(L.tlm[5], 2), oc.u16(L.tlm[1], 4) & 0x3FFF))
            print("%s: %d 点" % ("正転" if sgn > 0 else "逆転", len(samples[sgn])), flush=True)
    finally:
        for _ in range(3):
            L.cfg(0)
            time.sleep(0.05)
    return samples


def solve(a, b):
    n = len(b)
    m = [list(a[i]) + [b[i]] for i in range(n)]
    scale = max(abs(m[i][i]) for i in range(n)) or 1.0
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-9 * scale:
            return None
        m[c], m[p] = m[p], m[c]
        for r in range(c + 1, n):
            f = m[r][c] / m[c][c]
            for k in range(c, n + 1):
                m[r][k] -= f * m[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def regressors(sgn, mech):
    t = 2.0 * math.pi * mech / 16384.0
    return [1.0, float(sgn), math.cos(t), math.sin(t), math.cos(2.0 * t), math.sin(2.0 * t)]


def fit(samples):
    pts = [(sgn, oc.wrap(c - e), m) for sgn in (+1, -1) for c, e, m in samples.get(sgn, [])]
    if len(pts) < 12:
        return None
    sx = sum(math.sin(2.0 * math.pi * d / 65536.0) for _s, d, _m in pts)
    cx = sum(math.cos(2.0 * math.pi * d / 65536.0) for _s, d, _m in pts)
    ctr = math.atan2(sx, cx) * 65536.0 / (2.0 * math.pi)
    rows = [(regressors(s, m), oc.deg(ctr + oc.wrap(int(round(d - ctr)))), s) for s, d, m in pts]
    ata = [[sum(x[i] * x[j] for x, _y, _s in rows) for j in range(6)] for i in range(6)]
    aty = [sum(x[i] * y for x, y, _s in rows) for i in range(6)]
    k = solve(ata, aty)
    if k is None:
        return None
    r0 = [y - k[0] - k[1] * s for x, y, s in rows]
    res = dict((s, [y - sum(ki * xi for ki, xi in zip(k, x)) for x, y, s_ in rows if s_ == s]) for s in (+1, -1))
    allr = res[+1] + res[-1]

    def sd(v):
        if not v:
            return None
        mu = sum(v) / len(v)
        return (sum((q - mu) ** 2 for q in v) / len(v)) ** 0.5
    return dict(mean=k[0], lag=k[1], c1=k[2], s1=k[3], c2=k[4], s2=k[5],
                width0=max(r0) - min(r0), rms0=(sum(q * q for q in r0) / len(r0)) ** 0.5,
                rms=(sum(q * q for q in allr) / len(allr)) ** 0.5, max=max(abs(q) for q in allr),
                sd={+1: sd(res[+1]), -1: sd(res[-1])})


def coverage(pts):
    h = [0] * N_BINS
    for _c, _e, m in pts:
        h[(m * N_BINS) // 16384] += 1
    return h


CORR_KEYS = ("c1", "s1", "c2", "s2")


def corr_add(corr, add):
    return [int(round(c + 10.0 * a)) for c, a in zip(corr, add)]


def corr_range_bad(new_corr):
    bad = []
    for pid, v in zip(krcbus.ENC_CORR_PARAMS, new_corr):
        lo, hi = krcbus.PARAMS[pid][4], krcbus.PARAMS[pid][5]
        if not lo <= v <= hi:
            bad.append("新しい%sが %+.1f°（%+.0f〜%+.0f° の外：基板は端に丸めてしまう）"
                       % (krcbus.PARAMS[pid][1], v * 0.1, lo * 0.1, hi * 0.1))
    return bad


def want_values(new_off, new_corr):
    return dict([(16, int(new_off))] + [(p, int(v)) for p, v in zip(krcbus.ENC_CORR_PARAMS, new_corr)])


def new_values(fr, pp, dr, off, corr):
    delta = (((fr["mean"] + 180.0) % 360.0) - 180.0) * 65536.0 / 360.0
    d_off = -delta / (dr * pp * 4.0)
    new_off = int(round(off + d_off)) % 16384
    return new_off, d_off, corr_add(corr, [fr[k] for k in CORR_KEYS])


def judge(samples, fr, tr0, tr1, new_corr=None):
    st = dict((s, oc.sweep_stats([[(c, e) for c, e, _m in samples.get(s, [])]])) for s in (+1, -1))
    if fr is not None:
        for s in (+1, -1):
            if st[s]["n"] and fr["sd"][s] is not None:
                st[s] = dict(st[s], mean=(fr["mean"] + s * fr["lag"]) * 65536.0 / 360.0,
                             sd=fr["sd"][s] * 65536.0 / 360.0)
    bad = oc.judge(st, tr0, tr1)
    for s, name in ((+1, "正転"), (-1, "逆転")):
        pts = samples.get(s, [])
        if len(pts) >= oc.MIN_PTS:
            h = coverage(pts)
            if min(h) < BIN_MIN:
                bad.append("%sで θm が 1 回転を覆っていない（%d 区間の点の数 %s：どの区間にも %d 点要る）"
                           % (name, N_BINS, h, BIN_MIN))
    if fr is None:
        bad.append("当てはめられない（点が少ない・θm が偏っている）")
    elif fr["rms"] > RES_RMS_MAX_DEG:
        bad.append("当てはめた後の残り RMS %.1f° が大きい（%.0f° まで：2 次までの形で説明できていない）"
                   % (fr["rms"], RES_RMS_MAX_DEG))
    if new_corr is not None:
        bad += corr_range_bad(new_corr)
    return bad


def apply_and_readback(L, want):
    L.drain()
    n0 = L.n13
    for pid, v in want.items():
        L.cfg(krcbus.CFG_SET_PARAM, pid, v)
        time.sleep(0.02)
    ok = L.wait(lambda: all(L.prm_n.get(p, -1) >= n0 + 2 for p in want))
    if not ok:
        print("パラメータ %s を送った → **読み返せません**（塊 13 が来ない：tlm 0 か間引きが大きい）。"
              "krc.py param で確かめてから保存してください" % "・".join(str(p) for p in want))
        return False
    good = True
    for pid, v in want.items():
        got = L.prm[pid]
        if got != v:
            good = False
        print("  %2d %-14s 送った %6d → 基板の読み返し %6d%s"
              % (pid, krcbus.PARAMS[pid][0], v, got, "" if got == v else "　**食い違い**（遮断中は受けない）"))
    print("入れました（保存はまだ）" if good else "**入っていないものがあります**")
    return good


def main(argv, sock=None):
    node, apply_ = parse_args(argv)
    print("軸 %d の電気角オフセットとエンコーダの補正を測ります%s"
          % (node, "（apply：測った値をパラメータ 16・39〜42 に入れる・保存はしない）" if apply_ else ""), flush=True)
    L = oc.Link(node, sock)
    stt, why = oc.read_state(L)
    if why is not None:
        print(why)
        return 1
    pp, dr, off, e0, tr0 = stt
    corr = read_corr(L)
    if corr is None:
        print("補正のパラメータ 39〜42 が読めません（塊 13 に無い：古いファームか、tlm の間引きが大きい）")
        return 1
    print("極対数 %d・向き %+d・いまのオフセット %d・いまの補正 [0.1°] cos θ %d / sin θ %d / cos 2θ %d / sin 2θ %d"
          % ((pp, dr, off) + tuple(corr)), flush=True)
    if tr0 != 0:
        print("遮断中（理由 %d）なので測れません（「遮断を解除」してから）" % tr0)
        return 1
    if pp <= 0:
        print("極対数が 0 と読めました（較正が入っていない）。測れません")
        return 1
    print("整列モードで正転・逆転を %d 電気回転ずつ（%.1f 電気回転/s・約 %.0f 秒）引き回します"
          % (pp + REVS_EXTRA, REV_PER_S, 2.0 * (pp + REVS_EXTRA) / REV_PER_S + 1.0), flush=True)
    t_start = time.time()
    samples = measure(L, e0, pp)
    L.wait(lambda: L.trip(t_start) is not None)
    tr1 = L.trip(t_start)
    for sgn, name in ((+1, "正転"), (-1, "逆転")):
        s = oc.sweep_stats([[(c, e) for c, e, _m in samples[sgn]]])
        if s["n"]:
            print("%s: 指令−エンコーダ 平均 %+.1f°（%d 点・エンコーダが動いた量は指令の %.0f %%）"
                  % (name, oc.deg(s["mean"]), s["n"], (100.0 * s["enc"] / s["cmd"]) if s["cmd"] else float("nan")))
    fr = fit(samples)
    new = None
    if fr is not None:
        print("当てはめる前（平均と遅れだけ除く）: 幅 %.1f°・RMS %.1f°" % (fr["width0"], fr["rms0"]))
        print("当てはめ: 平均 %+.1f°・摩擦の遅れ %.1f°・1 次 cos %+.1f° sin %+.1f°（振幅 %.1f°）・"
              "2 次 cos %+.1f° sin %+.1f°（振幅 %.1f°）"
              % (fr["mean"], fr["lag"], fr["c1"], fr["s1"], math.hypot(fr["c1"], fr["s1"]),
                 fr["c2"], fr["s2"], math.hypot(fr["c2"], fr["s2"])))
        print("当てはめた後の残り: RMS %.1f°・最大 %.1f°" % (fr["rms"], fr["max"]))
        new = new_values(fr, pp, dr, off, corr)
    bad = judge(samples, fr, tr0, tr1, None if new is None else new[2])
    if bad:
        print("**測れていないので結論を出しません**%s: %s" % ("（apply もしない）" if apply_ else "", "／".join(bad)))
        return 1
    new_off, d_off, new_corr = new
    print("オフセット %d → %d（%+.1f カウント）" % (off, new_off, d_off))
    print("補正 [0.1°]: " + "・".join("%s %d → %d" % (krcbus.PARAMS[p][0], a, b)
                                     for p, a, b in zip(krcbus.ENC_CORR_PARAMS, corr, new_corr)), flush=True)
    if apply_:
        if not apply_and_readback(L, want_values(new_off, new_corr)):
            return 1
        print("確かめるなら、もう一度 enc_cal.py %d（残りが小さく出るはず）。残すなら python3 krc.py -n %d param save"
              % (node, node))
        return 0
    print("（入れるなら enc_cal.py %d apply。保存は python3 krc.py -n %d param save）" % (node, node))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
