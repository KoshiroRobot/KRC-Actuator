import csv
import math
import sys

ALPHA = 0.00393
MIN_PTS_PHASE = 5
R0_LO, R0_HI = 0.8, 1.05


def r0_why(R0, rmin, rmax):
    if R0_LO * rmin <= R0 <= R0_HI * rmax:
        return None
    return ("R0 %.4f Ω が測った抵抗 %.4f〜%.4f Ω から外れる（最小の %.2f 倍〜最大の %.2f 倍の外：当てはめが形を"
            "捉えていない。記録が短い・周囲温度が違う）" % (R0, rmin, rmax, R0_LO, R0_HI))


def read_rows(path):
    good, bad = [], 0
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            try:
                good.append((float(r["t"]), r["phase"], float(r["v_cmd_mv"]) / 1000.0, float(r["i_ma"]) / 1000.0,
                             float(r["board_c"]) if r["board_c"] else float("nan"), float(r["therm_pct"])))
            except (TypeError, ValueError, KeyError):
                bad += 1
    return good, bad


def main(argv):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except AttributeError:
        pass
    import numpy as np
    if len(argv) < 2:
        print("使い方: python thermal_fit.py <csv> [Ta（周囲）]")
        return 1
    path = argv[1]
    rows, nbad = read_rows(path)
    if nbad:
        print("読めない行を %d 行飛ばしました（途中で切れた行など）" % nbad)
    if not rows:
        print("**当てはめられません**: CSV に読める行がありません（thermal_id が始めてすぐ止まった・殺された）")
        return 1
    t = np.array([x[0] for x in rows])
    ph = [x[1] for x in rows]
    v = np.array([x[2] for x in rows])
    i = np.array([x[3] for x in rows])
    bc = np.array([x[4] for x in rows])
    th = np.array([x[5] for x in rows])
    Ta = float(argv[2]) if len(argv) > 2 else float(bc[0])

    halves = []
    start = 0
    for k in range(1, len(t) + 1):
        if k == len(t) or v[k] != v[start] or ph[k] != ph[start]:
            seg = range(start, k)
            tend = t[k - 1]
            tail = [j for j in seg if t[j] >= tend - 1.0]
            if len(tail) >= 3:
                halves.append((t[tail].mean(), v[start], i[tail].mean(), ph[start]))
            start = k
    Rk = []
    for a, b in zip(halves, halves[1:]):
        if a[3] == b[3] and abs(b[2] - a[2]) > 0.3:
            Rk.append(((a[0] + b[0]) / 2, (b[1] - a[1]) / (b[2] - a[2]), b[3]))
    n_heat = len([x for x in Rk if x[2] == "heat"])
    n_cool = len([x for x in Rk if x[2] == "cool"])
    if n_heat < MIN_PTS_PHASE or n_cool < MIN_PTS_PHASE:
        print("**当てはめられません**: 抵抗の点が加熱 %d 個・冷却 %d 個（どちらも %d 個以上要る：途中で止まった記録・"
              "加熱か冷却の段が無い）。全体 %.0f s" % (n_heat, n_cool, MIN_PTS_PHASE, t[-1]))
        return 1
    Rt = np.array([x[0] for x in Rk])
    Rv = np.array([x[1] for x in Rk])
    print("抵抗の点 %d 個（加熱 %d・冷却 %d）・範囲 %.4f〜%.4f Ω・加熱 %.0f s / 全体 %.0f s" % (
        len(Rk), n_heat, n_cool, Rv.min(), Rv.max(), max(x[0] for x in Rk if x[2] == "heat"), t[-1]))

    Ri = np.interp(t, Rt, Rv)
    P = 1.5 * Ri * i ** 2
    dt = np.diff(t, prepend=t[0])

    def resp(tau):
        h = np.zeros_like(t)
        acc = 0.0
        for k in range(len(t)):
            a = math.exp(-dt[k] / tau)
            acc = acc * a + (1 - a) * P[k]
            h[k] = acc
        return h, np.exp(-t / tau)

    def fit1(tau):
        h, g = resp(tau)
        hk = np.interp(Rt, t, h)
        gk = np.interp(Rt, t, g)
        A = np.vstack([np.ones_like(Rt), gk, hk]).T
        x, res, *_ = np.linalg.lstsq(A, Rv, rcond=None)
        err = Rv - A @ x
        return float(np.sqrt(np.mean(err ** 2))), x

    best = None
    for tau in np.concatenate([np.arange(3, 60, 1.0), np.arange(60, 1200, 5.0), np.arange(1200, 6000, 50.0)]):
        e, x = fit1(tau)
        if x[2] <= 0:
            continue
        if best is None or e < best[0]:
            best = (e, tau, x)
    if best is None:
        print("**当てはめられません**: どの τ でも熱抵抗が正にならない（温めて抵抗が上がっていない）")
        return 1
    e, tau, (R0, a, b) = best
    Rth = b / (R0 * ALPHA)
    T0w = Ta + a / (R0 * ALPHA)
    print("\n【一次】τ = %.0f s・Rth = %.3f K/W・R0(%.1f℃) = %.4f Ω・始めの巻線 %.1f ℃・残差 %.4f Ω（≈ %.1f K）"
          % (tau, Rth, Ta, R0, T0w, e, e / (R0 * ALPHA)))
    why = r0_why(R0, float(Rv.min()), float(Rv.max()))
    if why is not None:
        print("**推奨は出しません**: " + why)
        return 1

    best2 = None
    taus = [3, 5, 8, 12, 18, 25, 35, 50, 70, 100, 140, 200, 280, 400, 560, 800, 1100, 1600, 2400, 3500]
    H = {tt: resp(tt) for tt in taus}
    for ia_, t1 in enumerate(taus):
        for t2 in taus[ia_ + 1:]:
            h1, g1 = H[t1]
            h2, g2 = H[t2]
            A = np.vstack([np.ones_like(Rt), np.interp(Rt, t, g1), np.interp(Rt, t, g2),
                           np.interp(Rt, t, h1), np.interp(Rt, t, h2)]).T
            x, *_ = np.linalg.lstsq(A, Rv, rcond=None)
            if x[3] < 0 or x[4] < 0:
                continue
            err = float(np.sqrt(np.mean((Rv - A @ x) ** 2)))
            if best2 is None or err < best2[0]:
                best2 = (err, t1, t2, x)
    if best2:
        e2, t1, t2, x = best2
        print("【二次】τ1 = %d s（Rth1 %.3f K/W）・τ2 = %d s（Rth2 %.3f K/W）・残差 %.4f Ω"
              % (t1, x[3] / (x[0] * ALPHA), t2, x[4] / (x[0] * ALPHA), e2))

    Rhot = R0 * (1 + ALPHA * 40)
    for dT_allow in (50.0,):
        dT_ref = dT_allow / 1.2
        Ir = math.sqrt(dT_ref / (3 * Rth * Rhot))
        I_cont = math.sqrt(dT_allow / (1.5 * Rth * Rhot))
        print("\n【ファームへ】許す上昇 %.0f K（周囲 30℃ で巻線 80℃）・遮断 120%%のまま：" % dT_allow)
        print("  温度の時定数 t_tau = %.0f s・定格電流 irated = %.2f A（流し続けてよい電流 ≈ %.2f A）" % (tau, Ir, I_cont))
        for I in (4.0, 5.3, 6.51):
            u = I * I / (2 * Ir * Ir)
            ttrip = tau * math.log(u / (u - 1.2)) if u > 1.2 else float("inf")
            print("   %.2f A で遮断まで %s" % (I, "止まらない" if ttrip == float("inf") else "%.0f 秒" % ttrip))
    print("\nいまのファーム（τ 10 s・定格 2.49 A）の見積もりの最後: %d %%・実測の巻線の上昇の最大 ≈ %.1f K"
          % (th[-1], (Rv.max() / R0 - 1) / ALPHA))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
