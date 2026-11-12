#!/usr/bin/env python3
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import krcbus
import offset_cal as oc
import enc_cal as ec

N_BINS = 36
REST_S = 10.0
GO_FILE = os.path.expanduser("~/offset_full_go")
N_DEF, N_MIN, N_MAX = 3, 1, 20
MODES = ("corr", "off", "gear")
WAVE_N = 360


def usage():
    return ("使い方: python3 offset_full.py <node> [apply [--corr]] [--push] [回数=3]   （node は 1〜%d。"
            "apply＝オフセットだけ入れて補正 39〜42 は 0・apply --corr＝補正 4 つも入れる・保存はしない）" % krcbus.NODE_MAX)


def parse_args(argv):
    a = list(argv[1:])
    apply_ = "apply" in a
    push = "--push" in a
    corr = "--corr" in a
    a = [x for x in a if x not in ("apply", "--push", "--corr")]
    try:
        n = int(a[1]) if len(a) > 1 else N_DEF
    except ValueError:
        n = 0
    if (not a or len(a) > 2 or not krcbus.valid_node(a[0]) or not N_MIN <= n <= N_MAX
            or (corr and not apply_)):
        print(usage())
        raise SystemExit(1)
    return int(a[0]), apply_, ("corr" if corr else "off"), n, push


def ring_mean(samples):
    m = {}
    for s in (+1, -1):
        bins = [[] for _ in range(N_BINS)]
        for c, e, th in samples.get(s, []):
            bins[int(th * N_BINS / 16384) % N_BINS].append(oc.deg(oc.wrap(c - e)))
        if any(not b for b in bins):
            return None
        means = [sum(b) / len(b) for b in bins]
        m[s] = sum(means) / len(means)
    return (m[+1] + m[-1]) / 2.0, (m[+1] - m[-1]) / 2.0


def round_seconds(pp):
    return 2.0 * (pp + ec.REVS_EXTRA) / ec.REV_PER_S + 1.0


def begin(L):
    stt, why = oc.read_state(L)
    if why is not None:
        return None, why
    pp, dr, off, _e0, tr0 = stt
    corr = ec.read_corr(L)
    if corr is None:
        return None, "補正のパラメータ 39〜42 が読めません（古いファームか、tlm の間引きが大きい）"
    return dict(pp=pp, dr=dr, off=off, corr=corr, tr0=tr0), None


def trip_why(tr0):
    if tr0 != 0:
        return "遮断中（理由 %d）なので測れません" % tr0
    return None


FIT_KEYS = ec.CORR_KEYS + ("rms", "max", "width0", "rms0")


def one_round(L, pp, order=(+1, -1)):
    stt, why = oc.read_state(L)
    if why is not None:
        return dict(val=None, lag=None, fit=None, bad=[], why=why)
    e0, tr0 = stt[3], stt[4]
    t_start = time.time()
    samples = ec.measure(L, e0, pp, order=order)
    L.wait(lambda: L.trip(t_start) is not None)
    tr1 = L.trip(t_start)
    fr = ec.fit(samples)
    bad = ec.judge(samples, fr, tr0, tr1)
    rm = ring_mean(samples)
    if rm is None:
        bad.append("θm の 36 区間のどこかに点が無い（1 回転を覆っていない）")
    if bad:
        return dict(val=None, lag=None, fit=None, bad=bad, why=None)
    return dict(val=rm[0], lag=rm[1], fit=dict((k, fr[k]) for k in FIT_KEYS), bad=[], why=None)


def fail_head(k):
    return "%d 回目は測れていないので結論を出しません" % k


def summarize(vals, off, dr, pp, fits=None, corr=None):
    mean = sum(vals) / len(vals)
    sd = (sum((v - mean) ** 2 for v in vals) / (len(vals) - 1)) ** 0.5 if len(vals) > 1 else float("nan")
    d_off = -(mean * 65536.0 / 360.0) / (dr * pp * 4.0)
    new_off = int(round(off + d_off)) % 16384
    r = dict(mean=mean, sd=sd, span=max(vals) - min(vals), d_off=d_off, new_off=new_off)
    if fits:
        r.update(_corr_part(fits, corr))
    return r


def _corr_part(fits, corr):
    def avg(k):
        return sum(f[k] for f in fits) / len(fits)
    add = [avg(k) for k in ec.CORR_KEYS]
    new = ec.corr_add(corr, add)
    wave = [0.1 * sum(c * x for c, x in zip(new, ec.regressors(0, k * 16384.0 / WAVE_N)[2:])) for k in range(WAVE_N)]
    rms_after = avg("rms")
    return dict(corr_now=list(corr), corr_add=add, corr_new=new, corr_bad=ec.corr_range_bad(new),
                rms_now=avg("rms0"), width_now=avg("width0"),
                rms_before=math.sqrt(sum(w * w for w in wave) / WAVE_N + rms_after ** 2),
                width_before=max(wave) - min(wave),
                rms_after=rms_after, max_after=avg("max"))


def corr_why(res):
    if res.get("corr_new") is None:
        return "補正を当てはめていません（オフセットだけなら入れられます）"
    if res["corr_bad"]:
        return "補正は入れられません（%s）。オフセットだけなら入れられます" % "／".join(res["corr_bad"])
    return None


def apply_values(res, mode="corr"):
    if mode == "off":
        return ec.want_values(res["new_off"], [0] * len(krcbus.ENC_CORR_PARAMS))
    if mode not in ("corr", "gear"):
        raise ValueError("入れ方は %s のどれかです（%r）" % ("・".join(MODES), mode))
    why = corr_why(res)
    if why is not None:
        raise ValueError(why)
    want = ec.want_values(res["new_off"], res["corr_new"])
    if mode == "gear":
        del want[16]
    return want


class Stopped(Exception):
    pass


class StopLink(oc.Link):

    def __init__(self, node, sock=None, alive=None):
        oc.Link.__init__(self, node, sock)
        self._mu = threading.RLock()
        self.why = None
        self.alive = alive

    def cfg(self, sub, p1=0, p2=0):
        if sub != 0 and self.alive is not None:
            w = self.alive()
            if w is not None:
                self.stop(w)
        with self._mu:
            if self.why is not None:
                if sub != 0:
                    raise Stopped(self.why)
                return
            oc.Link.cfg(self, sub, p1, p2)

    def stop(self, why):
        with self._mu:
            if self.why is None:
                self.why = str(why)
            for _ in range(3):
                try:
                    oc.Link.cfg(self, 0)
                except OSError:
                    pass


def main(argv):
    node, apply_, mode, n, push = parse_args(argv)
    print("軸 %d の電気角オフセットを 1 周の平均で決めます（%d 回）%s"
          % (node, n, "" if not apply_ else
             "（apply --corr：オフセットと補正 39〜42 を入れる・保存はしない）" if mode == "corr" else
             "（apply：オフセットを入れ、補正 39〜42 を 0 に戻す・保存はしない）"), flush=True)
    L = oc.Link(node)
    st, why = begin(L)
    if why is not None:
        print(why)
        return 1
    pp, dr, off, corr = st["pp"], st["dr"], st["off"], st["corr"]
    print("極対数 %d・向き %+d・いまのオフセット %d・いまの補正 [0.1°] %s（1 周の平均には効かない）"
          % (pp, dr, off, corr), flush=True)
    why = trip_why(st["tr0"])
    if why is not None:
        print(why)
        return 1
    vals, fits = [], []
    for k in range(n):
        if k:
            if push and sys.stdin is not None and sys.stdin.isatty():
                input("ローターを手で押し込み直して、Enter を押してください（%d 回目の前）" % (k + 1))
            elif push:
                if os.path.exists(GO_FILE):
                    os.remove(GO_FILE)
                print("ローターを手で押し込み直してください（%d 回目の前）。済んだら touch %s" % (k + 1, GO_FILE), flush=True)
                while not os.path.exists(GO_FILE):
                    time.sleep(0.2)
                os.remove(GO_FILE)
            else:
                time.sleep(REST_S)
        r = one_round(L, pp)
        if r["why"] is not None:
            print(r["why"])
            return 1
        if r["bad"]:
            print("**%s**: %s" % (fail_head(k + 1), "／".join(r["bad"])))
            return 1
        vals.append(r["val"])
        fits.append(r["fit"])
        print("%d 回目: 1 周の平均のずれ %+.2f°・摩擦の遅れ %.2f°" % (k + 1, r["val"], r["lag"]), flush=True)
        f = r["fit"]
        print("   補正の残り: cos θ %+.2f° sin θ %+.2f°・cos 2θ %+.2f° sin 2θ %+.2f°（当てはめた後の残り RMS %.2f°）"
              % (f["c1"], f["s1"], f["c2"], f["s2"], f["rms"]), flush=True)
    s = summarize(vals, off, dr, pp, fits, corr)
    print("平均 %+.2f°（ばらつき %.2f°・幅 %.2f°）→ オフセット %d → %d（%+.1f カウント）"
          % (s["mean"], s["sd"], s["span"], off, s["new_off"], s["d_off"]), flush=True)
    print("補正 [0.1°]: " + "・".join("%s %d → %d" % (krcbus.PARAMS[p][0], a, b)
                                     for p, a, b in zip(krcbus.ENC_CORR_PARAMS, s["corr_now"], s["corr_new"])))
    print("ずれ（電気角）: 補正なし 幅 %.1f°・RMS %.1f° → 補正後の残り RMS %.1f°（最大 %.1f°）%s"
          % (s["width_before"], s["rms_before"], s["rms_after"], s["max_after"],
             "" if not any(s["corr_now"]) else "（いまの補正のまま: 幅 %.1f°・RMS %.1f°）" % (s["width_now"], s["rms_now"])),
          flush=True)
    cwhy = corr_why(s)
    if cwhy is not None:
        print("**%s**" % cwhy)
    if apply_:
        if mode == "corr" and cwhy is not None:
            print("apply --corr はしません（何も入れていません）")
            return 1
        if not ec.apply_and_readback(L, apply_values(s, mode)):
            return 1
        print("残すなら python3 krc.py -n %d param save" % node)
        return 0
    print("（入れるなら offset_full.py %d %s。保存は python3 krc.py -n %d param save）"
          % (node, "apply（オフセットだけ・補正は 0）" if cwhy is not None else
             "apply --corr（補正 4 つも）か apply（オフセットだけ・補正は 0）", node))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
