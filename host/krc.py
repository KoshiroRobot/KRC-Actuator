#!/usr/bin/env python3
"""KRC FOC Driver v2 - 立ち上げ・同定・制御のツール

python-can は使わない。Linux の SocketCAN を素の socket で叩く。

  ./krc.py [-n ノードID] <サブコマンド> ...

【立ち上げ】1台ずつバスに繋いだ状態で
  provision <新ID>   ノードIDを振る → 自動整列 → 較正をフラッシュに保存（全自動）
  zero               いまの位置を0にする（機構の基準姿勢で）
  limit <下限> <上限> 可動範囲[rad]を設定（壁を外すなら limit off）
  home [減速比]       原点出し：ストッパに当てて可動範囲を決め、真ん中を原点に、壁を置く
  save               設定をフラッシュに保存
  param              CAN から変えられるパラメータを一覧（基板の読み返し）
  param <名前|番号> <値>  パラメータを 1 つ変える（例: param imax 6.5）。残すなら param save
  param default      パラメータを全部既定に戻す（残すなら param save）
  param save         パラメータだけをフラッシュに焼く（cfg 23。較正と壁はそのまま・焼けたかを確かめる）
  status             いまの状態を表示

【同定】組み上がった実機で1回まわす
  identify           R / L / Ke / Kt / J / 摩擦 を測って推奨ゲインまで出す
  ripple  [rad/s]    トルクリップルの空間周波数を出す（正体の切り分け）
  noise              静止・回転中の電流の暴れ方と、測定ノイズの床
  sinenoise [振幅 周波数 kp kd]  正弦波で動かしているときの電流
  gains   <帯域Hz>   電流ループの帯域を変える（音を耳で比べるため）

【制御】
  enter / exit       モーターモードに入る / 出る
  hold  <kp> <kd> <秒>          いまの位置で保持する
  step  <rad> <kp> <kd> <秒>    いまの位置から <rad> だけ動かす
  vel   <rad/s> <kd> <秒>
  torque <N·m> <秒>

  ⚠ hold と step は**始める前にいまの位置を原点にする**（0xFE）。
    2026-09-07 まで、どこに居るかを見ずにいきなり目標 0 rad を送っていたので、
    連続回転させたあとに打つと 33 rad 離れたところから全力で戻った（§7-18）。
    sine / noise / sinenoise / identify は元から原点を取り直している。

【安全】
  estop              一斉停止（全ノード・ラッチする）
  clear              遮断の解除

⚠ ノードIDを振る前の基板は全部 id=1。
  未設定の基板を2台以上同じバスに繋がないこと。1台ずつ provision する。
"""
import math
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from krcbus import pos_gate, valid_node, limits_lsb, NODE_MAX, RAD_PER_COUNT
from krcbus import PARAMS, PARAM_BY_KEY, param_lsb, param_value, params_from_tlm13, CFG_SET_PARAM
from krcbus import PARAM_PAGES, TLM_LOOP_S, PARAM_SAVE_TEXT, param_save_result, tlm13_wait_s
from krcbus import PARAMS_NO_DEFAULT, COAST_WAIT_S, COAST_STOP_RPM, COAST_MAX_MS, coast_still
from krcbus import selftest_why, cal_from_tlm8, cal_why
from krcbus import SCALE_PARAMS, SCALE_LEGACY, ScaleRead
from krcbus import board_mode_why, BOARD_MODE_WAIT, t_max_why

NODE = 1

P_MAX = 12.5
V_MAX = 100.0
KP_MAX = 500.0
KD_MAX = 5.0
T_MAX = 3.0
I_MAX = 10.0

RATE = 100.0
CAP_N = 201
FS = None
MA_PER_LSB = 20.14


def id_estop():
    return 0x080


def id_imp():
    return 0x100 + NODE


def id_cfg():
    return 0x120 + NODE


def id_duty():
    return 0x140 + NODE


def id_fb():
    return 0x200 + NODE


def id_tlm(n):
    return 0x300 + NODE * 0x10 + n


sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
sock.bind(("can0",))
sock.setblocking(False)


def send(cid, data):
    sock.send(struct.pack("=IB3x8s", cid, len(data), data.ljust(8, b"\x00")))


def recv_all(secs, want=None):
    out = {}
    end = time.time() + secs
    while time.time() < end:
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0003)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        cid &= 0x1FFFFFFF
        if want is not None and cid not in want:
            continue
        out.setdefault(cid, []).append(frame[8:8 + dlc])
    return out


def tlm(n, secs=0.4):
    got = recv_all(secs, want={id_tlm(n)})
    v = got.get(id_tlm(n))
    return v[-1] if v else None


def collect(ns, secs, feed=None, feed_hz=10.0):
    want = {id_tlm(n): n for n in ns}
    got = {}
    end = time.time() + secs
    nxt = 0.0
    while time.time() < end:
        now = time.time()
        if feed is not None and now >= nxt:
            feed()
            nxt = now + 1.0 / feed_hz
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0005)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        cid &= 0x1FFFFFFF
        if cid in want:
            got[want[cid]] = frame[8:8 + dlc]
    return got


def u16(d, i):
    return (d[i] << 8) | d[i + 1]


def s16(d, i):
    v = u16(d, i)
    return v - 65536 if v > 32767 else v


def s32(d, i):
    v = (d[i] << 24) | (d[i + 1] << 16) | (d[i + 2] << 8) | d[i + 3]
    return v - (1 << 32) if v >= (1 << 31) else v


def pack(x, lo, hi, bits):
    x = max(lo, min(hi, x))
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def unpack(v, lo, hi, bits):
    return v * (hi - lo) / ((1 << bits) - 1) + lo


def mit_cmd(p, v, kp, kd, t):
    pi = pack(p, -P_MAX, P_MAX, 16)
    vi = pack(v, -V_MAX, V_MAX, 12)
    ki = pack(kp, 0.0, KP_MAX, 12)
    di = pack(kd, 0.0, KD_MAX, 12)
    ti = pack(t, -T_MAX, T_MAX, 12)
    return bytes([pi >> 8, pi & 0xFF,
                  vi >> 4, ((vi & 0xF) << 4) | (ki >> 8), ki & 0xFF,
                  di >> 4, ((di & 0xF) << 4) | (ti >> 8), ti & 0xFF])


def parse_fb(d):
    pi = (d[1] << 8) | d[2]
    vi = (d[3] << 4) | (d[4] >> 4)
    ii = ((d[4] & 0xF) << 8) | d[5]
    return (unpack(pi, -P_MAX, P_MAX, 16),
            unpack(vi, -V_MAX, V_MAX, 12),
            unpack(ii, -I_MAX, I_MAX, 12))


def poll_fb(deadline):
    fb = None
    while time.time() < deadline:
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0003)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        if (cid & 0x1FFFFFFF) == id_fb():
            fb = parse_fb(frame[8:8 + dlc])
    return fb


def special(code):
    send(id_imp(), bytes([0xFF] * 7 + [code]))
    return poll_fb(time.time() + 0.05)


def pos_true(secs=0.4):
    d = tlm(6, secs)
    if d is None or len(d) < 4:
        return None
    c = (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]
    if c >= (1 << 31):
        c -= (1 << 32)
    return c * RAD_PER_COUNT


def enter_pos_mode(label):
    fb = special(0xFC)
    pt = pos_true()
    _p, why = pos_gate(pt, fb[0] if fb else 0.0, fb is not None)
    if why is not None:
        special(0xFD)
        print("**%s に入りません**: %s" % (label, why))
        print("  → テレメトリを有効にする（./krc.py tlm 1）か、")
        print("     機構を安全な姿勢にしてから ./krc.py zero を打つこと")
        return False

    special(0xFE)

    fbz = special(0xFC)
    if (fbz is None) or (abs(fbz[0]) > 0.5):
        special(0xFD)
        print("**%s に入りません**: 原点を取り直したのに位置が %s のまま"
              % (label, "読めない（返事が来ない）" if fbz is None
                 else "%.2f rad" % fbz[0]))
        return False
    print("  いまの位置を原点にしました（目標 0 rad = いまの位置）")
    return True


def cfg(sub, p1=0, p2=0):
    p1 &= 0xFFFF
    p2 &= 0xFFFF
    send(id_cfg(), bytes([0xA5, sub, p1 >> 8, p1 & 0xFF,
                          p2 >> 8, p2 & 0xFF, 0, 0]))


def imp_hold(p, v, kp, kd, t, secs, log=None):
    dt = 1.0 / RATE
    t0 = time.time()
    for i in range(max(1, int(secs * RATE))):
        send(id_imp(), mit_cmd(p, v, kp, kd, t))
        fb = poll_fb(t0 + (i + 1) * dt)
        if log is not None and fb:
            log.append((time.time() - t0, fb))


def capture(src, div, arm_fn, secs=3.0):
    cfg(14, src, div)
    time.sleep(0.05)
    arm_fn()
    got = {}
    end = time.time() + secs
    wp = None
    while time.time() < end and len(got) < CAP_N:
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0003)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        if (cid & 0x1FFFFFFF) != id_tlm(9):
            continue
        d = frame[8:8 + dlc]
        idx = u16(d, 0)
        if idx == 0xFFFF:
            wp = u16(d, 2)
            continue
        for j in range(3):
            if idx + j < CAP_N:
                got[idx + j] = s16(d, 2 + j * 2)
    if len(got) < CAP_N:
        return None, None
    return [got[i] for i in range(CAP_N)], wp


def sync_fs():
    global FS
    d = tlm(12)
    FS = d[5] * 1000.0 if (d and 10 <= d[5] <= 100) else None
    return FS


def require_fs(step):
    if sync_fs() is None:
        print("  !! 制御周期が基板から読めない（テレメトリ 0x30C が来ない）。"
              "%s を中止する" % step)
        print("     ./krc.py tlm 1 でテレメトリを有効にしてから、もう一度")
        sys.exit(1)
    return FS


def trip_reason():
    d = tlm(4)
    return d[6] if d else None


def pole_pairs():
    d = tlm(8)
    if d is None:
        return None
    pp = d[2] & 0x7F
    return pp if pp else None


def require_ok(step):
    r = trip_reason()
    if r is None:
        print("  !! %s の途中で、遮断しているかどうかが読めない"
              "（テレメトリ 0x304 が来ない）。中止する" % step)
        print("     ./krc.py tlm 1 でテレメトリを有効にしてから、もう一度")
        sys.exit(1)
    if r:
        print("  !! %s の途中で遮断した（理由 %d）。中止する" % (step, r))
        print("     clear で解除してから、原因を確認すること")
        sys.exit(1)


def measure_R():
    pts = []
    for mv in (1000, 2000, 3000, 4000):
        for _ in range(8):
            cfg(1, mv, 0)
            time.sleep(0.1)
        got = collect([5], 0.4, feed=lambda mv=mv: cfg(1, mv, 0))
        if 5 in got:
            pts.append((mv / 1000.0, s16(got[5], 6) / 1000.0))
    cfg(0)
    require_ok("相抵抗の測定")
    if len(pts) < 3:
        return None, pts
    n = len(pts)
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sxx = sum(p[0] * p[0] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    slope = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    return (1.0 / slope if slope > 0 else None), pts


def measure_L(R):
    buf, _ = capture(0, 1, lambda: cfg(9, 3000, 0))
    cfg(0)
    require_ok("インダクタンスの測定")
    if buf is None:
        return None, None
    final = sum(buf[150:200]) / 50.0
    if final < 200:
        return None, None
    target = 0.632 * final
    k = None
    for i in range(1, CAP_N):
        if buf[i] >= target:
            prev = buf[i - 1]
            frac = (target - prev) / max(1.0, (buf[i] - prev))
            k = (i - 1) + frac
            break
    if k is None:
        return None, None
    tau = (k - 1.0) / FS
    return (tau * R if R else None), tau


def travel_rad():
    d = tlm(12)
    if d is None:
        return None, False
    lo, hi = s16(d, 0) / 100.0, s16(d, 2) / 100.0
    return (None if lo >= hi else (hi - lo)), True


def vel_burst(w, secs, kd=0.1):
    special(0xFC)
    time.sleep(0.05)
    log = []
    imp_hold(0.0, w, 0.0, kd, 0.0, secs, log=log)
    got = collect([5, 7], 0.25,
                  feed=lambda: send(id_imp(), mit_cmd(0.0, w, 0.0, kd, 0.0)))
    special(0xFD)
    tail = [f for (t, f) in log if t > secs * 0.55]
    if len(tail) < 4 or not all(k in got for k in (5, 7)):
        return None
    return (sum(f[1] for f in tail) / len(tail),
            s16(got[7], 2) / 1000.0,
            sum(f[2] for f in tail) / len(tail))


def sweep(speeds, tr):
    secs = 1.0
    speeds = list(speeds)

    if tr is not None:
        wmax = (tr * 0.35) / secs
        scale = min(1.0, wmax / max(speeds))
        speeds = [w * scale for w in speeds]

    out = []
    for w in speeds:
        a = vel_burst(+w, secs)
        time.sleep(0.4)
        b = vel_burst(-w, secs)
        time.sleep(0.4)
        require_ok("速度掃引")
        if a and b:
            out.append((a, b))
    return out


def measure_Ke(R, sw):
    best = None
    for a, b in sw:
        dw = a[0] - b[0]
        if abs(dw) < 6.0:
            continue
        ke = ((a[1] - b[1]) - R * (a[2] - b[2])) / dw
        if ke > 0 and (best is None or abs(dw) > abs(best[1])):
            best = (ke, dw, a, b)
    if best is None:
        return None, None
    return best[0], best


def run_and_capture_fn(cmdfn, secs):
    got = {}
    dt = 1.0 / RATE
    t0 = time.time()
    nxt = 0.0
    while time.time() < t0 + secs:
        now = time.time()
        if now >= nxt:
            send(id_imp(), cmdfn(now - t0))
            nxt = now + dt
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0003)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        if (cid & 0x1FFFFFFF) != id_tlm(9):
            continue
        d = frame[8:8 + dlc]
        idx = u16(d, 0)
        if idx == 0xFFFF:
            continue
        for j in range(3):
            if idx + j < CAP_N:
                got[idx + j] = s16(d, 2 + j * 2)
    return got


def run_and_capture(p, v, kp, kd, t, secs):
    got = {}
    dt = 1.0 / RATE
    t0 = time.time()
    nxt = 0.0
    while time.time() < t0 + secs:
        now = time.time()
        if now >= nxt:
            send(id_imp(), mit_cmd(p, v, kp, kd, t))
            nxt = now + dt
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0003)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        if (cid & 0x1FFFFFFF) != id_tlm(9):
            continue
        d = frame[8:8 + dlc]
        idx = u16(d, 0)
        if idx == 0xFFFF:
            continue
        for j in range(3):
            if idx + j < CAP_N:
                got[idx + j] = s16(d, 2 + j * 2)
    return got


def measure_J(Kp, step=0.05):
    for div in (20, 60):
        for kd in (0.02, 0.01, 0.004):
            special(0xFC)
            time.sleep(0.1)
            special(0xFE)
            time.sleep(0.1)
            imp_hold(0.0, 0.0, Kp, kd, 0.0, 0.5)

            cfg(14, 4, div)
            time.sleep(0.05)
            cfg(16)
            got = run_and_capture(step, 0.0, Kp, kd, 0.0,
                                  0.4 + CAP_N * div / FS)
            special(0xFD)
            require_ok("慣性の測定")
            if len(got) < CAP_N:
                continue
            y = [got[i] for i in range(CAP_N)]

            final = sum(y[150:200]) / 50.0
            amp = final - y[0]
            if abs(amp) < 10:
                continue
            pk_i = max(range(1, 150), key=lambda i: y[i])
            os_ratio = (y[pk_i] - final) / amp
            if os_ratio < 0.02:
                continue
            tr_i = min(range(pk_i, min(pk_i + 100, CAP_N)),
                       key=lambda i: y[i])
            dt_s = div / FS
            half_period = (tr_i - pk_i) * dt_s
            if half_period <= 2.0 * dt_s:
                continue
            lz = math.log(os_ratio)
            zeta = -lz / math.sqrt(math.pi ** 2 + lz * lz)
            wn = (math.pi / half_period) / math.sqrt(max(1e-6, 1 - zeta * zeta))
            return Kp / (wn * wn), dict(kd=kd, div=div, overshoot=os_ratio,
                                        zeta=zeta, wn=wn, y=y)
    return None, None


def measure_friction(Kt, sw):
    pts = []
    for a, b in sw:
        wm = (abs(a[0]) + abs(b[0])) / 2.0
        tq = (a[2] - b[2]) * Kt / 2.0
        pts.append((wm, tq))
    if len(pts) < 2:
        return None, None, pts
    n = len(pts)
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sxx = sum(p[0] * p[0] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    den = n * sxx - sx * sx
    b = (n * sxy - sx * sy) / den
    tc = (sy - b * sx) / n
    return tc, b, pts


def do_identify():
    require_fs("同定")
    print("=== KRC アクチュエータ 同定（ノード %d、制御 %.0f kHz）===" % (NODE, FS / 1000.0))
    d = tlm(8)
    if d is None:
        print("テレメトリが来ない。ノードIDと配線を確認すること")
        return 1
    if not (d[2] & 0x80):
        print("!! フラッシュに較正が無い。先に provision を済ませること")
        return 1
    r = trip_reason()
    if r is None:
        print("!! 遮断しているかどうかが読めない（テレメトリ 0x304）。中止する")
        return 1
    if r:
        print("!! 遮断中。clear してから実行すること")
        return 1
    print("   極対数 %d / エンコーダの向き %+d / オフセット %d"
          % (d[2] & 0x7F, -1 if d[3] == 0xFF else 1, u16(d, 0)))
    print()

    R, rpts = measure_R()
    if R is None:
        print("[1/5] 相抵抗      測定できず %s" % (rpts,))
        return 1
    print("[1/5] 相抵抗      R  = %.3f Ω" % R)
    print("      " + "  ".join("%.0fV→%.2fA" % p for p in rpts))

    L, tau = measure_L(R)
    if L is None:
        print("[2/5] インダクタンス 測定できず（電流が立ち上がらない）")
        return 1
    print("[2/5] インダクタンス L  = %.3f mH   (時定数 %.0f µs)"
          % (L * 1000.0, tau * 1e6))

    tr, tr_known = travel_rad()
    if not tr_known:
        print("      可動範囲が読めない（テレメトリ 0x30C が来ない）。"
              "壁があるかどうか分からないまま回すのは危ないので中止する")
        return 1
    print("      可動範囲 %s → 速度掃引は%s"
          % ("%.2f rad" % tr if tr else "無効",
             "その中で往復する" if tr else "往復（片道で回さない）"))
    sw = sweep((8.0, 16.0, 24.0), tr)
    if len(sw) < 2:
        print("[3/5] 逆起電力    測定できず（回らなかった）")
        return 1

    Ke, kr = measure_Ke(R, sw)
    if Ke is None:
        print("[3/5] 逆起電力    測定できず（往復の速度差が足りない）")
        if tr is not None:
            print("      可動範囲 %.1f rad ではロータを十分に回せない。" % tr)
            print("      いったん `limit 0 0` で範囲を外して測り、あとで戻すこと")
        return 1
    Kt = 1.5 * Ke
    print("[3/5] 逆起電力    Ke = %.4f V·s/rad   (+%.1f / %.1f rad/s の差分から)"
          % (Ke, kr[2][0], kr[3][0]))
    print("      トルク定数  Kt = %.4f N·m/A" % Kt)

    J, jd = measure_J(10.0)
    if J is None:
        print("[4/5] 慣性        測定できず（行き過ぎが読めない）")
    else:
        print("[4/5] 慣性        J  = %.3g kg·m²   "
              "(Kd=%.3f, 行き過ぎ %.1f%%, ζ=%.2f, ωn=%.0f rad/s)"
              % (J, jd["kd"], jd["overshoot"] * 100, jd["zeta"], jd["wn"]))

    tc, b, fpts = measure_friction(Kt, sw)
    if tc is None:
        print("[5/5] 摩擦        測定できず %s" % (fpts,))
    else:
        print("[5/5] 摩擦        クーロン %.4f N·m / 粘性 %.5f N·m·s/rad" % (tc, b))
        print("      " + "  ".join("%.1frad/s→%.3fN·m" % p for p in fpts))

    print()
    print("--- 推奨ゲイン ---")
    w_i = 2 * math.pi * 800.0
    print("電流ループ 800Hz : Kp = %.2f V/A (=%d mV/A), Ki = %.0f V/(A·s)"
          % (L * w_i, int(L * w_i * 1000), R * w_i))
    print("  → cfg 7 で送る:  cansend can0 %03X#A507%04X%04X0000"
          % (id_cfg(), int(L * w_i * 1000) & 0xFFFF, int(R * w_i) & 0xFFFF))
    if J:
        for f in (20.0, 50.0):
            wn = 2 * math.pi * f
            kp = J * wn * wn
            kd = 2.0 * J * wn - (b if b else 0.0)
            print("位置ループ %2.0fHz ζ=1 : Kp = %.2f N·m/rad, Kd = %.4f N·m·s/rad"
                  % (f, kp, max(0.0, kd)))
        print("  ※ 速度オブザーバは 400Hz。位置ループはその 1/5 以下に留めること")
    return 0


def do_provision(new_id):
    global NODE
    print("=== 立ち上げ（現在のID %d → %d）===" % (NODE, new_id))
    if tlm(0) is None:
        print("!! テレメトリが来ない。ID %d の基板がバスに居ない" % NODE)
        return 1
    d1 = tlm(1)
    if d1 is None:
        print("!! 自己確認の印が読めない（テレメトリ 0x%03X が来ない）。何も変えずに中止" % id_tlm(1))
        return 1
    why = selftest_why(d1[6])
    if why:
        print("!! 自己確認 フラグ 0x%02X **異常**：%s。何も変えずに中止" % (d1[6], "／".join(why)))
        return 1

    cfg(17, new_id)
    time.sleep(0.3)
    NODE = new_id
    if tlm(0) is None:
        print("!! ID %d に変わらなかった" % new_id)
        return 1
    print("[1/4] ノードID %d に変更（テレメトリ 0x%03X）" % (new_id, id_tlm(0)))

    print("[2/4] 自動整列 … ロータが約1秒動く")
    cfg(11)
    time.sleep(2.5)
    r = trip_reason()
    if r is None:
        print("!! 整列のあと、遮断しているかどうかが読めない（テレメトリ 0x304）。中止")
        return 1
    if r:
        print("!! 整列中に遮断した。中止")
        return 1

    cfg(12)
    time.sleep(0.8)
    d = tlm(8)
    if d is None or not (d[2] & 0x80):
        print("!! フラッシュへの保存に失敗")
        return 1
    print("[3/4] 保存完了  オフセット %d / 極対数 %d / 向き %+d"
          % (u16(d, 0), d[2] & 0x7F, -1 if d[3] == 0xFF else 1))

    d1 = tlm(1)
    if d1 is None:
        print("[4/4] 自己確認  **読めない**（テレメトリ 0x%03X が来ない）" % id_tlm(1))
        return 1
    why = selftest_why(d1[6])
    print("[4/4] 自己確認  フラグ 0x%02X %s" % (d1[6], "OK" if not why else "**異常**：" + "／".join(why)))
    if why:
        return 1
    print()
    print("次: 機構を基準姿勢にして  ./krc.py -n %d zero" % new_id)
    print("    可動範囲を決めて      ./krc.py -n %d limit <下限> <上限>" % new_id)
    print("    保存                  ./krc.py -n %d save" % new_id)
    return 0


def do_status():
    for n, name in ((0, "生存"), (1, "疎通"), (4, "熱/ISR"), (6, "位置"),
                    (8, "較正"), (12, "可動範囲")):
        d = tlm(n)
        print("  0x%03X %-8s %s" % (id_tlm(n), name,
                                    d.hex(" ") if d else "(来ない)"))
    d = tlm(1)
    if d:
        why = selftest_why(d[6])
        print("  → 自己確認 %s" % ("OK" if not why else "**異常**：" + "／".join(why)))
    d = tlm(8)
    if d:
        print("  → 較正 %s / 極対数 %d / 向き %+d / Kp %d mV/A / Ki %d"
              % ("あり" if d[2] & 0x80 else "**なし**", d[2] & 0x7F,
                 -1 if d[3] == 0xFF else 1, u16(d, 4), u16(d, 6)))
    d = tlm(12)
    if d:
        lo, hi = s16(d, 0) / 100.0, s16(d, 2) / 100.0
        print("  → 可動範囲 %s"
              % ("無効" if lo >= hi else "%.2f 〜 %.2f rad" % (lo, hi)))
    r = trip_reason()
    if r is None:
        print("  → 遮断しているかどうかは読めない（テレメトリ 0x304 が来ない）")
    elif r:
        print("  → **遮断中**（理由 %d）。clear で解除" % r)
    return 0


def run(p, v, kp, kd, t, secs, label, every=5, settle=0.0, arm=None):
    if kp > 0.0 and not enter_pos_mode(label):
        return 1
    dt = 1.0 / RATE
    print("t[s]   pos[rad]  vel[rad/s]  iq[A]   （pos は ±%.1f rad で飽和）" % P_MAX)
    t0 = time.time()
    for i in range(int(settle * RATE)):
        send(id_imp(), mit_cmd(0.0, 0.0, kp, kd, 0.0))
        poll_fb(t0 + (i + 1) * dt)
    if arm is not None:
        cfg(14, arm[0], arm[1])
        cfg(16)
    t1 = time.time()
    for i in range(int(secs * RATE)):
        send(id_imp(), mit_cmd(p, v, kp, kd, t))
        fb = poll_fb(t1 + (i + 1) * dt)
        if fb and (i % every == 0):
            print("%5.2f  %8.4f  %9.3f  %6.3f"
                  % (time.time() - t1, fb[0], fb[1], fb[2]))
    print("--- %s 終了 ---" % label)
    return 0


def do_sine(A, freq, kp, kd, secs, quiet=False):
    w = 2.0 * math.pi * freq
    dt = 1.0 / RATE
    log = []
    special(0xFC)
    time.sleep(0.05)
    special(0xFE)
    time.sleep(0.1)

    t0 = time.time()
    n = int(secs * RATE)
    for i in range(n):
        t = i * dt
        pc = A * math.sin(w * t)
        vc = A * w * math.cos(w * t)
        send(id_imp(), mit_cmd(pc, vc, kp, kd, 0.0))
        fb = poll_fb(t0 + (i + 1) * dt)
        if fb:
            log.append((t, pc, fb[0], fb[1], fb[2]))
    special(0xFD)
    require_ok("正弦波追従")

    use = [r for r in log if r[0] > 1.0 / freq]
    if len(use) < 20:
        print("  データが足りない")
        return None

    def bin_at(vals, ts):
        re = sum(v * math.cos(w * t) for v, t in zip(vals, ts))
        im = sum(-v * math.sin(w * t) for v, t in zip(vals, ts))
        return complex(re, im)

    ts = [r[0] for r in use]
    C = bin_at([r[1] for r in use], ts)
    P = bin_at([r[2] for r in use], ts)
    gain = abs(P) / abs(C) if abs(C) > 0 else 0.0
    phase = math.degrees(math.atan2((P / C).imag, (P / C).real)) if abs(C) > 0 else 0.0
    err = [r[2] - r[1] for r in use]
    rms = math.sqrt(sum(e * e for e in err) / len(err))
    peak_i = max(abs(r[4]) for r in use)

    if not quiet:
        print("   t[s]  指令[rad]  実測[rad]  速度[rad/s]  iq[A]")
        for r in use[:int(RATE / freq / 6) * 6:max(1, int(RATE / freq / 12))]:
            print("  %5.2f  %8.4f  %9.4f  %10.2f  %6.3f" % r)
    print("  → %4.1f Hz  振幅 %.0f%%  位相 %+.1f°  誤差RMS %.4f rad  最大電流 %.2f A"
          % (freq, gain * 100, phase, rms, peak_i))
    return gain, phase, rms


def spectrum(y, fs, top=6):
    n = len(y)
    m = sum(y) / n
    y = [v - m for v in y]
    out = []
    for k in range(1, n // 2):
        re = sum(y[i] * math.cos(2 * math.pi * k * i / n) for i in range(n))
        im = sum(y[i] * math.sin(2 * math.pi * k * i / n) for i in range(n))
        out.append((math.hypot(re, im) * 2.0 / n, k * fs / n))
    out.sort(reverse=True)
    return out[:top]


def cap_stats(y, label, fs=None):
    fs = FS if fs is None else fs
    n = len(y)
    m = sum(y) / n
    rms = math.sqrt(sum((v - m) ** 2 for v in y) / n)
    pp = max(y) - min(y)
    print("  %-14s 平均 %6.0f mA / 変動RMS %5.1f mA / p-p %5.0f mA"
          % (label, m, rms, pp))
    sp = spectrum(y, fs)
    print("       主な成分: " + "  ".join("%.0fHz:%.0fmA" % (f, a) for a, f in sp))
    return rms


def do_noise(kp=18.0, kd=0.12, w=20.0):
    require_fs("騒音の測定")
    print("=== 電流の暴れ方（%.0fkHz記録・201点=%.1fms）==="
          % (FS / 1000.0, CAP_N / FS * 1000.0))

    special(0xFC)
    time.sleep(0.05)
    imp_hold(0.0, 0.0, 0.0, 0.0, 0.0, 0.5)
    cfg(14, 1, 1)
    time.sleep(0.05)
    cfg(16)
    got = run_and_capture(0.0, 0.0, 0.0, 0.0, 0.0, 1.2)
    special(0xFD)
    if len(got) >= CAP_N:
        cap_stats([got[i] for i in range(CAP_N)], "無トルク(床)")
    time.sleep(0.8)

    special(0xFC)
    time.sleep(0.05)
    special(0xFE)
    time.sleep(0.1)
    imp_hold(0.0, 0.0, kp, kd, 0.0, 0.6)
    cfg(14, 1, 1)
    time.sleep(0.05)
    cfg(16)
    got = run_and_capture(0.0, 0.0, kp, kd, 0.0, 1.2)
    special(0xFD)
    if len(got) >= CAP_N:
        cap_stats([got[i] for i in range(CAP_N)], "静止保持中")
    else:
        print("  静止保持中     記録できず")
    time.sleep(1.0)

    special(0xFC)
    time.sleep(0.05)
    imp_hold(0.0, w, 0.0, 0.1, 0.0, 0.8)
    cfg(14, 1, 1)
    time.sleep(0.05)
    cfg(16)
    got = run_and_capture(0.0, w, 0.0, 0.1, 0.0, 1.2)
    special(0xFD)
    require_ok("騒音の測定")
    if len(got) >= CAP_N:
        cap_stats([got[i] for i in range(CAP_N)], "%.0f rad/s 回転中" % w)
        pp = pole_pairs()
        if pp is None:
            print("       （極対数が読めないので電気周波数は出さない）")
        else:
            fe = w / (2 * math.pi) * pp
            print("       （このとき電気周波数は %.0f Hz、その6倍が %.0f Hz）"
                  % (fe, fe * 6))
    else:
        print("  回転中         記録できず")
    return 0


def do_sinenoise(A=1.0, freq=2.0, kp=18.0, kd=0.12):
    require_fs("正弦波中の測定")
    w = 2.0 * math.pi * freq
    print("=== 正弦波で動かしているときの電流（振幅 %.1f rad, %.1f Hz）===" % (A, freq))

    def cmd(t):
        return mit_cmd(A * math.sin(w * t), A * w * math.cos(w * t), kp, kd, 0.0)

    for div, label in ((int(FS / freq / CAP_N), "1周期の包絡"), (1, "細部")):
        div = max(1, div)
        special(0xFC)
        time.sleep(0.05)
        special(0xFE)
        time.sleep(0.1)

        got = {}
        armed = False
        t_arm = 0.0
        dt = 1.0 / RATE
        runup = 1.0 / freq
        total = runup + 0.3 + CAP_N * div / FS
        t0 = time.time()
        nxt = 0.0
        while True:
            t = time.time() - t0
            if t >= total:
                break
            if t >= nxt:
                send(id_imp(), cmd(t))
                nxt = t + dt
            if (not armed) and (t >= runup):
                cfg(14, 1, div)
                cfg(16)
                t_arm = t
                armed = True
            try:
                frame = sock.recv(16)
            except BlockingIOError:
                time.sleep(0.0003)
                continue
            cid, dlc = struct.unpack("=IB3x", frame[:8])
            if (cid & 0x1FFFFFFF) != id_tlm(9):
                continue
            d = frame[8:8 + dlc]
            idx = u16(d, 0)
            if idx == 0xFFFF:
                continue
            for j in range(3):
                if idx + j < CAP_N:
                    got[idx + j] = s16(d, 2 + j * 2)

        special(0xFD)
        require_ok("正弦波中の測定")
        if len(got) < CAP_N:
            print("  %s: 記録できず" % label)
            continue
        y = [got[i] for i in range(CAP_N)]
        span = CAP_N * div / FS
        print("  %s（%.0f ms ぶん、%.0f Hz サンプル）" % (label, span * 1000, FS / div))
        cap_stats(y, "  iq", fs=FS / div)

        if div > 1:
            m = 8
            seg = CAP_N // m
            ph0 = math.degrees(w * t_arm) % 360.0
            rough = []
            for i in range(m):
                s = y[i * seg:(i + 1) * seg]
                d = [s[j + 1] - s[j] for j in range(len(s) - 1)]
                ph = (ph0 + 360.0 * (i + 0.5) / m) % 360.0
                rough.append((ph, math.sqrt(sum(v * v for v in d) / len(d))))
            rough.sort()
            print("       位相ごとの荒さ（隣接差のRMS[mA]、0°=位置0で速度+最大）:")
            print("       " + "  ".join("%3.0f°:%3.0f" % p for p in rough))
        time.sleep(0.8)
    return 0


def ripple_label(cyc, pp):
    if pp is None:
        return "偏心（1回転に1回）" if cyc < 3 else "?（極対数が読めない）"
    if abs(cyc - 6 * pp) < 12:
        return "電気6次高調波"
    if abs(cyc - 12 * pp) < 20:
        return "電気12次高調波"
    if cyc < 3:
        return "偏心（1回転に1回）"
    if abs(cyc - 2 * pp) < 6:
        return "電気基本波（磁極）"
    return "コギングか機械系"


def do_ripple(w=5.0, kd=0.15):
    require_fs("リップルの測定")
    pp = pole_pairs()
    div = max(1, int(FS * 0.2 / CAP_N))
    fs_eff = FS / div
    rev_s = abs(w) / (2.0 * math.pi)
    print("=== トルクリップルの正体（%.1f rad/s = %.3f 回転/秒）===" % (w, rev_s))
    print("    窓 %.0f ms / %.0f Hz サンプル / 分解能 %.1f 周期・回転"
          % (CAP_N * div / FS * 1000.0, fs_eff, (fs_eff / CAP_N) / rev_s))

    special(0xFC)
    time.sleep(0.05)
    imp_hold(0.0, w, 0.0, kd, 0.0, 1.0)
    cfg(14, 1, div)
    time.sleep(0.03)
    cfg(16)
    got = run_and_capture_fn(lambda t: mit_cmd(0.0, w, 0.0, kd, 0.0),
                             0.3 + CAP_N * div / FS)
    special(0xFD)
    require_ok("リップルの測定")
    if len(got) < CAP_N:
        print("  記録できず")
        return 1

    y = [got[i] for i in range(CAP_N)]

    def amp_at(cycles_per_rev):
        f = cycles_per_rev * rev_s
        k = int(round(f * CAP_N / fs_eff))
        if k < 1 or k >= CAP_N // 2:
            return None
        m = sum(y) / len(y)
        yy = [v - m for v in y]
        best = 0.0
        for kk in (k - 1, k, k + 1):
            if kk < 1 or kk >= CAP_N // 2:
                continue
            re = sum(yy[i] * math.cos(2 * math.pi * kk * i / CAP_N)
                     for i in range(CAP_N))
            im = sum(yy[i] * math.sin(2 * math.pi * kk * i / CAP_N)
                     for i in range(CAP_N))
            best = max(best, math.hypot(re, im) * 2.0 / CAP_N)
        return best

    if pp is None:
        print("    極対数が読めない（テレメトリ 0x308）ので、電気次数は出さない")
    else:
        a6 = amp_at(6 * pp)
        a12 = amp_at(12 * pp)
        print("    電気6次(%d周期/回転) %s / 12次(%d) %s"
              % (6 * pp, "%.0f mA" % a6 if a6 is not None else "測れず",
                 12 * pp, "%.0f mA" % a12 if a12 is not None else "測れず"))

    sp = spectrum(y, fs_eff, top=8)
    print("    平均 %.0f mA / 変動RMS %.1f mA"
          % (sum(y) / len(y), math.sqrt(sum((v - sum(y) / len(y)) ** 2
                                            for v in y) / len(y))))
    print("    成分（振幅 / 時間の周波数 / 空間の周期数 / 正体）:")
    for amp, f in sp:
        cyc = f / rev_s
        print("      %5.0f mA  %6.1f Hz  %6.1f 周期/回転   %s"
              % (amp, f, cyc, ripple_label(cyc, pp)))
    return 0


def gains_ints(bw_hz, R, L):
    w = 2.0 * math.pi * bw_hz
    return int(L * w * 1000.0), int(R * w)


def gains_why(bw_hz, R, L):
    if not (bw_hz > 0.0 and R > 0.0 and L > 0.0):
        return ("帯域・R・L はどれも 0 より大きい数で（帯域 %g Hz・R %g Ω・L %g H）。0 や負では電流ループが止まるか正帰還になる"
                % (bw_hz, R, L))
    kp_mv, ki = gains_ints(bw_hz, R, L)
    bad = []
    for pid, v in ((13, kp_mv), (14, ki)):
        _key, name, _unit, _lsb, lo, hi, _d = PARAMS[pid]
        if not (lo <= v <= hi):
            bad.append("%s %d（送れるのは %d〜%d）" % (name, v, lo, hi))
    if bad:
        return ("帯域 %g Hz では %s。int16 に折り返すと負のゲイン（正帰還）になる" % (bw_hz, "・".join(bad)))
    return None


def do_gains(bw_hz=800.0, R=1.62, L=0.00078):
    why = gains_why(bw_hz, R, L)
    if why is not None:
        print("gains を送りません: %s" % why)
        return 1
    kp_mv, ki = gains_ints(bw_hz, R, L)
    cfg(7, kp_mv, ki)
    time.sleep(0.2)
    d = tlm(8)
    print("電流ループ %.0f Hz → Kp %d mV/A, Ki %d" % (bw_hz, kp_mv, ki))
    if d:
        print("  読み返し: Kp %d mV/A, Ki %d" % (u16(d, 4), u16(d, 6)))
    return 0


NEED = {
    "provision": (1, "新しいノードID"),
    "limit": (1, "下限 rad・上限 rad（または off）"),
    "tlm": (1, "間引きの数"),
    "hold": (3, "Kp・Kd・秒"),
    "step": (4, "rad・Kp・Kd・秒"),
    "vel": (3, "速度 rad/s・Kd・秒"),
    "torque": (2, "トルク N·m・秒"),
    "sine": (5, "振幅 rad・周波数 Hz・Kp・Kd・秒"),
    "bode": (3, "振幅 rad・Kp・Kd"),
}

CFG_I16_MAX = 32767
LIMIT_RAD_PER_LSB = 0.01
TLM_DIV_MAX = 255


def check_cfg_args(cmd, a, f):
    if cmd == "limit" and len(a) == 1 and str(a[0]).lower() == "off":
        return None
    if cmd == "limit":
        if len(f) < 2:
            return "limit は 下限 rad・上限 rad の2つ（壁を外すなら limit off）"
        lo, hi = f[0], f[1]
        p1, p2 = limits_lsb(lo, hi)
        for x, p in ((lo, p1), (hi, p2)):
            if abs(p) > CFG_I16_MAX:
                return ("%.2f rad は基板に送れる範囲（±%.2f rad）の外です"
                        "（int16 に折り返して別の範囲になる）"
                        % (x, CFG_I16_MAX * LIMIT_RAD_PER_LSB))
        if lo >= hi or p1 >= p2:
            return ("下限 %.2f ≥ 上限 %.2f（ファームは下限 ≥ 上限を『無効』と読む。"
                    "無効にしたいなら意図を確かめてから）" % (lo, hi))
    elif cmd == "tlm":
        if f[0] != int(f[0]) or not (0 <= f[0] <= TLM_DIV_MAX):
            return ("間引きの数は 0〜%d の整数です: %s（0 は塊0 以外を止める）"
                    % (TLM_DIV_MAX, a[0]))
    elif cmd == "provision":
        if not valid_node(a[0]):
            return "新しいノードID として読めません（1〜%d の整数）: %s" % (NODE_MAX, a[0])
    return None


def do_home(ratio=1.0):
    import krcbus as _kb
    bus = _kb.KrcBus([NODE])

    def deg(r):
        return r / ratio * 180.0 / math.pi
    bad = bus.driving_why()
    if bad:
        for n, w in bad:
            print("home を始めません: ノード%d の%s" % (n, w))
        return 1
    try:
        bus.clear()
        time.sleep(0.2)
        bus.zero()
        bus.enter()
        print("原点出し ノード %d（減速比 %.1f）  押す力 %s N·m  速度 出力 %.0f°/s"
              % (NODE, ratio, "/".join("%.2f" % (k * _kb.HOME_V_DES) for k in _kb.HOME_KD_STEPS),
                 deg(_kb.HOME_V_DES)))

        def prog(sign, force, pos):
            s = str(sign)
            print("    %s方向 力 %.2f N·m で止まった%s: %+.3f rad（出力 %+.1f°）"
                  % ("＋" if s.startswith("+") else "−", force, "（当て直し）" if s.endswith("p") else "",
                     pos, deg(pos)), flush=True)
        r = bus.home(NODE, progress=prog)
        if not r["ok"]:
            print("原点出しできませんでした: %s" % r["why"])
            return 1
        print("可動範囲 %.3f rad ＝ 出力 %.1f°（原点は真ん中・ストッパは ±%.1f°）"
              % (r["span"], deg(r["span"]), deg(r["hi"])))
        print("壁 ±%.3f rad（出力 ±%.1f°）を置いた。**フラッシュには保存していない**"
              % (r["lim"], deg(r["lim"])))
        if r["reach"] < r["lim"]:
            print("⚠ 壁が位置指令の届く範囲（±%.1f rad）の外。位置指令で行けるのは出力 ±%.1f° まで"
                  % (_kb.POS_SAFE_RAD, deg(r["reach"])))
        return 0
    finally:
        try:
            bus.exit()
        except Exception:
            pass


PARAM_FIRST_S = 1.2


def flush_rx():
    while True:
        try:
            sock.recv(16)
        except BlockingIOError:
            return


def tlm13_frames(first_s=PARAM_FIRST_S):
    end = time.time() + first_s
    div = None
    while time.time() < end:
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0003)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        cid &= 0x1FFFFFFF
        d = frame[8:8 + dlc]
        if cid == id_tlm(12) and div is None and len(d) >= 5 and d[4] > 0:
            div = d[4]
            end = max(end, time.time() + tlm13_wait_s(div))
        elif cid == id_tlm(8):
            _TLM8[0] = d
        elif cid == id_tlm(4):
            _TLM4[0] = d
            _TLM4[1] += 1
        elif cid == id_tlm(13):
            yield d


_TLM8 = [None]
_TLM4 = [None, 0]


def board_why():
    why = board_mode_why(_TLM4[0], _TLM4[1], final=False)
    if why == BOARD_MODE_WAIT:
        for _d in tlm13_frames():
            why = board_mode_why(_TLM4[0], _TLM4[1], final=False)
            if why != BOARD_MODE_WAIT:
                break
    if why == BOARD_MODE_WAIT:
        why = board_mode_why(_TLM4[0], _TLM4[1], final=True)
    return why


def read_cal():
    if _TLM8[0] is None:
        for _d in tlm13_frames():
            if _TLM8[0] is not None:
                break
    return cal_from_tlm8(_TLM8[0])


def sync_scale():
    global P_MAX, V_MAX, KP_MAX, KD_MAX, T_MAX, I_MAX
    _TLM8[0] = None
    rd = ScaleRead()
    for d in tlm13_frames():
        rd.feed(d)
        if rd.result() is not None:
            break
    r = rd.result()
    if r is None:
        return None
    src, sc = r
    P_MAX, V_MAX, KP_MAX, KD_MAX, T_MAX, I_MAX = sc["p"], sc["v"], sc["kp"], sc["kd"], sc["t"], sc["i"]
    return src, sc


MIT_CMDS = ("enter", "exit", "zero", "hold", "step", "vel", "torque", "noise", "sinenoise", "ripple", "gains",
            "sine", "bode", "identify")
CLOSED_LOOP_CMDS = ("enter", "hold", "step", "vel", "torque", "noise", "sinenoise", "ripple", "sine", "bode", "identify")


def read_param_now(pid):
    for d in tlm13_frames():
        for p, v in params_from_tlm13(d):
            if p == pid:
                return v
    return None


def read_params(want=None):
    out = {}
    first = True
    for d in tlm13_frames():
        skip = first and want is not None
        first = False
        if skip:
            continue
        for pid, v in params_from_tlm13(d):
            out[pid] = v
        if (want in out) if want is not None else all(k in out for k in PARAMS):
            break
    return out


NO_TLM13_TEXT = ("**読み返せません**（%.1f 秒待っても塊 12・13 が来ない。tlm 0 か、間引きが大きい"
                 "（tlm N で N×%.1f 秒に 1 本）か、古いファーム）" % (PARAM_FIRST_S, TLM_LOOP_S))


COAST_FIRST_S = PARAM_FIRST_S


def wait_coast_over():
    t0 = time.time()
    end = t0 + COAST_WAIT_S
    rpm = None
    told = False
    while time.time() < end:
        if not told and rpm is None and time.time() - t0 > COAST_FIRST_S:
            told = True
            print("回転数が読めない（%.1f 秒待っても塊 6 が来ない：tlm 0 か間引きが大きい）ので、ファームの惰行の上限"
                  "（%.1f 秒）が過ぎるまで待ってから焼きます" % (COAST_FIRST_S, COAST_MAX_MS / 1000.0), flush=True)
        try:
            frame = sock.recv(16)
        except BlockingIOError:
            time.sleep(0.0003)
            continue
        cid, dlc = struct.unpack("=IB3x", frame[:8])
        cid &= 0x1FFFFFFF
        d = frame[8:8 + dlc]
        if cid != id_tlm(6) or len(d) < 6:
            continue
        rpm = struct.unpack(">h", bytes(d[4:6]))[0]
        if coast_still(rpm):
            if told:
                print("止まりました（%d rpm・%.1f 秒待った）" % (rpm, time.time() - t0), flush=True)
            return True
        if not told:
            told = True
            print("まだ回っている（%d rpm）ので、止まるまで待ってから焼きます（惰行中に焼くと、消去のあいだ電圧が"
                  "凍ったまま回るロータにかかる）" % rpm, flush=True)
    print("⚠ %.1f 秒待っても止まったと読めません（%s）。ファームの惰行の上限は過ぎたので焼きます"
          % (COAST_WAIT_S, "回転数が読めない" if rpm is None else "いま %d rpm" % rpm), flush=True)
    return False


def do_param(a):
    if len(a) == 1 and a[0] == "save":
        wait_coast_over()
        c0 = None
        for d in tlm13_frames():
            c0 = d[6] if len(d) >= 7 else None
            break
        if c0 is None:
            flush_rx()
            cfg(23)
            print("保存を送りました。%s ── 焼けた回数が%s" % (PARAM_SAVE_TEXT["unread"], NO_TLM13_TEXT))
            return 1
        flush_rx()
        cfg(23)
        res, c1, n = None, None, 0
        for d in tlm13_frames():
            n += 1
            c1 = d[6] if len(d) >= 7 else None
            res = param_save_result(c0, c1, n)
            if res is not None:
                break
        if res is None:
            print("保存を送りました。" + PARAM_SAVE_TEXT["unknown" if n == 0 else "few"])
            return 1
        print(("保存しました（焼けた回数 %d → %d）" % (c0, c1)) if res == "saved" else
              ("%s ── 焼けた回数 %d → %d" % (PARAM_SAVE_TEXT[res], c0, c1)))
        return 0 if res == "saved" else 1
    if len(a) == 1 and a[0] == "default":
        cfg(CFG_SET_PARAM, 0, 0)
        print("パラメータを全部既定に戻しました（残すなら param save）")
        a = []
    if len(a) == 2:
        k = a[0]
        pid = PARAM_BY_KEY.get(k)
        if pid is None:
            try:
                pid = int(k)
            except ValueError:
                pid = None
        if pid not in PARAMS:
            print("そのパラメータはありません: %s（名前: %s）" % (k, " ".join(v[0] for v in PARAMS.values())))
            return 1
        try:
            val = float(a[1])
        except ValueError:
            print("値が数として読めません: %s" % a[1])
            return 1
        if not math.isfinite(val):
            print("値に非有限（nan / inf）は使えません")
            return 1
        x = param_lsb(pid, val)
        p_t, p_kt = PARAM_BY_KEY["t_max"], PARAM_BY_KEY["kt"]
        if pid in (p_t, p_kt):
            other = p_t if pid == p_kt else p_kt
            o = read_param_now(other)
            if o is None:
                print("%s を送りません: %sが読み返せない（組で検めるため。%s）"
                      % (PARAMS[pid][1], PARAMS[other][1], NO_TLM13_TEXT))
                return 1
            why = t_max_why(param_value(p_t, x if pid == p_t else o), param_value(p_kt, x if pid == p_kt else o))
            if why is not None:
                print("%s を送りません: %s" % (PARAMS[pid][1], why))
                return 1
        flush_rx()
        cfg(CFG_SET_PARAM, pid, x)
        got = read_params(want=pid)
        name, unit = PARAMS[pid][1], PARAMS[pid][2]
        if pid not in got:
            print("%s に %g %s を送りました。%s" % (name, param_value(pid, x), unit, NO_TLM13_TEXT))
            return 1
        ok = (got[pid] == x)
        print("%s: 送った %g %s → 基板 %g %s%s（残すなら param save）"
              % (name, param_value(pid, x), unit, param_value(pid, got[pid]), unit,
                 "" if ok else "　**食い違い**（遮断中は受けない）"))
        return 0 if ok else 1
    if a:
        print("使い方: param ／ param <名前|番号> <値> ／ param default ／ param save")
        return 1
    got = read_params()
    if not got:
        print(NO_TLM13_TEXT)
        return 1
    for pid in sorted(PARAMS):
        k, name, unit, _lsb, _lo, _hi, d = PARAMS[pid]
        v = got.get(pid)
        print("%2d %-8s %-20s %10s %-10s（%s）"
              % (pid, k, name, "–" if v is None else ("%g" % param_value(pid, v)), unit,
                 "既定なし" if k in PARAMS_NO_DEFAULT else ("既定 %g" % param_value(pid, d))))
    return 0


def main():
    global NODE
    argv = sys.argv[1:]
    global RATE
    node_given = False
    while len(argv) >= 2 and argv[0] in ("-n", "-r"):
        if argv[0] == "-n":
            if not valid_node(argv[1]):
                print("立ち上げられません: 軸番号として読めません（1〜%d）: %s"
                      % (NODE_MAX, argv[1]))
                return 2
            NODE = int(argv[1])
            node_given = True
        else:
            try:
                r = float(argv[1])
            except (TypeError, ValueError):
                print("立ち上げられません: 送信レートが数として読めません: %s"
                      % argv[1])
                return 2
            if not math.isfinite(r) or r <= 0.0:
                print("立ち上げられません: 送信レートは 0 より大きい数で"
                      "指定してください: %s" % argv[1])
                return 2
            RATE = r
        argv = argv[2:]
    if not argv:
        print(__doc__)
        return 1
    cmd, a = argv[0], argv[1:]
    if cmd == "estop":
        print("全軸: estop")
    elif cmd != "provision":
        print("軸 %d%s: %s" % (NODE, "" if node_given else "（-n 省略・既定）", " ".join(argv)))
    if cmd == "param":
        return do_param(a)
    lim_off = (cmd == "limit" and len(a) == 1 and str(a[0]).lower() == "off")
    if lim_off:
        f = []
    else:
        try:
            f = [float(x) for x in a] if a else []
        except ValueError:
            print("引数が数として読めません: %s" % " ".join(a))
            return 1
    bad = [x for x, y in zip(a, f) if not math.isfinite(y)]
    if bad:
        print("引数に非有限の値は使えません（nan / inf）: %s" % " ".join(bad))
        return 1
    if cmd in NEED and len(f) < NEED[cmd][0] and not lim_off:
        print("%s は %d つの引数が要ります（%s）。いま %d つ"
              % (cmd, NEED[cmd][0], NEED[cmd][1], len(f)))
        print(__doc__)
        return 1
    why = check_cfg_args(cmd, a, f)
    if why is not None:
        print("%s を送りません: %s" % (cmd, why))
        return 1
    if cmd in MIT_CMDS:
        r = sync_scale()
        if r is None:
            print("%s を送りません: 基板の目盛りが読めない（塊 13 が来ない。tlm 0 になっていないか）" % cmd)
            return 1
        print("目盛り（%s）: 位置 ±%g rad・速度 ±%g rad/s・トルク ±%g N·m" % (r[0], r[1]["p"], r[1]["v"], r[1]["t"]))
    if cmd in CLOSED_LOOP_CMDS:
        why = cal_why(read_cal())
        if why is not None:
            print("%s を送りません: %s" % (cmd, why))
            return 1
        why = board_why()
        if why is not None:
            print("%s を送りません: ノード%d の%s" % (cmd, NODE, why))
            return 1

    if cmd == "estop":
        send(id_estop(), b"\x00")
        print("一斉停止を送信。解除は clear")
    elif cmd == "clear":
        send(id_duty(), bytes([0xA5, 2, 0, 0, 0, 0, 0, 0]))
        time.sleep(0.6)
        r = trip_reason()
        print("遮断を解除 →",
              "**確かめられない**（テレメトリ 0x304 が来ない）" if r is None
              else ("OK" if r == 0 else "**まだ遮断中**（理由 %d）" % r))
    elif cmd == "provision":
        return do_provision(int(f[0]))
    elif cmd == "identify":
        return do_identify()
    elif cmd == "status":
        return do_status()
    elif cmd == "limit" and len(a) == 1 and str(a[0]).lower() == "off":
        cfg(18, 0, 0)
        print("可動範囲を外した（無効）")
    elif cmd == "limit":
        p1, p2 = limits_lsb(f[0], f[1])
        cfg(18, p1, p2)
        print("可動範囲 %.2f 〜 %.2f rad（save で保存）" % (p1 * LIMIT_RAD_PER_LSB, p2 * LIMIT_RAD_PER_LSB))
    elif cmd == "home":
        return do_home(float(f[0]) if f else 1.0)
    elif cmd == "tlm":
        cfg(19, int(f[0]))
        print("テレメトリ間引き = %d" % int(f[0]))
    elif cmd == "save":
        wait_coast_over()
        cfg(12)
        time.sleep(0.8)
        d = tlm(8)
        if d is None:
            print("保存 → **確かめられない**（テレメトリ 0x308 が来ない）。"
                  "./krc.py status で較正の有無を見ること")
        elif (d[2] & 0x80) == 0:
            print("保存 → **失敗**（較正がフラッシュに入っていない）")
        else:
            print("保存 → 較正は入っている。ただし**今回書けたかは"
                  "確かめられない**（基板は保存の可否を申告しない。"
                  "通電中の保存は黙って捨てられるので、脱力させてから打つこと）")
    elif cmd == "enter":
        print("enter ->", special(0xFC))
    elif cmd == "exit":
        print("exit  ->", special(0xFD))
    elif cmd == "zero":
        print("zero  ->", special(0xFE))
    elif cmd == "hold":
        return run(0.0, 0.0, f[0], f[1], 0.0, f[2], "位置保持")
    elif cmd == "step":
        return run(f[0], 0.0, f[1], f[2], 0.0, f[3], "位置ステップ",
                   every=1, settle=0.5, arm=(4, 20))
    elif cmd == "vel":
        return run(0.0, f[0], 0.0, f[1], 0.0, f[2], "速度制御",
                   every=10, settle=0.3, arm=(3, 20))
    elif cmd == "torque":
        return run(0.0, 0.0, 0.0, 0.0, f[0], f[1], "トルク制御", every=10)
    elif cmd == "noise":
        return do_noise(*(f if f else []))
    elif cmd == "sinenoise":
        return do_sinenoise(*(f if f else []))
    elif cmd == "ripple":
        return do_ripple(*(f if f else []))
    elif cmd == "gains":
        return do_gains(*(f if f else []))
    elif cmd == "sine":
        do_sine(f[0], f[1], f[2], f[3], f[4])
    elif cmd == "bode":
        print("=== 正弦波追従（振幅 %.2f rad, Kp=%.1f, Kd=%.3f）===" % (f[0], f[1], f[2]))
        for fr in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
            do_sine(f[0], fr, f[1], f[2], max(1.5, 4.0 / fr), quiet=True)
            time.sleep(0.5)
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
