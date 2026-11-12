#!/usr/bin/env python3
"""KRC アクチュエータ - 多軸を一定周期で回すためのバス層

歩容コントローラの土台。脚の形（リンク長・自由度）に依らない部分だけを持つ。

  from krcbus import KrcBus
  bus = KrcBus(nodes=[1, 2, 3], rate=200)
  bus.enter()
  bus.spin(3.0, lambda t, b: b.set(1, p=0.5*math.sin(2*math.pi*t), v=..., kp=18, kd=0.12))
  bus.exit()

単体でも走る:
  ./krcbus.py bench [ノード数] [Hz] [秒]   … 周期の揺れとバス占有を測る
  ./krcbus.py demo  [ノードID]             … 1軸を正弦波で回す
  ./krcbus.py safety [ノードID]            … 非常停止が効くかを確かめる
  ./krcbus.py pos   [ノードID]             … 飽和しない位置と、位置モードに入れるか

⚠ スケールはファーム側の MIT_*_MAX と必ず一致させること（krc.py と同じ）。
"""
import math
import os
import select
import socket
import struct
import sys
import time

P_MAX = 95.5
V_MAX = 100.0
KP_MAX = 500.0
KD_MAX = 5.0
T_MAX = 3.0
I_MAX = 10.0

VMAX_FRAC = 0.90 / 1.7320508
CFG_SET_PARAM = 22
CFG_SAVE = 12
CFG_SAVE_PARAM = 23
PARAMS = {
    1:  ("imax",   "電流の上限",        "A",         0.001,  500,  8000, 3000),
    2:  ("kt",     "Kt（トルク定数）",  "N·m/A",     0.0001, 1000, 10000, 3000),
    3:  ("irated", "定格電流（温度推定）", "A",       0.001,  500,  6500, 2490),
    4:  ("t_trip", "温度で遮断",        "%",         1,      101,  250,  120),
    5:  ("t_clear", "温度で復帰",       "%",         1,      50,   240,  100),
    6:  ("t_tau",  "温度の時定数",      "s",         1,      1,    600,  10),
    7:  ("timeout", "指令が途切れて止まる時間", "ms",    1,      500,  2000, 500),
    8:  ("vbus",   "母線電圧",          "V",         0.001,  12000, 30000, 25200),
    9:  ("lim_kp", "壁の硬さ",          "N·m/rad",   0.1,    0,    1000, 200),
    10: ("lim_kd", "壁の減衰",          "N·m·s/rad", 0.001,  0,    1000, 150),
    11: ("harm",   "高調波の打ち消し",  "0/1",       1,      0,    1,    0),
    12: ("harm_mu", "その学習の速さ",   "×1e-6",     1,      -5000, 5000, 500),
    13: ("cur_kp", "電流 PI の Kp",     "V/A",       0.001,  1,    32767, 3520),
    14: ("cur_ki", "電流 PI の Ki",     "V/(A·s)",   1,      1,    32767, 8040),
    15: ("decoup", "非干渉化",          "‰",         1,      0,    1000, 0),
    16: ("enc_offset", "電気角オフセット", "カウント",  1,      0,    16383, 0),
    17: ("p_max",  "位置の目盛り",      "rad",       0.01,   100,  32767, 9550),
    18: ("v_max",  "速度の目盛り",      "rad/s",     0.1,    100,  10000, 1000),
    19: ("kp_max", "Kp の目盛り",       "N·m/rad",   0.1,    10,   32767, 5000),
    20: ("kd_max", "Kd の目盛り",       "N·m·s/rad", 0.001,  10,   32767, 5000),
    21: ("t_max",  "トルクの目盛り",    "N·m",       0.001,  100,  32767, 3000),
    22: ("i_max",  "電流の目盛り",      "A",         0.001,  1000, 32767, 10000),
    23: ("vmax_frac", "出力電圧の上限", "比",        0.001,  500,  950,  900),
    24: ("hi_duty", "電流を信じないデューティ", "比", 0.001,  500,  950,  800),
    25: ("obs_hz", "速度オブザーバの帯域", "Hz",     1,      50,   2000, 400),
    26: ("extrap", "指令の補間の最長",  "ms",        0.1,    30,   1000, 250),
    27: ("acc_max", "補間の加速度の上限", "rad/s²",  1,      0,    32767, 3000),
    28: ("harm_max", "高調波の係数の上限", "V",      0.001,  0,    5000, 1000),
    29: ("harm_min", "高調波を学ぶ最低速度", "rad/s", 0.01,  0,    10000, 200),
    30: ("l_uh",   "インダクタンス",    "µH",        1,      10,   32767, 780),
    31: ("ke",     "逆起電力定数",      "V·s/rad",   0.0001, 10,   32767, 1960),
    32: ("ntc_b",  "サーミスタの B 定数", "",        1,      1000, 10000, 4100),
    33: ("align_mv", "自動整列の電圧",  "V",         0.001,  200,  6000, 2000),
    34: ("t_amb",  "温度の見積もりの周囲", "℃",      1,      0,    60,   30),
    35: ("t_rise", "100 % での上昇",    "K",         1,      10,   200,  68),
    36: ("t_tref", "抵抗の基準温度",    "℃",         1,      20,   200,  66),
    37: ("lost_hold", "途切れたあと持つ", "ms",      1,      0,    10000, 2000),
    38: ("lost_rel",  "そのあと力を抜く", "ms",      1,      0,    10000, 3000),
    39: ("enc_c1", "エンコーダ補正 cos θ",  "°",     0.1,    -900, 900,  0),
    40: ("enc_s1", "エンコーダ補正 sin θ",  "°",     0.1,    -900, 900,  0),
    41: ("enc_c2", "エンコーダ補正 cos 2θ", "°",     0.1,    -900, 900,  0),
    42: ("enc_s2", "エンコーダ補正 sin 2θ", "°",     0.1,    -900, 900,  0),
    43: ("b_trip",   "基板温度で遮断",     "℃",     1,      40,   120,  80),
    44: ("b_clear",  "基板温度で復帰",     "℃",     1,      30,   119,  70),
    45: ("w_derate", "巻線温度で絞り始め", "℃",     1,      40,   200,  100),
    46: ("b_derate", "基板温度で絞り始め", "℃",     1,      30,   120,  70),
}
ENC_CORR_PARAMS = (39, 40, 41, 42)
SCALE_WAIT_S = 4.0
SCALE_PARAMS = {17: ("p", 0.01), 18: ("v", 0.1), 19: ("kp", 0.1), 20: ("kd", 0.001), 21: ("t", 0.001), 22: ("i", 0.001)}
SCALE_PER_AXIS = ("kp",)
SCALE_LEGACY = dict(p=12.5, v=100.0, kp=500.0, kd=5.0, t=3.0, i=10.0)
PARAMS_HIDDEN = ("enc_offset", "p_max", "v_max", "kp_max", "kd_max", "t_max", "i_max", "obs_hz",
                 "enc_c1", "enc_s1", "enc_c2", "enc_s2")
PARAMS_NO_DEFAULT = ("enc_offset",)
PARAM_BY_KEY = dict((v[0], k) for k, v in PARAMS.items())
TLM_LOOP_S = 0.1
PARAM_PAGES = (max(PARAMS) + 1) // 2


def tlm13_wait_s(div):
    return PARAM_PAGES * div * TLM_LOOP_S * 2.0

PARAM_SAVE_TEXT = dict(
    soon="保存待ち（まもなく基板に保存）",
    wait="通電中（解除すると基板に保存）",
    coast="回っている（止まってから基板に保存）",
    tripped="遮断中（遮断を解除すると基板に保存）",
    saving="保存中…",
    saved="基板に保存済み",
    failed="保存できませんでした（焼けた回数が増えない：通電中・遮断中・較正の無い基板は焼かない）",
    unknown="保存を確かめられません（塊 13 が来ない）",
    few="保存をまだ確かめられません（保存のあとの塊 13 が 1 本だけ：2 本目で決まる。tlm の間引きが大きい）",
    unread="保存を確かめられません（保存を送る前に焼けた回数が読めていなかった）",
    reset="保存を確かめられません（焼けた回数が送った保存の数と合わない：基板の再起動など）",
    differs="保存していません（基板の値が画面の送った値と違う：基板の再起動・CAN の途切れ・別の送り手など。"
            "値を入れ直すと保存し直します）",
    verify="保存を確かめています（焼けた回数は増えた・送った値の読み返しを待っています）",
)


def param_save_result(c0, c1, n_after, m=1):
    if c0 is None or c1 is None:
        return None
    if c1 == (c0 + m) & 0xFF:
        return "saved"
    if n_after < 2:
        return None
    return "failed" if c1 == c0 else "reset"


def param_lsb(pid, value):
    _k, _n, _u, lsb, lo, hi, _d = PARAMS[pid]
    try:
        q = float(value) / lsb
    except OverflowError:
        q = math.inf if value > 0 else -math.inf
    if q != q:
        raise ValueError("値が数ではありません（NaN）")
    if q >= hi:
        return hi
    if q <= lo:
        return lo
    return max(lo, min(hi, int(round(q))))


def param_value(pid, lsb_value):
    return lsb_value * PARAMS[pid][3]


def params_from_tlm13(d):
    out = []
    if d is None or len(d) < 6:
        return out
    for i in (0, 3):
        pid = d[i]
        if pid in PARAMS:
            out.append((pid, struct.unpack(">h", bytes(d[i + 1:i + 3]))[0]))
    return out

def scale_expect():
    return dict(p=P_MAX, v=V_MAX, kp=KP_MAX, kd=KD_MAX, t=T_MAX, i=I_MAX)


class ScaleRead(object):

    def __init__(self):
        self.got = {}
        self.maxid = 0
        self.wraps = 0

    def feed(self, d):
        for pid, v in params_from_tlm13(d):
            self.got[pid] = v
        if d is not None and len(d) >= 8:
            self.maxid = max(self.maxid, d[0], d[3])
            self.wraps += (d[7] == 0)

    def result(self):
        if all(k in self.got for k in SCALE_PARAMS):
            return "読み返し", dict((key, self.got[pid] * lsb) for pid, (key, lsb) in SCALE_PARAMS.items())
        if self.wraps >= 2 and self.maxid < min(SCALE_PARAMS):
            return "古いファーム", dict(SCALE_LEGACY)
        return None


ID_ESTOP = 0x080
NODE_MAX = 15


def valid_node(n):
    try:
        n = int(n)
    except (TypeError, ValueError, OverflowError):
        return False
    return 1 <= n <= NODE_MAX


SELFTEST_BITS = ((0x01, "DRV と SPI が通らない"), (0x02, "エンコーダが読めない"),
                 (0x10, "DRV が 3xPWM になっていない"), (0x20, "DRV の設定が入っていない"),
                 (0x40, "ゲート出力が切れている"))
SELFTEST_BAD = ((0x04, "エンコーダの磁石が見えない"),
                (0x08, "DRV が nFAULT を出している（チャージポンプ・電源電圧・過熱・配線）"))


def selftest_why(flags):
    why = [t for b, t in SELFTEST_BITS if not (flags & b)]
    why += [t for b, t in SELFTEST_BAD if flags & b]
    return why


def cal_from_tlm8(d):
    if d is None or len(d) < 3:
        return None
    return bool(d[2] & 0x80)


def cal_why(cal):
    if cal is False:
        return "較正（電気角オフセット）が入っていません（provision してから。オフセット 0 で転流すると逆向きに回りうる）"
    if cal is None:
        return "較正があるかをまだ読めていません（テレメトリの塊 8。数秒待つ・tlm が止まっていないか）"
    return None


def foc_mode_of(d):
    if d is None or len(d) < 8:
        return None
    return d[7]


BOARD_DRIVING_TEXT = ("基板はまだモーターモードです（指令が途切れて持っている／力を抜いている最中か、ほかの道具が"
                      "通電中）。止まってから")
BOARD_MODE_UNREAD_TEXT = ("基板のモードが読めません（テレメトリの塊 4 が来ない。tlm 0 になっていないか）。"
                          "持っている脚へ 0xFC・0xFD を送らないよう、読めるまで送りません")
BOARD_DRIVING_CLI_TEXT = BOARD_DRIVING_TEXT + "もう一度打ってください（いま 0xFC・0xFD を送ると、持っている脚がその場で落ちる）"
BOARD_MODE_WAIT = "wait"


def board_mode_why(d4, n_after, final=True):
    mode = foc_mode_of(d4) if n_after > 0 else None
    if mode == FOC_MODE_IDLE:
        return None
    if mode is not None and (n_after >= 2 or final):
        return BOARD_DRIVING_CLI_TEXT
    if not final:
        return BOARD_MODE_WAIT
    return BOARD_MODE_UNREAD_TEXT


def t_max_why(t_max, kt):
    if not (kt > 0.0):
        return "Kt が 0 以下では電流に直せません（Kt %g N·m/A）" % kt
    lim = 32767.0 / 1000.0 * kt
    if t_max > lim:
        return ("トルクの目盛り %g N·m は Kt %g N·m/A では大きすぎる（%.3f N·m まで）。ファームはトルクを電流（mA）の "
                "int16 に詰めるので、それを超えるトルクの指令は向きが反転して逆向きに全力が出る"
                % (t_max, kt, lim))
    return None

KT_NM_PER_A = 0.30
KE_V_S_PER_RAD = 0.21
COPPER_ALPHA = 0.00393
TEMP_INVALID_RAW = -32768
THERM_TRIP_PCT = 120
THERM_CLEAR_PCT = 100
FOC_TIMEOUT_MS = 500
COAST_STOP_RPM = 20
COAST_MAX_MS = 5000
COAST_WAIT_S = COAST_MAX_MS / 1000.0 * 1.1


def coast_still(rpm):
    return rpm is not None and abs(rpm) <= COAST_STOP_RPM


FOC_MODE_IDLE = 0

RAD_PER_COUNT = 2.0 * math.pi / 16384.0

HOME_V_DES = 4.0
HOME_KD_STEPS = (0.10, 0.14, 0.18)
HOME_STALL_S = 0.8
HOME_STALL_RAD = 0.03
HOME_MAX_TRAVEL = 4.0 * math.pi * 9.0
HOME_DIR_TIMEOUT_S = 90.0
HOME_NO_TLM_S = 1.0
HOME_PROBE_KD = (0.06, 0.10, 0.14, 0.18)
HOME_BACKOFF_RAD = 0.6
HOME_BACKOFF_S = 0.7
HOME_PROBE_NEAR_RAD = 0.5
HOME_MARGIN_RAD = 3.0 / 180.0 * math.pi * 9.0
HOME_SETTLE_KP = 3.0
HOME_SETTLE_KD = 0.08
HOME_ACK_S = 3.0
LIMIT_RAD_PER_LSB = 0.01
CFG_I16_MAX = 32767


def limits_lsb(lo, hi):
    return (int(math.floor(lo / LIMIT_RAD_PER_LSB + 1e-9)),
            int(math.ceil(hi / LIMIT_RAD_PER_LSB - 1e-9)))


def limits_from_tlm12(d):
    lo = struct.unpack(">h", bytes(d[0:2]))[0] * LIMIT_RAD_PER_LSB
    hi = struct.unpack(">h", bytes(d[2:4]))[0] * LIMIT_RAD_PER_LSB
    return lo, hi

POS_SAFE_RAD = P_MAX - 0.5

FRAME_S_8 = 130e-6
FRAME_S_6 = 110e-6


def _pack(x, lo, hi, bits):
    if not math.isfinite(x):
        raise ValueError("MIT に非有限の値は積めない: %r" % (x,))
    x = max(lo, min(hi, x))
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def _unpack(v, lo, hi, bits):
    return v * (hi - lo) / ((1 << bits) - 1) + lo


def _pos_cmd_edge():
    lo, hi = -P_MAX, P_MAX

    def _sendable(c):
        x = _unpack(c, lo, hi, 16)
        return _pack(x, lo, hi, 16) == c and not fb_saturated(x)
    c_lo = _pack(-POS_SAFE_RAD, lo, hi, 16)
    c_hi = _pack(POS_SAFE_RAD, lo, hi, 16)
    while not _sendable(c_lo):
        c_lo += 1
    while not _sendable(c_hi):
        c_hi -= 1
    return _unpack(c_lo, lo, hi, 16), _unpack(c_hi, lo, hi, 16)


FB_FRESH_S = 0.5

BOARD_FRESH_S = 2.0

UPTIME_WRAP = 1 << 16

TICK_WRAP_S = (((1 << 32) - 1) // 1000) % UPTIME_WRAP

REBOOT_WHY = "基板が再起動した"
REBOOT_WHY_EN = "board restarted"


class WhyEn(str):
    def __new__(cls, ja, en=None):
        s = str.__new__(cls, ja)
        s.en = en
        return s


class EnterRefused(Exception):
    pass


TRIP_SETTLE_S = 0.6
TRIP_ASK_S = 0.06


def trip_sequence(action, nodes):
    nodes = list(nodes)
    if action == "estop":
        steps = [(0.0, ID_ESTOP, b"\x00")]
        settle = TRIP_ASK_S
    elif action == "clear":
        steps = [(0.0, 0x140 + n, bytes([0xA5, 2, 0, 0, 0, 0, 0, 0]))
                 for n in nodes]
        settle = TRIP_SETTLE_S
    else:
        raise ValueError("遮断の操作は estop か clear のどちらか: %r" % (action,))
    steps += [(settle, 0x100 + n, bytes([0xFF] * 7 + [0xFD])) for n in nodes]
    return steps


def trip_assume(action):
    return True if action == "estop" else None


def fb_saturated(fb_pos):
    return abs(fb_pos) > POS_SAFE_RAD


def pos_gate(pos_true, fb_pos, fb_alive):
    if pos_true is None:
        if not fb_alive:
            return None, WhyEn("基板から返事が無い（電源・配線・ノードIDを確かめること）",
                               "no reply from the board (check power, wiring and node ID)")
        if fb_saturated(fb_pos):
            return None, WhyEn("位置が %.2f rad で飽和していて、累積位置（テレメトリ）も"
                               "読めないので、本当の位置が分からない" % fb_pos,
                               "the position is saturated at %.2f rad and the accumulated position (telemetry) "
                               "is not readable, so the true position is unknown" % fb_pos)
        return fb_pos, None
    if abs(pos_true) > POS_SAFE_RAD:
        return None, WhyEn("原点から %.2f rad 離れている（位置モードに居られるのは "
                           "±%.1f rad まで）" % (pos_true, POS_SAFE_RAD),
                           "%.2f rad away from home (position modes allow up to ±%.1f rad)"
                           % (pos_true, POS_SAFE_RAD))
    return pos_true, None


POS_CMD_LO, POS_CMD_HI = _pos_cmd_edge()


def sine_gate(center, amp, lo=None, hi=None):
    lo = -POS_SAFE_RAD if lo is None else max(lo, -POS_SAFE_RAD)
    hi = POS_SAFE_RAD if hi is None else min(hi, POS_SAFE_RAD)
    if center + amp > hi or center - amp < lo:
        if lo == -hi:
            return ("中心 %.2f rad ＋ 振幅 %.2f rad が ±%.1f rad を超える"
                    "（中心か振幅を小さくしてください）" % (center, amp, hi))
        return ("中心 %.2f rad ± 振幅 %.2f rad が %.2f〜%.2f rad を超える"
                "（中心か振幅を小さくしてください）" % (center, amp, lo, hi))
    return None


class HomingStepper(object):

    def __init__(self, node, v_des=HOME_V_DES, kd_steps=HOME_KD_STEPS, stall_s=HOME_STALL_S,
                 stall_rad=HOME_STALL_RAD, max_travel=HOME_MAX_TRAVEL, margin=HOME_MARGIN_RAD,
                 timeout=HOME_DIR_TIMEOUT_S, fold_sign=None, half_design=None):
        self.node = int(node)
        self.fold_sign = None if fold_sign is None else (1.0 if fold_sign > 0 else -1.0)
        self.half_design = None if half_design is None else float(half_design)
        if (self.fold_sign is None) != (self.half_design is None):
            raise ValueError("fold_sign と half_design は両方渡す（片側だけの原点出し）")
        self.v_des = float(v_des)
        self.kd_steps = tuple(kd_steps)
        self.stall_s = float(stall_s)
        self.stall_rad = float(stall_rad)
        self.max_travel = float(max_travel)
        self.margin = float(margin)
        self.timeout = float(timeout)
        self.phase = "start"
        self.k = 0
        self.sign = 1.0
        self.p_start = None
        self.last_p = None
        self.last_t = None
        self.t_phase = None
        self.seen_t = None
        self.hi = None
        self.lo = None
        self.off = None
        self.tgo = None
        self.trace = []
        self._p_before_zero = None
        self.probe_kd = tuple(HOME_PROBE_KD)
        self.j = 0
        self.p_max = None
        self.pts = []
        self.stops = {}
        self.result = dict(ok=False, why=None, hi=None, lo=None, span=None, lim=None,
                           trace=self.trace, reach=None, stops=self.stops, lim_seen=None)
        self._zero_t = None
        self._walls = None

    def zeroed(self, now):
        self._zero_t = now

    def _fail(self, why):
        self.phase = "fail"
        self.result["ok"] = False
        self.result["why"] = why
        return None, "fail"

    def _settle_cmd(self):
        return (0.0, 0.0, HOME_SETTLE_KP, HOME_SETTLE_KD, 0.0)

    def _moved_since_zero(self, pos):
        before = self._p_before_zero or 0.0
        return abs(before) <= self.stall_rad or abs(pos - before) > self.stall_rad

    def _dragged_back(self, pos, tol):
        before = self._p_before_zero or 0.0
        return abs(pos - before) > self.stall_rad and abs(pos) >= tol

    def _walls_acked(self, lim):
        if lim is None or self._walls is None:
            return False
        lo_r, hi_r, t = lim
        if not (t > self.t_phase):
            return False
        want = limits_lsb(self._walls[0], self._walls[1])
        got = (int(round(lo_r / LIMIT_RAD_PER_LSB)), int(round(hi_r / LIMIT_RAD_PER_LSB)))
        return got[0] < got[1] and abs(got[0] - want[0]) <= 1 and abs(got[1] - want[1]) <= 1

    def _unwalled(self, lim):
        if lim is None:
            return False
        lo_r, hi_r, t = lim
        if not (t > self.t_phase):
            return False
        got = (int(round(lo_r / LIMIT_RAD_PER_LSB)), int(round(hi_r / LIMIT_RAD_PER_LSB)))
        return got[0] >= got[1]

    def _put_walls(self, now, lo, hi):
        self._walls = (float(lo), float(hi))
        self.phase = "walls"
        self.t_phase = now
        return self._settle_cmd(), ("walls", lo, hi)

    def _stall_cmd(self):
        kd = self.kd_steps[self.k]
        return (0.0, self.sign * self.v_des, 0.0, kd, 0.0)

    def _probe_cmd(self):
        return (0.0, self.sign * self.v_des, 0.0, self.probe_kd[self.j], 0.0)

    @staticmethod
    def fit_stop(pts):
        n = len(pts)
        if n == 0:
            return None, None
        fs = [f for f, _ in pts]
        ps = [p for _, p in pts]
        mf = sum(fs) / n
        mp = sum(ps) / n
        sxx = sum((f - mf) ** 2 for f in fs)
        if sxx < 1e-12:
            return mp, 0.0
        k = sum((f - mf) * (p - mp) for f, p in pts) / sxx
        return mp - k * mf, k

    def _side_done(self, now, pos, pos_t=None):
        if pos_t is None:
            pos_t = now
        p0, k = self.fit_stop(self.pts)
        side = "+" if self.sign > 0 else "-"
        self.stops[side] = dict(p0=p0, k=k, pts=list(self.pts))
        if self.fold_sign is not None:
            half = self.half_design
            self.hi, self.lo = half, -half
            self.result.update(hi=half, lo=-half, span=2.0 * half, fold_p0=p0, fold_k=k)
            mid = p0 - self.fold_sign * half
            self.off = mid - pos
            self.phase = "zero_lo"
            self.t_phase = now
            self._p_before_zero = pos
            return self._stall_cmd(), "zero"
        if self.sign > 0:
            self.hi = p0
            self.sign = -1.0
            self.k = 0
            self.p_start = pos
            self.last_p, self.last_t, self.t_phase = pos, pos_t, now
            self.phase = "sweep"
            return self._stall_cmd(), None
        self.lo = p0
        span = self.hi - self.lo
        if span <= 2.0 * self.margin:
            return self._fail("可動範囲 %.3f rad が余白 %.3f×2 より狭い" % (span, self.margin))
        self.result.update(hi=span / 2.0, lo=-span / 2.0, span=span)
        self.off = (p0 + span / 2.0) - pos
        self.phase = "zero_lo"
        self.t_phase = now
        self._p_before_zero = pos
        return self._stall_cmd(), "zero"

    def step(self, now, pos, tripped, lim=None, pos_t=None, rebooted=None):
        ph = self.phase
        if pos_t is None:
            pos_t = now
        if ph in ("done", "fail"):
            return self._settle_cmd() if ph == "done" else None, ph
        if tripped:
            return self._fail("trip（遮断した）")
        if rebooted:
            return self._fail("%s（起動からの秒 %d → %d）" % (REBOOT_WHY, rebooted[0], rebooted[1]))
        if ph == "start":
            self.phase = "unwall"
            self.sign = 1.0 if self.fold_sign is None else self.fold_sign
            self.k = 0
            self.p_start = pos
            self.last_p, self.last_t, self.t_phase = pos, pos_t, now
            self.seen_t = now
            return None, "limits_off"
        if ph == "unwall":
            if now - self.t_phase > self.timeout:
                return self._fail("%s方向: timeout" % ("＋" if self.sign > 0 else "−"))
            if pos is None:
                if now - self.seen_t > HOME_NO_TLM_S:
                    return self._fail("%s方向: no_tlm（位置が読めない）" % ("＋" if self.sign > 0 else "−"))
                return self._stall_cmd(), None
            self.seen_t = now
            if self.p_start is None:
                self.p_start = pos
            if abs(pos - self.p_start) > self.max_travel:
                return self._fail("%s方向: no_stop（%.0f rad 回ってもストッパが無い）"
                                  % ("＋" if self.sign > 0 else "−", self.max_travel))
            if self._unwalled(lim):
                self.phase = "sweep"
                self.last_p, self.last_t = pos, pos_t
                return self._stall_cmd(), None
            if now - self.t_phase > HOME_ACK_S:
                return self._fail("壁を外せなかった（cfg 18 の申告が来ない：申告 %s）"
                                  % ("無し" if lim is None else "%.2f〜%.2f" % (lim[0], lim[1])))
            return self._stall_cmd(), None
        if ph == "sweep":
            if now - self.t_phase > self.timeout:
                return self._fail("%s方向: timeout" % ("＋" if self.sign > 0 else "−"))
            if pos is None:
                if now - self.seen_t > HOME_NO_TLM_S:
                    return self._fail("%s方向: no_tlm（位置が読めない）" % ("＋" if self.sign > 0 else "−"))
                return self._stall_cmd(), None
            self.seen_t = now
            if self.p_start is None:
                self.p_start = pos
                self.last_p, self.last_t = pos, pos_t
            if self.last_p is None or abs(pos - self.last_p) > self.stall_rad:
                self.last_p, self.last_t = pos, pos_t
            if abs(pos - self.p_start) > self.max_travel:
                return self._fail("%s方向: no_stop（%.0f rad 回ってもストッパが無い）"
                                  % ("＋" if self.sign > 0 else "−", self.max_travel))
            if pos_t - self.last_t >= self.stall_s:
                self.trace.append(("+" if self.sign > 0 else "-", self.kd_steps[self.k] * self.v_des, pos))
                if self.k + 1 < len(self.kd_steps):
                    self.k += 1
                    self.last_p, self.last_t = pos, pos_t
                    return self._stall_cmd(), None
                self.p_max = pos
                self.pts = [(self.kd_steps[self.k] * self.v_des, pos)]
                self.j = 0
                self.phase = "backoff"
                self.t_phase = now
                self._p_back = pos - self.sign * HOME_BACKOFF_RAD
                return (self._p_back, 0.0, HOME_SETTLE_KP, HOME_SETTLE_KD, 0.0), None
            return self._stall_cmd(), None
        if ph == "backoff":
            if now - self.t_phase >= HOME_BACKOFF_S:
                self.phase = "probe"
                self.t_phase = now
                self.last_p, self.last_t = pos, pos_t
                self.seen_t = now
                return self._probe_cmd(), None
            return (self._p_back, 0.0, HOME_SETTLE_KP, HOME_SETTLE_KD, 0.0), None
        if ph == "probe":
            if now - self.t_phase > self.timeout:
                return self._fail("%s方向: 当て直しで timeout" % ("＋" if self.sign > 0 else "−"))
            if pos is None:
                if now - self.seen_t > HOME_NO_TLM_S:
                    return self._fail("%s方向: 当て直しで no_tlm（位置が読めない）" % ("＋" if self.sign > 0 else "−"))
                return self._probe_cmd(), None
            self.seen_t = now
            if self.last_p is None or abs(pos - self.last_p) > self.stall_rad:
                self.last_p, self.last_t = pos, pos_t
            if pos_t - self.last_t >= self.stall_s:
                f = self.probe_kd[self.j] * self.v_des
                near = abs(pos - self.p_max) <= HOME_PROBE_NEAR_RAD
                self.trace.append(("+p" if self.sign > 0 else "-p", f, pos))
                if near:
                    self.pts.append((f, pos))
                self.j += 1
                if self.j < len(self.probe_kd):
                    self.phase = "backoff"
                    self.t_phase = now
                    self._p_back = self.p_max - self.sign * HOME_BACKOFF_RAD
                    return (self._p_back, 0.0, HOME_SETTLE_KP, HOME_SETTLE_KD, 0.0), None
                self.k = len(self.kd_steps) - 1
                return self._side_done(now, pos, pos_t)
            return self._probe_cmd(), None
        if ph == "zero_lo":
            tol = max(0.5, 0.25 * abs(self._p_before_zero or 0.0))
            if pos is not None and self._dragged_back(pos, tol):
                return self._fail("原点を移せない（下端。0xFE が効かず読みが中間の値：移す前 %.2f → いま %.2f）"
                                  % (self._p_before_zero or 0.0, pos))
            if self._zero_t is not None and now - self._zero_t >= 0.35 and pos is not None \
                    and abs(pos) < tol and self._moved_since_zero(pos):
                self._zero_t = None
                self.phase = "go_mid"
                self.t_phase = now
                self.tgo = max(0.8, abs(self.off) / self.v_des)
                self.last_p, self.last_t = pos, pos_t
                self.seen_t = now
                return self._settle_cmd(), None
            if now - self.t_phase > HOME_ACK_S:
                return self._fail("原点を移せない（下端。移す前 %.2f → いま %s）"
                                  % (self._p_before_zero or 0.0, "?" if pos is None else "%.2f" % pos))
            return self._stall_cmd(), None
        if ph == "go_mid":
            if now - self.t_phase > self.timeout:
                return self._fail("真ん中へ寄る途中で timeout")
            if pos is None:
                if now - self.seen_t > HOME_NO_TLM_S:
                    return self._fail("真ん中へ寄る途中で no_tlm（位置が読めない）")
                return (self.off * min(1.0, (now - self.t_phase) / self.tgo), 0.0,
                        HOME_SETTLE_KP, HOME_SETTLE_KD, 0.0), None
            self.seen_t = now
            if self.last_p is None or abs(pos - self.last_p) > self.stall_rad:
                self.last_p, self.last_t = pos, pos_t
            frac = min(1.0, (now - self.t_phase) / self.tgo)
            if now - self.t_phase >= self.tgo + 0.8 and pos_t - self.last_t >= self.stall_s:
                if abs(pos - self.off) > HOME_PROBE_NEAR_RAD:
                    return self._fail("真ん中へ寄れない（固着か障害物：目標 %.2f・いま %.2f）"
                                      % (self.off, pos))
                self.phase = "zero_mid"
                self.t_phase = now
                self._p_before_zero = pos
                return (0.0, 0.0, HOME_SETTLE_KP, HOME_SETTLE_KD, 0.0), "zero"
            return (self.off * frac, 0.0, HOME_SETTLE_KP, HOME_SETTLE_KD, 0.0), None
        if ph == "zero_mid":
            if pos is not None and self._dragged_back(pos, 0.5):
                return self._fail("原点を移せない（真ん中。0xFE が効かず目標 0 へ引き戻された：移す前 %.2f → いま %.2f）"
                                  % (self._p_before_zero or 0.0, pos))
            if self._zero_t is not None and now - self._zero_t >= 0.35 and pos is not None \
                    and abs(pos) < 0.5 and self._moved_since_zero(pos):
                self._zero_t = None
                if self.fold_sign is not None:
                    half = self.half_design
                    w_fold = self.fold_sign * half
                    w_ext = -self.fold_sign * half
                    lim = half
                    self.result.update(lim=lim, reach=min(half, POS_SAFE_RAD))
                    return self._put_walls(now, min(w_fold, w_ext), max(w_fold, w_ext))
                lim = self.result["hi"] - self.margin
                self.result.update(lim=lim, reach=min(lim, POS_SAFE_RAD))
                return self._put_walls(now, -lim, lim)
            if now - self.t_phase > HOME_ACK_S:
                return self._fail("原点を移せない（真ん中）")
            return self._settle_cmd(), None
        if ph == "walls":
            if self._walls_acked(lim):
                self.result.update(lim_seen=(lim[0], lim[1]), ok=True)
                self.phase = "done"
                return self._settle_cmd(), "done"
            if now - self.t_phase > HOME_ACK_S:
                return self._fail("壁を置けなかった（cfg 18 の申告が来ない：送った %.2f〜%.2f・申告 %s）"
                                  % (self._walls[0], self._walls[1],
                                     "無し" if lim is None else "%.2f〜%.2f" % (lim[0], lim[1])))
            return self._settle_cmd(), None
        return self._fail("不明な段階 %r" % ph)


class Axis(object):

    def __init__(self, node):
        self.node = node
        self.p = 0.0
        self.v = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.t = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.iq = 0.0
        self.tripped = None
        self.fb_count = 0
        self.fb_missing = 0
        self.fb_t = 0.0
        self.trip_t = 0.0
        self.tx_t = 0.0
        self.tx_seq = 0
        self.assume_seq = None
        self.uptime_s = None
        self._reboot = None

    def note_uptime(self, sec):
        prev = self.uptime_s
        self.uptime_s = sec
        if prev is None or sec >= prev:
            return
        if sec <= 1 and (prev >= UPTIME_WRAP - 1 or TICK_WRAP_S - 1 <= prev <= TICK_WRAP_S):
            return
        self._reboot = (prev, sec)

    def take_reboot(self):
        m = self._reboot
        self._reboot = None
        return m

    def carry_boot(self, old):
        if old is None:
            return
        self.uptime_s = old.uptime_s
        self._reboot = old.take_reboot()

    def set_tripped(self, value, assumed=False, reply=False):
        if assumed:
            self.assume_seq = self.tx_seq
        elif reply and self.tx_seq == self.assume_seq and value is not True:
            return
        self.tripped = value
        self.trip_t = time.time()

    def trip_now(self):
        if self.tripped is None:
            return None
        if self.tx_t <= self.trip_t:
            return self.tripped
        if (time.time() - self.trip_t) <= FB_FRESH_S:
            return self.tripped
        return None


class KrcBus(object):

    def __init__(self, nodes, rate=200.0, iface="can0"):
        self.axes = dict((n, Axis(n)) for n in nodes)
        self.order = list(nodes)
        self.rate = float(rate)
        self.iface = iface
        self._socket = socket
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.setblocking(False)
        self.ifindex = self._ifindex()
        self.listen_t = time.time()
        self.period_log = []
        self.late_count = 0
        self.tlm = {}
        self.tlm_t = {}
        self.tlm_n = {}
        self.params = {}
        self.prm_maxid = {}
        self.prm_wraps = {}
        self.cal = {}
        self.rx_total = 0
        self._mode_ref = {}

    def _ifindex(self):
        fn = getattr(self._socket, "if_nametoindex", None)
        if fn is None:
            return None
        try:
            return int(fn(self.iface))
        except (OSError, ValueError, TypeError):
            return None

    def iface_changed(self):
        now = self._ifindex()
        return now is not None and self.ifindex is not None and now != self.ifindex

    def reopen(self):
        old = self.sock
        s = self._socket.socket(self._socket.AF_CAN, self._socket.SOCK_RAW,
                                self._socket.CAN_RAW)
        try:
            s.bind((self.iface,))
            s.setblocking(False)
        except OSError:
            if s is not old:
                try:
                    s.close()
                except OSError:
                    pass
            raise
        self.sock = s
        self.ifindex = self._ifindex()
        self.listen_t = time.time()
        if s is not old:
            try:
                old.close()
            except OSError:
                pass

    def _send(self, cid, data):
        a = self.axes.get(cid - 0x100)
        if a is not None:
            a.tx_t = time.time()
            a.tx_seq += 1
        self.sock.send(struct.pack("=IB3x8s", cid, len(data), data.ljust(8, b"\x00")))
        if a is not None and len(data) >= 8 and data[:7] == b"\xFF" * 7 and data[7] == 0xFD:
            self._mode_ref[a.node] = self.tlm_n.get((a.node, 4), 0)
        if cid == ID_ESTOP:
            for ax in self.axes.values():
                ax.set_tripped(True, assumed=True)

    def scale_of(self, n):
        got = {}
        for pid, (k, lsb) in SCALE_PARAMS.items():
            r = self.params.get((n, pid))
            if r is not None:
                got[k] = r[0] * lsb
        want = scale_expect()
        if len(got) == len(SCALE_PARAMS):
            bad = [k for k in SCALE_PARAMS.values()
                   if abs(got[k[0]] - want[k[0]]) > k[1] * 0.51
                   and not (k[0] in SCALE_PER_AXIS and 0.0 < got[k[0]] <= want[k[0]] + k[1] * 0.51)]
            if bad:
                return ("mismatch", got, "基板の目盛りがホストと違う（%s）。krc.py param で合わせるか、ホストの P_MAX などを直す"
                        % "・".join("%s 基板 %g / ホスト %g" % (k[0], got[k[0]], want[k[0]]) for k in bad))
            return ("ok", got, None)
        if self.prm_wraps.get(n, 0) >= 2 and self.prm_maxid.get(n, 0) < min(SCALE_PARAMS):
            return ("legacy", dict(SCALE_LEGACY),
                    "ファームが古い（位置の目盛り ±%.1f rad のまま・目盛りを返さない）。焼き直すまで通電しない" % SCALE_LEGACY["p"])
        return ("unknown", None, "基板の目盛りをまだ読めていない（テレメトリの塊 13。数秒待つ・tlm が止まっていないか）")

    def cal_of(self, n):
        return self.cal.get(n)

    def board_mode_why(self, n, final=True):
        return board_mode_why(self.tlm.get((n, 4)), self.tlm_n.get((n, 4), 0) - self._mode_ref.get(n, 0), final)

    def _wait_known(self, mode=True):
        self._drain_now()
        t0 = time.time()
        t_end = t0 + SCALE_WAIT_S

        def pending(n):
            return (self.scale_of(n)[0] == "unknown" or self.cal_of(n) is None
                    or (mode and self.board_mode_why(n, final=False) == BOARD_MODE_WAIT))
        while time.time() < t_end and any(pending(n) for n in self.order):
            self._drain(time.time() + 0.05)
            divs = [self.tlm[(n, 12)][4] for n in self.order
                    if len(self.tlm.get((n, 12), b"")) >= 5 and self.tlm[(n, 12)][4] > 0]
            if divs:
                t_end = max(t_end, t0 + tlm13_wait_s(max(divs)))

    def _drain_now(self, cap=1000):
        for _ in range(cap):
            r, _, _ = select.select([self.sock], [], [], 0)
            if not r:
                return
            self._drain(time.time() + 0.001)

    def driving_why(self):
        self._wait_known(mode=True)
        return [(n, w) for n in self.order for w in (self.board_mode_why(n),) if w is not None]

    def _scale_for(self, n):
        st, sc, _why = self.scale_of(n)
        return sc if sc is not None else scale_expect()

    def _mit(self, a):
        sc = self._scale_for(a.node)
        pi = _pack(a.p, -sc["p"], sc["p"], 16)
        vi = _pack(a.v, -sc["v"], sc["v"], 12)
        ki = _pack(a.kp, 0.0, sc["kp"], 12)
        di = _pack(a.kd, 0.0, sc["kd"], 12)
        ti = _pack(a.t, -sc["t"], sc["t"], 12)
        return bytes([pi >> 8, pi & 0xFF,
                      vi >> 4, ((vi & 0xF) << 4) | (ki >> 8), ki & 0xFF,
                      di >> 4, ((di & 0xF) << 4) | (ti >> 8), ti & 0xFF])

    def _drain(self, deadline, max_wait=None):
        m0 = time.monotonic()
        while True:
            timeout = deadline - time.time()
            if max_wait is not None:
                left = max_wait - (time.monotonic() - m0)
                if left < timeout:
                    timeout = left
            if timeout <= 0:
                return
            r, _, _ = select.select([self.sock], [], [], timeout)
            if not r:
                return
            try:
                frame = self.sock.recv(16)
            except BlockingIOError:
                continue
            cid, dlc = struct.unpack("=IB3x", frame[:8])
            cid &= 0x1FFFFFFF
            d = frame[8:8 + dlc]
            self.rx_total += 1

            if 0x201 <= cid <= 0x20F:
                a = self.axes.get(cid - 0x200)
                if a is None or dlc < 6:
                    continue
                a.set_tripped(bool(d[0] & 0x80), reply=True)
                pi = (d[1] << 8) | d[2]
                vi = (d[3] << 4) | (d[4] >> 4)
                ii = ((d[4] & 0xF) << 8) | d[5]
                sc = self._scale_for(a.node)
                a.pos = _unpack(pi, -sc["p"], sc["p"], 16)
                a.vel = _unpack(vi, -sc["v"], sc["v"], 12)
                a.iq = _unpack(ii, -sc["i"], sc["i"], 12)
                a.fb_count += 1
                a.fb_t = time.time()
            elif 0x300 <= cid <= 0x3FF:
                node = (cid - 0x300) // 0x10
                idx = (cid - 0x300) % 0x10
                self.tlm[(node, idx)] = d
                self.tlm_t[(node, idx)] = time.time()
                self.tlm_n[(node, idx)] = self.tlm_n.get((node, idx), 0) + 1
                if idx == 13:
                    now13 = time.time()
                    for pid, v in params_from_tlm13(d):
                        self.params[(node, pid)] = (v, now13)
                    if len(d) >= 8:
                        self.prm_maxid[node] = max(self.prm_maxid.get(node, 0), d[0], d[3])
                        if d[7] == 0:
                            self.prm_wraps[node] = self.prm_wraps.get(node, 0) + 1
                if idx == 8:
                    c8 = cal_from_tlm8(d)
                    if c8 is not None:
                        self.cal[node] = c8
                if idx == 0 and len(d) >= 8:
                    a = self.axes.get(node)
                    if a is not None:
                        a.note_uptime((d[6] << 8) | d[7])

    def pos_true(self, node, max_age=1.0):
        return self.pos_true_t(node, max_age)[0]

    def pos_true_t(self, node, max_age=1.0):
        key = (node, 6)
        d = self.tlm.get(key)
        if d is None or len(d) < 4:
            return None, None
        t = self.tlm_t.get(key, 0.0)
        if (time.time() - t) > max_age:
            return None, None
        c = (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]
        if c >= (1 << 31):
            c -= (1 << 32)
        return c * RAD_PER_COUNT, t

    def pos_gate(self, node, since_fb=None):
        a = self.axes[node]
        if since_fb is None:
            alive = a.fb_count > 0 and (time.time() - a.fb_t) <= FB_FRESH_S
        else:
            alive = a.fb_count > since_fb
        return pos_gate(self.pos_true(node), a.pos, alive)

    def _wait_tlm(self, idx, timeout=0.5, fresh=False):
        if fresh:
            for n in self.order:
                self.tlm.pop((n, idx), None)
                self.tlm_t.pop((n, idx), None)
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._drain(min(deadline, time.time() + 0.02))
            if all((n, idx) in self.tlm for n in self.order):
                return True
        return False

    def set(self, node, p=None, v=None, kp=None, kd=None, t=None):
        a = self.axes[node]
        if p is not None:
            a.p = p
        if v is not None:
            a.v = v
        if kp is not None:
            a.kp = kp
        if kd is not None:
            a.kd = kd
        if t is not None:
            a.t = t

    def get(self, node):
        a = self.axes[node]
        return (a.pos, a.vel, a.iq)

    def tlm_gone(self, node):
        seen = [t for (n, _i), t in list(self.tlm_t.items()) if n == node]
        return bool(seen) and (time.time() - max(seen)) > BOARD_FRESH_S

    def is_tripped(self, node):
        v = self.axes[node].trip_now()
        if v is not None and self.tlm_gone(node)                 and (time.time() - self.axes[node].trip_t) > FB_FRESH_S:
            return None
        return v

    def any_tripped(self):
        st = dict((n, self.is_tripped(n)) for n in self.order)
        return ([n for n in self.order if st[n] is True],
                [n for n in self.order if st[n] is None])

    def _special(self, code):
        for n in self.order:
            self._send(0x100 + n, bytes([0xFF] * 7 + [code]))
            time.sleep(0.002)

    def enter(self, tlm_wait=0.3):
        self._wait_known(mode=True)
        bad = [(n, self.scale_of(n)[2]) for n in self.order if self.scale_of(n)[0] != "ok"]
        if bad:
            raise EnterRefused("モーターモードに入りませんでした（目盛り）:\n"
                               + "\n".join("  ノード%d: %s" % (n, w) for n, w in bad))
        bad = [(n, cal_why(self.cal_of(n))) for n in self.order if cal_why(self.cal_of(n)) is not None]
        if bad:
            raise EnterRefused("モーターモードに入りませんでした（較正）:\n"
                               + "\n".join("  ノード%d: %s" % (n, w) for n, w in bad))
        bad = [(n, w) for n in self.order for w in (self.board_mode_why(n),) if w is not None]
        if bad:
            raise EnterRefused("モーターモードに入りませんでした（基板のモード）:\n"
                               + "\n".join("  ノード%d の%s" % (n, w) for n, w in bad))
        before =dict((n, self.axes[n].fb_count) for n in self.order)
        self._special(0xFC)
        self._wait_tlm(6, timeout=tlm_wait, fresh=True)

        refused = []
        for n in self.order:
            a = self.axes[n]
            pt, why = self.pos_gate(n, since_fb=before[n])
            if why is None:
                a.p = pt
                a.v = 0.0
            else:
                refused.append((n, why))

        if refused:
            self._special(0xFD)
            raise EnterRefused(
                "モーターモードに入りませんでした:\n"
                + "\n".join("  ノード%d: %s" % (n, w) for n, w in refused)
                + "\n  → zero() で原点を取り直すか、テレメトリを有効にすること"
                  "（python3 ~/krc.py tlm 1）")

    def exit(self):
        try:
            self._drain_now()
        except OSError:
            pass
        self._special(0xFD)

    def zero(self, settle=0.3, flush=0.15):
        self._special(0xFE)
        for n in self.order:
            a = self.axes[n]
            a.p = 0.0
            a.v = 0.0
            a.pos = 0.0
        self._drain(time.time() + flush)
        self._wait_tlm(6, timeout=settle, fresh=True)

    def _special_one(self, node, code):
        self._send(0x100 + int(node), bytes([0xFF] * 7 + [code]))

    def zero_one(self, node, settle=0.3, flush=0.15):
        node = int(node)
        self._special_one(node, 0xFE)
        a = self.axes[node]
        a.p = 0.0
        a.v = 0.0
        a.pos = 0.0
        self._drain(time.time() + flush)
        self._wait_tlm(6, timeout=settle, fresh=True)

    def set_param(self, node, pid, value):
        x = param_lsb(pid, value)
        self._cfg(node, CFG_SET_PARAM, pid, x)
        return x

    def param_reported(self, node, pid):
        r = self.params.get((node, pid))
        return None if r is None else (param_value(pid, r[0]), r[1])

    def _cfg(self, node, sub, p1=0, p2=0):
        p1 &= 0xFFFF
        p2 &= 0xFFFF
        self._send(0x120 + int(node), bytes([0xA5, sub, p1 >> 8, p1 & 0xFF,
                                               p2 >> 8, p2 & 0xFF, 0, 0]))

    def set_limits(self, node, lo, hi):
        if not (lo < hi):
            raise ValueError("下限 %.3f >= 上限 %.3f（無効にするなら limits_off）" % (lo, hi))
        p1, p2 = limits_lsb(lo, hi)
        for x, p in ((lo, p1), (hi, p2)):
            if abs(p) > CFG_I16_MAX:
                raise ValueError("%.2f rad は基板に送れる範囲の外" % x)
        if not (p1 < p2):
            raise ValueError("下限 %.3f と上限 %.3f は 0.01 rad 刻みで同じ壁になる（無効にするなら limits_off）" % (lo, hi))
        self._cfg(node, 18, p1, p2)

    def limits_off(self, node):
        self._cfg(node, 18, 0, 0)

    def limits_reported(self, node):
        key = (int(node), 12)
        d = self.tlm.get(key)
        if d is None or len(d) < 4:
            return None
        lo, hi = limits_from_tlm12(d)
        return lo, hi, self.tlm_t.get(key, 0.0)

    def home(self, node, v_des=HOME_V_DES, kd_steps=HOME_KD_STEPS, stall_s=HOME_STALL_S,
             stall_rad=HOME_STALL_RAD, max_travel=HOME_MAX_TRAVEL, margin=HOME_MARGIN_RAD,
             timeout=HOME_DIR_TIMEOUT_S, progress=None):
        node = int(node)
        st = HomingStepper(node, v_des=v_des, kd_steps=kd_steps, stall_s=stall_s,
                           stall_rad=stall_rad, max_travel=max_travel, margin=margin,
                           timeout=timeout)
        last_sign = [None]
        ax = self.axes[node]
        ax.take_reboot()

        def upd(t, b):
            now = time.time()
            pos, pos_t = b.pos_true_t(node)
            cmd, act = st.step(now, pos, b.is_tripped(node), b.limits_reported(node), rebooted=ax.take_reboot(),
                               pos_t=pos_t)
            if act == "limits_off":
                b.limits_off(node)
            elif act == "zero":
                b.zero_one(node)
                st.zeroed(time.time())
            elif isinstance(act, tuple) and act[0] == "walls":
                b.set_limits(node, act[1], act[2])
            if progress and st.trace and st.trace[-1] != last_sign[0]:
                last_sign[0] = st.trace[-1]
                progress(*st.trace[-1])
            if cmd is not None:
                b.set(node, p=cmd[0], v=cmd[1], kp=cmd[2], kd=cmd[3], t=cmd[4])
            if act in ("done", "fail"):
                raise StopIteration
        try:
            self.spin(2 * timeout + 30.0, upd)
        except StopIteration:
            pass
        if st.result["ok"]:
            self.spin(0.3, lambda t, b: b.set(node, p=0.0, v=0.0, kp=HOME_SETTLE_KP,
                                              kd=HOME_SETTLE_KD, t=0.0))
        else:
            self.spin(0.3, lambda t, b: b.set(node, p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0))
        return st.result

    def _trip_op(self, action):
        assume = trip_assume(action)
        if assume is not None:
            for n in self.order:
                self.axes[n].set_tripped(assume, assumed=True)
        t0 = time.time()
        for delay, cid, data in trip_sequence(action, self.order):
            if delay > 0.0:
                self._drain(t0 + delay)
            self._send(cid, data)
        self._drain(time.time() + TRIP_ASK_S)
        return dict((n, self.axes[n].tripped) for n in self.order)

    def estop(self):
        return self._trip_op("estop")

    def clear(self):
        return self._trip_op("clear")

    def cycle(self, deadline):
        for n in self.order:
            self._send(0x100 + n, self._mit(self.axes[n]))
        before = dict((n, self.axes[n].fb_count) for n in self.order)
        self._drain(deadline)
        for n in self.order:
            if self.axes[n].fb_count == before[n]:
                self.axes[n].fb_missing += 1

    def spin(self, secs, update=None):
        dt = 1.0 / self.rate
        t0 = time.time()
        n = int(secs * self.rate)
        prev = t0
        self.period_log = []
        self.late_count = 0
        for i in range(n):
            if update is not None:
                update(time.time() - t0, self)
            deadline = t0 + (i + 1) * dt
            self.cycle(deadline)
            now = time.time()
            self.period_log.append(now - prev)
            prev = now
            if now > deadline + dt * 0.5:
                self.late_count += 1

    def stats(self):
        p = self.period_log
        if not p:
            return None
        p = p[2:]
        m = sum(p) / len(p)
        sd = math.sqrt(sum((x - m) ** 2 for x in p) / len(p))
        return dict(mean_ms=m * 1000.0, sd_ms=sd * 1000.0,
                    max_ms=max(p) * 1000.0, min_ms=min(p) * 1000.0,
                    late=self.late_count, n=len(p))

    def bus_load(self):
        per_axis = FRAME_S_8 + FRAME_S_6
        return len(self.order) * self.rate * per_axis


def bench(n_axes=12, rate=200.0, secs=3.0):
    nodes = list(range(1, int(n_axes) + 1))
    bus = KrcBus(nodes, rate=rate)
    print("=== 周期の揺れ（%d軸ぶん送信 / %.0f Hz / %.1f 秒）===" % (len(nodes), rate, secs))
    print("  バス占有の見積り: %.1f %%" % (bus.bus_load() * 100.0))
    if bus.bus_load() > 1.0:
        print("  !! 1Mbps では入らない。レートか軸数を下げること")

    bus.spin(secs, lambda t, b: [b.set(n, p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0)
                                 for n in b.order])
    s = bus.stats()
    if s is None:
        print("  データなし")
        return 1
    print("  周期  平均 %.3f ms / 標準偏差 %.3f ms / 最大 %.3f ms / 最小 %.3f ms"
          % (s["mean_ms"], s["sd_ms"], s["max_ms"], s["min_ms"]))
    print("  目標 %.3f ms に対して、遅れた周期 %d / %d"
          % (1000.0 / rate, s["late"], s["n"]))
    for n in bus.order:
        a = bus.axes[n]
        if a.fb_count or a.fb_missing:
            print("  ノード%-2d フィードバック %d 本 / 欠測 %d"
                  % (n, a.fb_count, a.fb_missing))
    jitter_rad = s["sd_ms"] * 1e-3 * 10.0
    print("  → 周期の揺れが位置に与える影響（10rad/sで動作中）: %.4f rad" % jitter_rad)
    return 0


def demo(node=1, secs=4.0, rate=200.0):
    bus = KrcBus([int(node)], rate=rate)
    bus.zero()
    bus.enter()
    w = 2.0 * math.pi * 1.0

    def upd(t, b):
        b.set(int(node), p=0.6 * math.sin(w * t), v=0.6 * w * math.cos(w * t),
              kp=18.0, kd=0.12, t=0.0)

    bus.spin(secs, upd)
    bus.exit()
    s = bus.stats()
    a = bus.axes[int(node)]
    print("  周期 平均 %.3f ms / 標準偏差 %.3f ms / 最大 %.3f ms"
          % (s["mean_ms"], s["sd_ms"], s["max_ms"]))
    print("  フィードバック %d 本 / 欠測 %d" % (a.fb_count, a.fb_missing))
    print("  最後の状態: 位置 %.4f rad / 速度 %.2f rad/s / iq %.3f A"
          % (a.pos, a.vel, a.iq))
    return 0


def safety(node=1):
    n = int(node)
    bus = KrcBus([n], rate=200)
    bus.clear()
    bus.zero()
    bus.enter()
    print("=== 非常停止（ノード %d）===" % n)

    bus.spin(1.0, lambda t, b: b.set(n, p=0.0, v=20.0, kp=0.0, kd=0.1))
    print("  停止前   速度 %6.2f rad/s" % bus.get(n)[1])

    bus.estop()
    time.sleep(1.0)
    bus.spin(1.0, lambda t, b: b.set(n, p=0.0, v=20.0, kp=0.0, kd=0.1))
    v = bus.get(n)[1]
    trip = bus.is_tripped(n) is True
    miss = bus.axes[n].fb_missing
    print("  停止後   速度 %6.2f rad/s / 遮断の申告 %s / 欠測 %d"
          % (v, "あり" if trip else "**なし**", miss))
    print("           %s"
          % ("OK（止まって、かつ遮断を申告している）"
             if (abs(v) < 1.0 and trip) else "**期待どおりでない**"))

    bus.clear()
    bus.zero()
    bus.enter()
    bus.spin(1.0, lambda t, b: b.set(n, p=0.0, v=20.0, kp=0.0, kd=0.1))
    v2 = bus.get(n)[1]
    print("  解除後   速度 %6.2f rad/s  %s"
          % (v2, "OK（復帰した）" if abs(v2) > 5.0 else "**復帰できていない**"))
    bus.exit()
    return 0 if (abs(v) < 1.0 and trip and abs(v2) > 5.0) else 1


def pos(node=1, secs=0.6):
    n = int(node)
    bus = KrcBus([n], rate=200)
    bus._wait_tlm(6, timeout=float(secs), fresh=True)
    pt = bus.pos_true(n)
    print("=== 位置（ノード %d）===" % n)
    if pt is None:
        print("  累積位置が読めない。テレメトリが止まっているか、基板が喋っていない")
        print("  → python3 ~/krc.py tlm 1  で戻せる")
        return 1
    print("  累積位置   %+.4f rad（%+.2f 回転）" % (pt, pt / (2.0 * math.pi)))
    print("  送れる範囲 ±%.1f rad / 入ってよいのは ±%.1f rad まで" % (P_MAX, POS_SAFE_RAD))
    _p, why = bus.pos_gate(n)
    if why is None:
        print("  → 位置モードに入れる")
        return 0
    print("  → **入れない。** %s" % why)
    print("     このまま位置指令を出すと全力で戻ろうとする")
    print("     zero() で原点を取り直すこと（./krcbus.py demo は自分で取り直す）")
    return 1


def main():
    argv = sys.argv[1:]
    if not argv:
        print(__doc__)
        return 1
    if argv[0] == "bench":
        return bench(*[float(x) for x in argv[1:]])
    if argv[0] == "demo":
        return demo(*[float(x) for x in argv[1:]])
    if argv[0] == "safety":
        return safety(*[float(x) for x in argv[1:]])
    if argv[0] == "pos":
        return pos(*[float(x) for x in argv[1:]])
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
