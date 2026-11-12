#!/usr/bin/env python3
import argparse
import collections
import contextlib
import copy
import io
import json
import math
import os
import select
import shutil
import socket
import struct
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from krcbus import (KrcBus, P_MAX, V_MAX, KP_MAX, KD_MAX, T_MAX, I_MAX,
                    RAD_PER_COUNT, POS_SAFE_RAD, FB_FRESH_S, KT_NM_PER_A,
                    POS_CMD_LO, POS_CMD_HI, LIMIT_RAD_PER_LSB,
                    THERM_TRIP_PCT, THERM_CLEAR_PCT, trip_sequence, trip_assume,
                    valid_node, fb_saturated, BOARD_FRESH_S, NODE_MAX,
                    KE_V_S_PER_RAD, COPPER_ALPHA, TEMP_INVALID_RAW, sine_gate,
                    HomingStepper, FOC_TIMEOUT_MS, limits_from_tlm12, REBOOT_WHY, REBOOT_WHY_EN)
import krcleg
import krcbus
import offset_full

class Why(str):
    def __new__(cls, ja, en=None):
        s = str.__new__(cls, ja)
        s.en = en
        return s


def en_of(x):
    return getattr(x, "en", None)


def wjoin(parts, sep="／", en_sep=" / "):
    parts = list(parts)
    ens = [en_of(p) for p in parts]
    return Why(sep.join(parts), en_sep.join(ens) if parts and all(ens) else None)


PARAM_SAVE_DONE = ("saved", "failed", "reset", "differs")
GAIN_DEFAULT = (12.0, 0.15)
GAIN_BY_NODE = {5: (15.0, 0.01)}
PARAM_ECHO_S = 0.3

WATCHDOG_S = 1.5
STALL_MAX_S = 1.0
HIST_N = 4000
TRAJ_V_MAX = 40.0
TRAJ_A_MAX = 400.0
TRAJ_TAU_S = 0.08
STOP_EMA_S = 0.08
STOP_WARM_S = 0.03
STOP_N = 2
STOP_PEAK_S = 0.5
TRAJ_F_SNAP_RAD = 0.02
TRAJ_SNAP_RAD = 1e-4
TRAJ_DT_MAX = 0.05
HANDOFF_N = 200
SAMPLE_HZ = 100.0

TEMP_DT = 0.1
TEMP_HIST_DT = 1.0
TEMP_VIEW_S = 60.0
TEMP_VIEW_LO_C, TEMP_VIEW_HI_C = 15.0, 85.0
VEL_VIEW_K = 1.2
VEL_VIEW_STEP = 10.0
TEMP_FIT_WIN = 100
TEMP_IQ_MIN = 0.3
TEMP_IQ_SPREAD = 0.25
TEMP_EMF_RATIO = 0.5
TEMP_TAU = 20.0
TEMP_FIT_RMS_V = 0.35
TEMP_DV_TAU = 60.0
TEMP_FILE_SHOW_AFTER = 1
TEMP_R0_AFTER = 100
TEMP_STALE_S = 10.0
TEMP_GAP_CAP_S = 1.0
TEMP_DV_BOUND_V = 0.20
TEMP_ERR_BUDGET_C = 10.0
TEMP_DV_SE_K = 2.0
TEMP_DV_SE_BETTER = 0.8
TEMP_FIT_MIN_PTS = 20
WIND_REF_PATH = os.environ.get("KRC_WIND_REF") or os.path.expanduser("~/krc_wind_ref.json")
WIND_REF_ACTIONS = ("wind_ref_set", "wind_ref_clear")
WIND_REF_LOG_DIR = os.environ.get("KRC_WIND_REF_LOG") or os.path.expanduser("~/krc_wind_ref_log")
TEMP_REF_GAP_S = 3.0
TEMP_REF_RUN_S = 60.0
TEMP_REF_MIN_S = 5.0
TEMP_REF_MIN_PTS = 20
TEMP_REF_IQ_REL = 0.10
TEMP_REF_OUT_K = 4.0
TEMP_REF_T0_S = 1.0

MAX_BODY = 65536

UI_KP_MAX = 80.0
UI_KD_MAX = 1.0
UI_T_MAX = 2.4
UI_V_MAX = 30.0
UI_AMP_MAX = 3.0
UI_FREQ_MAX = 5.0

TRIP_NAMES = {0: "正常", 1: "nFAULT（DRV8323）", 2: "過電流",
              3: "巻線温度", 4: "非常停止", 5: "基板温度（過熱・読めない）"}
TRIP_NAMES_EN = {0: "OK", 1: "nFAULT (DRV8323)", 2: "Overcurrent",
                 3: "Winding temperature", 4: "E-stop", 5: "Board temperature (overheat / unreadable)"}
TRIP_CODE_THERMAL = 3
TRIP_CODE_BOARD_HOT = 5


TRIP_UNKNOWN_TEXT = Why("遮断しているか分かりません（基板の返事もテレメトリの理由も"
                        "来ていない）。「遮断を解除」を押すと基板に聞き直します",
                        "Not known whether it is tripped (no reply and no trip reason in the telemetry). "
                        "Press \"Clear trip\" to ask the board again")
TRIP_NOREPLY_TEXT = Why("基板から返事がありません。基板の電源と CAN の配線を確かめてください（テレメトリを止めているだけなら"
                        "「遮断を解除」で聞き直せます）",
                        "No reply from the board. Check the board power and the CAN wiring (if only the telemetry is off, "
                        "\"Clear trip\" asks the board again)")


def trip_state(fb_tripped, reason):
    if fb_tripped is True or (reason is not None and reason != 0):
        return True
    if fb_tripped is False or reason == 0:
        return False
    return None


def trip_label(tripped, reason):
    if not tripped and not reason:
        return None
    if reason is None:
        return Why("読めない（テレメトリ 0x304 が来ない）", "reason unknown (telemetry 0x304 not arriving)")
    if reason == 0:
        return TRIP_REASON_PENDING
    return Why(TRIP_NAMES.get(reason, str(reason)), TRIP_NAMES_EN.get(reason, str(reason)))


TRIP_REASON_PENDING = Why("理由を確かめています", "checking the reason")

NO_TARGET_TEXT = "その軸は動かす対象ではありません"

TRIP_ACTION_TEXT = "「遮断を解除」を押してください。"
TRIP_ACTION_TEXT_EN = "Press \"Clear trip\"."
TRIP_CLEARING_TEXT = Why("解除の返事を待っています。", "Waiting for the board to confirm the clear.")
TRIP_CLEAR_WAIT_S = 1.0
SCALE_WHY_EN = {"unknown": "board scale not read yet (wait a few seconds)",
                "mismatch": "board scale differs from the host (fix with krc.py param)",
                "legacy": "old firmware without scale readback (reflash before arming)"}
CAL_WHY_EN = {False: "not calibrated (no electrical offset; provision first)",
              True: "calibration not read yet (telemetry block 8; wait a few seconds)"}
TRIP_OFF_AXIS_TEXT = ("この軸は「動かす」から外してあるので、"
                      "ほかの軸はこのまま通電できます。")


def _axes_names(nodes):
    nodes = list(nodes)
    return ("軸" + "・".join(str(n) for n in nodes), "Axis " + ", ".join(str(n) for n in nodes), len(nodes) == 1)


def arm_sent_note(nodes):
    ja, en, one = _axes_names(nodes)
    return Why("%s に通電を送りました（基板の返事を確かめています）" % ja,
               "%s: arming (waiting for the %s reply)" % (en, "board's" if one else "boards'"))


def arm_done_note(nodes):
    ja, en, _one = _axes_names(nodes)
    return Why("%s をモーターモードに入れました" % ja, "%s armed (motor mode)" % en)

MODES = ("free", "damp", "hold", "sine", "vel", "torque", "kt_test", "pos_test")
TORQUE_MODES = ("torque", "kt_test")
KT_RAMP_S = 0.2
KT_SECS_DEF, KT_SECS_MIN, KT_SECS_MAX = 3.0, 0.5, 10.0
PT_ACTIONS = ("pt_start", "pt_stop")
PT_KEEP = PT_ACTIONS + ("node", "rec_start")
PT_PTS_DEF = "0, 45, 90, 135, 180, 225, 270, 315, 360, 315, 270, 225, 180, 135, 90, 45"
PT_SPEED_DEF = 180.0
PT_ACC_DEG = 2000.0
PT_DWELL_DEF, PT_DWELL_MAX = 1.0, 60.0
PT_STEP_DWELL_MIN = 0.05
PT_N_MIN, PT_N_MAX = 2, 64
GEAR_RATIO = krcleg.RATIO_DEFAULT
WCAL_ACTIONS = ("wcal_start", "wcal_cancel")
WCAL_AMPS_DEF, WCAL_AMPS_MIN = 3.0, 1.0
WCAL_SECS_DEF, WCAL_SECS_MIN, WCAL_SECS_MAX = KT_SECS_MAX, TEMP_REF_MIN_S + 2.0, KT_SECS_MAX
WCAL_LEAD_S = 0.15
WCAL_POLL_S = 0.05
POS_MODES = ("hold", "sine", "leg_sine", "leg_traj", "pos_test")
LOST_HOLD_MODES = POS_MODES + ("homing",)
LEG_AUTO_MODES = ("leg_sine", "leg_traj")
HOMING_POWER_LOST = "通電が落ちた（解除・停止・画面の応答切れ）"
REC_DIR = os.environ.get("KRC_REC_DIR") or os.path.expanduser("~/krc_rec")
DEMO_ZERO_PATH = os.environ.get("KRC_DEMO_ZERO") or os.path.expanduser("~/krc_demo_zero.json")
REC_FLUSH_S = 0.5
REC_COLS = ("t", "node", "mode", "cmd_p", "cmd_v", "kp", "kd", "tff", "pos", "vel", "iq",
            "tau_motor", "tout", "tin", "tau_ext_out", "tau_fric_out", "tau_grav_out",
            "temp_board", "temp_wind")
GRAV_SAMPLE_S = 1.0
LEG_TRAJ_CHECK_SUB = 8
MANUAL_POS_MODES = ("hold",)
REZERO_TRIES = 3
LEG_SINE_CHECK_N = 64
LEG_SINE_DH_S = 0.001
JT_N_MIN, JT_N_MAX = 8, 2000
JT_NODES_MAX = 12
JT_WRAP_RAD = 4.0 * math.pi
CFG_SET_TLM = 19
JT_T_MIN, JT_T_MAX = 0.1, 20.0
JT_ONCE_T_MAX = 60.0
JT_SPEED_MIN, JT_SPEED_MAX = 0.05, 1.0
JT_SPEED_RATE = 1.0
JT_BLEND_S = 0.6
JT_FOLLOW_WN = 8.0
JT_APPROACH_V = 3.0
JT_GLIDE_MIN = 0.05
JT_V_LIMIT = 60.0
JT_STEP_MAX = 0.6
JT_ARRIVE_RAD = 1e-3
JT_KEEP = ("jtraj", "jtraj_speed", "jtraj_stop", "jtraj_gain", "node", "rec_start")
JT_TFF_MAX = 5.0
JT_TFF_K_MAX = 1.5
OFFCAL_BUSY = ("running", "wait_push", "seek", "back")
OCG_SEEK_V = 3.0
OCG_KP, OCG_KD = GAIN_DEFAULT
OCG_STOP_IQ = 1.0
OCG_STOP_VSLOW = 0.5
OCG_SEEK_MAX_REV = 9.5
OCG_BACK_REV = 1.5
OCG_ARRIVE_RAD = 0.05
OCG_IDLE_WAIT_S = 3.0
OFFCAL_ACTIONS = ("offcal_start", "offcal_next", "offcal_cancel", "offcal_apply", "offcal_save")
OFFCAL_SP_TEXT = {0xFC: Why("通電（0xFC）が来た", "arm command 0xFC"), 0xFD: Why("解除（0xFD）が来た", "disarm command 0xFD"),
                  0xFE: Why("原点を移す指令（0xFE）が来た", "re-zero command 0xFE")}


class BadValue(ValueError):
    pass


def num(v, name):
    if isinstance(v, bool):
        raise BadValue("%s が数として読めません: %r" % (name, v))
    try:
        x = float(v)
    except (TypeError, ValueError, OverflowError):
        raise BadValue("%s が数として読めません: %r" % (name, v))
    if not math.isfinite(x):
        raise BadValue("%s が数として読めません: %r" % (name, v))
    return x


def num_int(v, name):
    if isinstance(v, bool):
        raise BadValue("%s が整数として読めません: %r" % (name, v))
    if isinstance(v, float) and math.isfinite(v) and v != int(v):
        raise BadValue("%s が整数として読めません（小数）: %r" % (name, v))
    try:
        return int(v)
    except (TypeError, ValueError, OverflowError):
        raise BadValue("%s が整数として読めません: %r" % (name, v))


def no_nan(o):
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return dict((k, no_nan(v)) for k, v in o.items())
    if isinstance(o, (list, tuple)):
        return [no_nan(v) for v in o]
    return o


def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def s16(d, i):
    v = (d[i] << 8) | d[i + 1]
    return v - 65536 if v > 32767 else v


def u16(d, i):
    return (d[i] << 8) | d[i + 1]


def discover(iface="can0", secs=2.0):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        s.bind((iface,))
        s.setblocking(False)
        found = set()
        until = time.time() + float(secs)
        while True:
            left = until - time.time()
            if left <= 0:
                break
            r, _, _ = select.select([s], [], [], left)
            if not r:
                continue
            try:
                frame = s.recv(16)
            except BlockingIOError:
                continue
            cid = struct.unpack("=I", frame[:4])[0] & 0x1FFFFFFF
            n = nodes_of(cid)
            if n:
                found.add(n)
        return sorted(found)
    finally:
        s.close()


def nodes_of(cid):
    if 0x300 <= cid <= 0x3FF:
        n = (cid - 0x300) // 0x10
    elif 0x201 <= cid <= 0x20F:
        n = cid - 0x200
    else:
        return 0
    return n if valid_node(n) else 0


class GuiBus(KrcBus):
    pass


class TempHist(object):

    def __init__(self, view_s=TEMP_VIEW_S, dt=TEMP_HIST_DT):
        self.view_s = float(view_s)
        self.dt = float(dt)
        self.cap = int(math.ceil(self.view_s / self.dt)) + 2
        self.pts = []

    def add(self, t, board_c, wind_c):
        if self.pts and (t - self.pts[-1][0]) < self.dt * 0.999:
            return False
        self.pts.append((t, board_c, wind_c))
        if len(self.pts) > self.cap:
            self.pts = self.pts[-self.cap:]
        return True

    def recent(self, now):
        lo = now - self.view_s
        return [p for p in self.pts if p[0] >= lo]


WindBase = collections.namedtuple("WindBase", "r0 iq0 inv0 t0 src dv dv_se date")
WIND_BASE_NONE = WindBase(None, None, None, None, None, None, None, None)


def wind_base_of(ent):
    return WindBase(ent["r"], ent.get("iq"), ent["inv"], ent["t"], "file",
                    ent.get("dv"), ent.get("dv_se"), ent.get("date"))


def _wind_ref_entry(v):
    if not isinstance(v, dict):
        return None, "値が表ではない"

    def fin(k, lo, hi):
        x = v.get(k)
        if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or not (lo <= x <= hi):
            raise ValueError(k)
        return float(x)
    try:
        out = dict(r=fin("r", 0.3, 10.0), t=fin("t", -40.0, 150.0), inv=fin("inv", 1e-3, 1.0 / TEMP_IQ_MIN))
        if v.get("iq") is not None:
            out["iq"] = fin("iq", TEMP_IQ_MIN, 1e3)
        if v.get("dv") is not None or v.get("dv_se") is not None:
            out["dv"], out["dv_se"] = fin("dv", -1.5, 1.5), fin("dv_se", 1e-9, 1.5)
    except ValueError as e:
        return None, "%s が無いか範囲の外（%r）" % (e, v.get(str(e)))
    if v.get("date") is not None:
        out["date"] = str(v["date"])[:40]
    fit = v.get("fit")
    if isinstance(fit, dict):
        keep = {}
        for k, x in fit.items():
            if k in ("n", "secs", "slope", "se", "se_c") and isinstance(x, (int, float)) and not isinstance(x, bool) \
                    and math.isfinite(x):
                keep[k] = x
            elif k in ("start", "log") and isinstance(x, str):
                keep[k] = x[:120]
        if keep:
            out["fit"] = keep
    return out, None


def _wind_ref_keep(path):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    bad, k = "%s.bad-%s" % (path, stamp), 0
    while os.path.exists(bad):
        k += 1
        bad = "%s.bad-%s-%d" % (path, stamp, k)
    try:
        shutil.copyfile(path, bad)
        return "原本は %s に残しました" % bad, "original kept as %s" % bad
    except OSError as e:
        return "原本を写せませんでした: %s" % e, "could not copy the original: %s" % e


def wind_ref_load(path=None):
    path = path or WIND_REF_PATH
    first_ja, first_en = "起動後の最初の抵抗を基準にしています", "using the first resistance after start-up"
    try:
        with io.open(path, encoding="utf-8-sig") as f:
            d = json.load(f)
    except FileNotFoundError:
        return {}, {}
    except OSError as e:
        return {}, {None: Why("基準ファイル %s が開けないので、%s（%s）" % (path, first_ja, e),
                              "Can't open the reference file %s, %s (%s)" % (path, first_en, e))}
    except ValueError as e:
        d, what, what_en = None, "JSON として読めない: %s" % e, "not valid JSON: %s" % e
    else:
        what, what_en = "中身が軸ごとの表ではない", "not a per-axis table"
    if not isinstance(d, dict):
        kept, kept_en = _wind_ref_keep(path)
        return {}, {None: Why("基準ファイル %s が読めないので、%s（%s。%s）" % (path, first_ja, what, kept),
                              "Can't read the reference file %s, %s (%s; %s)" % (path, first_en, what_en, kept_en))}
    refs, bad = {}, []
    for k, v in d.items():
        try:
            n = int(k)
        except (TypeError, ValueError):
            bad.append((None, "鍵 %r が軸番号ではない" % (k,)))
            continue
        ent, why = _wind_ref_entry(v)
        if ent is None:
            bad.append((n, why))
        else:
            refs[n] = ent
    errs = {}
    if bad:
        kept, kept_en = _wind_ref_keep(path)
        for n, why in bad:
            who = "" if n is None else " の軸 %d" % n
            errs[n] = Why("基準ファイル %s%s が読めないので、%s（%s。%s）" % (path, who, first_ja, why, kept),
                          "Can't read %s in the reference file %s, %s (%s; %s)"
                          % ("an entry" if n is None else "axis %d" % n, path, first_en, why, kept_en))
    return refs, errs


WIND_RUN_COLS = ("t_s", "epoch", "iq_a", "w_rads", "vq_v", "r_app_ohm", "board_c", "kind", "fit")


def wind_run_save(node, fit, log_dir=None):
    log_dir = log_dir or WIND_REF_LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    name = "%d_%s.csv" % (node, time.strftime("%Y%m%d-%H%M%S", time.localtime(fit["t_start"])))
    path, k = os.path.join(log_dir, name), 0
    while os.path.exists(path):
        k += 1
        name = "%d_%s-%d.csv" % (node, time.strftime("%Y%m%d-%H%M%S", time.localtime(fit["t_start"])), k)
        path = os.path.join(log_dir, name)
    used = fit["used"]

    def num(x, nd):
        return "" if x is None else ("%.*f" % (nd, x))
    with io.open(path, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(WIND_RUN_COLS) + "\n")
        for i, p in enumerate(fit["pts"]):
            t, iq, w, vq, r, bc, kind = p
            f.write("%s,%s,%s,%s,%s,%s,%s,%s,%d\n" % (num(t - fit["t_start"], 3), num(t, 3), num(iq, 4), num(w, 4),
                                                   num(vq, 4), num(r, 5), num(bc, 2), kind, 1 if i in used else 0))
        f.flush()
        os.fsync(f.fileno())
    return name


def wind_ref_save(refs, path=None):
    path = path or WIND_REF_PATH
    tmp = path + ".tmp"
    with io.open(tmp, "w", encoding="utf-8") as f:
        json.dump(dict((str(int(n)), e) for n, e in sorted(refs.items())), f, ensure_ascii=False, indent=1)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


class WindRun(object):
    __slots__ = ("t0", "t_last", "open", "pts", "after_gap", "prev")

    def __init__(self, now, after_gap=False, prev=None):
        self.t0 = now
        self.t_last = now
        self.open = True
        self.pts = []
        self.after_gap = after_gap
        self.prev = prev

    def chain_pts(self, since=None):
        runs, r = [], self
        while r is not None:
            runs.append(r)
            r = r.prev
        out = []
        for r in reversed(runs):
            out.extend(p for p in list(r.pts) if since is None or p[0] >= since)
        return out


class WCal(object):
    __slots__ = ("node", "pulse", "t_req", "t_fit", "amps", "secs", "prev", "stage", "saving", "res", "msg", "why")

    def __init__(self, node, pulse, prev):
        self.node = node
        self.pulse = pulse
        self.t_req = pulse["t0"]
        self.t_fit = pulse["t0"] + KT_RAMP_S + pulse["secs"] - WCAL_LEAD_S
        self.amps = pulse["amps"]
        self.secs = pulse["secs"]
        self.prev = prev
        self.stage = "running"
        self.saving = False
        self.res = self.msg = self.why = None


class WindingTemp(object):

    def __init__(self, ke=KE_V_S_PER_RAD, alpha=COPPER_ALPHA, ref=None):
        self.ke = float(ke)
        self.alpha = float(alpha)
        self._base_lock = threading.Lock()
        self.base = WIND_BASE_NONE if ref is None else wind_base_of(ref)
        self.samples = collections.deque(maxlen=TEMP_FIT_WIN)
        self.r = None
        self.iq_now = None
        self.inv_now = None
        self.dv = None
        self.dv_se = None
        self.t_last = 0.0
        self.n_ok = 0
        self.n_push = 0
        self.t_push = 0.0
        self.last = None
        self.n_lowi = 0
        self.n_emf = 0
        self.last_sd = None
        self.last_rms = None
        self.n_rms_rej = 0
        self.run = None

    def _base_field(name):
        def get(self):
            return getattr(self.base, name)

        def put(self, v):
            with self._base_lock:
                self.base = self.base._replace(**{name: v})
        return property(get, put)
    r0 = _base_field("r0")
    iq0 = _base_field("iq0")
    inv0 = _base_field("inv0")
    t0 = _base_field("t0")
    src = _base_field("src")
    del _base_field

    def set_base(self, ent):
        with self._base_lock:
            self.base = WIND_BASE_NONE if ent is None else wind_base_of(ent)

    def dv_pair(self, b=None):
        b = self.base if b is None else b
        if self.dv is not None:
            return self.dv, self.dv_se
        return b.dv, b.dv_se

    def push(self, iq_a, w_rads, vq_v, now, board_c):
        self.n_push += 1
        self.t_push = now
        if abs(iq_a) < TEMP_IQ_MIN:
            self.n_lowi += 1
            self.last = "lowi"
            if self.run is not None:
                self.run.open = False
            return
        emf = self.ke * w_rads
        if abs(emf) > TEMP_EMF_RATIO * abs(vq_v):
            self.n_emf += 1
            self.last = "emf"
            self._run_add(now, iq_a, w_rads, vq_v, None, board_c, "emf")
            return
        s = 1.0 if iq_a > 0 else -1.0
        ai = abs(iq_a)
        u = (vq_v - emf) * s
        r_inst = u / ai
        if not (0.3 <= r_inst <= 10.0):
            self.last = "range"
            self._run_add(now, iq_a, w_rads, vq_v, r_inst, board_c, "range")
            return
        self.last = "ok"
        self._run_add(now, iq_a, w_rads, vq_v, r_inst, board_c, "ok")
        self.samples.append((ai, u, now))
        if self.r is None:
            a = 1.0
        else:
            a = max(min(now - self.t_last, TEMP_GAP_CAP_S) / TEMP_TAU,
                    1.0 / (self.n_ok + 1.0))
        self.r = r_inst if self.r is None else self.r + a * (r_inst - self.r)
        self.iq_now = ai if self.iq_now is None else self.iq_now + a * (ai - self.iq_now)
        self.inv_now = (1.0 / ai) if self.inv_now is None else self.inv_now + a * (1.0 / ai - self.inv_now)
        self.t_last = now
        self.n_ok += 1
        self._fit(now)
        if self.base.r0 is None and board_c is not None and self.n_ok >= TEMP_R0_AFTER:
            with self._base_lock:
                if self.base.r0 is None:
                    self.base = WindBase(self.r, self.iq_now, self.inv_now, float(board_c), "first",
                                         None, None, None)

    def _fit(self, now):
        pts = [p for p in self.samples if (now - p[2]) <= TEMP_FIT_WIN * TEMP_DT]
        if len(pts) < TEMP_FIT_MIN_PTS:
            return
        xs = [p[0] for p in pts]
        n = float(len(pts))
        mx = sum(xs) / n
        sd = math.sqrt(sum((x - mx) ** 2 for x in xs) / n)
        self.last_sd = sd
        if sd < TEMP_IQ_SPREAD:
            return
        sx = sum(xs)
        sy = sum(p[1] for p in pts)
        sxx = sum(x * x for x in xs)
        sxy = sum(p[0] * p[1] for p in pts)
        den = n * sxx - sx * sx
        if den <= 0:
            return
        r = (n * sxy - sx * sy) / den
        dv = (sy - r * sx) / n
        rms = math.sqrt(sum((p[1] - (r * p[0] + dv)) ** 2 for p in pts) / n)
        self.last_rms = rms
        if rms > TEMP_FIT_RMS_V:
            self.n_rms_rej += 1
            return
        if not (0.3 <= r <= 10.0 and abs(dv) <= 1.5):
            return
        se = rms * math.sqrt(sxx / den)
        if self.dv is None or se < TEMP_DV_SE_BETTER * self.dv_se:
            self.dv, self.dv_se = dv, se
        else:
            a = TEMP_DT / TEMP_DV_TAU
            self.dv += a * (dv - self.dv)
            self.dv_se += a * (se - self.dv_se)

    def _run_add(self, now, iq_a, w_rads, vq_v, r_app, board_c, kind):
        run = self.run
        if run is None or not run.open or now - run.t_last > TEMP_REF_GAP_S:
            gap = run is not None and run.open
            prev = run if gap else None
            r = prev
            while r is not None:
                if r.prev is not None and r.prev.t0 < now - TEMP_REF_RUN_S:
                    r.prev = None
                r = r.prev
            run = WindRun(now, after_gap=gap, prev=prev)
            self.run = run
        run.t_last = now
        if now - run.t0 <= TEMP_REF_RUN_S:
            run.pts.append((now, iq_a, w_rads, vq_v, r_app, board_c, kind))

    def live_run(self, now):
        run = self.run
        if run is None or not run.open or now - run.t_last > TEMP_REF_GAP_S:
            return None
        return run

    def ref_fit(self, now, node):
        run = self.live_run(now)
        flow_ja = "当てはめられるのは電流を流して押さえている間だけです"
        flow_en = "it can only be fitted while current is flowing"
        if run is None:
            return None, Why("軸%d の電流が流れていません（%s）" % (node, flow_ja),
                             "Axis %d: no current flowing (%s)" % (node, flow_en))
        if run.after_gap:
            return None, Why("軸%d は流している途中でテレメトリが %.0f 秒より長く途切れたので、流し始めが分かりません"
                             "（覚えません。電流をいったん止めて、流し直してください）" % (node, TEMP_REF_GAP_S),
                             "Axis %d: telemetry dropped for more than %.0f s while the current was flowing, so the start "
                             "is unknown (nothing stored; stop the current and start again)" % (node, TEMP_REF_GAP_S))
        pts = list(run.pts)
        t_start = run.t0
        t_skip = t_start + KT_RAMP_S + TEMP_DT
        idx = [i for i, p in enumerate(pts) if p[6] == "ok" and p[0] >= t_skip]
        span = (pts[idx[-1]][0] - t_start) if idx else 0.0
        if span < TEMP_REF_MIN_S or len(idx) < TEMP_REF_MIN_PTS:
            return None, Why("軸%d は流し始めからの記録がまだ短いです（%.1f / %.0f 秒・%d / %d 点。電流を流し続けてください）"
                             % (node, span, TEMP_REF_MIN_S, len(idx), TEMP_REF_MIN_PTS),
                             "Axis %d: too little since the current started (%.1f / %.0f s, %d / %d points; keep it flowing)"
                             % (node, span, TEMP_REF_MIN_S, len(idx), TEMP_REF_MIN_PTS))
        bc0 = next((p[5] for p in pts if p[5] is not None and p[0] - t_start <= TEMP_REF_T0_S), None)
        if bc0 is None:
            return None, Why("軸%d は流し始めの基板温度が読めていません（電流をいったん止めて、流し直してください）" % node,
                             "Axis %d: board temperature unread when the current started (stop and restart the current)" % node)
        ais = [abs(pts[i][1]) for i in idx]
        m_ai = sum(ais) / len(ais)
        rel = math.sqrt(sum((x - m_ai) ** 2 for x in ais) / len(ais)) / m_ai
        if rel > TEMP_REF_IQ_REL:
            return None, Why("軸%d は流し始めから電流が一定ではありません（ばらつき %.0f%%・%.0f%% まで。同じ電流で流し直してください）"
                             % (node, 100.0 * rel, 100.0 * TEMP_REF_IQ_REL),
                             "Axis %d: current not steady since it started (spread %.0f%%, limit %.0f%%; restart at one current)"
                             % (node, 100.0 * rel, 100.0 * TEMP_REF_IQ_REL))

        def line(use):
            xs = [pts[i][0] - t_start for i in use]
            ys = [pts[i][4] for i in use]
            n = float(len(use))
            sx, sy = sum(xs), sum(ys)
            sxx = sum(x * x for x in xs)
            sxy = sum(x * y for x, y in zip(xs, ys))
            den = n * sxx - sx * sx
            if den <= 0:
                return None
            b = (n * sxy - sx * sy) / den
            a = (sy - b * sx) / n
            res = [y - (a + b * x) for x, y in zip(xs, ys)]
            s = math.sqrt(sum(e * e for e in res) / max(n - 2.0, 1.0))
            return a, b, s, s * math.sqrt(sxx / den), res
        f = line(idx)
        if f is not None and f[2] > 0:
            keep = [i for i, e in zip(idx, f[4]) if abs(e) <= TEMP_REF_OUT_K * f[2]]
            if len(keep) < len(idx) and len(keep) >= TEMP_REF_MIN_PTS:
                idx, f = keep, line(keep)
        if f is None:
            return None, Why("軸%d の流し始めの R を当てはめられません（時刻が揃っていない）" % node,
                             "Axis %d: can't fit the starting R (no time spread)" % node)
        a, b, s, se, _ = f
        if not (0.3 <= a <= 10.0):
            return None, Why("軸%d の流し始めの R が範囲の外です（%.3f Ω）" % (node, a),
                             "Axis %d: starting R out of range (%.3f Ω)" % (node, a))
        se_c = se / (a * self.alpha)
        invs = [1.0 / abs(pts[i][1]) for i in idx]
        return dict(r=a, t=float(bc0), iq=sum(abs(pts[i][1]) for i in idx) / len(idx), inv=sum(invs) / len(invs),
                    n=len(idx), secs=pts[idx[-1]][0] - t_start, slope=b, se=se, se_c=se_c, t_start=t_start,
                    pts=pts, used=frozenset(idx)), None

    def mismatch_c(self, b=None):
        b = self.base if b is None else b
        if b.r0 is None or self.inv_now is None or b.inv0 is None:
            return 0.0
        dv, se = self.dv_pair(b)
        bound = (TEMP_DV_BOUND_V if dv is None
                 else min(TEMP_DV_BOUND_V, TEMP_DV_SE_K * se))
        return bound / (b.r0 * self.alpha) * abs(self.inv_now - b.inv0)

    def gated(self, b=None):
        return self.mismatch_c(b) > TEMP_ERR_BUDGET_C

    def celsius(self):
        b = self.base
        if self.r is None or b.r0 is None:
            return None
        if b.src == "file" and self.n_ok < TEMP_FILE_SHOW_AFTER:
            return None
        if self.gated(b):
            return None
        dv = self.dv_pair(b)[0]
        dv = 0.0 if dv is None else dv
        r = self.r - dv * (self.inv_now or 0.0)
        r0 = b.r0 - dv * (b.inv0 or 0.0)
        if r0 <= 0:
            return None
        return b.t0 + (r / r0 - 1.0) / self.alpha

    def fresh(self, now, max_age=TEMP_STALE_S):
        return self.r is not None and (now - self.t_last) <= max_age


def parse_traj(v):
    rows = []
    if isinstance(v, str):
        for ln in v.replace("\r", "\n").split("\n"):
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = [x for x in ln.replace("\t", ",").replace(" ", ",").split(",") if x != ""]
            if len(parts) < 2:
                return (None, "「t,Θout」の 2 列が要ります: %r" % ln[:40])
            rows.append((parts[0], parts[1]))
    elif isinstance(v, (list, tuple)):
        for x in v:
            if not isinstance(x, (list, tuple)) or len(x) < 2:
                return (None, "点は [t, Θout] の形")
            rows.append((x[0], x[1]))
    else:
        return (None, "軌道がありません（points か text）")
    pts = []
    for i, (a, b) in enumerate(rows):
        try:
            t = num(a, "t")
            y = num(b, "Θout")
        except BadValue as e:
            return (None, "%d 行目: %s" % (i + 1, e))
        pts.append((t, y))
    if len(pts) < 2:
        return (None, "点が 2 つ以上要ります")
    if abs(pts[0][0]) > 1e-12:
        return (None, "最初の点の t は 0")
    for i in range(1, len(pts)):
        if not (pts[i][0] > pts[i - 1][0]):
            return (None, "t は単調増加（%d 行目）" % (i + 1))
    return (pts, None)


def parse_pt_degs(v):
    if isinstance(v, str):
        toks = []
        for part in v.replace("、", ",").replace("，", ",").split(","):
            toks.extend(part.split())
    elif isinstance(v, (list, tuple)):
        toks = list(v)
    else:
        return None, Why("角度の並びがありません（例：0, 45, 90, 135）", "No angle list (e.g. 0, 45, 90, 135)")
    out = []
    for x in toks:
        try:
            d = num(x, "角度")
        except BadValue:
            return None, Why("角度の並びに読めない字があります: %r（数をカンマか空白で区切ってください）" % (x,),
                             "Unreadable entry in the angle list: %r (separate numbers with commas or spaces)" % (x,))
        out.append(d)
    if len(out) < PT_N_MIN:
        return None, Why("角度は %d 点以上入れてください（いま %d 点）" % (PT_N_MIN, len(out)),
                         "Enter at least %d angles (got %d)" % (PT_N_MIN, len(out)))
    if len(out) > PT_N_MAX:
        return None, Why("角度は %d 点までです（いま %d 点）" % (PT_N_MAX, len(out)),
                         "At most %d angles (got %d)" % (PT_N_MAX, len(out)))
    return out, None


class RecWriter(object):

    T_FMT = "%.3f"

    def __init__(self, rec_dir, name=""):
        try:
            os.makedirs(rec_dir, exist_ok=True)
        except FileExistsError:
            raise NotADirectoryError("記録の置き場がディレクトリではありません: %s" % rec_dir)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.name = "%s_%s.csv" % (stamp, name) if name else "%s.csv" % stamp
        self.path = os.path.join(rec_dir, self.name)
        self.t0 = time.time()
        self.rows = 0
        self.written = 0
        self.err = None
        self._q = collections.deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._f = io.open(self.path, "x", encoding="utf-8", newline="")
        self._f.write(",".join(REC_COLS) + "\n")
        self._th = threading.Thread(target=self._run, name="krc-rec", daemon=True)
        self._th.start()

    def put(self, row):
        if self.err is not None or self._stop.is_set():
            return False
        with self._lock:
            self._q.append(row)
        self.rows += 1
        return True

    @staticmethod
    def _fmt(v):
        if v is None:
            return ""
        if isinstance(v, float):
            return "%.6g" % v
        return str(v)

    def _drain(self):
        with self._lock:
            rows = list(self._q)
            self._q.clear()
        if rows:
            self._f.write("".join((self.T_FMT % r[0]) + "," + ",".join(self._fmt(x) for x in r[1:]) + "\n"
                                  for r in rows))
            self._f.flush()
            self.written += len(rows)

    def _run(self):
        try:
            while not self._stop.wait(REC_FLUSH_S):
                self._drain()
            self._drain()
            self._f.close()
        except Exception as e:
            self.err = "%s: %s" % (type(e).__name__, e)
            with self._lock:
                self._q.clear()
            try:
                self._f.close()
            except Exception:
                pass

    def close(self):
        self._stop.set()
        self._th.join(timeout=2.0)
        if self._th.is_alive():
            return "書き終わるのを待ちきれませんでした（SD が詰まっている）"
        return self.err


class AxUI(object):

    _COUNTED = {"mode": "mode_n", "p_target": "tgt_n", "center": "tgt_n"}

    def __setattr__(self, k, v):
        c = AxUI._COUNTED.get(k)
        if c is not None:
            object.__setattr__(self, c, self.__dict__.get(c, 0) + 1)
        object.__setattr__(self, k, v)

    def __init__(self, node):
        self.node = int(node)
        self.on = True
        self.start_zeroed = False
        self.mode = "free"
        self.p_target = 0.0
        self.v_target = 0.0
        self.t_target = 0.0
        self.kt_pulse = None
        self.pt = None
        self.kp, self.kd = GAIN_BY_NODE.get(self.node, GAIN_DEFAULT)
        self.amp = 0.5
        self.freq = 1.0
        self.center = 0.0
        self.phase_t0 = time.time()
        self.traj_p = None
        self.traj_v = 0.0
        self.traj_t = 0.0
        self.traj_f = 0.0
        self.glide_v = TRAJ_V_MAX
        self.stop_iq = 0.0
        self.stop_dir = 1.0
        self.stop_hit = None
        self.stop_t0 = None
        self.stop_tl = 0.0
        self.stop_ema = 0.0
        self.stop_n = 0
        self.stop_skip = 0.0
        self.stop_from = None
        self.stop_vslow = 0.0
        self.stop_fb = -1
        self.stop_ema_s = STOP_EMA_S
        self.leg_lo = 70.0
        self.leg_hi = 130.0
        self.leg_period = 2.0
        self.leg_t0 = 0.0
        self.leg_phase0 = 0.0
        self.leg_run = False
        self.leg_des_tout = None
        self.leg_vmax = None
        self.leg_traj_pts = None
        self.leg_traj_loop = True
        self.leg_traj_speed = 1.0
        self.leg_traj_name = ""
        self.grav_samp = None
        self.hist = collections.deque(maxlen=HIST_N)


class OffCal(object):

    def __init__(self, node, n, cid, kind="bare", side=1):
        self.node = int(node)
        self.n = int(n)
        self.cid = cid
        self.mu = threading.Lock()
        self.kind = kind
        self.side = 0 if side == 0 else (1 if side > 0 else -1)
        self.hit = None
        self.rest_until = None
        self.stage = "prep" if (kind == "gear" and self.side != 0) else "running"
        self.k = 1
        self.t0 = None
        self.est_s = None
        self.st = None
        self.vals = []
        self.fits = []
        self.res = None
        self.why = None
        self.stop_why = None
        self.link = None
        self.go = threading.Event()
        self.last_poll = 0.0
        self.beat()
        self.want = None
        self.mode = None
        self.apply_err = None
        self.save_err = None
        self.save_text = None

    def busy(self):
        return self.stage in OFFCAL_BUSY

    def beat(self):
        self.last_poll = time.monotonic()


class Controller(threading.Thread):

    daemon = True

    def __init__(self, nodes, rate, iface="can0", auto_scan=False):
        threading.Thread.__init__(self)
        bad = [n for n in nodes if not valid_node(n)]
        if bad:
            raise BadValue("軸番号として読めません（1〜%d）: %s"
                           % (NODE_MAX, "・".join(str(n) for n in bad)))
        ns = sorted(set(int(n) for n in nodes))
        try:
            rate = float(rate)
        except (TypeError, ValueError):
            raise BadValue("送信レートが数として読めません: %r" % (rate,))
        if not math.isfinite(rate) or rate <= 0.0:
            raise BadValue("送信レートは 0 より大きい数で指定してください: %r"
                           % (rate,))
        self.ax = dict((n, AxUI(n)) for n in ns)
        self.order = sorted(self.ax)
        self.node = self.order[0] if self.order else None
        self.rate = rate
        self.bus = GuiBus(list(self.order), rate=self.rate, iface=iface)
        self.lock = threading.Lock()

        self.armed = False
        self.target = "all"
        self.arm_gen = 0
        self.arm_wait = None
        self.arm_nodes = []
        self._arm_add_n = None
        self._last_sp = {}
        self._sp_seq = 0
        self._sp_fc_seq = {}

        self.last_poll = 0.0
        self.arm_cid = None
        self.wd_fired = False
        self.stall = None
        self._stall_n = 0
        self._note = ""
        self.note_seq = 0
        self.note = ""
        self._estop_note_seq = -1
        self._clear_t = None
        self._held_note = None
        self.seq = 0
        self.periods = collections.deque(maxlen=400)
        self._mit_mono = {}
        self._fb_mono = {}
        self._held = {}
        self._gain_tx = {}
        self._fd_queued = set()
        self.pending = []
        self._zero_t = {}
        self._rz_tok = {}
        self._rz_seq = 0
        self._op_jt_why = None
        self._param_sent = {}
        self._param_dirty = {}
        self._param_saving = {}
        self._param_new = {}
        self._param_save_pids = {}
        self._param_quiet = {}
        self._coast_t0 = {}
        self._op_queued = True
        self.stopping = False
        self._last_sample = 0.0
        self._last_temp = 0.0
        self.auto_scan = bool(auto_scan)
        self._last_scan = 0.0
        self.t_start = time.time()
        self.temps = {}
        self.wind = {}
        self.wind_refs, self.wind_ref_errs = wind_ref_load()
        self._wind_ref_lock = threading.Lock()
        self.homing = {}
        self.jt = None
        self.homing_kind = {}
        self.leg_last = {}
        self.leg_cal, _cal_why = krcleg.load_all_why()
        if _cal_why:
            self.note = _cal_why
        self.rec = None
        self.rec_last = None
        self._temp_seen = {}
        self.offcal = {}
        self.wcal = {}
        self._wcal_lock = threading.Lock()
        self._wcal_starting = set()

    @property
    def note(self):
        return self._note

    @note.setter
    def note(self, v):
        self._note = "" if v is None else str(v)
        self._note_en = en_of(v)
        self.note_seq += 1

    @property
    def note_en(self):
        return getattr(self, "_note_en", None)

    def _later(self, dt, fn, keep=False):
        if not self._op_queued:
            self._op_queued = True
            self._supersede()
        self.pending.append((time.time() + dt, fn, bool(keep)))

    def _supersede(self):
        now = time.time()
        self.pending = [it for it in self.pending if it[2] or it[0] <= now]
        self._void_arming()

    def _void_arming(self):
        self.arm_gen += 1
        self.arm_wait = None

    def _arming(self):
        return self.arm_wait is not None and self.arm_wait == self.arm_gen

    def _arming_nodes(self):
        if not self._arming():
            return []
        if self.armed:
            return [] if self._arm_add_n is None else [self._arm_add_n]
        return list(self.arm_nodes)

    def _arm_voided(self, ns):
        with self.lock:
            if self._arming():
                return
            drive = self.driven_nodes()
            left = [n for n in ns if n in self.ax and n not in drive
                    and self._last_sp.get(n) != 0xFD and self._trip_state(n) is not True]
            if not left:
                return
            self.arm_nodes = [k for k in self.arm_nodes if k not in left]
            if not self.arm_nodes:
                self.armed = False
            msg = ("%s の通電は、確かめる前に別の操作が来たので取り消しました（通電するなら押し直してください）"
                   % "・".join("軸%d" % n for n in left))
            self.note = (self.note + "／" + msg) if self.note else msg
        for n in left:
            self._send_special(0xFD, n)

    def _send_special(self, code, node):
        self._offcal_stop(int(node), OFFCAL_SP_TEXT.get(code, "特殊コマンドが来た"), running_only=True)
        n = int(node)
        if code == 0xFD:
            self._fd_queued.discard(n)
        t_tx = time.time()
        try:
            self.bus._send(0x100 + n, bytes([0xFF] * 7 + [code]))
        except Exception:
            if code == 0xFD:
                self._fd_lost(n)
            raise
        if code in (0xFC, 0xFD):
            self._gain_tx[n] = (t_tx, code)
        self._last_sp[int(node)] = code
        self._sp_seq += 1
        if code == 0xFC:
            self._sp_fc_seq[int(node)] = self._sp_seq

    def live_nodes(self):
        return [n for n in self.order if self.ax[n].on]

    def driven_nodes(self):
        if not self.armed:
            return []
        return [n for n in self.arm_nodes if n in self.ax and self.ax[n].on]

    def _t_max(self, n, default=UI_T_MAX):
        r = self.bus.param_reported(n, 1)
        kt = self._kt(n, None if default is None else KT_NM_PER_A)
        if r is None or kt is None:
            return default
        return min(T_MAX, r[0] * kt)

    def _kp_max(self, n):
        st, sc, _why = self.bus.scale_of(n)
        if st != "ok" or not sc:
            return UI_KP_MAX
        return min(UI_KP_MAX, round(sc["kp"], 4))

    def _gain_sent(self, n, key, x):
        if x is None or not math.isfinite(x):
            return None
        sc = self.bus._scale_for(n)[key]
        return krcbus._unpack(krcbus._pack(x, 0.0, sc, 12), 0.0, sc, 12)

    def _v_max(self, n):
        vb = self.bus.param_reported(n, 8)
        kt = self.bus.param_reported(n, 2)
        if vb is None or kt is None or kt[0] <= 0:
            return UI_V_MAX
        return min(V_MAX, vb[0] * krcbus.VMAX_FRAC / (kt[0] / 1.5))

    def _kt(self, n, default=KT_NM_PER_A):
        r = self.bus.param_reported(n, 2)
        return r[0] if r is not None else default

    def _therm(self, n):
        rt = self.bus.param_reported(n, 4)
        rc = self.bus.param_reported(n, 5)
        return (rt[0] if rt is not None else THERM_TRIP_PCT,
                rc[0] if rc is not None else THERM_CLEAR_PCT)

    def _param_save_tick(self):
        drive = self.driven_nodes()
        now = time.time()
        self._coast_track(drive, now)
        for n, rec in list(self._param_saving.items()):
            q = self._param_quiet.get(n)
            if rec[0] in PARAM_SAVE_DONE or not isinstance(q, float):
                continue
            t_last = self.bus.tlm_t.get((n, 0), 0.0)
            if t_last > rec[1]:
                self._param_quiet[n] = (t_last - q) > FB_FRESH_S
        if self._arming():
            return
        for n, t in list(self._param_dirty.items()):
            if n not in self.ax:
                self._param_dirty.pop(n, None)
                continue
            if n in drive or n in self.homing or (now - t) < 1.0 or self._trip_state(n) is True:
                continue
            if self._param_coasting(n, now):
                continue
            if self._offcal_busy(n):
                continue
            b = self._board(n)
            if not b.get("alive"):
                continue
            self._param_dirty.pop(n, None)
            new = self._param_new.pop(n, set())
            prev = self._param_saving.get(n)
            chained = prev is not None and prev[0] is not None and prev[0] not in PARAM_SAVE_DONE
            pids = (set(self._param_save_pids.get(n, ())) if chained else set()) | new
            self._param_save_pids[n] = pids
            self._param_quiet[n] = float(self.bus.tlm_t.get((n, 0), 0.0))
            k13 = self.bus.tlm_n.get((n, 13), 0)
            if self._param_echo(n, new)[0]:
                self._param_saving[n] = ("differs", now, k13, 1)
                continue
            cnt, m = self._param_save_count(n), 1
            if chained:
                cnt, m = prev[0], prev[3] + 1
            self._param_saving[n] = (cnt, now, k13, m)
            self.pending.append((now, lambda k=n: self.bus._cfg(k, krcbus.CFG_SAVE_PARAM), True))

    def _param_echo(self, n, pids, after=0.0):
        bad, unread = [], []
        for pid in sorted(pids):
            snt = self._param_sent.get((n, pid))
            if snt is None:
                continue
            r = self.bus.params.get((n, pid))
            if r is None or r[1] <= max(snt[1] + PARAM_ECHO_S, after):
                unread.append(pid)
            elif r[0] != snt[0]:
                bad.append(pid)
        return bad, unread

    def _param_save_count(self, n):
        d = self.bus.tlm.get((n, 13))
        return None if (d is None or len(d) < 8) else d[6]

    def _coast_track(self, drive, now):
        for n in self.order:
            if n in drive or self._board_driving(n, now):
                self._coast_t0[n] = now
                continue
            d = self.bus.tlm.get((n, 6))
            t6 = self.bus.tlm_t.get((n, 6), 0.0)
            if d is None or len(d) < 8 or now - t6 > BOARD_FRESH_S:
                continue
            t0 = self._coast_t0.get(n)
            if not krcbus.coast_still(s16(d, 4)):
                if t0 is None:
                    self._coast_t0[n] = now
            elif t0 is not None and t6 >= t0 + krcbus.TLM_LOOP_S:
                self._coast_t0.pop(n, None)

    def _board_driving(self, n, now):
        d = self.bus.tlm.get((n, 4))
        t4 = self.bus.tlm_t.get((n, 4))
        if self._board_held(n, now):
            return True
        m = krcbus.foc_mode_of(d)
        return (m is not None and t4 is not None and now - t4 < krcbus.COAST_WAIT_S
                and m != krcbus.FOC_MODE_IDLE)

    def _board_held(self, n, now):
        held = self._held.get(n)
        if held is None:
            return False
        t4 = self.bus.tlm_t.get((n, 4))
        if t4 is not None and t4 > held[0]:
            d = self.bus.tlm.get((n, 4))
            if krcbus.foc_mode_of(d) == krcbus.FOC_MODE_IDLE:
                self._held.pop(n, None)
                return False
            if now - t4 < held[1]:
                return True
            self._held.pop(n, None)
            return False
        if now - held[0] < held[1]:
            return True
        self._held.pop(n, None)
        return False

    def _fd_lost(self, n):
        tx = self._gain_tx.get(n)
        if tx is None or tx[1] != "mit" or n in self._held:
            return
        hold_s = self._lost_hold_s(n)
        if time.time() - tx[0] < hold_s:
            self._held[n] = (tx[0], hold_s)

    def _own_remnant(self, n, now):
        if n in self._fd_queued:
            return True
        tx = self._gain_tx.get(n)
        if tx is None or tx[1] not in (0xFC, 0xFD) or now - tx[0] >= krcbus.COAST_WAIT_S:
            return False
        a = self.bus.axes.get(n)
        return a is not None and a.fb_count > 0 and a.fb_t >= tx[0]

    def _lost_hold_s(self, n):
        ms = 0.0
        for pid in (7, 37, 38):
            r = self.bus.param_reported(n, pid)
            ms += float(r[0]) if r is not None else krcbus.param_value(pid, krcbus.PARAMS[pid][5])
        return ms / 1000.0 * 1.1

    def _let_board_hold(self, nodes, why, why_en):
        now = time.time()
        homed = [n for n in nodes if n in self.homing]
        for n in nodes:
            self._abort_homing(n, "指令が途切れた（基板が持っている）")
            self._wcal_stop([n], Why(why, why_en))
            self.ax[n].mode = "free"
            self._held[n] = (now, self._lost_hold_s(n))
            self._mit_mono.pop(n, None)
            self._fb_mono.pop(n, None)
        self.arm_nodes = [k for k in self.arm_nodes if k not in nodes]
        if not self.arm_nodes:
            self.armed = False
        secs = max(self._held[n][1] for n in nodes)
        head = ("%s。基板は最後の目標を同じ硬さで持ってから、ゆっくり力を抜いて自分で止まります（§7-63・最大 約 %.0f 秒）。"
                "脱力の指令も解除（0xFD）も送っていません（送ると脚がその場で落ちる）" % (why, secs))
        head_en = ("%s. The board holds its last target, then slowly releases and stops by itself (up to ~%.0f s). "
                   "No limp command or disarm was sent (it would drop the leg)" % (why_en, secs))
        act, act_en = "。基板が止まってから通電し直してください", ". Re-arm after the board has stopped"
        hom = ("。軸%s の原点出しは中止しました（基板が止まってからやり直してください）"
               % "・".join(str(n) for n in homed) if homed else "")
        hom_en = (". Homing on axis %s was aborted (redo it after the board has stopped)"
                  % ", ".join(str(n) for n in homed) if homed else "")
        self.note = Why(head + act + hom, head_en + act_en + hom_en)
        self._held_note = (self.note_seq, list(nodes), Why(head + hom, head_en + hom_en))

    def _param_coasting(self, n, now):
        t0 = self._coast_t0.get(n)
        return t0 is not None and (now - t0) < krcbus.COAST_WAIT_S

    def _param_view(self, n):
        now = time.time()
        out = {}
        for pid, (key, _name, _unit, _lsb, _lo, _hi, _d) in krcbus.PARAMS.items():
            r = self.bus.params.get((n, pid))
            snt = self._param_sent.get((n, pid))
            out[key] = dict(v=(None if r is None else krcbus.param_value(pid, r[0])),
                            age=(None if r is None else round(now - r[1], 1)),
                            sent=(None if snt is None else krcbus.param_value(pid, snt[0])),
                            miss=bool(self._param_echo(n, (pid,))[0]))
        save = None
        if n in self._param_dirty:
            save = ("wait" if n in self.driven_nodes() or self._board_driving(n, now) else
                    "tripped" if self._trip_state(n) is True else
                    "coast" if self._param_coasting(n, now) else "soon")
        elif n in self._param_saving:
            c0, t0, k0, m = self._param_saving[n]
            if c0 in PARAM_SAVE_DONE:
                save = c0
                if c0 == "saved" and now - t0 > 5.0:
                    self._param_saving.pop(n, None)
                    save = None
            else:
                k = self.bus.tlm_n.get((n, 13), 0) - k0
                r = krcbus.param_save_result(c0, self._param_save_count(n), k, m)
                bad, unread = self._param_echo(n, self._param_save_pids.get(n, ()), after=t0)
                if bad:
                    r = "differs"
                elif r == "failed" and self._param_quiet.get(n) is True:
                    self._param_saving.pop(n, None)
                    self._param_quiet.pop(n, None)
                    self._param_new.setdefault(n, set()).update(self._param_save_pids.get(n, ()))
                    self._param_dirty[n] = now - 1.0
                    return out, "soon"
                elif r == "saved" and unread:
                    return out, "verify"
                if r is not None:
                    self._param_saving[n] = (r, now, k0, m)
                    save = r
                elif c0 is None or now - t0 > 3.0:
                    save = ("unknown" if k <= 0 else "unread" if c0 is None else "few")
                else:
                    save = "saving"
        return out, save

    def _start_zero_tick(self):
        if self._arming():
            return
        drive = self.driven_nodes()
        now = time.time()
        for n in self.order:
            a = self.ax[n]
            if a.start_zeroed or n in drive or n in self.homing:
                continue
            if self._board_driving(n, now):
                continue
            if self._offcal_busy(n):
                continue
            b = self._board(n)
            if not b.get("alive") or b.get("lim_on") is None:
                continue
            a.start_zeroed = True
            if not self._may_rezero(n)[0]:
                continue
            self.pending.append((now, lambda k=n: self._zero_now(k), False))
            self.pending.append((now + 0.15, lambda k=n: self._drop_pos_tlm(k), True))
            self.pending.append((now + 0.30, lambda k=n: self._sync_targets(k), True))

    def _rezero_seq(self, m, nodes, tries):
        toks = dict((n, self._rz_tok.get(n)) for n in nodes)
        if tries == 0:
            at = self._later
        else:
            at = self._queue
        for n in nodes:
            at(0.0, lambda k=n: self._rz_zero(k, toks[k]))
        for n in nodes:
            at(0.15, lambda k=n: self._drop_pos_tlm(k), keep=True)
        for n in nodes:
            at(0.30, lambda k=n: self._rz_sync(k, toks[k]), keep=True)
        at(0.35, lambda: self._enter_rezero(m, nodes, tries + 1, toks), keep=True)

    def _queue(self, dt, fn, keep=False):
        self.pending.append((time.time() + dt, fn, bool(keep)))

    def _rz_begin(self, nodes):
        toks = {}
        for n in nodes:
            a = self.ax[n]
            self._rz_seq += 1
            toks[n] = self._rz_tok[n] = dict(seq=self._rz_seq, ax=a, mode_n=a.mode_n, tgt_n=a.tgt_n)
        return toks

    def _rz_live(self, n, tok, what):
        a = self.ax.get(n)
        if tok is None or a is None or self._rz_tok.get(n) is not tok or tok["ax"] is not a:
            return False
        if a.mode_n != tok["mode_n"]:
            return False
        return what != "target" or a.tgt_n == tok["tgt_n"]

    def _rz_own(self, n, tok):
        a = self.ax.get(n)
        if tok is not None and a is not None and self._rz_tok.get(n) is tok:
            tok["tgt_n"] = a.tgt_n

    def _rz_zero(self, n, tok):
        self._zero_now(n)
        self._rz_own(n, tok)

    def _rz_sync(self, n, tok, last=False):
        if self._rz_live(n, tok, "target"):
            self._sync_targets(n)
            self._rz_own(n, tok)
        if last and self._rz_tok.get(n) is tok:
            self._rz_tok.pop(n, None)

    def _enter_zero_mode(self, m, nodes, toks):
        with self.lock:
            live = [n for n in nodes if self._rz_live(n, toks.get(n), "mode")]
            for n in nodes:
                if self._rz_tok.get(n) is toks.get(n):
                    self._rz_tok.pop(n, None)
            if live:
                self._enter_mode(m, live)

    def _enter_rezero(self, m, nodes, tries, toks):
        with self.lock:
            drive = self.driven_nodes()
            live = [n for n in nodes if self._rz_live(n, toks.get(n), "mode") and n in drive]
            again = [n for n in live if tries < REZERO_TRIES and self._needs_rezero(n, m)]
            ready = [n for n in live if n not in again]
            if again:
                self._rezero_seq(m, again, tries)
        if ready:
            for n in ready:
                self._rz_tok.pop(n, None)
            self._enter_mode(m, ready)
            left = [n for n in ready if self.ax[n].mode != m]
            if left and tries > 1:
                self.note += "（回り続けていて %d 回取り直しても離れました。止まってから押してください）" % tries

    def _enter_mode(self, m, nodes):
        busy = [w for w in (self._leg_busy_why(n) for n in nodes) if w is not None]
        if busy:
            self.note = "／".join(busy)
            return
        bad = self._gate_some(nodes)
        if bad:
            self.note = ("原点を取り直しても位置モードに入れません: %s"
                         % "／".join("軸%d %s" % (n, w) for n, w in bad))
            return
        if m == "sine":
            bad = [(n, sine_gate(self.ax[n].center, self.ax[n].amp, *self._p_bound(n)))
                   for n in nodes]
            bad = [(n, w) for n, w in bad if w is not None]
            if bad:
                self.note = ("原点を取り直しても正弦波に入れません: %s"
                             % "／".join("軸%d %s" % (n, w) for n, w in bad))
                return
        for n in nodes:
            if self.ax[n].mode != m:
                self.ax[n].phase_t0 = time.time()
            self.ax[n].mode = m
        self.note = ""

    def _targets(self, params):
        if "node" in params:
            n = num_int(params["node"], "軸番号")
            return [n] if (n in self.ax and self.ax[n].on) else []
        return self.edit_nodes()

    def _node_why(self, params):
        if "node" not in params:
            return None
        n = num_int(params["node"], "軸番号")
        if n not in self.ax:
            return Why("軸 %r は居ません" % (n,), "Axis %r not found" % (n,))
        if not self.ax[n].on:
            return Why("軸%d は動かす対象ではありません（止めてあります）" % n, "Axis %d is not enabled" % n)
        return None

    def edit_nodes(self):
        if self.target == "all":
            return self.live_nodes()
        n = num_int(self.target, "対象の軸番号")
        return [n] if (n in self.ax and self.ax[n].on) else []

    def _tlm(self, n, idx, max_age=1.0):
        d = self.bus.tlm.get((n, idx))
        if d is None or len(d) < 8:
            return None
        if (time.time() - self.bus.tlm_t.get((n, idx), 0.0)) > max_age:
            return None
        return d

    def _pos_true(self, n):
        return self.bus.pos_true(n)

    def _pos_now(self, n):
        a = self.bus.axes[n]
        fb_fresh = (a.fb_count > 0
                    and (time.time() - a.fb_t) <= FB_FRESH_S
                    and a.fb_t > self._zero_t.get(n, 0.0))
        if fb_fresh and not fb_saturated(a.pos):
            return a.pos
        return self._pos_true(n)

    def _id_now(self, n):
        d = self._tlm(n, 5)
        return None if d is None else s16(d, 6) / 1000.0

    def _live(self, n, armed):
        a = self.bus.axes[n]
        fb_fresh = a.fb_count > 0 and (time.time() - a.fb_t) <= FB_FRESH_S
        pos = self._pos_now(n)
        if armed and fb_fresh:
            return pos, a.vel, a.iq
        vel, iq = (a.vel, a.iq) if fb_fresh else (None, None)
        d = self._tlm(n, 10)
        if d is not None:
            vel = s16(d, 2) / 10.0
        d = self._tlm(n, 5)
        if d is not None:
            iq = s16(d, 4) / 1000.0
        return pos, vel, iq

    def _restart_sine(self, n):
        if self.ax[n].mode == "sine":
            self.ax[n].phase_t0 = time.time()

    def _zero_now(self, n):
        self._send_special(0xFE, n)
        self.ax[n].p_target = 0.0
        self.ax[n].center = 0.0
        self._snap_traj(n, 0.0)
        self._restart_sine(n)

    def _trip_op(self, action):
        assume = trip_assume(action)
        if assume is not None:
            for n in self.order:
                self.bus.axes[n].set_tripped(assume, assumed=True)
        self._clear_t = time.time() if action == "clear" else None
        ask = bytes([0xFF] * 7 + [0xFD])
        for delay, cid, data in trip_sequence(action, list(self.order)):
            if data == ask and valid_node(cid - 0x100):
                self._later(delay, lambda k=cid - 0x100, q=self._sp_seq: self._trip_ask(k, q), keep=True)
            else:
                self._later(delay, lambda c=cid, d=data: self.bus._send(c, d))

    def _trip_ask(self, n, seq):
        if self._sp_fc_seq.get(n, 0) > seq:
            return
        self._send_special(0xFD, n)

    def _trip_state(self, n):
        return trip_state(self.bus.is_tripped(n), self._board(n).get("trip"))

    def _any_tripped(self):
        st = [self._trip_state(n) for n in self.live_nodes()]
        if any(x is True for x in st):
            return True
        if any(x is None for x in st):
            return None
        return False

    def _trip_hold(self, n):
        b = self._board(n)
        r = b.get("trip")
        if r == TRIP_CODE_BOARD_HOT:
            tc = b.get("temp_c")
            if tc is None:
                return Why("軸%d は基板温度が読めないので、「遮断を解除」は効きません（読めて冷えたと分かるまで基板は"
                           "戻しません。温度センサーと配線を確かめてください）" % n,
                           "Axis %d: board temperature unreadable, \"Clear trip\" has no effect until it reads cool "
                           "(check the sensor and wiring)" % n)
            rc = self.bus.param_reported(n, krcbus.PARAM_BY_KEY["b_clear"])
            clr = rc[0] if rc is not None else krcbus.PARAMS[krcbus.PARAM_BY_KEY["b_clear"]][6]
            if tc >= clr:
                return Why("軸%d は基板が冷えるまで「遮断を解除」は効きません（いま %.1f ℃）" % (n, tc),
                           "Axis %d: \"Clear trip\" has no effect until the board cools (now %.1f °C)" % (n, tc))
        elif r == TRIP_CODE_THERMAL:
            pct = b.get("therm_pct")
            if pct is None or pct >= self._therm(n)[1]:
                return Why("軸%d は巻線の熱の見積もりが下がるまで「遮断を解除」は効きません（いま %s）"
                           % (n, "読めない" if pct is None else "%d %%" % pct),
                           "Axis %d: \"Clear trip\" has no effect until the winding cools (now %s)"
                           % (n, "unknown" if pct is None else "%d %%" % pct))
        return None

    def _trip_action(self, nodes):
        now = time.time()
        trip = [n for n in nodes if self._trip_state(n) is True]
        hold = [(n, self._trip_hold(n)) for n in trip]
        holds = [w for _n, w in hold if w is not None]
        free = [n for n, w in hold if w is None]
        press = (TRIP_CLEARING_TEXT if free and all(self._clearing(n, now) for n in free)
                 else Why(TRIP_ACTION_TEXT, TRIP_ACTION_TEXT_EN))
        if not holds:
            return press
        if len(holds) == len(trip):
            return wjoin(holds)
        return wjoin([press] + holds, sep=" ", en_sep=" ")

    def _clearing(self, n, now):
        t0 = self._clear_t
        if t0 is None or not (0.0 <= now - t0 < TRIP_CLEAR_WAIT_S):
            return False
        return self.bus.axes[n].trip_t < t0 + krcbus.TRIP_SETTLE_S

    def _arm_block(self, node=None):
        if node is not None:
            return self._arm_block_axis(node)
        if self.armed:
            return Why("すでに通電中です（解除してから押してください）", "Already armed (disarm first)")
        if self._arming():
            return Why("通電の手続き中です（基板の返事を確かめています）", "Arming (waiting for the board's reply)")
        if not self.order:
            return Why("軸が見つかりません（基板の電源と CAN の配線を確かめて「再探索」）",
                       "No axes found (check board power and CAN wiring, then Rescan)")
        if not self.live_nodes():
            return Why("動かす軸が1つも選ばれていません", "No axis is enabled")
        if self._any_tripped() is True:
            act = self._trip_action(self.live_nodes())
            return Why("遮断中なので通電できません。" + act, "Tripped, cannot arm. " + (en_of(act) or TRIP_ACTION_TEXT_EN))
        for n in self.live_nodes():
            st, _sc, why = self.bus.scale_of(n)
            if st != "ok":
                return Why("軸%d: %s" % (n, why), "Axis %d: %s" % (n, SCALE_WHY_EN.get(st, st)))
        now = time.time()
        busy = [n for n in self.live_nodes() if self._board_busy(n, now)]
        for n in self.live_nodes():
            why = self._arm_board_why(n, now)
            if why is not None:
                return self._board_busy_why(busy) if n in busy else why
        return None

    def _board_busy(self, n, now):
        return self._board_held(n, now) or (self._board_driving(n, now) and not self._own_remnant(n, now))

    @staticmethod
    def _board_busy_why(nodes):
        ns = "・".join(str(n) for n in nodes)
        en = ("Axis %d board is" % nodes[0]) if len(nodes) == 1 else ("Axis %s boards are" % ", ".join(str(n) for n in nodes))
        return Why("軸%s の%s通電してください" % (ns, krcbus.BOARD_DRIVING_TEXT),
                   "%s still in motor mode (holding or releasing after lost commands, or another tool "
                   "is driving it). Arm after it has stopped" % en)

    def _arm_board_why(self, n, now):
        if self._board_busy(n, now):
            return self._board_busy_why([n])
        cal = self.bus.cal_of(n)
        why = krcbus.cal_why(cal)
        if why is not None:
            return Why("軸%d: %s" % (n, why), "Axis %d: %s" % (n, CAL_WHY_EN[cal is None]))
        return None

    def _arm_block_axis(self, n):
        if self._arming():
            return Why("通電の手続き中です（基板の返事を確かめています）", "Arming (waiting for the board's reply)")
        if n not in self.ax:
            return Why("軸%d は居ません" % n, "Axis %d not found" % n)
        why = self._node_why({"node": n})
        if why is not None:
            return why
        if n in self.driven_nodes():
            return Why("軸%d はすでに通電中です" % n, "Axis %d is already armed" % n)
        if self._trip_state(n) is True:
            act = self._trip_action([n])
            return Why(("軸%d は遮断中なので通電できません。" % n) + act,
                       ("Axis %d is tripped, cannot arm. " % n) + (en_of(act) or TRIP_ACTION_TEXT_EN))
        st, _sc, why = self.bus.scale_of(n)
        if st != "ok":
            return Why("軸%d: %s" % (n, why), "Axis %d: %s" % (n, SCALE_WHY_EN.get(st, st)))
        return self._arm_board_why(n, time.time())

    def _arm_add_confirm(self, n, gen):
        if gen != self.arm_gen:
            self._arm_voided([n])
            return
        self.arm_wait = None
        a = self.bus.axes[n]
        fresh = a.fb_count > 0 and (time.time() - a.fb_t) <= FB_FRESH_S
        if self.armed and self._trip_state(n) is False and fresh:
            if n not in self.arm_nodes:
                self.arm_nodes.append(n)
            self.note = arm_done_note([n])
            return
        self._send_special(0xFD, n)
        if self._trip_state(n):
            why = "遮断しています"
        elif not self.armed:
            why = "ほかの軸が解除されました"
        else:
            why = "返事がありません"
        self.note = ("軸%d を通電できませんでした: %s" % (n, why)
                     + "（遮断なら「遮断を解除」、返事が無いなら電源・配線・ノードID）")

    def _may_rezero(self, n):
        b = self._board(n)
        if b.get("lim_on") is None:
            return False, Why("壁の有無がまだ読めていないので原点を動かしません",
                              "the home is not moved: whether the axis has walls is not read yet")
        if b.get("lim_on"):
            return False, Why("壁（可動範囲）のある軸なので原点を動かしません（動かすと壁が機構に対してずれる）",
                              "the home is not moved: this axis has walls (moving it would shift them against the mechanism)")
        if self._leg_cal(n).valid:
            return False, Why("脚の較正（原点出し）が残っている軸なので原点を動かしません",
                              "the home is not moved: the leg calibration (homing) is still valid")
        return True, None

    def _needs_rezero(self, n, m):
        if self.ax[n].mode == m:
            return False
        if not self._may_rezero(n)[0] or not self._board(n).get("alive"):
            return False
        _p, why = self._gate(n)
        if why is not None:
            return True
        if m == "sine":
            pos = self._pos_now(n)
            if pos is not None and sine_gate(pos, self.ax[n].amp, *self._p_bound(n)) is not None:
                return True
        return False

    def _arm_confirm(self, gen=None, ns=None, retry=0):
        if gen is not None and gen != self.arm_gen:
            if ns:
                self._arm_voided(ns)
            return
        self.arm_wait = None
        if not self.arm_nodes:
            self.armed = False
            self.note = ("通電できませんでした: 手続きの途中で制御が止まりました"
                         "（押し直してください）")
            return
        bad = []
        for n in self.arm_nodes:
            a = self.bus.axes[n]
            fresh = a.fb_count > 0 and (time.time() - a.fb_t) <= FB_FRESH_S
            if self._trip_state(n) is False and fresh:
                continue
            bad.append((n, "遮断しています" if self._trip_state(n)
                        else "返事がありません"))
        if bad and retry == 0 and all(w == "返事がありません" for _, w in bad):
            self.arm_wait = gen
            for n, _ in bad:
                self._send_special(0xFC, n)
            with self.lock:
                self.pending.append((time.time() + 0.15,
                                     lambda g=gen, ns2=ns: self._arm_confirm(g, ns2, retry=1), True))
            return
        if not bad:
            self.armed = True
            self.note = arm_done_note(self.arm_nodes)
            return
        self.armed = False
        for n in self.arm_nodes:
            self._send_special(0xFD, n)
        self.arm_nodes = []
        self.note = ("通電できませんでした: "
                     + "／".join("軸%d %s" % (n, w) for n, w in bad)
                     + "（遮断なら「遮断を解除」、返事が無いなら電源・配線・ノードID）")

    def _drop_pos_tlm(self, n):
        self.bus.tlm.pop((n, 6), None)
        self.bus.tlm_t.pop((n, 6), None)
        self._zero_t[n] = time.time()

    def _gate(self, n):
        return self.bus.pos_gate(n)

    def _fb_fresh(self, n):
        a = self.bus.axes[n]
        return a.fb_count > 0 and (time.time() - a.fb_t) <= FB_FRESH_S

    def _gate_some(self, nodes):
        return [(n, why) for n, why in
                ((n, self._gate(n)[1]) for n in nodes)
                if why is not None]

    def _pos_mode_nodes(self):
        return [n for n in self.live_nodes() if self.ax[n].mode in POS_MODES]

    def _sync_pos(self, n, mode=None):
        pos, why = self._gate(n)
        if why is not None:
            self.note = ("軸%d のいまの位置が決められないので目標を更新しませんでした: %s"
                         % (n, why))
            return None
        pos = self._pos_now(n)
        if pos is None:
            self.note = ("軸%d のいまの位置が決められないので目標を更新しませんでした: "
                         "原点を移した後の位置がまだ読めない"
                         "（テレメトリを有効にしてください）" % n)
            return None
        if (self.ax[n].mode if mode is None else mode) == "sine":
            why = sine_gate(pos, self.ax[n].amp, *self._p_bound(n))
            if why is not None:
                self.note = "軸%d 正弦波の中心をここに置けません: %s" % (n, why)
                return None
        return pos

    def _apply_fields(self, n, vals, lo, hi):
        now = time.time()
        a = self.ax[n]
        for attr, v in vals:
            if attr in ("p_target", "center"):
                v = clamp(v, lo, hi)
            if attr == "freq" and a.mode == "sine" and v > 0 and a.freq > 0 and v != a.freq:
                a.phase_t0 = now - (now - a.phase_t0) * a.freq / v
            setattr(a, attr, v)

    def _put_target(self, n, pos):
        self.ax[n].p_target = pos
        self.ax[n].center = pos
        self._snap_traj(n, pos)
        self._restart_sine(n)

    def _snap_traj(self, n, pos):
        self.ax[n].traj_p = pos
        self.ax[n].traj_v = 0.0
        self.ax[n].traj_f = pos
        self.ax[n].traj_t = time.time()

    def _traj_step(self, now, n, target):
        a = self.ax[n]
        if not math.isfinite(target):
            return target, 0.0
        if a.traj_p is None:
            pos = self._pos_now(n)
            a.traj_p = target if pos is None else pos
            a.traj_v = 0.0
            a.traj_t = now
            a.traj_f = a.traj_p
        dt = now - a.traj_t
        a.traj_t = now
        dt = clamp(dt, 0.0, TRAJ_DT_MAX)
        a.traj_f += (target - a.traj_f) * (dt / (TRAJ_TAU_S + dt))
        if abs(target - a.traj_f) <= TRAJ_F_SNAP_RAD:
            a.traj_f = target
        goal = a.traj_f
        d = goal - a.traj_p
        if abs(d) <= TRAJ_SNAP_RAD and abs(a.traj_v) <= TRAJ_A_MAX * dt:
            a.traj_p, a.traj_v = goal, 0.0
            return goal, 0.0
        v_want = math.copysign(min(TRAJ_V_MAX, a.glide_v, math.sqrt(2.0 * TRAJ_A_MAX * abs(d))), d)
        a.traj_v += clamp(v_want - a.traj_v, -TRAJ_A_MAX * dt, TRAJ_A_MAX * dt)
        p_next = a.traj_p + a.traj_v * dt
        if (p_next - goal) * d >= 0.0:
            p_next = goal
        a.traj_p = p_next
        return p_next, a.traj_v

    def _sync_targets(self, n, mode=None):
        pos = self._sync_pos(n, mode)
        if pos is None:
            return False
        self._put_target(n, pos)
        return True

    def _sync_targets_some(self, nodes, mode=None):
        got = []
        for n in nodes:
            pos = self._sync_pos(n, mode)
            if pos is None:
                return False
            got.append((n, pos))
        for n, pos in got:
            self._put_target(n, pos)
        return True

    def _sync_all(self):
        ok = True
        for n in self.live_nodes():
            if not self._sync_targets(n):
                ok = False
        return ok

    def do(self, action, params):
        if action == "rec_stop":
            return self._rec_stop()
        if action == "leg_traj":
            return self._leg_traj_cmd(params)
        if action in WIND_REF_ACTIONS:
            return self._wind_ref_cmd(action, params)
        if action == "demo_tlm":
            return self.demo_tlm(params.get("div"))
        if action == "demo_zero_save":
            return self.demo_zero_save(params.get("refs"))
        if action in WCAL_ACTIONS:
            return self._wcal_cmd(action, params)
        with self._op_lock():
            self._op_queued = False
            self._jt_yield(action, params)
            self._pt_yield(action, params)
            if action in PT_ACTIONS:
                return self._pt_op(action, params)
            if action in ("jtraj", "jtraj_speed", "jtraj_stop", "jtraj_gain"):
                return self._jt_cmd(action, params)
            if action == "arm":
                one = params.get("node")
                one = None if one is None else num_int(one, "軸番号")
                blocked = self._arm_block(one)
                if blocked is not None:
                    self.note = blocked
                    return {"ok": False, "err": self.note}
                self._offcal_yield([one] if one is not None else self.live_nodes(), Why("通電が来た", "armed"))
                if one is not None and self.armed:
                    self._abort_homing(one, HOMING_POWER_LOST)
                    self.ax[one].mode = "free"
                    self.ax[one].start_zeroed = True
                    self.bus.axes[one].fb_missing = 0
                    self.bus.axes[one].fb_count = 0
                    self._later(0.0, lambda k=one: self._send_special(0xFC, k))
                    self.arm_wait = self.arm_gen
                    self._arm_add_n = one
                    self._later(0.12, lambda k=one: self._sync_targets(k))
                    self._later(0.15, lambda k=one, g=self.arm_gen: self._arm_add_confirm(k, g), keep=True)
                    self.note = arm_sent_note([one])
                    return {"ok": True}
                self.last_poll = time.monotonic()
                self.arm_cid = params.get("cid")
                self.wd_fired = False
                self.arm_nodes = [one] if one is not None else list(self.live_nodes())
                for n in self.arm_nodes:
                    self._abort_homing(n, HOMING_POWER_LOST)
                    self.ax[n].mode = "free"
                    self.ax[n].start_zeroed = True
                for n in self.arm_nodes:
                    self.bus.axes[n].fb_missing = 0
                    self.bus.axes[n].fb_count = 0
                    self._later(0.0, lambda k=n: self._send_special(0xFC, k))
                self.arm_wait = self.arm_gen
                self._later(0.12, self._sync_all)
                self._later(0.15,
                            lambda g=self.arm_gen, ns=list(self.arm_nodes): self._arm_confirm(g, ns),
                            keep=True)
                self.note = arm_sent_note(self.arm_nodes)

            elif action == "disarm":
                one = params.get("node")
                if one is not None:
                    one = num_int(one, "軸番号")
                    if one not in self.ax:
                        return {"ok": False, "err": "軸 %r は居ません" % (one,)}
                    self._abort_homing(one, HOMING_POWER_LOST)
                    self._wcal_stop([one], Why("解除が来た", "disarmed"))
                    was = one in self.arm_nodes
                    trying = one in self._arming_nodes()
                    self._fd_queued.add(one)
                    self.arm_nodes = [k for k in self.arm_nodes if k != one]
                    if not self.arm_nodes:
                        self.armed = False
                    if was or trying:
                        self._later(0.0, lambda k=one: self._send_special(0xFD, k))
                    else:
                        self.pending.append((time.time(), lambda k=one: self._send_special(0xFD, k), False))
                    self.note = (Why("軸%d を解除しました" % one, "Axis %d disarmed" % one) if was else
                                 Why("軸%d の通電を取り消しました" % one, "Axis %d arming cancelled" % one) if trying else
                                 Why("軸%d は通電していません" % one, "Axis %d is not armed" % one))
                    return {"ok": True}
                self.armed = False
                self.arm_nodes = []
                self._wcal_stop(self.order, Why("解除が来た", "disarmed"))
                for n in self.order:
                    self._fd_queued.add(n)
                    self._later(0.0, lambda k=n: self._send_special(0xFD, k))
                self.note = Why("解除しました（全軸）", "All axes disarmed")

            elif action == "estop":
                self.armed = False
                self.arm_nodes = []
                self._offcal_yield(self.order, Why("非常停止が来た", "E-stop"))
                self._wcal_stop(self.order, Why("非常停止が来た", "E-stop"))
                self._trip_op("estop")
                self.note = Why("非常停止（ラッチ）。復帰には「遮断を解除」", "E-STOP (latched). Press \"Clear trip\" to recover")
                self._estop_note_seq = self.note_seq

            elif action == "clear":
                self.armed = False
                self.arm_nodes = []
                self._offcal_yield(self.order, Why("遮断の解除が来た", "trip cleared"), running_only=True)
                self._wcal_stop(self.order, Why("遮断の解除が来た", "trip cleared"))
                self._trip_op("clear")
                self.note = Why("遮断の解除を送りました（約1秒かかります）", "Clearing the trip (takes about 1 s)")

            elif action == "zero":
                zs = self._targets(params)
                if not zs:
                    return {"ok": False, "err": NO_TARGET_TEXT}
                busy = [w for w in (self._leg_busy_why(n) for n in zs) if w is not None]
                if busy:
                    self.note = "／".join(busy)
                    return {"ok": False, "err": self.note}
                self._offcal_yield(zs, "原点を移す操作が来た")
                toks = self._rz_begin(zs)
                for n in zs:
                    self._later(0.0, lambda k=n: self._rz_zero(k, toks[k]))
                for n in zs:
                    self._later(0.15, lambda k=n: self._drop_pos_tlm(k),
                                keep=True)
                for n in zs:
                    self._later(0.30, lambda k=n: self._rz_sync(k, toks[k], last=True),
                                keep=True)
                self.note = ("いまの位置を原点にしました（%s。フラッシュ未保存）"
                             % "・".join("軸%d" % n for n in zs))

            elif action == "mode":
                m = params.get("mode", "free")
                if m not in MODES:
                    return {"ok": False, "err": "不明なモード"}
                why = self._node_why(params)
                if why is not None:
                    return {"ok": False, "err": why}
                tg = self._targets(params)
                rz = []
                if m in ("hold", "sine"):
                    rz = [n for n in tg if self._needs_rezero(n, m)]
                    drive = self.driven_nodes()
                    cold = [n for n in rz if n not in drive]
                    if cold:
                        trying = self._arming_nodes()
                        self.note = wjoin(
                            Why("軸%d は通電の手続き中です（基板の返事を確かめてから押してください）" % n,
                                "Axis %d is arming (wait for the board's reply)" % n) if n in trying
                            else Why("軸%d は通電していません（通電してから押すと、いまの位置を原点にして入ります）" % n,
                                     "Axis %d is not armed (arm it first; it will re-zero here and enter)" % n)
                            for n in cold)
                        return {"ok": False, "err": self.note}
                if m in POS_MODES:
                    bad = self._gate_some([n for n in tg if n not in rz])
                    if bad:
                        why = "／".join("軸%d %s" % (n, w) for n, w in bad)
                        return {"ok": False, "err": "位置モードに入れません: %s" % why}
                busy = [w for w in (self._leg_busy_why(n) for n in tg) if w is not None]
                if busy:
                    self.note = "／".join(busy)
                    return {"ok": False, "err": self.note}
                changed = [n for n in tg if self.ax[n].mode != m and n not in rz]
                if changed:
                    if m in POS_MODES:
                        if not self._sync_targets_some(changed, mode=m):
                            return {"ok": False, "err": self.note}
                    for n in changed:
                        self.ax[n].phase_t0 = time.time()
                self._offcal_yield(tg, Why("モードの変更が来た", "mode changed"))
                self._wcal_stop(tg, Why("モードの変更が来た", "mode changed"))
                for n in tg:
                    if n not in rz:
                        self.ax[n].mode = m
                self.note = ""
                for n in tg:
                    self._rz_tok.pop(n, None)
                    if self.ax[n].kt_pulse is not None:
                        self._kt_cancel(n)
                if rz:
                    for n in rz:
                        self.ax[n].mode = "damp"
                    self._rz_begin(rz)
                    self._rezero_seq(m, list(rz), 0)
                    self.note = Why("いまの位置を原点にして%sに入ります（%s）"
                                    % ("位置保持" if m == "hold" else "正弦波",
                                       "・".join("軸%d" % n for n in rz)),
                                    "Re-zeroing here and entering %s (%s)"
                                    % ("Hold" if m == "hold" else "Sine", ", ".join("axis %d" % n for n in rz)))

            elif action == "leg_home":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                if self._trip_state(n) is True:
                    self.note = "軸%d は遮断中です" % n
                    return {"ok": False, "err": self.note}
                if not self.armed:
                    self.note = "原点出しは通電してから押してください"
                    return {"ok": False, "err": self.note}
                if n not in self.driven_nodes():
                    self.note = "軸%d はこの通電の相手になっていません（「動かす」を入れて通電し直してください）" % n
                    return {"ok": False, "err": self.note}
                if n in self.homing:
                    return {"ok": False, "err": "軸%d は原点出しの最中です" % n}
                cal = self._leg_cal(n)
                if cal.sign is None:
                    self.note = "軸%d 先に「畳み切りは＋側か−側か」を選んでください（その向きにだけ当てます）" % n
                    return {"ok": False, "err": self.note}
                self.homing[n] = HomingStepper(n, fold_sign=(1.0 if cal.plus_end == "fold" else -1.0), half_design=krcleg.design_half_rad())
                self.homing_kind[n] = "knee"
                self.ax[n].mode = "homing"
                self.note = "軸%d 原点出しを始めました（ストッパにゆっくり当てます。止めるなら「解除」＝脱力して中止）" % n

            elif action == "joint_home":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                if self._trip_state(n) is True:
                    self.note = "軸%d は遮断中です" % n
                    return {"ok": False, "err": self.note}
                if not self.armed:
                    self.note = "原点出しは通電してから押してください"
                    return {"ok": False, "err": self.note}
                if n not in self.driven_nodes():
                    self.note = "軸%d はこの通電の相手になっていません" % n
                    return {"ok": False, "err": self.note}
                if n in self.homing:
                    return {"ok": False, "err": "軸%d は原点出しの最中です" % n}
                sg = num(params.get("stop_sign"), "stop_sign")
                if sg not in (1.0, -1.0):
                    return {"ok": False, "err": "stop_sign は +1 か −1（ストッパがモーターの＋側なら +1）"}
                hd = num(params.get("half_deg"), "half_deg")
                if not (5.0 <= hd <= 170.0):
                    return {"ok": False, "err": "half_deg は 5〜170°（出力の °）"}
                ks = num(params.get("kd_scale", 1.0), "kd_scale")
                if not (0.5 <= ks <= 2.0):
                    return {"ok": False, "err": "kd_scale は 0.5〜2"}
                st_new = HomingStepper(n, fold_sign=sg, half_design=math.radians(hd) * krcleg.RATIO_DEFAULT,
                                       kd_steps=tuple(k * ks for k in krcbus.HOME_KD_STEPS))
                st_new.probe_kd = tuple(k * ks for k in krcbus.HOME_PROBE_KD)
                self.homing[n] = st_new
                self.homing_kind[n] = "joint"
                self.ax[n].mode = "homing"
                self.note = ("軸%d 原点出しを始めました（%s側のストッパにゆっくり当てます。止めるなら「解除」）"
                             % (n, "＋" if sg > 0 else "−"))

            elif action == "leg_set":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                tout = num(params.get("tout"), "tout")
                cal = self._leg_cal(n)
                if n in self.homing or not cal.valid:
                    self.note = "軸%d は同期していません（%s）" % (
                        n, "原点出しの最中です" if n in self.homing else cal.why_invalid())
                    return {"ok": False, "err": self.note}
                pos = self._pos_now(n)
                m = cal.tout_to_motor(tout, pos)
                if m is None:
                    self.note = "軸%d Θout %.1f° はリンクが閉じません" % (n, tout)
                    return {"ok": False, "err": self.note}
                lo, hi = self._p_cmd_bound(n)
                if m < lo - 1e-9 or m > hi + 1e-9:
                    self.note = ("軸%d Θout %.1f° は壁の外です（モーター %.2f rad、壁 %.2f〜%.2f）"
                                 % (n, tout, m, lo, hi))
                    return {"ok": False, "err": self.note}
                if n in self.homing:
                    return {"ok": False, "err": "軸%d は原点出しの最中です" % n}
                if self.ax[n].mode != "hold":
                    bad = self._gate_some([n])
                    if bad:
                        self.note = "位置モードに入れません: %s" % "／".join("軸%d %s" % (k, w) for k, w in bad)
                        return {"ok": False, "err": self.note}
                    if not self._sync_targets_some([n], mode="hold"):
                        return {"ok": False, "err": self.note}
                    self.ax[n].mode = "hold"
                self._apply_fields(n, [("p_target", m)], lo, hi)
                self.note = ""

            elif action == "leg_sine":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                lo_deg = num(params.get("lo"), "lo")
                hi_deg = num(params.get("hi"), "hi")
                period = num(params.get("period"), "period")
                if not (hi_deg > lo_deg):
                    return {"ok": False, "err": "最大が最小より大きくありません"}
                if not (period > 0.0):
                    return {"ok": False, "err": "周期は正の数"}
                off = krcleg.wrap180(lo_deg) - lo_deg
                lo_deg += off
                hi_deg += off
                cal = self._leg_cal(n)
                if n in self.homing or not cal.valid:
                    self.note = "軸%d は同期していません（%s）" % (
                        n, "原点出しの最中です" if n in self.homing else cal.why_invalid())
                    return {"ok": False, "err": self.note}
                err, _m, vmax = self._leg_sine_plan(n, lo_deg, hi_deg, period)
                if err:
                    self.note = "軸%d 正弦波: %s" % (n, err)
                    return {"ok": False, "err": self.note}
                a = self.ax[n]
                mid = 0.5 * (lo_deg + hi_deg)
                amp = 0.5 * (hi_deg - lo_deg)
                r = self._leg_enter_auto(n, "leg_sine", lo_deg, hi_deg, vmax)
                if not r.get("ok"):
                    return r
                a.leg_lo, a.leg_hi, a.leg_period = lo_deg, hi_deg, period
                a.leg_phase0 = math.asin(clamp((r["start"] - mid) / amp, -1.0, 1.0))
                return r

            elif action == "grav_set":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                cal = self._leg_cal(n)
                g = copy.copy(cal.grav)
                if "on" in params:
                    on = params["on"]
                    if not isinstance(on, bool):
                        raise BadValue("on が真偽として読めません: %r" % (on,))
                    g.on = on
                if "a_nm" in params:
                    g.a_nm = num(params["a_nm"], "a_nm")
                if "phase_deg" in params:
                    g.phase_deg = num(params["phase_deg"], "phase_deg")
                err = self._grav_put(n, cal, g)
                if err is not None:
                    return {"ok": False, "err": err}
                self.note = ""
                return {"ok": True, "grav": cal.grav.to_dict()}

            elif action == "grav_sample":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                why = self._grav_sample_why(n)
                if why is not None:
                    return {"ok": False, "err": "軸%d: %s" % (n, why)}
                self.ax[n].grav_samp = dict(t_end=time.time() + GRAV_SAMPLE_S, n=0, iq_sum=0.0, tout_sum=0.0,
                                            key=self._leg_map_key(n), p_target=self.ax[n].p_target)
                self.note = ""
                return {"ok": True}

            elif action == "grav_fit":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                cal = self._leg_cal(n)
                g = copy.copy(cal.grav)
                g.samples = list(cal.grav.samples)
                if not g.fit():
                    return {"ok": False, "err": "標本が足りません（違う角で 2 つ以上）"}
                err = self._grav_put(n, cal, g)
                if err is not None:
                    return {"ok": False, "err": err}
                self.note = "軸%d 重力補償: A %.3f N·m・位相 %.1f°（標本 %d）" % (n, g.a_nm, g.phase_deg, len(g.samples))
                return {"ok": True, "grav": cal.grav.to_dict()}

            elif action == "grav_clear":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                cal = self._leg_cal(n)
                err = self._grav_put(n, cal, krcleg.Grav())
                if err is not None:
                    return {"ok": False, "err": err}
                self.ax[n].grav_samp = None
                self.note = ""
                return {"ok": True}

            elif action == "rec_start":
                if self.rec is not None:
                    return {"ok": False, "err": "記録中です（%s）" % self.rec.name}
                name = "".join(ch for ch in str(params.get("name", ""))
                               if ch.isascii() and (ch.isalnum() or ch in "_-"))[:40]
                try:
                    self.rec = RecWriter(REC_DIR, name)
                except FileExistsError as e:
                    self.rec = None
                    return {"ok": False, "err": "同じ秒に同じ名前の記録があります（1 秒待って押し直してください）: %s"
                            % os.path.basename(e.filename or "")}
                except OSError as e:
                    self.rec = None
                    return {"ok": False, "err": "記録が始められません: %s" % e}
                self.note = ""
                return {"ok": True, "file": self.rec.name}

            elif action == "leg_cal":
                n = num_int(params.get("node"), "node")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                busy = self._leg_busy_why(n, walking=True)
                if busy is not None:
                    self.note = busy
                    return {"ok": False, "err": self.note}
                old = self._leg_cal(n)
                cal = copy.copy(old)
                if "plus_end" in params:
                    w = str(params["plus_end"])
                    if w not in ("extend", "fold"):
                        return {"ok": False, "err": "plus_end は extend / fold"}
                    cal.set_plus_end(w)
                if params.get("ref_clear"):
                    cal.clear_ref()
                if "tout_now" in params:
                    tout_now = num(params.get("tout_now"), "tout_now")
                    pos = self._pos_true(n)
                    if pos is None:
                        return {"ok": False, "err": "軸%d の位置が読めないので基準を置けません" % n}
                    if cal.half is None or cal.sign is None:
                        return {"ok": False, "err": "軸%d 基準の前に原点出しと端の選択が要ります（%s）" % (n, cal.why_invalid())}
                    if not cal.set_from_tout(pos, tout_now):
                        return {"ok": False, "err": "軸%d Θout %.1f° はリンクが閉じません" % (n, tout_now)}
                cals = dict(self.leg_cal)
                cals[n] = cal
                try:
                    krcleg.save_all(cals)
                except OSError as e:
                    self.note = "軸%d 較正を保存できませんでした（何も変えていません）: %s" % (n, e)
                    return {"ok": False, "err": self.note}
                self.leg_cal[n] = cal
                self.note = ("軸%d 側を替えたので原点出しをやり直してください（原点は側から決まる）" % n
                             if (old.half is not None and cal.half is None) else "")
                return {"ok": True, "cal": cal.to_dict()}

            elif action == "set":
                fields = (("p", "p_target", POS_CMD_LO, POS_CMD_HI),
                          ("v", "v_target", -V_MAX, V_MAX),
                          ("t", "t_target", -T_MAX, T_MAX),
                          ("kp", "kp", 0.0, UI_KP_MAX),
                          ("kd", "kd", 0.0, UI_KD_MAX),
                          ("amp", "amp", 0.0, UI_AMP_MAX),
                          ("freq", "freq", 0.05, UI_FREQ_MAX),
                          ("center", "center", POS_CMD_LO, POS_CMD_HI),
                          ("glide", "glide_v", 0.05, TRAJ_V_MAX),
                          ("stop_iq", "stop_iq", 0.0, 10.0),
                          ("stop_dir", "stop_dir", -1.0, 1.0),
                          ("stop_skip", "stop_skip", 0.0, 50.0),
                          ("stop_vslow", "stop_vslow", 0.0, 1.0),
                          ("stop_ema_s", "stop_ema_s", 0.005, 100.0))
                why = self._node_why(params)
                if why is not None:
                    return {"ok": False, "err": why}
                if "t" in params:
                    kts = [n for n in self._targets(params) if self.ax[n].mode == "kt_test"]
                    if kts:
                        self.note = "／".join("軸%d はトルク定数のモードです（トルクのつまみは使いません。電流のボタンで"
                                             "流し、「止める」で止めます）" % n for n in kts)
                        return {"ok": False, "err": self.note}
                vals = [(attr, clamp(num(params[key], key), lo, hi))
                        for key, attr, lo, hi in fields if key in params]
                nv = dict(vals)
                busy = [w for w in (self._leg_busy_why(n, walking="p" in params) for n in self._targets(params)) if w is not None]
                if busy:
                    self.note = "／".join(busy)
                    return {"ok": False, "err": self.note}
                bounds = dict((n, self._p_cmd_bound(n)) for n in self._targets(params))
                if "center" in nv or "amp" in nv:
                    for n in self._targets(params):
                        glo, ghi = self._p_bound(n)
                        lo, hi = bounds[n]
                        why = sine_gate(clamp(nv.get("center", self.ax[n].center), lo, hi),
                                        nv.get("amp", self.ax[n].amp), glo, ghi)
                        if why is not None:
                            self.note = "軸%d %s" % (n, why)
                            return {"ok": False, "err": self.note}
                for n in self._targets(params):
                    lo, hi = bounds[n]
                    tm = self._t_max(n)
                    vm = self._v_max(n)
                    km = self._kp_max(n)
                    vn = [(a_, clamp(v_, -tm, tm) if a_ == "t_target" else
                           (clamp(v_, -vm, vm) if a_ == "v_target" else
                            (min(v_, km) if a_ == "kp" else v_))) for a_, v_ in vals]
                    self._apply_fields(n, vn, lo, hi)
                    if "stop_iq" in params:
                        a_ = self.ax[n]
                        a_.stop_dir = 1.0 if a_.stop_dir >= 0 else -1.0
                        a_.stop_hit = None
                        a_.stop_t0 = None
                        a_.stop_n = 0
                        a_.stop_from = None

            elif action == "tq_pulse":
                if "node" not in params:
                    return {"ok": False, "err": "node（軸番号）が要ります"}
                n = num_int(params["node"], "軸番号")
                amps = num(params.get("amps"), "amps")
                secs = clamp(num(params.get("secs", KT_SECS_DEF), "secs"), KT_SECS_MIN, KT_SECS_MAX)
                why = self._kt_why(n)
                if why is not None:
                    self.note = why
                    return {"ok": False, "err": self.note}
                if abs(amps) < 0.01:
                    return {"ok": False, "err": Why("電流が 0 A です（流す電流を選んでください）", "Current is 0 A (choose a current)")}
                kt = self._kt(n, None)
                tm = self._t_max(n, None)
                tau = clamp(amps * kt, -tm, tm)
                a = self.ax[n]
                now = time.time()
                v0 = 0.0
                if a.kt_pulse is not None:
                    v0, ph = self._kt_pulse_at(a.kt_pulse, now)
                    if ph == "done":
                        v0 = 0.0
                a.kt_pulse = dict(t0=now, v0=v0, tau=tau, amps=tau / kt, secs=secs, down_t=None, down_v=0.0)
                cut = abs(tau) < abs(amps * kt) - 1e-12
                self.note = Why("軸%d に %+.2f A（τ %+.4f N·m・Kt %.4f）を %.1f 秒流します%s"
                                % (n, tau / kt, tau, kt, secs,
                                   ("（上限 %.2f A に詰めました）" % (tm / kt)) if cut else ""),
                                "Axis %d: %+.2f A (τ %+.4f N·m, Kt %.4f) for %.1f s%s"
                                % (n, tau / kt, tau, kt, secs,
                                   (" (capped at %.2f A)" % (tm / kt)) if cut else ""))
                return {"ok": True, "amps": tau / kt, "tau": tau, "secs": secs}

            elif action == "tq_stop":
                n = num_int(params.get("node"), "軸番号")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                p = self.ax[n].kt_pulse
                if p is None:
                    self.note = Why("軸%d は電流を流していません" % n, "Axis %d is not driving current" % n)
                    return {"ok": True}
                now = time.time()
                v, ph = self._kt_pulse_at(p, now)
                if ph == "done":
                    self._kt_cancel(n)
                elif ph in ("up", "hold"):
                    p["down_t"], p["down_v"] = now, v
                self.note = Why("軸%d の電流を %.1f 秒かけて 0 に戻します" % (n, KT_RAMP_S),
                                "Axis %d: ramping current to 0 over %.1f s" % (n, KT_RAMP_S))

            elif action == "here":
                why = self._node_why(params)
                if why is not None:
                    return {"ok": False, "err": why}
                busy = [w for w in (self._leg_busy_why(n, walking=True) for n in self._targets(params)) if w is not None]
                if busy:
                    self.note = "／".join(busy)
                    return {"ok": False, "err": self.note}
                ok = True
                for n in self._targets(params):
                    if not self._sync_targets(n):
                        ok = False
                if not ok:
                    return {"ok": False, "err": self.note}

            elif action == "node":
                n = num_int(params.get("node", self.node), "軸番号")
                if n not in self.ax:
                    return {"ok": False, "err": "その軸は居ません"}
                self.node = n
                self.note = "軸 %d を表示しています" % n

            elif action == "zero_mode":
                m = params.get("mode", "hold")
                if m not in MODES:
                    return {"ok": False, "err": "不明なモード"}
                zs = self._targets(params)
                if not zs:
                    return {"ok": False, "err": NO_TARGET_TEXT}
                busy = [w for w in (self._leg_busy_why(n) for n in zs) if w is not None]
                if busy:
                    self.note = "／".join(busy)
                    return {"ok": False, "err": self.note}
                self._offcal_yield(zs, "原点を移す操作が来た")
                toks = self._rz_begin(zs)
                for n in zs:
                    self._later(0.0, lambda k=n: self._rz_zero(k, toks[k]))
                for n in zs:
                    self._later(0.15, lambda k=n: self._drop_pos_tlm(k), keep=True)
                for n in zs:
                    self._later(0.30, lambda k=n: self._rz_sync(k, toks[k]), keep=True)
                self._later(0.35,
                            lambda mm=m, ns=list(zs): self._enter_zero_mode(mm, ns, toks),
                            keep=True)
                self.note = ("原点を取り直して %s に入ります（%s）"
                             % (m, "・".join("軸%d" % n for n in zs)))

            elif action == "target":
                t = params.get("target", "all")
                if t != "all" and num_int(t, "対象の軸番号") not in self.ax:
                    return {"ok": False, "err": "その軸は居ません"}
                self.target = "all" if t == "all" else num_int(t, "対象の軸番号")

            elif action == "axis":
                n = num_int(params.get("node", 0), "軸番号")
                if n not in self.ax:
                    return {"ok": False, "err": "その軸は居ません"}
                on = params.get("on", True)
                if not isinstance(on, bool):
                    raise BadValue("on が真偽として読めません: %r" % (on,))
                if self.armed or self._arming():
                    return {"ok": False,
                            "err": "通電中は軸の入り切りを変えられません。"
                                   "先に「解除」を押してください"}
                self.ax[n].on = on
                if not on:
                    self._abort_homing(n, HOMING_POWER_LOST)

            elif action == "param":
                n = num_int(params.get("node"), "軸番号")
                if n not in self.ax:
                    return {"ok": False, "err": "軸 %r は居ません" % (n,)}
                k = params.get("key")
                pid = krcbus.PARAM_BY_KEY.get(k) if isinstance(k, str) else None
                if pid is None:
                    return {"ok": False, "err": "そのパラメータはありません: %r" % (k,)}
                if k in krcbus.PARAMS_HIDDEN:
                    self.note = ("%sは画面からは変えられません（krc.py param か offset_cal.py で）"
                                 % krcbus.PARAMS[pid][1])
                    return {"ok": False, "err": self.note}
                v = num(params.get("value"), krcbus.PARAMS[pid][1])
                if self._offcal_busy(n):
                    self.note = "軸%d は電気角オフセットを測っている最中です（中止するか、測り終えてから）" % n
                    return {"ok": False, "err": self.note}
                if self._trip_state(n) is True:
                    self.note = "軸%d は遮断中なので基板が設定を受けません（「遮断を解除」してから）" % n
                    return {"ok": False, "err": self.note}
                why = self._leg_busy_why(n)
                if why is not None:
                    self.note = why
                    return {"ok": False, "err": self.note}
                if not self._board(n).get("alive") and any(k_[0] == n for k_ in list(self.bus.tlm_t)):
                    self.note = "軸%d の基板から返事がありません（設定を受け取れません。電源と CAN を確かめてから）" % n
                    return {"ok": False, "err": self.note}
                x = krcbus.param_lsb(pid, v)
                if k == "kt":
                    p_t = krcbus.PARAM_BY_KEY["t_max"]
                    r_t = self.bus.param_reported(n, p_t)
                    why = ("%sが読み返せていません（組で検めるため。数秒待つ・tlm が止まっていないか）" % krcbus.PARAMS[p_t][1]
                           if r_t is None else krcbus.t_max_why(r_t[0], krcbus.param_value(pid, x)))
                    if why is not None:
                        self.note = "軸%d の%sを送りません: %s" % (n, krcbus.PARAMS[pid][1], why)
                        return {"ok": False, "err": self.note}
                self.pending.append((time.time(), lambda nn=n, pp=pid, vv=v: self.bus.set_param(nn, pp, vv), True))
                self._param_sent[(n, pid)] = (x, time.time())
                self._param_dirty[n] = time.time()
                self._param_new.setdefault(n, set()).add(pid)
                self.note = "軸%d の%sを %g %s にします" % (n, krcbus.PARAMS[pid][1], krcbus.param_value(pid, x),
                                                         krcbus.PARAMS[pid][2])

            elif action in OFFCAL_ACTIONS:
                return self._offcal_cmd(action, params)

            elif action == "rescan":
                if self.armed or self._arming():
                    return {"ok": False,
                            "err": "通電中は再探索できません。先に「解除」を"}
                added = self._rescan_locked()
                self.note = ("見つけた軸: %s（新しく %s）"
                             % ("・".join(str(n) for n in self.order),
                                "・".join(str(n) for n in added) if added
                                else "なし"))

            else:
                return {"ok": False, "err": "不明な操作"}
        return {"ok": True}

    def _offcal_busy(self, n):
        j = self.offcal.get(n)
        return j is not None and j.busy()

    def _offcal_why(self, n, own=False):
        if n not in self.ax:
            return Why("軸 %r は居ません" % (n,), "Axis %r not found" % (n,))
        why = self._node_why({"node": n})
        if why is not None:
            return why
        if not own and self._offcal_busy(n):
            return Why("軸%d は電気角オフセットを測っている最中です" % n, "Axis %d is already measuring the offset" % n)
        if n in self._arming_nodes():
            return Why("軸%d は通電の手続き中です（測るのは解除中の軸だけです）" % n,
                       "Axis %d is arming (only disarmed axes can be measured)" % n)
        if n in self.driven_nodes():
            return Why("軸%d は通電中です（測るのは解除中の軸だけです）" % n,
                       "Axis %d is armed (only disarmed axes can be measured)" % n)
        why = self._leg_busy_why(n)
        if why is not None:
            return why
        if self._trip_state(n) is True:
            act = self._trip_action([n])
            return Why(("軸%d は遮断中です。" % n) + act, ("Axis %d is tripped. " % n) + (en_of(act) or TRIP_ACTION_TEXT_EN))
        b = self._board(n)
        if not b.get("alive"):
            return Why("軸%d の基板から返事がありません（電源と CAN を確かめてから）" % n,
                       "No reply from axis %d (check power and CAN)" % n)
        if b.get("cal") is False:
            return Why("軸%d は較正が入っていません（極対数・向きが無いと測れない。krc.py の較正から）" % n,
                       "Axis %d is not calibrated (calibrate with krc.py first)" % n)
        if self._board_driving(n, time.time()):
            return Why("軸%d はほかの道具が通電しています（放されてから測ってください）" % n,
                       "Axis %d is energized by another tool (wait until it is released)" % n)
        saving = self._param_saving.get(n)
        if n in self._param_dirty or (saving is not None and saving[0] not in PARAM_SAVE_DONE):
            return Why("軸%d は基板の設定の保存待ちです（保存が済んでから）" % n,
                       "Axis %d is waiting to save board settings (wait until saved)" % n)
        miss = [p for p in (16,) + krcbus.ENC_CORR_PARAMS if self.bus.param_reported(n, p) is None]
        if miss:
            if self.bus.prm_wraps.get(n, 0) >= 2 and self.bus.prm_maxid.get(n, 0) < max(miss):
                return Why("軸%d のファームが古いので測れません（塊 13 にパラメータ %s が無い。焼き直してから）"
                           % (n, "・".join(str(p) for p in miss)),
                           "Axis %d firmware is too old to measure (no parameter %s; reflash first)"
                           % (n, ", ".join(str(p) for p in miss)))
            return Why("軸%d の電気角オフセット・補正（パラメータ 16・39〜42）をまだ読めていません（塊 13。数秒待つ・"
                       "tlm が止まっていないか）" % n,
                       "Axis %d: offset / correction (params 16, 39-42) not read yet (wait a few seconds)" % n)
        return None

    def _offcal_stop(self, n, why, running_only=False):
        j = self.offcal.get(n)
        if j is None:
            return False
        with j.mu:
            if not j.busy() or (running_only and j.stage != "running"):
                return False
            if j.stop_why is None:
                j.stop_why = str(why)
            L = j.link
        j.go.set()
        if L is not None:
            L.stop(why)
        return True

    def _offcal_yield(self, nodes, why, running_only=False):
        return [n for n in list(nodes) if self._offcal_stop(n, why, running_only)]

    def _offcal_alive(self, j):
        gap = time.monotonic() - j.last_poll
        if gap > WATCHDOG_S:
            return "画面からの応答が %.1f s 途切れた" % gap
        return None

    def _offcal_sock(self):
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            s.bind((self.bus.iface,))
        except OSError:
            s.close()
            raise
        return s

    def _offcal_end(self, j, stage, why=None, res=None):
        with j.mu:
            j.stage = stage
            j.why = why
            if res is not None:
                j.res = res
            j.t0 = None
            j.link = None

    def _offcal_run(self, j):
        L = None
        try:
            if j.kind == "gear" and j.side != 0:
                why = self._offcal_gear_seek(j)
                if why is not None:
                    self._offcal_end(j, "failed", why=why)
                    return
                with j.mu:
                    j.stage = "running"
                    j.t0 = None
            L = offset_full.StopLink(j.node, self._offcal_sock(), alive=lambda: self._offcal_alive(j))
            with j.mu:
                j.link = L
                w = j.stop_why
            if w is not None:
                raise offset_full.Stopped(w)
            st, why = offset_full.begin(L)
            if why is None:
                why = offset_full.trip_why(st["tr0"])
            if why is not None:
                self._offcal_end(j, "failed", why=why)
                return
            with j.mu:
                j.st = st
                j.est_s = offset_full.round_seconds(st["pp"])
            for k in range(1, j.n + 1):
                if k > 1 and j.kind == "gear":
                    with j.mu:
                        j.k = k
                        j.t0 = None
                        j.rest_until = time.time() + offset_full.REST_S
                    while time.time() < j.rest_until:
                        with j.mu:
                            if j.stop_why is not None:
                                raise offset_full.Stopped(j.stop_why)
                        L.drain()
                        time.sleep(0.1)
                    with j.mu:
                        j.rest_until = None
                elif k > 1:
                    with j.mu:
                        if j.stop_why is not None:
                            raise offset_full.Stopped(j.stop_why)
                        j.stage = "wait_push"
                        j.k = k
                        j.t0 = None
                        j.go.clear()
                    while not j.go.wait(0.1):
                        L.drain()
                    with j.mu:
                        if j.stop_why is not None:
                            raise offset_full.Stopped(j.stop_why)
                        j.stage = "running"
                with j.mu:
                    j.k = k
                    j.t0 = time.time()
                r = offset_full.one_round(L, st["pp"], order=((j.side, -j.side) if (j.kind == "gear" and j.side)
                                                               else (+1, -1)))
                if r["why"] is not None:
                    self._offcal_end(j, "failed", why=r["why"])
                    return
                if r["bad"]:
                    self._offcal_end(j, "failed", why="%s: %s" % (offset_full.fail_head(k), "／".join(r["bad"])))
                    return
                with j.mu:
                    j.vals.append((r["val"], r["lag"]))
                    j.fits.append(r["fit"])
            res = offset_full.summarize([v for v, _lag in j.vals], st["off"], st["dr"], st["pp"], j.fits, st["corr"])
            self._offcal_end(j, "done", res=res)
        except offset_full.Stopped as e:
            self._offcal_end(j, "failed", why=Why("中止しました（%s）。何も入れていません" % e,
                                                  "Cancelled%s. Nothing was written" % ((" (%s)" % en_of(e.args[0]))
                                                                                      if e.args and en_of(e.args[0]) else "")))
        except Exception as e:
            self._offcal_end(j, "failed", why=Why("測れませんでした（何も入れていません）: %s" % e,
                                                  "Measurement failed (nothing written): %s" % e))
        finally:
            if L is not None:
                try:
                    L.s.close()
                except OSError:
                    pass

    def _offcal_gear_why(self, n, stopper=True):
        if self.bus.params.get((n, 16)) is None:
            return Why("軸%d の基板のオフセット（パラメータ 16）をまだ読めていません（数秒待つ）" % n,
                       "Axis %d: board offset (param 16) not read yet (wait a few seconds)" % n)
        if self.bus.cal_of(n) is not True:
            return Why("軸%d の基板にオフセットが入っていません（較正なし。裸のカードで「電気角オフセットを測る」→「入れる」→"
                       "「保存する」を先に）" % n,
                       "Axis %d has no offset on the board (not calibrated; measure and save it bare first)" % n)
        if not stopper:
            return None
        b = self._board(n)
        if b.get("lim_on") is None:
            return Why("軸%d の壁（可動範囲）の有無がまだ読めていません" % n, "Axis %d: walls not read yet" % n)
        if b.get("lim_on"):
            return Why("軸%d は壁（可動範囲）があるので使えません（壁で止まってストッパと読み違える）" % n,
                       "Axis %d has walls set (it would stop at a wall, not the stopper)" % n)
        return self._arm_block_axis(n)

    def _offcal_gear_seek(self, j):
        n, side = j.node, j.side
        a = self.ax.get(n)
        if a is None:
            return Why("軸%d は居ません" % n, "Axis %d not found" % n)
        prev = (a.kp, a.kd, a.glide_v)
        armed_by_me = False
        holding = False

        def stopped():
            with j.mu:
                return j.stop_why

        def lost():
            if self._trip_state(n) is True:
                return Why("遮断した", "tripped")
            if armed_by_me and n not in self.driven_nodes():
                return Why("通電が落ちた（解除・非常停止・画面の応答切れ）", "power was lost (disarm, E-stop or page timeout)")
            if holding and (self.ax.get(n) is not a or a.mode != "hold"):
                return Why("位置保持が外れた（ほかの操作）", "position hold was left (another command)")
            return None

        def wait(pred, secs):
            end = time.time() + secs
            while True:
                w = stopped()
                if w is not None:
                    return w
                with self.lock:
                    why = lost()
                    if why is None and pred():
                        return None
                if why is not None:
                    return why
                if time.time() > end:
                    return "timeout"
                time.sleep(0.02)

        def fail(head, why):
            return Why("%s（%s）。何も入れていません" % (head, why),
                       "%s (%s). Nothing was written" % (en_of(head) or head, en_of(why) or why))

        def err_of(r):
            e = r.get("err")
            return Why(str(e), en_of(e) or str(e))

        try:
            r = self.do("arm", {"node": n, "cid": j.cid})
            if not r.get("ok"):
                return fail(Why("通電できませんでした", "Could not arm"), err_of(r))
            w = wait(lambda: n in self.driven_nodes(), 1.5)
            if w is not None:
                return fail(Why("通電できませんでした", "Could not arm"), Why(str(self.note), self.note_en or str(self.note))
                            if w == "timeout" else w)
            armed_by_me = True
            r = self.do("mode", {"node": n, "mode": "hold"})
            if not r.get("ok"):
                return fail(Why("位置保持に入れませんでした", "Could not enter hold"), err_of(r))
            w = wait(lambda: a.mode == "hold", 1.5)
            if w is not None:
                return fail(Why("位置保持に入れませんでした", "Could not enter hold"),
                            Why("時間切れ", "timed out") if w == "timeout" else w)
            holding = True
            with self.lock:
                pos0 = self._pos_now(n)
                lo, hi = self._p_cmd_bound(n)
            if pos0 is None:
                return fail(Why("位置が読めません", "Position unreadable"), Why("テレメトリ", "telemetry"))
            far = clamp(pos0 + side * OCG_SEEK_MAX_REV * 2.0 * math.pi, lo, hi)
            with j.mu:
                j.stage = "seek"
            r = self.do("set", {"node": n, "kp": OCG_KP, "kd": OCG_KD, "glide": OCG_SEEK_V, "stop_iq": OCG_STOP_IQ,
                                "stop_dir": float(side), "stop_skip": 0.0, "stop_vslow": OCG_STOP_VSLOW, "p": far})
            if not r.get("ok"):
                return fail(Why("ストッパへ動かせませんでした", "Could not move to the stopper"), err_of(r))
            w = wait(lambda: a.stop_hit is not None
                     or (a.traj_p is not None and a.traj_p == a.p_target and a.traj_v == 0.0),
                     abs(far - pos0) / OCG_SEEK_V + 5.0)
            if w is not None:
                return fail(Why("ストッパを探すのをやめました", "Stopper search stopped"),
                            Why("時間切れ", "timed out") if w == "timeout" else w)
            with self.lock:
                hit = a.stop_hit
            if hit is None:
                return fail(Why("ストッパに当たりませんでした", "Did not hit a stopper"),
                            Why("%s側へローター %.1f 回転（出力 %.0f°）動かしても当たらない・送れる端で止まった"
                                % ("＋" if side > 0 else "−", abs(far - pos0) / (2.0 * math.pi),
                                   math.degrees(abs(far - pos0)) / GEAR_RATIO),
                                "moved %.1f rotor turns without a hit" % (abs(far - pos0) / (2.0 * math.pi))))
            back = hit["pos"] - side * OCG_BACK_REV * 2.0 * math.pi
            if not (lo <= back <= hi):
                return fail(Why("ストッパから戻せません", "Cannot back off"), Why("戻す位置が送れる端の外", "outside the range"))
            with j.mu:
                j.stage = "back"
                j.hit = hit["pos"]
            r = self.do("set", {"node": n, "p": back})
            if not r.get("ok"):
                return fail(Why("ストッパから戻せませんでした", "Could not back off"), err_of(r))

            def arrived():
                p = self._pos_now(n)
                return (a.traj_p is not None and a.traj_p == a.p_target and a.traj_v == 0.0
                        and p is not None and abs(p - back) < OCG_ARRIVE_RAD)
            w = wait(arrived, OCG_BACK_REV * 2.0 * math.pi / OCG_SEEK_V + 5.0)
            if w is not None:
                return fail(Why("ストッパから戻れませんでした", "Could not back off"),
                            Why("時間切れ", "timed out") if w == "timeout" else w)
            t_dis = time.time()
            holding = False
            armed_by_me = False
            self.do("disarm", {"node": n})

            def idle():
                t4 = self.bus.tlm_t.get((n, 4))
                return (t4 is not None and t4 > t_dis + 0.05
                        and krcbus.foc_mode_of(self.bus.tlm.get((n, 4))) == krcbus.FOC_MODE_IDLE)
            w = wait(idle, OCG_IDLE_WAIT_S)
            if w is not None:
                return fail(Why("解除したあと基板が止まったと確かめられません", "Could not confirm the board stopped after disarm"),
                            Why("塊 4 が IDLE と言わない", "block 4 did not report IDLE") if w == "timeout" else w)
            return None
        finally:
            with self.lock:
                if self.ax.get(n) is a:
                    a.stop_iq = 0.0
                    a.kp, a.kd, a.glide_v = prev
                still = armed_by_me and n in self.driven_nodes() and self.ax.get(n) is a and a.mode == "hold"
            if still:
                self.do("disarm", {"node": n})

    def _offcal_cmd(self, action, params):
        n = num_int(params.get("node"), "軸番号")
        j = self.offcal.get(n)

        def no(why):
            self.note = why
            return {"ok": False, "err": why}

        if action == "offcal_start":
            cnt = num_int(params.get("n", offset_full.N_DEF), "回数")
            why = self._offcal_why(n)
            if why is not None:
                return no(why)
            if j is not None and j.stage in ("applying", "saving"):
                return no(Why("軸%d は測った値を%s最中です（済んでから）" % (n, "入れている" if j.stage == "applying" else "保存している"),
                              "Axis %d is %s the result (wait until done)" % (n, "writing" if j.stage == "applying" else "saving")))
            if j is not None and j.stage == "prep":
                return no(Why("軸%d はローター傾き補正を始めている最中です" % n, "Axis %d is starting the rotor tilt correction" % n))
            gear = params.get("gear") is True
            side = 1
            if gear:
                side = num(params.get("side", 1), "ストッパの側")
                if side not in (1.0, -1.0, 0.0):
                    return no(Why("ストッパの側は +1・−1・0（なし）のどれかです", "Stopper side must be +1, -1 or 0 (none)"))
                why = self._offcal_gear_why(n, stopper=(side != 0))
                if why is not None:
                    return no(why)
            if params.get("free") is not True:
                if gear and side == 0:
                    return no(Why("その場から両向きにローター 1 回転強まわして大丈夫か（途中に当たる物が無い・脚が付いていない）"
                                  "確かめてから、チェックを入れてください",
                                  "Make sure it can turn a bit over one rotor turn both ways (nothing in the way, no leg), "
                                  "then tick the check box"))
                if gear:
                    return no(Why("ストッパの側へ回して大丈夫か（途中に当たる物が無い・脚が付いていない）確かめてから、チェックを"
                                  "入れてください",
                                  "Make sure it can turn toward the stopper (nothing in the way, no leg), then tick the check box"))
                return no(Why("腕・脚を外して、ローターが 1 回転まわれる状態か確かめてから、チェックを入れてください"
                              "（脚に組んだまま使わない）",
                              "Remove the arm/leg so the rotor can turn a full revolution, then tick the check box"))
            if not offset_full.N_MIN <= cnt <= offset_full.N_MAX:
                return no(Why("回数は %d〜%d 回です" % (offset_full.N_MIN, offset_full.N_MAX),
                              "Runs must be %d-%d" % (offset_full.N_MIN, offset_full.N_MAX)))
            j = OffCal(n, cnt, params.get("cid"), kind=("gear" if gear else "bare"), side=int(side))
            self.offcal[n] = j
            threading.Thread(target=self._offcal_run, args=(j,), daemon=True).start()
            if gear and side == 0:
                self.note = Why("軸%d のローター傾き補正を測ります（ストッパなし・その場から %d 回）。回っているあいだは触らないでください"
                                % (n, cnt),
                                "Axis %d: measuring the rotor tilt correction (no stopper, from here, %d runs). Do not touch"
                                % (n, cnt))
                return {"ok": True}
            if gear:
                self.note = Why("軸%d のローター傾き補正を測ります（%s側のストッパへ当ててから %d 回）。動いているあいだは触らないでください"
                                % (n, "＋" if side > 0 else "−", cnt),
                                "Axis %d: measuring the rotor tilt correction (%s stopper first, %d runs). Do not touch"
                                % (n, "+" if side > 0 else "-", cnt))
                return {"ok": True}
            self.note = Why("軸%d の電気角オフセットを測ります（%d 回）。回っているあいだは触らないでください" % (n, cnt),
                            "Axis %d: measuring the offset (%d runs). Do not touch while it turns" % (n, cnt))
            return {"ok": True}
        if j is None:
            return no(Why("軸%d は電気角オフセットを測っていません" % n, "Axis %d is not measuring the offset" % n))
        if action == "offcal_next":
            if j.stage != "wait_push":
                return no(Why("軸%d は押し込み直しを待っていません" % n, "Axis %d is not waiting for a re-seat" % n))
            why = self._offcal_why(n, own=True)
            if why is not None:
                return no(why)
            with j.mu:
                j.cid = params.get("cid")
                j.beat()
                j.stage = "running"
                j.t0 = None
            j.go.set()
            self.note = Why("軸%d の %d 回目を回します。触らないでください" % (n, j.k),
                            "Axis %d: run %d. Do not touch" % (n, j.k))
            return {"ok": True}
        if action == "offcal_cancel":
            if j.stage == "prep":
                with j.mu:
                    if j.stop_why is None:
                        j.stop_why = Why("画面の「中止」", "Cancel pressed")
                j.go.set()
                self.note = Why("軸%d のローター傾き補正を中止しました（何も入れていません）" % n,
                                "Axis %d: rotor tilt correction cancelled (nothing written)" % n)
                return {"ok": True}
            if self._offcal_stop(n, Why("画面の「中止」", "Cancel pressed")):
                self.note = Why("軸%d の電気角オフセットの測定を中止しました（cfg 0 を送りました・何も入れていません）" % n,
                                "Axis %d: offset measurement cancelled (nothing written)" % n)
                return {"ok": True}
            if j.stage in ("applying", "saving"):
                return no(Why("軸%d は測った値を%s最中です（済んでから）" % (n, "入れている" if j.stage == "applying" else "保存している"),
                              "Axis %d is %s the result (wait until done)" % (n, "writing" if j.stage == "applying" else "saving")))
            del self.offcal[n]
            self.note = ""
            return {"ok": True}
        if action == "offcal_apply":
            if j.stage != "done" or j.res is None:
                return no(Why("軸%d は入れる結果がありません（測り終えてから）" % n, "Axis %d has no result to write yet" % n))
            why = self._offcal_why(n, own=True)
            if why is not None:
                return no(why)
            mode = params.get("mode")
            if mode not in offset_full.MODES:
                return no("入れ方が読めません（%r：%s のどれか。画面を読み直してから）" % (mode, "・".join(offset_full.MODES)))
            if (j.kind == "gear") != (mode == "gear"):
                return no(Why("軸%d: 入れ方 %s はこの測定には使えません（ローター傾き補正は「補正だけ入れる」、裸の測定は「補正も入れる」"
                              "か「オフセットだけ」）" % (n, mode),
                              "Axis %d: mode %s does not fit this measurement" % (n, mode)))
            if mode in ("corr", "gear"):
                why = offset_full.corr_why(j.res)
                if why is not None:
                    return no("軸%d: %s" % (n, why))
            was = dict([(16, j.st["off"])] + list(zip(krcbus.ENC_CORR_PARAMS, j.st["corr"])))
            moved = []
            for pid in sorted(was):
                r = self.bus.params.get((n, pid))
                if r is None or r[0] != was[pid]:
                    moved.append("%s %d → %s" % (krcbus.PARAMS[pid][0], was[pid], "読めない" if r is None else r[0]))
            if moved:
                return no("軸%d の電気角オフセット・補正が測り始めたときから変わっています（%s：基板の再起動・別の道具）。"
                          "測り直してください" % (n, "・".join(moved)))
            want = offset_full.apply_values(j.res, mode)
            now = time.time()
            for pid, x in sorted(want.items()):
                self.pending.append((now, lambda nn=n, pp=pid, vv=krcbus.param_value(pid, x): self.bus.set_param(nn, pp, vv),
                                     True))
                self._param_sent[(n, pid)] = (x, now)
            with j.mu:
                j.want = want
                j.mode = mode
                j.stage = "applying"
                j.apply_err = None
            if mode == "gear":
                self.note = Why("軸%d に補正 39〜42 を入れます（オフセットは裸の値 %d のまま・保存はまだ）" % (n, j.st["off"]),
                                "Axis %d: writing correction 39-42 (offset stays at the bare value %d; not saved yet)"
                                % (n, j.st["off"]))
                return {"ok": True}
            self.note = Why("軸%d に電気角オフセット %d %s（保存はまだ）"
                            % (n, j.res["new_off"], "と補正 39〜42 を入れます" if mode == "corr"
                               else "を入れます（補正 39〜42 は 0 に戻す）"),
                            "Axis %d: writing offset %d%s (not saved yet)"
                            % (n, j.res["new_off"], " and correction 39-42" if mode == "corr"
                               else " (correction 39-42 reset to 0)"))
            return {"ok": True}
        if j.stage != "applied" or not j.want:
            return no(Why("軸%d は先に「入れる」を押してください（入れた値を保存します）" % n,
                          "Axis %d: press \"Write\" first (saving writes what was written)" % n))
        self._param_dirty[n] = time.time() - 1.0
        self._param_new.setdefault(n, set()).update(j.want)
        with j.mu:
            j.stage = "saving"
            j.save_err = None
            j.save_text = None
        self.note = Why("軸%d の基板に保存します（解除中・止まっているときに焼きます）" % n,
                        "Axis %d: saving to the board (flashes while disarmed and still)" % n)
        return {"ok": True}

    def _offcal_tick(self):
        for n, j in list(self.offcal.items()):
            if j.stage == "applying":
                bad, unread = self._param_echo(n, j.want)
                if bad:
                    got = dict((p, self.bus.params.get((n, p))) for p in bad)
                    with j.mu:
                        j.stage = "done"
                        j.apply_err = ("入っていない値があります（%s：基板の読み返しが送った値と違う。遮断中・再起動・別の送り手）。"
                                       "もう一度「入れる」か、測り直してください"
                                       % "・".join("%s 送った %d → 基板 %s" % (krcbus.PARAMS[p][1], j.want[p],
                                                                            "–" if got[p] is None else got[p][0])
                                                  for p in bad))
                elif not unread:
                    with j.mu:
                        j.stage = "applied"
            elif j.stage == "saving" and n in self.ax:
                r = self._param_view(n)[1]
                with j.mu:
                    if r == "saved":
                        j.stage = "saved"
                        j.save_text = None
                    elif r in PARAM_SAVE_DONE:
                        j.stage = "applied"
                        j.save_err = krcbus.PARAM_SAVE_TEXT.get(r, r)
                    else:
                        j.save_text = krcbus.PARAM_SAVE_TEXT.get(r) if r else None

    def _offcal_gear_info(self, n):
        r = self.bus.params.get((n, 16))
        why = self._offcal_gear_why(n)
        wf = self._offcal_gear_why(n, stopper=False)
        return dict(board_off=(r[0] if r else None), gear_why=why, gear_why_en=en_of(why),
                    gear_why_free=wf, gear_why_free_en=en_of(wf))

    def _offcal_view(self, n, now):
        j = self.offcal.get(n)
        start_why = self._offcal_why(n)
        if j is None:
            pp = self._board(n).get("pole_pairs")
            return dict(stage="idle", start_why=start_why, start_why_en=en_of(start_why),
                        round_s=(offset_full.round_seconds(pp) if pp else None), **self._offcal_gear_info(n))

        def fin(x):
            return x if (x is not None and math.isfinite(x)) else None
        with j.mu:
            v = dict(stage=j.stage, k=j.k, n=j.n, est_s=j.est_s, why=j.why,
                     prep=(j.stage == "running" and j.t0 is None and j.rest_until is None),
                     vals=[[a, b] for a, b in j.vals],
                     apply_err=j.apply_err, save_err=j.save_err, save_text=j.save_text,
                     off=(j.st["off"] if j.st else None),
                     kind=j.kind, side=j.side, hit=j.hit,
                     rest_s=(max(0.0, j.rest_until - now) if j.rest_until is not None else None))
            if j.stage == "running" and j.t0 is not None and j.est_s:
                el = max(0.0, now - j.t0)
                v["left_s"] = max(0.0, j.est_s - el)
                v["frac"] = min(1.0, el / j.est_s)
            if j.res is not None:
                v.update(mean=fin(j.res["mean"]), sd=fin(j.res["sd"]), span=fin(j.res["span"]),
                         d_off=fin(j.res["d_off"]), new_off=j.res["new_off"], mode=j.mode,
                         corr_now=j.res.get("corr_now"), corr_new=j.res.get("corr_new"),
                         corr_why=offset_full.corr_why(j.res),
                         **dict((k, fin(j.res.get(k))) for k in ("rms_before", "width_before", "rms_after",
                                                                  "max_after", "rms_now", "width_now")))
        v["start_why"] = start_why
        v["next_why"] = self._offcal_why(n, own=True) if v["stage"] == "wait_push" else None
        v["apply_why"] = self._offcal_why(n, own=True) if v["stage"] == "done" else None
        v.update(self._offcal_gear_info(n))
        for k in ("start_why", "next_why", "apply_why", "why", "apply_err", "save_err", "corr_why"):
            if v.get(k) is not None:
                v[k + "_en"] = en_of(v[k])
        return v

    def _rescan_locked(self):
        seen = self._seen_nodes()
        added = sorted(seen - set(self.ax))
        for n in added:
            self.ax[n] = AxUI(n)
            ax = _new_axis(n)
            ax.carry_boot(self.bus.axes.get(n))
            self.bus.axes[n] = ax
            self.bus.axes[n].set_tripped(None)
            self._later(0.0, lambda k=n: self._send_special(0xFD, k))
        if added:
            self.order = sorted(self.ax)
            self.bus.order = list(self.order)
        if self.node is None and self.order:
            self.node = self.order[0]
        return added

    def _seen_nodes(self):
        now = time.time()
        seen = set()
        for (n, _idx) in list(self.bus.tlm.keys()):
            if not valid_node(n):
                continue
            if (now - self.bus.tlm_t.get((n, _idx), 0.0)) > BOARD_FRESH_S:
                continue
            seen.add(n)
        return seen

    def _prune_locked(self):
        if self.pending:
            return []
        if time.time() - self.bus.listen_t <= BOARD_FRESH_S:
            return []
        seen = self._seen_nodes()
        saving = set(n for n, rec in self._param_saving.items() if rec[0] not in PARAM_SAVE_DONE)
        now = time.time()
        gone = sorted(n for n in self.ax if n not in seen and n not in self.homing
                      and n not in self._param_dirty and n not in saving and not self._offcal_busy(n)
                      and not self._board_held(n, now))
        if not gone:
            return []
        for n in gone:
            del self.ax[n]
            self._param_saving.pop(n, None)
            self._param_dirty.pop(n, None)
            for d in (self._param_new, self._param_save_pids, self._param_quiet):
                d.pop(n, None)
            for key in [kk for kk in self._param_sent if kk[0] == n]:
                del self._param_sent[key]
        self.order = sorted(self.ax)
        self.bus.order = list(self.order)
        self.arm_nodes = [k for k in self.arm_nodes if k in self.ax]
        if self.node not in self.ax:
            self.node = self.order[0] if self.order else None
        if self.target != "all" and self.target not in self.ax:
            self.target = "all"
        return gone

    def _p_bound(self, n):
        lo, hi = -POS_SAFE_RAD, POS_SAFE_RAD
        b = self._board(n)
        if b.get("lim_on") and b.get("lim_lo") is not None and b.get("lim_hi") is not None:
            lo = max(lo, float(b["lim_lo"]) - LIMIT_RAD_PER_LSB)
            hi = min(hi, float(b["lim_hi"]) + LIMIT_RAD_PER_LSB)
        if not (lo < hi):
            return -POS_SAFE_RAD, POS_SAFE_RAD
        return lo, hi

    def _p_cmd_bound(self, n):
        lo, hi = self._p_bound(n)
        return max(lo, POS_CMD_LO), min(hi, POS_CMD_HI)

    def _leg_sine_tout(self, a, t):
        mid = 0.5 * (a.leg_lo + a.leg_hi)
        amp = 0.5 * (a.leg_hi - a.leg_lo)
        ph = 2.0 * math.pi * (math.fmod(t, a.leg_period) / a.leg_period) + a.leg_phase0
        return mid + amp * math.sin(ph)

    def _leg_traj_tout(self, a, t):
        pts = a.leg_traj_pts
        T = pts[-1][0]
        tt = t * a.leg_traj_speed
        if a.leg_traj_loop and T > 0.0:
            tt = math.fmod(tt, T)
        if tt <= 0.0:
            return pts[0][1]
        if tt >= T:
            return pts[-1][1]
        lo, hi = 0, len(pts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if pts[mid][0] <= tt:
                lo = mid
            else:
                hi = mid
        t0, y0 = pts[lo]
        t1, y1 = pts[hi]
        f = (tt - t0) / (t1 - t0) if t1 > t0 else 0.0
        return y0 + (y1 - y0) * f

    def _leg_tout_at(self, a, t):
        return self._leg_traj_tout(a, t) if a.mode == "leg_traj" else self._leg_sine_tout(a, t)

    def _grav_ff(self, n):
        cal = self._leg_cal(n)
        if not (cal.valid and cal.grav.on):
            return 0.0
        pos = self._pos_now(n)
        if pos is None:
            return 0.0
        tout = cal.motor_to_tout(pos)
        if tout is None:
            return 0.0
        tm = cal.motor_from_knee_nm(pos, cal.grav.tau_knee(tout))
        if tm is None or not math.isfinite(tm):
            return 0.0
        return clamp(tm, -T_MAX, T_MAX)

    def _leg_track_cmd(self, now, n):
        a = self.ax[n]
        cal = self._leg_cal(n)
        lo, hi = self._p_cmd_bound(n)
        tff = self._grav_ff(n)
        bad = (a.leg_period <= 0.0) if a.mode == "leg_sine" else (not a.leg_traj_pts)
        if not cal.valid or bad:
            p, v = self._traj_step(now, n, a.p_target)
            return (clamp(p, lo, hi), v, a.kp, a.kd, tff)
        if not a.leg_run:
            p, v = self._traj_step(now, n, a.p_target)
            if self._leg_arrived(n):
                a.leg_run = True
                a.leg_t0 = now
            else:
                return (clamp(p, lo, hi), v, a.kp, a.kd, tff)
        t = now - a.leg_t0
        near = a.traj_p if a.traj_p is not None else a.p_target
        ms = []
        for tt in (t - LEG_SINE_DH_S, t, t + LEG_SINE_DH_S):
            mm = cal.tout_to_motor(self._leg_tout_at(a, tt), near)
            if mm is None:
                p, v = self._traj_step(now, n, a.p_target)
                return (clamp(p, lo, hi), v, a.kp, a.kd, tff)
            ms.append(mm)
        a.leg_des_tout = self._leg_tout_at(a, t)
        a.traj_p = ms[1]
        v = clamp((ms[2] - ms[0]) / (2.0 * LEG_SINE_DH_S), -V_MAX, V_MAX)
        return (clamp(ms[1], lo, hi), v, a.kp, a.kd, tff)

    def _leg_enter_auto(self, n, mode, lo_deg, hi_deg, vmax):
        a = self.ax[n]
        cal = self._leg_cal(n)
        if a.mode != "hold":
            bad = self._gate_some([n])
            if bad:
                self.note = "位置モードに入れません: %s" % "／".join("軸%d %s" % (k, w) for k, w in bad)
                return {"ok": False, "err": self.note}
            if not self._sync_targets_some([n], mode="hold"):
                return {"ok": False, "err": self.note}
            a.mode = "hold"
        pos = self._pos_now(n)
        tout_now = cal.motor_to_tout(pos) if pos is not None else None
        start = clamp(tout_now if tout_now is not None else lo_deg, lo_deg, hi_deg)
        m_start = cal.tout_to_motor(start, pos)
        if m_start is None:
            return {"ok": False, "err": "軸%d 始点 Θout %.1f° はリンクが閉じません" % (n, start)}
        lo, hi = self._p_cmd_bound(n)
        a.leg_run = False
        a.leg_des_tout = None
        a.leg_vmax = vmax
        self._apply_fields(n, [("p_target", m_start)], lo, hi)
        a.mode = mode
        self.note = ("" if vmax <= V_MAX else
                     "軸%d 軌道: 速さ %.3g rad/s は CAN で送れる %.0f rad/s を超えるので、速度の前置きは頭打ち（追従は落ちる）"
                     % (n, vmax, V_MAX))
        return {"ok": True, "start": start, "vmax": vmax, "warn": self.note or None}

    def _leg_traj_plan(self, cal, bound, near, pts, speed):
        lo, hi = bound
        ms = []
        ts = []
        for k in range(len(pts) - 1):
            t0, y0 = pts[k]
            t1, y1 = pts[k + 1]
            for j in range(LEG_TRAJ_CHECK_SUB):
                f = j / float(LEG_TRAJ_CHECK_SUB)
                tout = y0 + (y1 - y0) * f
                mm = cal.tout_to_motor(tout, near)
                if mm is None:
                    return ("Θout %.1f° はリンクが閉じません" % tout, None)
                if mm < lo - 1e-9 or mm > hi + 1e-9:
                    return ("Θout %.1f° は壁の外です（モーター %.2f rad、壁 %.2f〜%.2f）" % (tout, mm, lo, hi), None)
                ms.append(mm)
                ts.append((t0 + (t1 - t0) * f) / speed)
                near = mm
        mm = cal.tout_to_motor(pts[-1][1], near)
        if mm is None or mm < lo - 1e-9 or mm > hi + 1e-9:
            return ("Θout %.1f° は壁の外か閉じません" % pts[-1][1], None)
        ms.append(mm)
        ts.append(pts[-1][0] / speed)
        vmax = 0.0
        for k in range(len(ms) - 1):
            dt = ts[k + 1] - ts[k]
            if dt > 0.0:
                vmax = max(vmax, abs(ms[k + 1] - ms[k]) / dt)
            else:
                vmax = math.inf
        if not math.isfinite(vmax):
            return ("時間の刻みが小さすぎて速さが計算できません", None)
        return (None, vmax)

    def _grav_put(self, n, cal, g):
        new = copy.copy(cal)
        new.grav = g
        cals = dict(self.leg_cal)
        cals[n] = new
        try:
            krcleg.save_all(cals)
        except OSError as e:
            return "軸%d 重力補償を保存できませんでした（何も変えていません）: %s" % (n, e)
        cal.grav = g
        return None

    def _rec_stop(self):
        with self.lock:
            r = self.rec
            if r is None:
                return {"ok": False, "err": "記録していません", "last": self.rec_last}
            self.rec = None
        err = r.close()
        with self.lock:
            self.rec_last = dict(file=r.name, rows=r.written, err=err)
            if err is None:
                self.note = ""
                return {"ok": True, "file": r.name, "rows": r.written}
            self.note = "記録 %s は途中で書けなくなりました（書けたのは %d 行）: %s" % (r.name, r.written, err)
            return {"ok": False, "err": self.note, "file": r.name, "rows": r.written}

    def _leg_traj_gate(self, n):
        cal = self._leg_cal(n)
        if n in self.homing or not cal.valid:
            self.note = "軸%d は同期していません（%s）" % (
                n, "原点出しの最中です" if n in self.homing else cal.why_invalid())
            return {"ok": False, "err": self.note}
        return None

    def _leg_map_key(self, n):
        cal = self._leg_cal(n)
        return (cal.plus_end, cal.tin0, cal.half) + tuple(self._p_cmd_bound(n))

    def _leg_traj_cmd(self, params):
        n = num_int(params.get("node"), "node")
        loop = params.get("loop", True)
        if not isinstance(loop, bool):
            raise BadValue("loop が真偽として読めません: %r" % (loop,))
        speed = num(params.get("speed", 1.0), "speed")
        if not (speed > 0.0):
            return {"ok": False, "err": "速さは正の数"}
        with self.lock:
            if n not in self.ax:
                return {"ok": False, "err": "軸 %r は居ません" % (n,)}
            gate = self._leg_traj_gate(n)
            if gate is not None:
                return gate
            cal = copy.copy(self._leg_cal(n))
            key = self._leg_map_key(n)
            bound = self._p_cmd_bound(n)
            near = self._pos_now(n)
        pts, err = parse_traj(params.get("points", params.get("text")))
        if err:
            return {"ok": False, "err": err}
        off = krcleg.wrap180(pts[0][1]) - pts[0][1]
        if off != 0.0:
            pts = [(t, y + off) for t, y in pts]
        err, vmax = self._leg_traj_plan(cal, bound, near, pts, speed)
        with self._op_lock():
            self._op_queued = False
            self._jt_yield("leg_traj", params)
            if err:
                self.note = "軸%d 軌道: %s" % (n, err)
                return {"ok": False, "err": self.note}
            gate = self._leg_traj_gate(n)
            if gate is not None:
                return gate
            if self._leg_map_key(n) != key:
                self.note = "軸%d 軌道: 検めている間に較正か壁が変わりました（もう一度）" % n
                return {"ok": False, "err": self.note}
            a = self.ax[n]
            r = self._leg_enter_auto(n, "leg_traj", pts[0][1], pts[0][1], vmax)
            if not r.get("ok"):
                return r
            a.leg_traj_pts = pts
            a.leg_traj_loop = loop
            a.leg_traj_speed = speed
            a.leg_traj_name = str(params.get("name", ""))[:40]
            r.update(n_pts=len(pts), total_s=pts[-1][0] / speed)
            return r

    def _leg_arrived(self, n):
        a = self.ax[n]
        return a.traj_p is not None and a.traj_p == a.p_target and a.traj_v == 0.0

    def _grav_hold_why(self, n):
        cal = self._leg_cal(n)
        if n in self.homing or not cal.valid:
            return "同期していません"
        if n not in self.driven_nodes() or self.ax[n].mode != "hold":
            return "位置保持で止めてから（保持中の電流を測ります）"
        if not self._leg_arrived(n):
            return "目標へ動いている途中です（着いてから）"
        return None

    def _grav_sample_why(self, n):
        why = self._grav_hold_why(n)
        if why is None and self.ax[n].grav_samp is not None:
            return "測っている最中です"
        return why

    def _grav_commit(self, n, tout_m, tk):
        while True:
            with self.lock:
                cal = self._leg_cal(n)
                g_old = cal.grav
                g = copy.copy(g_old)
                g.samples = list(g_old.samples) + [(tout_m, tk)]
                new = copy.copy(cal)
                new.grav = g
                cals = dict(self.leg_cal)
                cals[n] = new
            try:
                krcleg.save_all(cals)
            except OSError as e:
                with self.lock:
                    self.note = ("軸%d 重力の標本は保存できませんでした（JSON が書けない。メモリも変えていません）: %s"
                                 % (n, e))
                return False
            with self.lock:
                if self.leg_cal.get(n) is cal and cal.grav is g_old:
                    cal.grav = g
                    self.note = "軸%d 重力の標本: Θout %.1f° で膝 %.3f N·m（標本 %d）" % (n, tout_m, tk, len(g.samples))
                    return True

    def _leg_sine_plan(self, n, lo_deg, hi_deg, period):
        cal = self._leg_cal(n)
        lo, hi = self._p_cmd_bound(n)
        pos = self._pos_now(n)
        mid = 0.5 * (lo_deg + hi_deg)
        amp = 0.5 * (hi_deg - lo_deg)
        near = pos
        ms = []
        for k in range(LEG_SINE_CHECK_N + 1):
            tout = mid + amp * math.sin(2.0 * math.pi * k / LEG_SINE_CHECK_N)
            mm = cal.tout_to_motor(tout, near)
            if mm is None:
                return ("Θout %.1f° はリンクが閉じません" % tout, None, None)
            if mm < lo - 1e-9 or mm > hi + 1e-9:
                return ("Θout %.1f° は壁の外です（モーター %.2f rad、壁 %.2f〜%.2f）"
                        % (tout, mm, lo, hi), None, None)
            ms.append(mm)
            near = mm
        dt = period / LEG_SINE_CHECK_N
        vmax = (max(abs(ms[k + 1] - ms[k]) / dt for k in range(LEG_SINE_CHECK_N))
                if dt > 0.0 else math.inf)
        if not math.isfinite(vmax):
            return ("周期 %r は小さすぎて計算できません" % period, None, None)
        return (None, None, vmax)

    def _leg_busy_why(self, n, walking=False):
        if n in self.homing:
            return Why("軸%d は原点出しの最中です" % n, "Axis %d is homing" % n)
        if walking and self._leg_walking(n):
            return Why("軸%d は脚の軌道の走行中です（先に止めてください）" % n, "Axis %d is running a leg trajectory (stop it first)" % n)
        return None

    def _leg_walking(self, n):
        return n in self.driven_nodes() and self.ax[n].mode in LEG_AUTO_MODES

    def _abort_homing(self, n, why):
        st = self.homing.pop(n, None)
        kind = self.homing_kind.pop(n, None)
        if st is None:
            return
        if st.phase != "fail":
            st._fail(why)
        self.leg_last[n] = dict(st.result)
        self.ax[n].mode = "free"
        if kind != "joint":
            self._leg_cal(n).invalidate()
        self.note = "軸%d 原点出しできませんでした: %s" % (n, st.result["why"])

    def _homing_cmd(self, now, n):
        st = self.homing.get(n)
        a = self.ax[n]
        if st is None:
            a.mode = "free"
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        tripped = self._trip_state(n) is True
        if not tripped and n not in self.driven_nodes():
            self._abort_homing(n, HOMING_POWER_LOST)
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        pos, pos_t = self.bus.pos_true_t(n)
        cmd, act = st.step(now, pos, tripped, self.bus.limits_reported(n), pos_t=pos_t)
        if act == "limits_off":
            self.bus.limits_off(n)
        elif act == "zero":
            self._zero_now(n)
            self._queue(0.15, lambda k=n: self._drop_pos_tlm(k), keep=True)
            st.zeroed(now)
        elif isinstance(act, tuple) and act[0] == "walls":
            self.bus.set_limits(n, act[1], act[2])
        if act == "done" and self.homing_kind.get(n) == "joint":
            self.homing.pop(n, None)
            self.homing_kind.pop(n, None)
            self.leg_last[n] = dict(st.result)
            a.mode = "hold"
            r = st.result
            self.note = ("軸%d 原点出し 完了：ストッパ（力 0 へ外挿）から半幅 %.1f° 戻った所を原点にした・壁 ±%.2f rad"
                         % (n, math.degrees(st.half_design / krcleg.RATIO_DEFAULT), r["lim"]))
            return (0.0, 0.0, a.kp, a.kd, 0.0)
        if act == "done":
            cal = self._leg_cal(n)
            if cal.sign is None or st.fold_sign != cal.sign:
                self._abort_homing(n, "当てた側と「畳み切りは＋側か−側か」の選択が食い違う（原点出しをやり直してください）")
                return (0.0, 0.0, 0.0, 0.0, 0.0)
            self.homing.pop(n, None)
            self.homing_kind.pop(n, None)
            self.leg_last[n] = dict(st.result)
            a.mode = "hold"
            r = st.result
            cal.set_span(r["hi"])
            self.note = ("軸%d 原点出し 完了：可動範囲 %.2f rad・壁 ±%.2f rad（保存はしません）"
                         % (n, r["span"], r["lim"]))
            return (0.0, 0.0, a.kp, a.kd, 0.0)
        if act == "fail":
            self._abort_homing(n, st.result["why"])
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        if cmd is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        p, v, kp, kd, t = cmd
        return (clamp(p, POS_CMD_LO, POS_CMD_HI), v, kp, kd, t)

    def _leg_cal(self, n):
        c = self.leg_cal.get(n)
        if c is None:
            c = krcleg.Calib()
            self.leg_cal[n] = c
        return c

    def _leg_view(self, n):
        a = self.ax[n]
        cal = self._leg_cal(n)
        pos = self._pos_now(n)
        st = self.homing.get(n)
        synced = cal.valid and st is None
        why = ("原点出しの最中です" if st is not None else cal.why_invalid())
        tin = cal.motor_to_tin(pos) if (synced and pos is not None) else None
        tout = cal.motor_to_tout(pos) if (synced and pos is not None) else None
        lo, hi = self._p_bound(n)
        reach = krcleg.reachable_tout(cal, lo, hi) if synced else (None, None)
        drv = n in self.driven_nodes()
        walking = self._leg_walking(n)
        tgt_tout = None
        if drv and synced and a.mode in POS_MODES:
            tgt_tout = cal.motor_to_tout(a.p_target)
        if walking and synced and a.leg_run and a.leg_des_tout is not None:
            tgt_tout = a.leg_des_tout
        ends = cal.end_touts() if synced else (None, None)
        ext_nm = fric_nm = grav_out = None
        if synced and drv and pos is not None:
            lp, lv, li = self._live(n, True)
            if li is not None:
                ext_nm, fric_nm = cal.ext_torque_out(pos, li, self._kt(n))
            tout_g = cal.motor_to_tout(pos)
            if tout_g is not None and cal.grav.on:
                grav_out = cal.grav.tau_knee(tout_g)
        gs = a.grav_samp
        samp_why = self._grav_sample_why(n)
        return dict(node=n, on=a.on, mode=a.mode, cal=cal.to_dict(),
                    end_lo=ends[0], end_hi=ends[1],
                    ext_nm=ext_nm, fric_nm=fric_nm, grav_out=grav_out,
                    grav=dict(cal.grav.to_dict(), sampling=(gs is not None),
                              can_sample=(samp_why is None),
                              sample_block=(None if samp_why is None else "軸%d: %s" % (n, samp_why)),
                              ff_motor=(self._grav_ff(n) if synced else 0.0)),
                    traj=dict(n_pts=(len(a.leg_traj_pts) if a.leg_traj_pts else 0),
                              total_s=((a.leg_traj_pts[-1][0] / a.leg_traj_speed) if a.leg_traj_pts else None),
                              loop=a.leg_traj_loop, speed=a.leg_traj_speed, name=a.leg_traj_name,
                              on=(walking and a.mode == "leg_traj"),
                              run=(walking and a.mode == "leg_traj" and a.leg_run)),
                    rec=self.rec_status(),
                    sine=dict(lo=a.leg_lo, hi=a.leg_hi, period=a.leg_period,
                              run=(walking and a.mode == "leg_sine" and a.leg_run),
                              on=(walking and a.mode == "leg_sine"), vmax=a.leg_vmax),
                    cal_block=self._leg_busy_why(n, walking=True),
                    synced=synced, sync_why=why,
                    slope=cal.slope_deg_per_rad(),
                    motor_rad=pos, tin=tin, tout=tout,
                    target_tout=tgt_tout, p_target=a.p_target,
                    reach_lo=reach[0], reach_hi=reach[1],
                    p_lo=lo, p_hi=hi,
                    homing=(None if st is None else dict(phase=st.phase, trace=list(st.trace))),
                    last=self.leg_last.get(n), tripped=self._trip_state(n))

    def rec_status(self):
        r = self.rec
        if r is None:
            return dict(on=False, last=self.rec_last)
        return dict(on=True, file=r.name, rows=r.rows, written=r.written,
                    secs=round(time.time() - r.t0, 1), last=self.rec_last)

    def rec_snapshot(self):
        files = []
        try:
            for fn in os.listdir(REC_DIR):
                if fn.endswith(".csv"):
                    p = os.path.join(REC_DIR, fn)
                    files.append(dict(name=fn, bytes=os.path.getsize(p), mtime=os.path.getmtime(p)))
        except OSError:
            pass
        files.sort(key=lambda x: x["mtime"], reverse=True)
        with self.lock:
            return dict(dir=REC_DIR, rec=self.rec_status(), files=files[:50], total=len(files))

    def leg_snapshot(self):
        with self.lock:
            return dict(design=krcleg.design(), armed=self.armed, note=self.note,
                        note_seq=self.note_seq, arm_block=self._arm_block(),
                        axes=[self._leg_view(n) for n in self.order])

    def _stop_check(self, now, n):
        a = self.ax[n]
        bus = self.bus.axes[n]
        h = a.stop_hit
        if h is not None and now - h["t"] < STOP_PEAK_S:
            h["peak"] = max(h["peak"], abs(bus.iq))
            pn = self._pos_now(n)
            if pn is not None and a.stop_dir * (pn - h["reach"]) > 0:
                h["reach"] = pn
        if not a.stop_iq:
            return
        pos = self._pos_now(n)
        if pos is None:
            return
        moving = a.traj_p is not None and abs(a.traj_v) > 0.2
        if not moving:
            a.stop_t0 = None
            a.stop_n = 0
            return
        if a.stop_from is None:
            a.stop_from = pos
        if a.stop_dir * (pos - a.stop_from) < a.stop_skip:
            a.stop_t0 = None
            a.stop_n = 0
            return
        iq = bus.iq
        if a.stop_t0 is None:
            if abs(a.traj_v) < 0.95 * min(a.glide_v, TRAJ_V_MAX):
                return
            a.stop_t0 = now
            a.stop_tl = now
            a.stop_ema = iq
            a.stop_n = 0
            return
        if bus.fb_count == a.stop_fb:
            return
        a.stop_fb = bus.fb_count
        dt = max(0.0, now - a.stop_tl)
        a.stop_tl = now
        d = a.stop_dir * (iq - a.stop_ema)
        if now - a.stop_t0 >= STOP_WARM_S:
            slow = (not a.stop_vslow) or (a.stop_dir * bus.vel < a.stop_vslow * abs(a.traj_v))
            a.stop_n = a.stop_n + 1 if (d > a.stop_iq and slow) else 0
            if a.stop_n >= STOP_N:
                a.stop_hit = dict(t=now, pos=pos, p_cmd=a.traj_p, v=a.traj_v, iq=iq, ema=a.stop_ema,
                                  peak=abs(iq), reach=pos)
                a.stop_iq = 0.0
                a.stop_t0 = None
                self._put_target(n, pos)
                if self.jt is not None and n in self.jt["nodes"]:
                    self._jt_end(Why("軸%d が軌道の始点へ寄る途中で当たったので、再生をやめてその場で位置保持にしました" % n,
                                     "Axis %d hit something while approaching the start; playback ended and it holds in place" % n))
                return
        if d < a.stop_iq * 0.5:
            a.stop_ema += (iq - a.stop_ema) * min(1.0, dt / max(1e-3, a.stop_ema_s))

    def _kt_why(self, n, any_mode=False):
        why = self._node_why({"node": n})
        if why is not None:
            return why
        if self._trip_state(n) is True:
            return Why("軸%d は遮断中です" % n, "Axis %d is tripped" % n)
        if n not in self.driven_nodes():
            if n in self._arming_nodes():
                return Why("軸%d は通電の手続き中です（基板の返事を確かめています）" % n,
                           "Axis %d is arming (waiting for the board's reply)" % n)
            return Why("軸%d は通電していません（通電してから流してください）" % n, "Axis %d is not armed (arm it first)" % n)
        if not any_mode and self.ax[n].mode != "kt_test":
            return Why("軸%d はトルク定数のモードではありません（「トルク定数」を押してから）" % n,
                       "Axis %d is not in Kt test mode (press \"Kt test\" first)" % n)
        if self._kt(n, None) is None or self._t_max(n, None) is None:
            return Why("軸%d の Kt・電流の上限が基板から読めていません（塊 13 の読み返しがまだ来ていない。"
                       "既定の Kt %.2f では送りません）" % (n, KT_NM_PER_A),
                       "Axis %d: Kt / current limit not read from the board yet" % n)
        return None

    @staticmethod
    def _kt_pulse_at(p, now):
        def s(x):
            return 0.5 - 0.5 * math.cos(math.pi * clamp(x, 0.0, 1.0))
        if p["down_t"] is not None:
            x = (now - p["down_t"]) / KT_RAMP_S
            if x >= 1.0:
                return 0.0, "done"
            return p["down_v"] * (1.0 - s(x)), "down"
        e = max(0.0, now - p["t0"])
        if e < KT_RAMP_S:
            return p["v0"] + (p["tau"] - p["v0"]) * s(e / KT_RAMP_S), "up"
        e -= KT_RAMP_S
        if e < p["secs"]:
            return p["tau"], "hold"
        x = (e - p["secs"]) / KT_RAMP_S
        if x >= 1.0:
            return 0.0, "done"
        return p["tau"] * (1.0 - s(x)), "down"

    def _kt_cancel(self, n):
        self.ax[n].kt_pulse = None
        self.ax[n].t_target = 0.0

    def _kt_cmd(self, now, n):
        a = self.ax[n]
        p = a.kt_pulse
        if p is None:
            a.t_target = 0.0
            return 0.0
        v, ph = self._kt_pulse_at(p, now)
        if ph == "done":
            self._kt_cancel(n)
            return 0.0
        tm = self._t_max(n)
        a.t_target = clamp(v, -tm, tm)
        return a.t_target

    def _kt_pulse_view(self, n, now):
        p = self.ax[n].kt_pulse
        if p is None:
            return None
        v, ph = self._kt_pulse_at(p, now)
        kt = self._kt(n)
        if p["down_t"] is not None:
            left = KT_RAMP_S - (now - p["down_t"])
        else:
            left = 2.0 * KT_RAMP_S + p["secs"] - max(0.0, now - p["t0"])
        return dict(on=(ph != "done"), amps=p["amps"], tau=p["tau"], now_tau=v, now_amps=v / kt,
                    left=max(0.0, left), phase=ph, secs=p["secs"])

    def _pt_why(self, n):
        why = self._node_why({"node": n})
        if why is not None:
            return why
        if self._trip_state(n) is True:
            return Why("軸%d は遮断中です" % n, "Axis %d is tripped" % n)
        if n not in self.driven_nodes():
            if n in self._arming_nodes():
                return Why("軸%d は通電の手続き中です（基板の返事を確かめています）" % n,
                           "Axis %d is arming (waiting for the board's reply)" % n)
            return Why("軸%d は通電していません（通電してから始めてください）" % n, "Axis %d is not armed (arm it first)" % n)
        if self.ax[n].mode != "pos_test":
            return Why("軸%d は位置制御テストのモードではありません（「位置制御テスト」を押してから）" % n,
                       "Axis %d is not in Position test mode (press \"Position test\" first)" % n)
        return None

    def _pt_op(self, action, params):
        if "node" not in params:
            return {"ok": False, "err": "node（軸番号）が要ります"}
        n = num_int(params["node"], "軸番号")
        if n not in self.ax:
            return {"ok": False, "err": Why("軸 %r は居ません" % (n,), "Axis %r not found" % (n,))}
        a = self.ax[n]
        if action == "pt_stop":
            if a.pt is None:
                self.note = Why("軸%d は位置制御テストを走らせていません" % n, "Axis %d: position test is not running" % n)
                return {"ok": True, "idle": True}
            self._pt_stop_req(n)
            self.note = Why("軸%d の位置制御テストを止めます（その場で位置保持）" % n,
                            "Axis %d: stopping the position test (holding in place)" % n)
            return {"ok": True}
        why = self._pt_why(n)
        if why is not None:
            self.note = why
            return {"ok": False, "err": why}
        if a.pt is not None:
            return {"ok": False, "err": Why("軸%d は位置制御テストを走らせています（止めてから始めてください）" % n,
                                            "Axis %d: position test is already running (stop it first)" % n)}
        degs, why = parse_pt_degs(params.get("pts"))
        if why is not None:
            self.note = why
            return {"ok": False, "err": why}
        style = params.get("style", "smooth")
        if style not in ("smooth", "step"):
            return {"ok": False, "err": "style は smooth か step"}
        speed = num(params.get("speed", PT_SPEED_DEF), "速さ")
        dwell = num(params.get("dwell", PT_DWELL_DEF), "止まる時間")
        ratio = num(params.get("ratio", 1.0), "減速比")
        if ratio not in (1.0, GEAR_RATIO):
            return {"ok": False, "err": Why("ratio は 1（モーター軸）か %g（出力軸）です（いま %g）" % (GEAR_RATIO, ratio),
                                            "ratio must be 1 (motor shaft) or %g (output shaft) (got %g)" % (GEAR_RATIO, ratio))}
        vmax = math.degrees(self._v_max(n)) / ratio
        if not (0.0 < speed <= vmax + 1e-9):
            why = Why("速さは 0 より大きく、この軸の速度の端 %.0f °/s までにしてください（いま %g °/s）" % (vmax, speed),
                      "Speed must be above 0 and at most this axis's limit %.0f °/s (got %g °/s)" % (vmax, speed))
            self.note = why
            return {"ok": False, "err": why}
        step = (style == "step")
        dmin = PT_STEP_DWELL_MIN if step else 0.0
        if not (dmin <= dwell <= PT_DWELL_MAX):
            why = Why("止まる時間は %g〜%g 秒にしてください（いま %g 秒%s）"
                      % (dmin, PT_DWELL_MAX, dwell, "・段のときは 0 にできません" if step and dwell < dmin else ""),
                      "Dwell must be %g to %g s (got %g s)" % (dmin, PT_DWELL_MAX, dwell))
            self.note = why
            return {"ok": False, "err": why}
        lo, hi = self._p_cmd_bound(n)
        bad = [d for d in degs if not (lo - 1e-9 <= math.radians(d * ratio) <= hi + 1e-9)]
        if bad:
            why = Why("範囲の外の角度があります: %s°（この軸に送れるのは %.1f〜%.1f°）"
                      % ("・".join("%g" % d for d in bad), math.degrees(lo) / ratio, math.degrees(hi) / ratio),
                      "Angle out of range: %s° (this axis accepts %.1f to %.1f°)"
                      % (", ".join("%g" % d for d in bad), math.degrees(lo) / ratio, math.degrees(hi) / ratio))
            self.note = why
            return {"ok": False, "err": why}
        start = a.traj_p if a.traj_p is not None else self._pos_now(n)
        if start is None or not math.isfinite(start):
            why = Why("軸%d のいまの位置が読めないので始められません" % n, "Axis %d: current position unreadable" % n)
            self.note = why
            return {"ok": False, "err": why}
        P = [math.radians(d * ratio) for d in degs]
        V = math.radians(speed * ratio)
        A = math.radians(PT_ACC_DEG * ratio)
        a.pt = dict(P=P, deg=degs, step=step, V=V, A=A, dwell=dwell, k=0, lap=0, stage="approach",
                    seg=self._pt_seg(start, P[0], 0.0, V, A), d_t0=0.0, clk=0.0, t_last=None,
                    p=start, v=0.0, stop_at_end=False, ratio=ratio)
        sh, sh_en = (("出力軸 %g:1・" % ratio, "output shaft %g:1, " % ratio) if ratio != 1.0 else ("", ""))
        self.note = Why("軸%d 位置制御テストを始めます（%s%d 点・%s・%g °/s・止まる %g 秒）。まず最初の点 %g° へ寄ります"
                        % (n, sh, len(P), "段" if step else "なめらか", speed, dwell, degs[0]),
                        "Axis %d: position test started (%s%d points, %s, %g °/s, dwell %g s). Approaching %g° first"
                        % (n, sh_en, len(P), "step" if step else "smooth", speed, dwell, degs[0]))
        return {"ok": True}

    @staticmethod
    def _pt_seg(p0, p1, t0, V, A):
        D = abs(p1 - p0)
        s = 1.0 if p1 >= p0 else -1.0
        if D <= 1e-12 or V <= 0.0 or A <= 0.0:
            return dict(kind="move", p0=p0, p1=p1, s=s, D=0.0, Vp=0.0, Ta=0.0, T=0.0, t0=t0)
        Vp = V
        Ta = math.pi * Vp / (2.0 * A)
        if D < Vp * Ta:
            Vp = math.sqrt(2.0 * A * D / math.pi)
            Ta = math.pi * Vp / (2.0 * A)
        T = 2.0 * Ta + (D - Vp * Ta) / Vp
        return dict(kind="move", p0=p0, p1=p1, s=s, D=D, Vp=Vp, Ta=Ta, T=T, t0=t0)

    @staticmethod
    def _pt_seg_at(g, clk):
        t = clk - g["t0"]
        if g["kind"] == "stop":
            Ta, x0, v0 = g["Ta"], g["x0"], g["v0"]
            if t >= Ta:
                return g["p1"], 0.0, True
            t = max(0.0, t)
            return (x0 + 0.5 * v0 * (t + Ta / math.pi * math.sin(math.pi * t / Ta)),
                    0.5 * v0 * (1.0 + math.cos(math.pi * t / Ta)), False)
        T, Ta, Vp, s = g["T"], g["Ta"], g["Vp"], g["s"]
        if t >= T:
            return g["p1"], 0.0, True
        if t <= 0.0:
            return g["p0"], 0.0, False
        if t < Ta:
            x = 0.5 * Vp * (t - Ta / math.pi * math.sin(math.pi * t / Ta))
            v = 0.5 * Vp * (1.0 - math.cos(math.pi * t / Ta))
        elif t <= T - Ta:
            x = 0.5 * Vp * Ta + Vp * (t - Ta)
            v = Vp
        else:
            u = T - t
            x = g["D"] - 0.5 * Vp * (u - Ta / math.pi * math.sin(math.pi * u / Ta))
            v = 0.5 * Vp * (1.0 - math.cos(math.pi * u / Ta))
        return g["p0"] + s * x, s * v, False

    def _pt_tick(self, now, n):
        pt = self.ax[n].pt
        if pt["t_last"] is None:
            pt["t_last"] = now
        elif now != pt["t_last"]:
            pt["clk"] += clamp(now - pt["t_last"], 0.0, TRAJ_DT_MAX)
            pt["t_last"] = now
        clk = pt["clk"]
        N = len(pt["P"])
        for _i in range(4 * N + 8):
            stg = pt["stage"]
            if stg in ("approach", "move", "stop"):
                g = pt["seg"]
                p, v, done = self._pt_seg_at(g, clk)
                if not done:
                    pt["p"], pt["v"] = p, v
                    return p, v, False
                if stg == "stop" or pt["stop_at_end"]:
                    pt["p"], pt["v"] = p, 0.0
                    return p, 0.0, True
                pt["stage"] = "dwell"
                pt["d_t0"] = g["t0"] + (g["T"] if g["kind"] == "move" else g["Ta"])
                continue
            p = pt["P"][pt["k"]]
            t1 = pt["d_t0"] + pt["dwell"]
            if clk < t1:
                pt["p"], pt["v"] = p, 0.0
                return p, 0.0, False
            k0 = pt["k"]
            pt["k"] = (k0 + 1) % N
            if pt["k"] == 0:
                pt["lap"] += 1
            if pt["step"]:
                pt["d_t0"] = t1
            else:
                pt["seg"] = self._pt_seg(pt["P"][k0], pt["P"][pt["k"]], t1, pt["V"], pt["A"])
                pt["stage"] = "move"
        return pt["p"], pt["v"], False

    def _pt_cmd(self, now, n):
        a = self.ax[n]
        if a.pt is not None:
            p, v, fin = self._pt_tick(now, n)
            if fin:
                self._pt_end(n, p)
                return p, 0.0
            a.traj_p, a.traj_v, a.traj_f, a.traj_t = p, v, p, now
            return p, v
        return self._traj_step(now, n, a.p_target)

    def _pt_stop_req(self, n):
        pt = self.ax[n].pt
        if pt is None or pt["stage"] == "stop" or pt["stop_at_end"]:
            return
        if pt["stage"] in ("approach", "move"):
            g = pt["seg"]
            t = pt["clk"] - g["t0"]
            if 0.0 < t < g["T"] and abs(pt["v"]) > 1e-9:
                if t > g["T"] - g["Ta"]:
                    pt["stop_at_end"] = True
                    return
                Ta = math.pi * abs(pt["v"]) / (2.0 * pt["A"])
                x0, v0 = pt["p"], pt["v"]
                pt["seg"] = dict(kind="stop", x0=x0, v0=v0, Ta=Ta, t0=pt["clk"], p1=x0 + 0.5 * v0 * Ta)
                pt["stage"] = "stop"
                return
        self._pt_end(n, pt["p"])

    def _pt_end(self, n, pos):
        a = self.ax[n]
        a.pt = None
        if a.mode != "pos_test" or pos is None or not math.isfinite(pos):
            return
        self._snap_traj(n, pos)
        a.p_target = pos
        a.center = pos

    def _pt_yield(self, action, params):
        if action in PT_KEEP or action in OFFCAL_ACTIONS or (
                action == "set" and "p" not in params and "center" not in params):
            return None
        run = [n for n in self.order if self.ax[n].pt is not None]
        if not run:
            return None
        if "node" in params:
            try:
                one = num_int(params["node"], "軸番号")
                run = [n for n in run if n == one]
            except BadValue:
                pass
        if not run:
            return None
        for n in run:
            self._pt_end(n, self.ax[n].pt["p"])
        if action in ("disarm", "estop", "clear"):
            return None
        why = Why("軸%s の位置制御テストを止めました（%s が来た）" % ("・".join(str(n) for n in run), action),
                  "Position test stopped on axis %s (%s received)" % (", ".join(str(n) for n in run), action))
        self._op_jt_why = why if self._op_jt_why is None else wjoin([self._op_jt_why, why])
        self.note = self._op_jt_why
        return why

    def _pt_lost(self, since=None):
        drive = self.driven_nodes()
        lost = [n for n in self.order
                if self.ax[n].pt is not None and (self.ax[n].mode != "pos_test" or n not in drive)]
        if not lost:
            return False
        for n in lost:
            self.ax[n].pt = None
        if since is None or self.note_seq == since:
            self.note = Why("軸%s の位置制御テストをやめました（通電・モードが外れた）" % "・".join(str(n) for n in lost),
                            "Position test ended on axis %s (power or mode was lost)" % ", ".join(str(n) for n in lost))
        return True

    def _pt_view(self, n, now):
        a = self.ax[n]
        pt = a.pt
        if pt is None or a.mode != "pos_test" or n not in self.driven_nodes():
            return None
        stg = "stop" if (pt["stage"] == "stop" or pt["stop_at_end"]) else pt["stage"]
        left = max(0.0, pt["d_t0"] + pt["dwell"] - pt["clk"]) if stg == "dwell" else None
        return dict(on=True, stage=stg, k=pt["k"] + 1, n=len(pt["P"]), deg=pt["deg"][pt["k"]],
                    left=left, step=pt["step"], lap=pt["lap"], ratio=pt.get("ratio", 1.0))

    def _command(self, now, n):
        a = self.ax[n]
        m = a.mode
        if a.kt_pulse is not None and (m != "kt_test" or n not in self.driven_nodes()):
            self._kt_cancel(n)
        if m != "hold" and m != "pos_test" and m not in LEG_AUTO_MODES:
            a.traj_p = None
        if m in LEG_AUTO_MODES:
            return self._leg_track_cmd(now, n)
        if m == "pos_test":
            p, v = self._pt_cmd(now, n)
            lo, hi = self._p_cmd_bound(n)
            return (clamp(p, lo, hi), v, a.kp, a.kd, self._grav_ff(n))
        if m == "free":
            return (0.0, 0.0, 0.0, 0.0, 0.0)
        if m == "homing":
            return self._homing_cmd(now, n)
        if m == "damp":
            return (0.0, 0.0, 0.0, a.kd, 0.0)
        if m in ("hold", "sine"):
            if m == "hold":
                if self.jt is not None and n in self.jt["nodes"]:
                    r = self._jt_step(now, n)
                    if r is not None:
                        lo, hi = self._p_cmd_bound(n)
                        return (clamp(r[0], lo, hi), r[1], r[3], r[4], self._grav_ff(n) if r[2] is None else r[2])
                self._stop_check(now, n)
                p, v = self._traj_step(now, n, a.p_target)
            else:
                w = 2.0 * math.pi * a.freq
                t = now - a.phase_t0
                p = a.center + a.amp * math.sin(w * t)
                v = a.amp * w * math.cos(w * t)
            lo, hi = self._p_cmd_bound(n)
            return (clamp(p, lo, hi), v, a.kp, a.kd, self._grav_ff(n))
        if m == "vel":
            return (0.0, a.v_target, 0.0, a.kd, 0.0)
        if m in TORQUE_MODES:
            return (0.0, 0.0, 0.0, 0.0, self._kt_cmd(now, n) if m == "kt_test" else a.t_target)
        return (0.0, 0.0, 0.0, 0.0, 0.0)

    def _jt_cmd(self, action, params):
        now = time.time()
        jt = self.jt
        if action == "jtraj_gain":
            if jt is None:
                return {"ok": False, "err": "軌道を再生していません"}
            g, err = self._jt_gains(params, len(jt["nodes"]))
            if err:
                return {"ok": False, "err": err}
            self._jt_apply_gains(jt["nodes"], g)
            return {"ok": True}
        if action == "jtraj_speed":
            if jt is None:
                return {"ok": False, "err": "軌道を再生していません"}
            sp = clamp(num(params.get("speed"), "速さ"), JT_SPEED_MIN, JT_SPEED_MAX)
            if jt["vpeak"] * sp / jt["T"] > JT_V_LIMIT:
                return {"ok": False, "err": "速さ %.2f では点列が %.0f rad/s を超えます" % (sp, JT_V_LIMIT)}
            jt["speed_goal"] = sp
            return {"ok": True}
        if action == "jtraj_stop":
            if jt is None:
                return {"ok": True, "idle": True}
            if jt.get("inc") is not None and jt["stage"] == "run" and jt.get("ramp"):
                to = params.get("to")
                if to is not None:
                    if not isinstance(to, (list, tuple)) or len(to) != len(jt["nodes"]):
                        return {"ok": False, "err": "to は軸の数だけの位置の並び"}
                    jt["soft_to"] = dict((n, num(v, "位置") - jt["shift"][k]) for k, (n, v) in enumerate(zip(jt["nodes"], to)))
                jt["stopping"] = True
                return {"ok": True}
            if jt.get("inc") is not None and jt["stage"] == "run":
                jt["stop_at_wrap"] = True
                return {"ok": True}
            to = params.get("to")
            stop_to = None
            if to is not None:
                if not isinstance(to, (list, tuple)) or len(to) != len(jt["nodes"]):
                    return {"ok": False, "err": "to は軸の数だけの位置の並び"}
                stop_to = {}
                for n, v in zip(jt["nodes"], to):
                    lo, hi = self._p_cmd_bound(n)
                    v = num(v, "位置")
                    if not (lo <= v <= hi):
                        return {"ok": False, "err": "軸%d の止める位置 %.3f が送れる端の外" % (n, v)}
                    stop_to[n] = v
            jt["stop_to"] = stop_to
            jt["stopping"] = True
            jt["speed_goal"] = 0.0
            if jt["stage"] != "run":
                self._jt_end(Why("軌道の再生を止めました（寄っている途中）", "Trajectory playback stopped (while approaching the start)"),
                             to=stop_to)
            return {"ok": True}
        nodes = params.get("nodes")
        if not isinstance(nodes, (list, tuple)) or not (1 <= len(nodes) <= JT_NODES_MAX):
            return {"ok": False, "err": "nodes は軸番号の並び（1〜%d 本）" % JT_NODES_MAX}
        nodes = [num_int(x, "軸番号") for x in nodes]
        if len(set(nodes)) != len(nodes) or any(n not in self.ax for n in nodes):
            return {"ok": False, "err": "nodes に居ない軸か重なりがある"}
        T = num(params.get("period"), "周期")
        t_max = JT_ONCE_T_MAX if params.get("once") is True else JT_T_MAX
        if not (JT_T_MIN <= T <= t_max):
            return {"ok": False, "err": "周期は %.1f〜%.0f s" % (JT_T_MIN, t_max)}
        pts = params.get("points")
        if not isinstance(pts, (list, tuple)) or not (JT_N_MIN <= len(pts) <= JT_N_MAX):
            return {"ok": False, "err": "points は %d〜%d 点" % (JT_N_MIN, JT_N_MAX)}
        same = params.get("same") is True
        P = []
        for row in pts:
            if not isinstance(row, (list, tuple)) or len(row) != (1 if same else len(nodes)):
                return {"ok": False, "err": "points の 1 点は%s" % ("位置 1 つ（same）" if same else "軸の数だけの位置")}
            r = [num(v, "位置") for v in row]
            if not all(math.isfinite(v) for v in r):
                return {"ok": False, "err": "points に数でない値"}
            P.append(r * len(nodes) if same else r)
        sp = clamp(num(params.get("speed", 1.0), "速さ"), JT_SPEED_MIN, JT_SPEED_MAX)
        once = params.get("once") is True
        inc = params.get("inc")
        if inc is not None:
            if once:
                return {"ok": False, "err": "once と inc は一緒に使えません"}
            if not isinstance(inc, (list, tuple)) or len(inc) != (1 if same else len(nodes)):
                return {"ok": False, "err": "inc は軸の数だけ（same なら 1 つ）"}
            inc = [num(v, "inc") for v in inc]
            if not all(math.isfinite(v) and abs(v) <= 2.0 * math.pi for v in inc):
                return {"ok": False, "err": "inc は ±2π rad まで"}
            if same:
                inc = inc * len(nodes)
        delays = params.get("delays")
        if delays is not None:
            if inc is None or not isinstance(delays, (list, tuple)) or len(delays) != len(nodes):
                return {"ok": False, "err": "delays は inc と一緒に・軸の数だけ"}
            delays = [num(v, "delays") for v in delays]
            if not all(0.0 <= v <= 60.0 for v in delays):
                return {"ok": False, "err": "delays は 0〜60 s"}
        ramp = 0.0 if params.get("ramp") is None else num(params.get("ramp"), "ramp")
        if ramp and (inc is None or not 0.0 < ramp <= 5.0):
            return {"ok": False, "err": "ramp は inc と一緒に・0〜5 s"}
        marks = params.get("marks")
        if marks is not None:
            if inc is None or not isinstance(marks, (list, tuple)) or len(marks) > JT_N_MAX:
                return {"ok": False, "err": "marks は inc と一緒に・位相の並び"}
            marks = sorted(num(v, "marks") for v in marks)
            if not all(0.0 < v < 1.0 for v in marks):
                return {"ok": False, "err": "marks は 0 と 1 の間"}
        if not self.armed:
            return {"ok": False, "err": "通電していません"}
        drive = self.driven_nodes()
        N = len(P)
        vpeak = 0.0
        for k, n in enumerate(nodes):
            a = self.ax[n]
            if n not in drive:
                return {"ok": False, "err": Why("軸%d は通電していません" % n, "Axis %d is not armed" % n)}
            if a.mode != "hold":
                return {"ok": False, "err": "軸%d が位置保持ではない（%s）" % (n, a.mode)}
            if n in self.homing:
                return {"ok": False, "err": "軸%d は原点出しの最中" % n}
            lo, hi = self._p_cmd_bound(n)
            for i in range(N):
                v = P[i][k]
                if not (lo <= v <= hi):
                    return {"ok": False, "err": "軸%d の %d 点目 %.3f rad が送れる端の外" % (n, i, v)}
                if once and i == N - 1:
                    break
                d = abs((P[0][k] + inc[k] if (inc is not None and i == N - 1) else P[(i + 1) % N][k]) - v)
                if d > JT_STEP_MAX:
                    return {"ok": False, "err": "軸%d の %d 点目の次の点まで %.2f rad（%.1f rad を超える）" % (n, i, d, JT_STEP_MAX)}
                vpeak = max(vpeak, d * ((N - 1) if once else N))
        if vpeak * sp / T > JT_V_LIMIT:
            return {"ok": False, "err": "点列の速さ %.0f rad/s（速さ %.2f）が上限 %.0f を超える" % (vpeak * sp / T, sp, JT_V_LIMIT)}
        name = str(params.get("name", ""))[:40]
        gains, err = self._jt_gains(params, len(nodes))
        if err:
            return {"ok": False, "err": err}
        ff = {}
        for key in ("tg", "ti"):
            v = params.get(key)
            if v is None:
                continue
            if not isinstance(v, (list, tuple)) or len(v) != N:
                return {"ok": False, "err": "%s は点の数だけ" % key}
            rows = []
            for row in v:
                if not isinstance(row, (list, tuple)) or len(row) != len(nodes):
                    return {"ok": False, "err": "%s の 1 点は軸の数だけ" % key}
                r = [num(x, key) for x in row]
                if any(abs(x) > JT_TFF_MAX for x in r):
                    return {"ok": False, "err": "%s は ±%g N·m まで" % (key, JT_TFF_MAX)}
                rows.append(r)
            ff[key] = rows
        sw = params.get("sw")
        if sw is not None:
            if not isinstance(sw, (list, tuple)) or len(sw) != N:
                return {"ok": False, "err": "sw は点の数だけ"}
            sw = [num(x, "sw") for x in sw]
            if any(not (0.0 <= x <= 1.0) for x in sw):
                return {"ok": False, "err": "sw は 0〜1"}
        if (jt is not None and jt["nodes"] == nodes and jt["stage"] == "run" and not jt["stopping"]
                and not once and not jt.get("once") and inc is None and not jt.get("inc")):
            jt["old"] = self._jt_now_traj(jt, now)
            jt["blend_t0"] = now
            jt["pts"], jt["T"], jt["vpeak"], jt["speed_goal"], jt["name"] = P, T, vpeak, sp, name
            jt["tg"], jt["ti"], jt["sw"] = ff.get("tg"), ff.get("ti"), sw
            for k, n in enumerate(nodes):
                a = self.ax[n]
                pT, dpT = self._jt_eval(P, jt["phase"], k)
                vT = dpT * jt["speed"] / T
                if a.traj_p is not None:
                    jt["err"][n] = [a.traj_p - pT, (a.traj_v or 0.0) - vT, now]
            self._jt_apply_gains(nodes, gains)
            return {"ok": True, "blend": True, "vpeak": vpeak / T}
        if jt is not None:
            self._jt_end("軌道を置き換えます")
        self.jt = None
        self.jt = dict(nodes=nodes, pts=P, T=T, vpeak=vpeak, phase=0.0, speed=0.0, speed_goal=sp, stage="approach",
                       t_adv=None, old=None, blend_t0=None, stop_to=None, stopping=False, name=name, t_start=now,
                       gain_prev=dict((n, (self.ax[n].kp, self.ax[n].kd)) for n in nodes),
                       tg=ff.get("tg"), ti=ff.get("ti"), sw=sw, kp_sw=None, kd_sw=None, tff_k=1.0,
                       err=dict((n, [0.0, 0.0, None]) for n in nodes), dt=0.0, once=once,
                       inc=inc, cycles=0, stop_at_wrap=False, shift=[0.0] * len(nodes), wraps=0, marks=marks or [],
                       t_run=0.0, delays=delays or [0.0] * len(nodes), ck=[0] * len(nodes), stop_tau=None,
                       ramp=ramp, rate=[0.0] * len(nodes), etime=[0.0] * len(nodes), soft_to=None)
        self._jt_apply_gains(nodes, gains)
        for k, n in enumerate(nodes):
            a = self.ax[n]
            a.glide_v = JT_APPROACH_V
            a.p_target = P[0][k]
            a.center = P[0][k]
        self._jt_glide_sync(nodes, dict((n, P[0][k]) for k, n in enumerate(nodes)))
        self.note = Why("軌道の始点へ寄っています（%s）" % (name or "jtraj"),
                        "Approaching the trajectory start (%s)" % (name or "jtraj"))
        return {"ok": True, "vpeak": vpeak / T}

    def _jt_now_traj(self, jt, now):
        if jt["old"] is None:
            return dict(pts=jt["pts"], tg=jt["tg"], ti=jt["ti"], sw=jt["sw"])
        w = self._jt_w(jt, now)[0]
        o = jt["old"]
        N = len(jt["pts"])
        nn = len(jt["nodes"])
        pts = [[(1.0 - w) * self._jt_eval(o["pts"], i / N, k)[0] + w * jt["pts"][i][k] for k in range(nn)] for i in range(N)]

        def mix2(key):
            new, old = jt.get(key), o.get(key)
            if new is None and old is None:
                return None
            return [[(1.0 - w) * (self._jt_lin(old, i / N, k) if old is not None else 0.0) +
                     w * (new[i][k] if new is not None else 0.0) for k in range(nn)] for i in range(N)]
        sw_new, sw_old = jt.get("sw"), o.get("sw")
        sw = None if (sw_new is None and sw_old is None) else [
            (1.0 - w) * (self._jt_lin(sw_old, i / N) if sw_old is not None else 0.0) + w * (sw_new[i] if sw_new is not None else 0.0)
            for i in range(N)]
        return dict(pts=pts, tg=mix2("tg"), ti=mix2("ti"), sw=sw)

    @staticmethod
    def _jt_gains(params, nn):
        out = {}
        if params.get("tff_k") is not None:
            k = num(params.get("tff_k"), "tff_k")
            if not (0.0 <= k <= JT_TFF_K_MAX):
                return None, "tff_k は 0〜%g" % JT_TFF_K_MAX
            out["tff_k"] = k
        for key, hi in (("kp", UI_KP_MAX), ("kd", UI_KD_MAX), ("kp_sw", UI_KP_MAX), ("kd_sw", UI_KD_MAX)):
            v = params.get(key)
            if v is None:
                continue
            if not isinstance(v, (list, tuple)) or len(v) != nn:
                return None, "%s は軸の数だけの並び" % key
            vv = [num(x, key) for x in v]
            if any(not (0.0 <= x <= hi) for x in vv):
                return None, "%s は 0〜%g" % (key, hi)
            out[key] = vv
        return (out or None), None

    def _jt_apply_gains(self, nodes, g):
        if not g:
            return
        jt = self.jt
        if jt is not None:
            for key in ("kp_sw", "kd_sw", "tff_k"):
                if key in g:
                    jt[key] = g[key]
        for k, n in enumerate(nodes):
            if "kp" in g:
                self.ax[n].kp = min(g["kp"][k], self._kp_max(n))
            if "kd" in g:
                self.ax[n].kd = g["kd"][k]

    @staticmethod
    def _jt_lin(A, phase, k=None):
        N = len(A)
        x = (phase % 1.0) * N
        i = int(x) % N
        j = (i + 1) % N
        f = x - math.floor(x)
        a, b = (A[i], A[j]) if k is None else (A[i][k], A[j][k])
        return a + (b - a) * f

    def _jt_extra(self, jt, now, k):
        def one(src):
            tg, ti, sw = src.get("tg"), src.get("ti"), src.get("sw")
            t = None
            if tg is not None or ti is not None:
                t = (self._jt_lin(tg, jt["phase"], k) if tg is not None else 0.0) + \
                    (jt["speed"] ** 2) * (self._jt_lin(ti, jt["phase"], k) if ti is not None else 0.0)
            return t, (self._jt_lin(sw, jt["phase"]) if sw is not None else 0.0)
        t, w_sw = one(jt)
        if jt["old"] is not None:
            w = self._jt_w(jt, now)[0]
            to, swo = one(jt["old"])
            if t is not None or to is not None:
                t = (to or 0.0) + ((t or 0.0) - (to or 0.0)) * w
            w_sw = swo + (w_sw - swo) * w
        return t, w_sw

    @staticmethod
    def _jt_eval(P, phase, k, once=False):
        N = len(P)
        if once:
            x = clamp(phase, 0.0, 1.0) * (N - 1)
            i = min(int(x), N - 2)
            d = P[i + 1][k] - P[i][k]
            return P[i][k] + d * (x - i), d * (N - 1)
        x = (phase % 1.0) * N
        i = int(x) % N
        j = (i + 1) % N
        f = x - math.floor(x)
        d = P[j][k] - P[i][k]
        return P[i][k] + d * f, d * N

    def _jt_glide_sync(self, nodes, tgt):
        d = {}
        for n in nodes:
            a = self.ax[n]
            here = a.traj_p if a.traj_p is not None else self._pos_now(n)
            d[n] = 0.0 if here is None else abs(tgt[n] - here)
        dm = max(d.values()) if d else 0.0
        for n in nodes:
            self.ax[n].glide_v = JT_APPROACH_V if dm <= 1e-9 else max(JT_GLIDE_MIN, JT_APPROACH_V * d[n] / dm)

    def _jt_yield(self, action, params):
        if self.jt is not None and action not in JT_KEEP and action not in OFFCAL_ACTIONS and not (
                action == "set" and "p" not in params and "center" not in params):
            why = Why("軌道の再生を止めました（%s が来た）" % action, "Trajectory playback stopped (%s received)" % action)
            self._jt_end(why)
            self._op_jt_why = why
            return why
        return None

    @contextlib.contextmanager
    def _op_lock(self):
        with self.lock:
            self._op_jt_why = None
            try:
                yield
            finally:
                why, self._op_jt_why = self._op_jt_why, None
                if why is not None and why not in (self.note or ""):
                    self.note = wjoin([why, Why(self.note, self.note_en)]) if self.note else why

    def _jt_end(self, why, to=None):
        jt = self.jt
        if jt is None:
            return
        self.jt = None
        for n, (kp, kd) in (jt.get("gain_prev") or {}).items():
            self.ax[n].kp, self.ax[n].kd = kp, kd
        held = [n for n in jt["nodes"] if self.ax[n].traj_p is not None and self.ax[n].mode == "hold"]
        for n in held:
            a = self.ax[n]
            self._snap_traj(n, a.traj_p)
            tgt = a.traj_p if not to or n not in to else to[n]
            a.p_target = tgt
            a.center = tgt
            a.glide_v = JT_APPROACH_V
        if to:
            self._jt_glide_sync(held, dict((n, self.ax[n].p_target) for n in held))
        self.note = why

    def _jt_lost(self, since=None):
        jt = self.jt
        if jt is None:
            return False
        drive = self.driven_nodes()
        if all(n in drive and self.ax[n].mode == "hold" for n in jt["nodes"]):
            return False
        why = Why("軌道の再生をやめました（通電・位置保持が外れた）", "Trajectory playback ended (power or position hold was lost)")
        if since is not None and self.note_seq != since and self.note and why not in self.note:
            why = wjoin([Why(self.note, self.note_en), why])
        self._jt_end(why)
        return True

    def _jt_advance(self, now):
        jt = self.jt
        if jt["t_adv"] == now:
            return
        dt = 0.0 if jt["t_adv"] is None else clamp(now - jt["t_adv"], 0.0, TRAJ_DT_MAX)
        jt["t_adv"] = now
        jt["dt"] = dt
        if self._jt_lost():
            return
        if jt["stage"] == "approach":
            if all(self.ax[n].traj_p is not None and abs(self.ax[n].traj_p - jt["pts"][0][k]) <= JT_ARRIVE_RAD
                   and self.ax[n].traj_v == 0.0 for k, n in enumerate(jt["nodes"])):
                jt["stage"] = "run"
                jt["phase"] = 0.0
                jt["speed"] = jt["speed_goal"] if (jt.get("once") or jt.get("inc")) else 0.0
                self.note = Why("軌道を再生しています（%s）" % (jt["name"] or "jtraj"),
                                "Playing trajectory (%s)" % (jt["name"] or "jtraj"))
            return
        sp = jt["speed"]
        sp += clamp(jt["speed_goal"] - sp, -JT_SPEED_RATE * dt, JT_SPEED_RATE * dt)
        jt["speed"] = sp
        if jt.get("once") and jt["phase"] + dt * sp / jt["T"] >= 1.0:
            self._jt_end(Why("再生し終えました（%s）" % (jt["name"] or "jtraj"), "Finished (%s)" % (jt["name"] or "jtraj")),
                         to=dict((n, jt["pts"][-1][k]) for k, n in enumerate(jt["nodes"])))
            return
        if jt.get("inc") is not None:
            self._seq_advance(jt, dt * sp)
            return
        jt["phase"] = (jt["phase"] + dt * sp / jt["T"]) % 1.0
        if jt["old"] is not None and now - jt["blend_t0"] >= JT_BLEND_S:
            jt["old"] = None
        if jt["stopping"] and sp <= 0.0 and all(abs(e[0]) < 1e-3 and abs(e[1]) < 1e-2 for e in jt["err"].values()):
            self._jt_end(Why("軌道の再生を止めました", "Trajectory playback stopped"), to=jt["stop_to"])

    def _seq_pos(self, jt, k, tau):
        P = jt["pts"]
        N = len(P)
        T = jt["T"]
        moving = True
        if jt["stop_tau"] is not None and tau >= jt["stop_tau"]:
            tau = jt["stop_tau"]
            moving = False
        if tau <= 0.0:
            return P[0][k] - jt["shift"][k], 0.0
        c = math.floor(tau / T)
        x = (tau / T - c) * N
        i = min(int(x), N - 1)
        a = P[i][k]
        b = P[0][k] + jt["inc"][k] if i == N - 1 else P[i + 1][k]
        return a + (b - a) * (x - i) + jt["inc"][k] * c - jt["shift"][k], ((b - a) * N if moving else 0.0)

    def _seq_advance(self, jt, d):
        T = jt["T"]
        jt["t_run"] += d
        t = jt["t_run"]
        dl = jt["delays"]
        if jt.get("ramp"):
            R = jt["ramp"]
            for k in range(len(jt["nodes"])):
                if jt["stopping"]:
                    r = max(0.0, jt["rate"][k] - d / R)
                    jt["etime"][k] += r * d
                else:
                    u = t - dl[k]
                    r = 0.0 if u <= 0.0 else min(1.0, u / R)
                    jt["etime"][k] = 0.0 if u <= 0.0 else (u * u / (2.0 * R) if u < R else u - R / 2.0)
                jt["rate"][k] = r
            k0 = min(range(len(dl)), key=lambda k: dl[k])
            jt["phase"] = (jt["etime"][k0] / T) % 1.0
            jt["cycles"] = int(jt["etime"][k0] // T)
            if jt["stopping"] and all(r == 0.0 for r in jt["rate"]):
                end = jt["soft_to"] or dict((n, self._seq_pos(jt, k, jt["etime"][k])[0]) for k, n in enumerate(jt["nodes"]))
                self._jt_end(Why("止めました（%s）" % (jt["name"] or "jtraj"), "Stopped (%s)" % (jt["name"] or "jtraj")), to=end)
            return
        wrap = []
        for k in range(len(jt["nodes"])):
            tau = t - dl[k]
            if jt["stop_tau"] is not None:
                tau = min(tau, jt["stop_tau"])
            c = math.floor(tau / T) if tau > 0 else 0
            if c != jt["ck"][k]:
                jt["ck"][k] = c
                if abs(jt["pts"][0][k] + jt["inc"][k] * c - jt["shift"][k]) >= JT_WRAP_RAD:
                    wrap.append(k)
        if wrap:
            self._jt_wrap(jt, wrap)
        k0 = min(range(len(dl)), key=lambda k: dl[k])
        tau0 = t - dl[k0]
        if jt["stop_at_wrap"] and jt["stop_tau"] is None:
            if tau0 <= 0.0:
                jt["stop_tau"] = 0.0
            else:
                c = math.floor(tau0 / T)
                cand = [(c + m) * T for m in list(jt["marks"]) + [1.0]] + [(c + 1 + m) * T for m in list(jt["marks"])]
                jt["stop_tau"] = min(x for x in cand if x >= tau0 - 1e-9)
        jt["phase"] = (tau0 / T) % 1.0 if tau0 > 0 else 0.0
        jt["cycles"] = jt["ck"][k0]
        if jt["stop_tau"] is not None and all(t - x >= jt["stop_tau"] for x in dl):
            end = dict((n, self._seq_pos(jt, k, jt["stop_tau"])[0]) for k, n in enumerate(jt["nodes"]))
            self._jt_end(Why("止めました（%s）" % (jt["name"] or "jtraj"), "Stopped (%s)" % (jt["name"] or "jtraj")), to=end)

    def _jt_wrap(self, jt, ks=None):
        ks = list(range(len(jt["nodes"]))) if ks is None else ks
        meas = dict((k, self._pos_now(jt["nodes"][k])) for k in ks)
        if any(v is None for v in meas.values()):
            return
        now = time.time()
        for k in ks:
            n = jt["nodes"][k]
            m = meas[k]
            self._send_special(0xFE, n)
            jt["shift"][k] += m
            a = self.ax[n]
            if a.traj_p is not None:
                a.traj_p -= m
            a.traj_f -= m
            a.p_target -= m
            a.center -= m
            a.stop_from = None
            self.pending.append((now + 0.15, lambda kk=n: self._drop_pos_tlm(kk), True))
        jt["wraps"] += 1

    @staticmethod
    def _jt_w(jt, now):
        if jt["old"] is None:
            return 1.0, 0.0
        u = clamp((now - jt["blend_t0"]) / JT_BLEND_S, 0.0, 1.0)
        return u * u * (3.0 - 2.0 * u), 6.0 * u * (1.0 - u) / JT_BLEND_S

    def _jt_pos(self, jt, now, k):
        if jt.get("inc") is not None:
            if jt.get("ramp"):
                p, dp = self._seq_pos(jt, k, jt["etime"][k])
                return p, dp * jt["rate"][k]
            return self._seq_pos(jt, k, jt["t_run"] - jt["delays"][k])
        return self._jt_eval(jt["pts"], jt["phase"], k, bool(jt.get("once")))

    def _jt_step(self, now, n):
        if self.jt is None:
            return None
        self._jt_advance(now)
        jt = self.jt
        if jt is None or jt["stage"] != "run" or n not in jt["nodes"]:
            return None
        k = jt["nodes"].index(n)
        pT, dp = self._jt_pos(jt, now, k)
        vT = dp * jt["speed"] / jt["T"]
        e = jt["err"].setdefault(n, [0.0, 0.0, None])
        if e[2] != now and jt.get("dt", 0.0) > 0.0 and (e[0] != 0.0 or e[1] != 0.0):
            e[2] = now
            dt = jt["dt"]
            wn = JT_FOLLOW_WN
            e[1] += (-wn * wn * e[0] - 2.0 * wn * e[1]) * dt
            e[0] += e[1] * dt
            if abs(e[0]) < 1e-6 and abs(e[1]) < 1e-5:
                e[0] = e[1] = 0.0
        p = pT + e[0]
        v = clamp(vT + e[1], -V_MAX, V_MAX)
        a = self.ax[n]
        a.traj_p = p
        a.traj_v = v
        a.traj_f = p
        a.traj_t = now
        a.p_target = p
        a.center = p
        t, w = self._jt_extra(jt, now, k)
        if t is not None:
            tm = self._t_max(n)
            t = clamp(jt["tff_k"] * t, -tm, tm)
        kp, kd = a.kp, a.kd
        if jt["kp_sw"] is not None:
            kp = min(kp + (jt["kp_sw"][k] - kp) * w, self._kp_max(n))
        if jt["kd_sw"] is not None:
            kd = kd + (jt["kd_sw"][k] - kd) * w
        jt.setdefault("last", {})[n] = (t, kp, kd)
        return p, v, t, kp, kd

    def _jt_snap(self):
        jt = self.jt
        if jt is None:
            return None
        return dict(stage=("stopping" if jt["stopping"] else jt["stage"]), phase=round(jt["phase"], 4),
                    speed=round(jt["speed"], 3), speed_goal=jt["speed_goal"], period=jt["T"], n=len(jt["pts"]),
                    nodes=list(jt["nodes"]), name=jt["name"], blending=jt["old"] is not None,
                    vpeak=round(jt["vpeak"] / jt["T"], 2),
                    kp=[self.ax[n].kp for n in jt["nodes"]], kd=[self.ax[n].kd for n in jt["nodes"]],
                    kp_sw=jt["kp_sw"], kd_sw=jt["kd_sw"], tff_k=jt["tff_k"], ff=jt["tg"] is not None or jt["ti"] is not None,
                    sw=jt["sw"] is not None, once=bool(jt.get("once")), cycles=jt.get("cycles", 0), wraps=jt.get("wraps", 0),
                    ramp=jt.get("ramp") or 0.0,
                    stop_at_wrap=bool(jt.get("stop_at_wrap")),
                    now=dict((str(n), [None if x is None else round(x, 4) for x in v]) for n, v in (jt.get("last") or {}).items()))

    def _reboot_tick(self):
        for n in self.order:
            mark = self.bus.axes[n].take_reboot()
            if mark is None:
                continue
            was_s, now_s = mark
            self.bus.axes[n].set_tripped(None)
            self._held.pop(n, None)
            self._offcal_stop(n, Why(REBOOT_WHY, REBOOT_WHY_EN))
            self._abort_homing(n, REBOOT_WHY)
            self._leg_cal(n).invalidate()
            if n in self.arm_nodes:
                self.ax[n].mode = "free"
                self.note = Why("軸%d の基板が再起動しました（起動からの秒 "
                                "%d → %d）。脱力にしました。目標を作り直す"
                                "のでモードを入れ直してください"
                                % (n, was_s, now_s),
                                "Axis %d board restarted (uptime %d → %d s). Set to free; re-enter the mode "
                                "(the targets are rebuilt)" % (n, was_s, now_s))
            else:
                self.note = Why("軸%d の基板が再起動しました（起動からの秒 "
                                "%d → %d）" % (n, was_s, now_s),
                                "Axis %d board restarted (uptime %d → %d s)" % (n, was_s, now_s))

    def _reply_gap_tick(self, mono, drive):
        for n in list(self._fb_mono):
            if n not in drive:
                self._fb_mono.pop(n, None)
        gap = []
        for n in drive:
            t = self._fb_mono.get(n)
            if t is None:
                self._fb_mono[n] = mono
                continue
            if self.ax[n].mode == "free":
                continue
            if mono - t > FOC_TIMEOUT_MS / 1000.0:
                gap.append((n, mono - t))
        held = [(n, g) for n, g in gap if self.ax[n].mode in LOST_HOLD_MODES]
        gap = [(n, g) for n, g in gap if self.ax[n].mode not in LOST_HOLD_MODES]
        if held:
            g = max(g for _n, g in held)
            self._let_board_hold([n for n, _g in held],
                                 "軸%s の基板から返事が %.2f s ありません" % ("・".join(str(n) for n, _g in held), g),
                                 "No reply from axis %s for %.2f s" % (", ".join(str(n) for n, _g in held), g))
        for n, g in gap:
            self._abort_homing(n, "基板から返事が %.2f s 無い" % g)
            self.ax[n].mode = "free"
        if gap:
            self.note = ("軸%s の基板から返事が %.2f s ありません（CAN が途切れた。基板は FOC_TIMEOUT %d ms のあと、"
                         "最後の指令のまま目標速度 0 で持ってから、ゆっくり力を抜いて止まる・§7-63）。"
                         "つながり直しても勝手に動き出さないよう脱力にしました。モードを入れ直してください"
                         % ("・".join(str(n) for n, _g in gap), max(g for _n, g in gap), FOC_TIMEOUT_MS))

    def run(self):
        dt = 1.0 / self.rate
        t0 = time.time()
        i = 0
        prev = t0
        while not self.stopping:
            try:
                now = time.time()

                with self.lock:
                    due = [f for (at, f, _k) in self.pending if at <= now]
                    self.pending = [it for it in self.pending if it[0] > now]
                    note_seq0 = self.note_seq

                for f in due:
                    try:
                        f()
                    except Exception as e:
                        self.note = "操作に失敗: %s" % e

                with self.lock:
                    if self.armed and self.last_poll > 0.0 and \
                            (time.monotonic() - self.last_poll) > WATCHDOG_S:
                        self.armed = False
                        self.arm_nodes = []
                        self.wd_fired = True
                        self.note = Why("画面からの応答が途切れたので停止しました", "Stopped: the page stopped responding")
                        for n in self.order:
                            self.pending.append(
                                (now, lambda k=n: self._send_special(0xFD, k),
                                 True))

                    self._reboot_tick()

                    mono = time.monotonic()
                    drive_now = self.driven_nodes()
                    for n in list(self._mit_mono):
                        if n not in drive_now:
                            self._mit_mono.pop(n, None)
                    gap = [(n, mono - self._mit_mono[n]) for n in drive_now
                           if n in self._mit_mono
                           and (mono - self._mit_mono[n]) > FOC_TIMEOUT_MS / 1000.0]
                    held = [(n, g) for n, g in gap if self.ax[n].mode in LOST_HOLD_MODES]
                    gap = [(n, g) for n, g in gap if self.ax[n].mode not in LOST_HOLD_MODES]
                    for n, g in gap:
                        self._mit_mono.pop(n, None)
                        self._abort_homing(n, "指令が %.2f s 途切れた" % g)
                        self.ax[n].mode = "free"
                    if gap:
                        self.note = ("軸%s への指令が %.2f s 途切れました（基板は FOC_TIMEOUT %d ms のあと、最後の指令の"
                                     "まま目標速度 0 で持ってから、ゆっくり力を抜いて止まる・§7-63）。"
                                     "脱力にしました。目標を作り直すのでモードを入れ直してください"
                                     % ("・".join(str(n) for n, _g in gap),
                                        max(g for _n, g in gap), FOC_TIMEOUT_MS))
                    if held:
                        g = max(g for _n, g in held)
                        self._let_board_hold([n for n, _g in held],
                                             "軸%s への指令が %.2f s 途切れました" % ("・".join(str(n) for n, _g in held), g),
                                             "Commands to axis %s stalled for %.2f s" % (", ".join(str(n) for n, _g in held), g))
                    self._reply_gap_tick(mono, drive_now)

                    if self.armed:
                        bad = self._gate_some(self._pos_mode_nodes())
                        stale = [(n, w) for n, w in bad if not self._fb_fresh(n)]
                        bad = [(n, w) for n, w in bad if self._fb_fresh(n)]
                        if stale:
                            self._let_board_hold([n for n, _w in stale],
                                                 "位置モードを続けられないので MIT の相手から外しました: %s"
                                                 % "／".join("軸%d %s" % (n, w) for n, w in stale),
                                                 "Position mode could not continue; stopped commanding axis %s (no reply)"
                                                 % ", ".join(str(n) for n, _w in stale))
                        for n, w in bad:
                            self.ax[n].mode = "free"
                        if bad:
                            msg = ("位置モードを続けられないので脱力にしました: %s"
                                   % "／".join("軸%d %s" % (n, w)
                                               for n, w in bad))
                            self.note = (wjoin([Why(self.note, self.note_en), msg]) if stale else msg)

                    self._jt_lost(since=note_seq0)
                    self._pt_lost(since=note_seq0)

                    live = self.live_nodes()
                    drive = self.driven_nodes()
                    cmd = dict((n, self._command(now, n)) for n in live)
                    shown = self._command(now, self.node) if self.node in cmd \
                        else (None, None, None, None, None)

                before = dict((n, self.bus.axes[n].fb_count) for n in drive)
                try:
                    for n in drive:
                        a = self.bus.axes[n]
                        a.p, a.v, a.kp, a.kd, a.t = cmd[n]
                        self.bus._send(0x100 + n, self.bus._mit(a))
                        self._mit_mono[n] = time.monotonic()
                        self._gain_tx[n] = (time.time(), "mit")
                except (TypeError, ValueError, OverflowError) as e:
                    with self.lock:
                        self.armed = False
                        self.arm_nodes = []
                        self.note = "指令を作れなかったので停止しました: %s" % e
                    for n in self.order:
                        try:
                            self._send_special(0xFD, n)
                        except OSError:
                            pass

                deadline = t0 + (i + 1) * dt
                tnow = time.time()
                if deadline > tnow + 10 * dt:
                    t0 = tnow
                    i = 0
                    deadline = t0 + dt
                self.bus._drain(deadline, max_wait=11 * dt)
                for n in drive:
                    if self.bus.axes[n].fb_count == before[n]:
                        self.bus.axes[n].fb_missing += 1
                    else:
                        self._fb_mono[n] = time.monotonic()

                end = time.time()
                self.periods.append(end - prev)
                prev = end

                rec = self.rec
                if rec is not None and rec.err is not None:
                    with self.lock:
                        if self.rec is rec:
                            self.rec = None
                            self.rec_last = dict(file=rec.name, rows=rec.written, err=rec.err)
                            self.note = ("記録 %s が書けなくなったので止めました（書けたのは %d 行）: %s"
                                         % (rec.name, rec.written, rec.err))
                    rec = None
                if rec is not None or any(self.ax[k].grav_samp is not None for k in self.order):
                    self._rec_tick(end, cmd, drive, rec)

                if (end - self._last_sample) >= (1.0 / SAMPLE_HZ):
                    self._last_sample = end
                    self.seq += 1
                    for n in self.order:
                        on = n in drive
                        lp, lv, li = self._live(n, on)
                        des = cmd[n][0] if (on and n in cmd) else None
                        self.ax[n].hist.append((self.seq, end, des, lp, lv, li, self._id_now(n)))

                if (end - self._last_temp) >= TEMP_DT:
                    self._last_temp = end
                    if self.bus.iface_changed():
                        try:
                            self.bus.reopen()
                            with self.lock:
                                self.note = "%s が作り直されていたので（USB の CAN を挿し直した）ソケットを開き直しました" % self.bus.iface
                        except Exception as e:
                            with self.lock:
                                self.note = "%s が作り直されたがソケットを開き直せない: %s" % (self.bus.iface, e)
                    for n in self.order:
                        self._temp_tick(n, end)
                    with self.lock:
                        self._start_zero_tick()
                        self._param_save_tick()
                        self._offcal_tick()
                    if self.auto_scan and (end - self._last_scan) >= BOARD_FRESH_S:
                        self._last_scan = end
                        with self.lock:
                            if not self.armed and not self._arming():
                                gone = self._prune_locked()
                                q, self._op_queued = self._op_queued, True
                                try:
                                    added = self._rescan_locked()
                                finally:
                                    self._op_queued = q
                                msg = []
                                if added:
                                    msg.append("軸 %s を見つけたので並べました"
                                               % "・".join(str(n) for n in added))
                                if gone:
                                    msg.append("軸 %s が見つからなくなったので外しました"
                                               % "・".join(str(n) for n in gone))
                                if msg:
                                    self.note = "／".join(msg)

                i += 1
                if end > deadline + 10 * dt:
                    t0 = end
                    i = 0
                if self._stall_n:
                    self._stall_n = 0
                    with self.lock:
                        self.stall = None
                        now_r = time.time()
                        still = [k for k in self.order if self._board_held(k, now_r)]
                        if still:
                            secs = max(0.0, max(self._held[k][0] + self._held[k][1] for k in still if k in self._held) - now_r)
                            self.note = Why("制御ループが立ち直りました。軸%s の基板は途切れた間も最後の目標を同じ硬さで持っていて、"
                                            "ゆっくり力を抜いてから自分で止まります（§7-63・あと最大 約 %.0f 秒）。止まってから通電し直して"
                                            "ください" % ("・".join(str(k) for k in still), secs),
                                            "Control loop recovered. Axis %s: the board is still holding its last target and will "
                                            "release and stop by itself (up to ~%.0f s more). Re-arm after it has stopped"
                                            % (", ".join(str(k) for k in still), secs))
                        else:
                            self.note = ("制御ループが立ち直りました。"
                                         "通電し直せます")

            except Exception as e:
                first = (self._stall_n == 0)
                self._stall_n += 1
                with self.lock:
                    was = self.armed or bool(self.arm_nodes)
                    self.armed = False
                    self.arm_nodes = []
                    self._void_arming()
                    self.stall = ("制御が止まっています: %s。"
                                  "原因が直れば自動でつなぎ直します"
                                  "（いまボタンを押しても基板には届きません）"
                                  % e)
                if first and was:
                    for n in self.order:
                        try:
                            self._send_special(0xFD, n)
                        except OSError:
                            pass
                time.sleep(min(dt * (2 ** min(self._stall_n, 12)),
                               STALL_MAX_S))
                try:
                    self.bus.reopen()
                except Exception:
                    pass
                t0 = time.time()
                i = 0
                prev = t0

    def _rec_tick(self, now, cmd, drive, rec):
        for n in self.order:
            a = self.ax[n]
            on = n in drive
            lp, lv, li = self._live(n, on)
            c = cmd.get(n) if on else None
            cal = self._leg_cal(n)
            synced = cal.valid and n not in self.homing
            tout = cal.motor_to_tout(lp) if (synced and lp is not None) else None
            tin = cal.motor_to_tin(lp) if (synced and lp is not None) else None
            ext = fric = grav_out = None
            if synced and lp is not None and li is not None:
                ext, fric = cal.ext_torque_out(lp, li, self._kt(n))
            if synced and tout is not None and cal.grav.on:
                grav_out = cal.grav.tau_knee(tout)
            gs = a.grav_samp
            if gs is not None:
                why = self._grav_hold_why(n)
                if why is None and (gs["key"] != self._leg_map_key(n) or gs["p_target"] != a.p_target):
                    why = "較正か壁、または目標が変わりました"
                if why is not None:
                    a.grav_samp = None
                    self.note = "軸%d 重力の標本は捨てました（測っている間に崩れた: %s）" % (n, why)
                else:
                    if li is not None and tout is not None:
                        gs["n"] += 1
                        gs["iq_sum"] += li
                        gs["tout_sum"] += tout
                    if now >= gs["t_end"]:
                        a.grav_samp = None
                        tk = None
                        if gs["n"] > 0:
                            tout_m = gs["tout_sum"] / gs["n"]
                            m_m = cal.tout_to_motor(tout_m, lp)
                            tk = cal.knee_from_motor_nm(m_m, self._kt(n) * gs["iq_sum"] / gs["n"]) if m_m is not None else None
                        if tk is not None:
                            threading.Thread(target=self._grav_commit, args=(n, tout_m, tk),
                                             name="krc-grav", daemon=True).start()
                        else:
                            self.note = "軸%d 重力の標本が取れませんでした（保持中の電流が読めない）" % n
            if rec is not None:
                th = self.temps.get(n)
                est = self.wind.get(n)
                b = self._board(n)
                wc = est.celsius() if (est is not None and est.fresh(now)) else None
                rec.put((now - rec.t0, n, a.mode,
                         c[0] if c else None, c[1] if c else None, c[2] if c else None,
                         c[3] if c else None, c[4] if c else None,
                         lp, lv, li, (None if li is None else self._kt(n) * li),
                         tout, tin, ext, fric, grav_out, b.get("temp_c"), wc))

    def _temp_tick(self, n, now):
        if n not in self.temps:
            self.temps[n] = TempHist()
            self.wind[n] = WindingTemp(ref=self.wind_refs.get(n))
        b = self._board(n)
        tc = b.get("temp_c")
        est = self.wind[n]
        if ("iq_ma" in b) and ("vq_mv" in b) and ("rpm" in b):
            key = (self.bus.tlm_t.get((n, 5)), self.bus.tlm_t.get((n, 7)))
            if key != self._temp_seen.get(n):
                self._temp_seen[n] = key
                est.push(b["iq_ma"] / 1000.0, b["rpm"] * 2.0 * math.pi / 60.0,
                         b["vq_mv"] / 1000.0, now, tc)
        wc = est.celsius() if est.fresh(now) else None
        self.temps[n].add(now, tc, wc)

    def _wind_ref_err(self, n):
        errs = self.wind_ref_errs
        return errs.get(n) or (None if n in self.wind_refs else errs.get(None))

    def _wind_ref_fit(self, n, now):
        est = self.wind.get(n)
        if n not in self.ax or est is None:
            return None, Why("軸%d は居ません" % n, "Axis %d is not on the bus" % n)
        return est.ref_fit(now, n)

    def _wcal_keep_later(self, j, base):
        est = self.wind.get(j.node)
        run = None if est is None else est.run
        pts = [] if run is None else run.chain_pts(j.t_req)
        if not pts:
            return

        def bg():
            w = self._wind_run_keep(j.node, base, j.t_req, pts=pts)
            with self._wcal_lock:
                if j.why is base:
                    j.why = w
        threading.Thread(target=bg, daemon=True).start()

    def _wind_run_keep(self, n, why, since=None, pts=None):
        if pts is None:
            est = self.wind.get(n)
            run = None if est is None else est.run
            pts = [] if run is None else run.chain_pts(since)
        if not pts:
            return why
        try:
            nm = wind_run_save(n, dict(t_start=(since if since is not None else pts[0][0]), pts=pts, used=set()))
        except OSError as e:
            return Why("%s（生の点の記録は書けませんでした: %s）" % (why, e),
                       "%s (couldn't write the raw points: %s)" % (en_of(why) or why, e))
        return Why("%s（生の点は %s に残しました）" % (why, nm), "%s (raw points saved to %s)" % (en_of(why) or why, nm))

    def _wind_ref_cmd(self, action, params, since=None):
        n = num_int(params.get("node"), "軸番号")
        log_name, log_err = None, None
        with self._wind_ref_lock:
            refs = dict(self.wind_refs)
            if action == "wind_ref_set":
                fit, why = self._wind_ref_fit(n, time.time())
                if why is not None:
                    return self._wind_ref_say(self._wind_run_keep(n, why, since), ok=False)
                try:
                    log_name = wind_run_save(n, fit)
                except OSError as e:
                    log_err = e
                dv, se = self.wind[n].dv_pair()
                fit_d = dict(n=fit["n"], secs=round(fit["secs"], 2), slope=round(fit["slope"], 6),
                             se=round(fit["se"], 6), se_c=round(fit["se_c"], 2),
                             start=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(fit["t_start"])))
                if log_name is not None:
                    fit_d["log"] = log_name
                raw = dict(r=round(fit["r"], 5), t=round(fit["t"], 2), date=time.strftime("%Y-%m-%d %H:%M"),
                           iq=round(fit["iq"], 3), inv=round(fit["inv"], 6), fit=fit_d)
                if dv is not None:
                    raw.update(dv=round(dv, 5), dv_se=round(se, 5))
                ent, bad = _wind_ref_entry(raw)
                if ent is None:
                    return self._wind_ref_say(Why("軸%d の巻線温度の基準を覚えられません（%s）" % (n, bad),
                                                  "Axis %d: can't store the winding reference (%s)" % (n, bad)), ok=False)
                refs[n] = ent
            else:
                if n not in refs:
                    return self._wind_ref_say(Why("軸%d には巻線温度の基準がありません" % n,
                                                  "Axis %d has no winding reference" % n), ok=False)
                del refs[n]
                ent = None
            try:
                wind_ref_save(refs)
            except OSError as e:
                return self._wind_ref_say(Why("巻線温度の基準を書けませんでした（何も変えていません）: %s" % e,
                                              "Couldn't write the winding reference (nothing changed): %s" % e), ok=False)
            self.wind_refs = refs
            dropped = [w for k, w in sorted(self.wind_ref_errs.items(), key=lambda kv: (kv[0] is not None, kv[0] or 0))
                       if k != n]
            self.wind_ref_errs = {}
            est = self.wind.get(n)
            if est is not None:
                est.set_base(ent)
        if dropped:
            gone = Why("基準ファイルの読めなかった行は、この書き直しで本体から外れました（%s）" % "／".join(dropped),
                       "Unreadable entries of the reference file were dropped from it by this rewrite (%s)"
                       % " / ".join(en_of(w) or w for w in dropped))
        else:
            gone = None
        if ent is not None:
            fd = ent.get("fit", {})
            if log_err is None:
                tail, tail_en = "生の点は %s" % log_name, "raw points in %s" % log_name
            else:
                tail = "生の点の記録は書けませんでした: %s" % log_err
                tail_en = "couldn't write the raw points: %s" % log_err
            msg = Why("軸%d の巻線温度の基準を覚えました（流し始めの R %.3f Ω・%.1f ℃・当てはめ %d 点 %.1f 秒 ±%.1f ℃。%s）"
                      % (n, ent["r"], ent["t"], fd.get("n", 0), fd.get("secs", 0.0), fd.get("se_c", 0.0), tail),
                      "Axis %d: winding reference stored (R at current start %.3f Ω, %.1f °C; fit %d pts, %.1f s, ±%.1f °C; %s)"
                      % (n, ent["r"], ent["t"], fd.get("n", 0), fd.get("secs", 0.0), fd.get("se_c", 0.0), tail_en))
        else:
            msg = Why("軸%d の巻線温度の基準を消しました（次に R が落ち着いたときの基板温度を基準にします）" % n,
                      "Axis %d: winding reference cleared (the next settled R uses the board temperature)" % n)
        return self._wind_ref_say(msg if gone is None else wjoin([msg, gone]))

    def _wind_ref_say(self, msg, ok=True):
        with self.lock:
            self.note = msg
        return {"ok": True, "msg": msg} if ok else {"ok": False, "err": msg}

    def _wcal_why(self, n, now):
        if n not in self.ax:
            return Why("軸 %r は居ません" % (n,), "Axis %r not found" % (n,))
        with self._wcal_lock:
            j = self.wcal.get(n)
            if n in self._wcal_starting or (j is not None and j.stage == "running"):
                return Why("軸%d は巻線温度の校正の最中です" % n, "Axis %d: winding calibration in progress" % n)
        if self.jt is not None:
            return Why("3 軸の軌道を再生しています（始めると再生が止まるので、止めてから始めてください）",
                       "A trajectory is playing (starting would stop it; stop it first)")
        why = self._kt_why(n, any_mode=True)
        if why is not None:
            return why
        if self.ax[n].kt_pulse is not None:
            return Why("軸%d は電流を流している最中です（止めてから始めてください）" % n,
                       "Axis %d is driving current (stop it first)" % n)
        est = self.wind.get(n)
        if est is not None and est.live_run(now) is not None:
            return Why("軸%d に電流が流れています（流し始めから測るので、電流を止めてから始めてください）" % n,
                       "Axis %d has current flowing (it measures from the start of the current; stop it first)" % n)
        return None

    def _wcal_stop(self, nodes, why):
        out = []
        with self._wcal_lock:
            for n in list(nodes):
                j = self.wcal.get(n)
                if j is not None and j.stage == "running" and not j.saving:
                    j.stage = "failed"
                    j.why = Why("中止しました（%s）。何も覚えていません" % why,
                                "Stopped (%s); nothing stored" % (en_of(why) or why))
                    self._wcal_keep_later(j, j.why)
                    out.append(n)
        return out

    def _wcal_cmd(self, action, params):
        n = num_int(params.get("node"), "軸番号")
        if action == "wcal_cancel":
            with self._wcal_lock:
                j = self.wcal.get(n)
                if j is None:
                    return {"ok": True}
                running = j.stage == "running"
                if not running:
                    del self.wcal[n]
            if running and self._wcal_stop([n], Why("画面の「中止」", "Cancel pressed")):
                self.do("tq_stop", {"node": n})
            return {"ok": True}
        amps = num(params.get("amps", WCAL_AMPS_DEF), "amps")
        secs = clamp(num(params.get("secs", WCAL_SECS_DEF), "secs"), WCAL_SECS_MIN, WCAL_SECS_MAX)
        why = None
        if params.get("cold") is not True or params.get("blocked") is not True:
            why = Why("確かめのチェック（室温まで冷えている・腕が当たって回らない）が入っていません",
                      "The checks (cooled to room temperature, arm blocked) are not ticked")
        elif abs(amps) < WCAL_AMPS_MIN:
            why = Why("電流が小さすぎます（%.1f A 以上）" % WCAL_AMPS_MIN, "Current too small (at least %.1f A)" % WCAL_AMPS_MIN)
        with self.lock:
            if why is None:
                why = self._wcal_why(n, time.time())
            if why is not None:
                self.note = why
                return {"ok": False, "err": why}
            prev = self.wind_refs.get(n)
            with self._wcal_lock:
                self._wcal_starting.add(n)
                self.wcal.pop(n, None)
        try:
            if self.ax[n].mode != "kt_test":
                r = self.do("mode", {"node": n, "mode": "kt_test"})
                if not r.get("ok"):
                    return r
            r = self.do("tq_pulse", {"node": n, "amps": amps, "secs": secs})
            if not r.get("ok"):
                return r
            with self.lock:
                p = self.ax[n].kt_pulse
                if p is None or self.ax[n].mode != "kt_test" or n not in self.driven_nodes():
                    self.note = Why("軸%d の巻線温度の校正を始められませんでした（直前に別の操作が来ました）" % n,
                                    "Axis %d: winding calibration not started (another command came in)" % n)
                    return {"ok": False, "err": self.note}
                j = WCal(n, p, prev)
                with self._wcal_lock:
                    self.wcal[n] = j
                self.note = Why("軸%d の巻線温度の校正を始めました（%+.2f A を %.1f 秒。終わる直前に流し始めの抵抗を当てはめて、"
                                "通れば基準として覚えます）" % (n, j.amps, j.secs),
                                "Axis %d: winding calibration started (%+.2f A for %.1f s; the starting resistance is "
                                "fitted just before the end and stored if it passes)" % (n, j.amps, j.secs))
            threading.Thread(target=self._wcal_run, args=(j,), daemon=True).start()
            return {"ok": True, "amps": j.amps, "secs": j.secs}
        finally:
            with self._wcal_lock:
                self._wcal_starting.discard(n)

    def _wcal_run(self, j):
        n = j.node
        while True:
            with self._wcal_lock:
                if j.stage != "running":
                    return
                a = self.ax.get(n)
                if a is None or a.kt_pulse is not j.pulse:
                    j.stage = "failed"
                    j.why = Why("電流が途中で止まりました（解除・非常停止・モードの変更・返事の途切れなど）。何も覚えていません",
                                "The current stopped midway (disarm, E-stop, mode change, lost replies, ...); nothing stored")
                    self._wcal_keep_later(j, j.why)
                    return
                wait = j.t_fit - time.time()
                if wait <= 0.0:
                    j.saving = True
                    break
            time.sleep(min(WCAL_POLL_S, wait))
        est = self.wind.get(n)
        run = None if est is None else est.run
        if run is not None and run.t0 < j.t_req:
            r = {"ok": False, "err": self._wind_run_keep(n, Why(
                "軸%d は流し始める前から電流が流れていました（流し始めの抵抗が取れないので覚えません。"
                "電流を止めてからもう一度）" % n,
                "Axis %d had current flowing before the start (no starting resistance, nothing "
                "stored; stop the current and try again)" % n), j.t_req)}
        else:
            r = self._wind_ref_cmd("wind_ref_set", {"node": n}, since=j.t_req)
        with self._wcal_lock:
            j.saving = False
            if r.get("ok"):
                j.stage, j.res, j.msg = "done", self.wind_refs.get(n), r.get("msg")
            else:
                j.stage, j.why = "failed", r.get("err")

    def _wcal_view(self, n, now):
        ref = self.wind_refs.get(n)
        with self._wcal_lock:
            j = self.wcal.get(n)
            if j is not None:
                v = dict(stage=j.stage, saving=j.saving, amps=j.amps, secs=j.secs, prev=j.prev, res=j.res,
                         msg=j.msg, why=j.why)
        if j is None:
            start_why = self._wcal_why(n, now)
            return dict(stage="idle", ref=ref, start_why=start_why, start_why_en=en_of(start_why))
        v["ref"] = ref
        if v["stage"] == "running":
            pv = self._kt_pulse_view(n, now) if j.pulse is self.ax[n].kt_pulse else None
            v["left"] = None if pv is None else pv["left"]
            v["total"] = 2.0 * KT_RAMP_S + j.secs
            est = self.wind.get(n)
            run = None if est is None else est.run
            v["pts"] = len(run.pts) if (run is not None and run.t0 >= j.t_req) else 0
            fit, fw = self._wind_ref_fit(n, now) if v["pts"] else (None, None)
            v["fit"] = None if fit is None else dict(r=fit["r"], se_c=fit["se_c"], n=fit["n"], secs=fit["secs"])
            v["fit_why"] = fw
        for k in ("why", "msg", "fit_why"):
            if v.get(k) is not None:
                v[k + "_en"] = en_of(v[k])
        return v

    def _temp_why(self, est, b, now, wc):
        if wc is not None:
            return None
        if est.n_push == 0 or (now - est.t_push) > BOARD_FRESH_S:
            return "no_tlm"
        if est.r is None:
            if est.last == "lowi":
                return "low_iq"
            if est.last == "emf":
                return "spinning"
            if est.last == "range":
                return "range"
            return "reading"
        base = est.base
        settling = base.src == "file" and est.n_ok < TEMP_FILE_SHOW_AFTER
        if base.r0 is None or settling:
            if base.r0 is None and b.get("temp_c") is None:
                return "no_board_temp"
            if est.last == "emf":
                return "spinning"
            if est.last == "range":
                return "range"
            return "settling" if settling else "basis"
        if est.gated():
            return "iq_mismatch"
        if not est.fresh(now):
            if est.last == "emf":
                return "spinning"
            if est.last == "range":
                return "range"
            return "stopped"
        return None

    def temps_snapshot(self):
        now = time.time()
        axes = []
        for n in self.order:
            th = self.temps.get(n)
            est = self.wind.get(n)
            err = self._wind_ref_err(n)
            wref = dict(ref=self.wind_refs.get(n), ref_err=err, ref_err_en=en_of(err))
            if th is None or est is None:
                axes.append(dict(node=n, dt=None, pts=[], board_c=None,
                                 wind_c=None, wind_ok=False, why="no_tlm",
                                 r=None, r0=None, t0=None, iq0=None, iq_now=None, ref_src=None, **wref))
                continue
            pts = th.recent(now)
            b = self._board(n)
            ok = est.fresh(now)
            wc = est.celsius() if ok else None
            base = est.base
            axes.append(dict(
                node=n, dt=th.dt,
                pts=[[round(p[0] - now, 1),
                      None if p[1] is None else round(p[1], 1),
                      None if p[2] is None else round(p[2], 1)] for p in pts],
                board_c=b.get("temp_c"),
                wind_c=(None if wc is None else round(wc, 1)),
                wind_ok=ok,
                why=self._temp_why(est, b, now, wc),
                r=(None if est.r is None else round(est.r, 4)),
                r0=(None if base.r0 is None else round(base.r0, 4)),
                t0=base.t0,
                iq0=(None if base.iq0 is None else round(base.iq0, 2)),
                ref_src=base.src, **wref,
                iq_now=(round(abs(b["iq_ma"]) / 1000.0, 2) if "iq_ma" in b
                        else (None if est.iq_now is None else round(est.iq_now, 2))),
                dbg=dict(push=est.n_push, lowi=est.n_lowi, emf=est.n_emf,
                         win=len(est.samples), sd=est.last_sd, rms=est.last_rms,
                         rms_rej=est.n_rms_rej, n_ok=est.n_ok,
                         dv=est.dv, dv_se=est.dv_se),
            ))
        return dict(t_start=self.t_start - now, axes=axes, view_s=TEMP_VIEW_S,
                    r0_after=TEMP_R0_AFTER, iq_min=TEMP_IQ_MIN)

    def _board(self, n):
        out = {}
        now = time.time()

        def fresh(idx):
            d = self.bus.tlm.get((n, idx))
            if d is None or (now - self.bus.tlm_t.get((n, idx), 0)) > BOARD_FRESH_S:
                return None
            return d

        d = fresh(0)
        if d and len(d) >= 8:
            raw = s16(d, 4)
            if raw != TEMP_INVALID_RAW:
                out["temp_c"] = raw / 10.0
            out["uptime_s"] = u16(d, 6)
        d = fresh(1)
        if d and len(d) >= 8:
            f = d[6]
            out["flags"] = f
            out["drv_ok"] = bool(f & 0x01)
            out["enc_ok"] = bool(f & 0x02)
            out["nomag"] = bool(f & 0x04)
            out["fault_now"] = bool(f & 0x08)
            out["pwm3x"] = bool(f & 0x10)
            out["cfg_ok"] = bool(f & 0x20)
            out["output_on"] = bool(f & 0x40)
            out["selftest"] = not krcbus.selftest_why(f)
            out["enc_raw"] = u16(d, 4)
        d = fresh(2)
        if d and len(d) >= 8:
            out["peak_a"] = d[7] / 10.0
        d = fresh(4)
        if d and len(d) >= 8:
            out["therm_pct"] = d[0]
            out["enc_err"] = d[1]
            out["isr_cycles"] = u16(d, 2)
            out["isr_max"] = u16(d, 4)
            out["trip"] = d[6]
            out["foc_mode"] = d[7]
        d = fresh(5)
        if d and len(d) >= 8:
            out["iq_ma"] = s16(d, 4)
            out["id_ma"] = s16(d, 6)
        d = fresh(6)
        if d and len(d) >= 8:
            out["rpm"] = s16(d, 4)
        d = fresh(7)
        if d and len(d) >= 8:
            out["vd_mv"] = s16(d, 0)
            out["vq_mv"] = s16(d, 2)
            out["vbus_mv"] = u16(d, 4)
        d = fresh(8)
        if d and len(d) >= 8:
            out["cal"] = krcbus.cal_from_tlm8(d)
            out["pole_pairs"] = d[2] & 0x7F
            out["dir"] = -1 if d[3] == 0xFF else 1
            out["cur_kp_mv_a"] = u16(d, 4)
            out["cur_ki"] = u16(d, 6)
        d = fresh(12)
        if d and len(d) >= 8:
            lo, hi = limits_from_tlm12(d)
            out["lim_lo"] = lo
            out["lim_hi"] = hi
            out["lim_on"] = lo < hi
            out["khz"] = d[5]
        out["alive"] = bool(out)
        return out

    def demo_snapshot(self, cid=None):
        with self.lock:
            if cid == self.arm_cid:
                self.last_poll = time.monotonic()
            drive = self.driven_nodes()
            axes = []
            for n in self.order:
                lp, _lv, _li = self._live(n, n in drive)
                b = self._board(n)
                axes.append(dict(node=n, on=self.ax[n].on, mode=self.ax[n].mode, armed=(n in drive),
                                 pos=lp, p_cmd=self.ax[n].traj_p, kp=self.ax[n].kp, kd=self.ax[n].kd,
                                 tripped=trip_state(self.bus.is_tripped(n), b.get("trip")),
                                 board=dict(alive=b.get("alive", False), temp_c=b.get("temp_c"), enc_raw=b.get("enc_raw"))))
            return no_nan(dict(nodes=list(self.order), axes=axes, armed=self.armed, note=self.note,
                               note_en=self.note_en, jt=self._jt_snap(),
                               tripped=any(a["tripped"] for a in axes), zero_ref=self._demo_zero()))

    def _demo_zero(self):
        z = getattr(self, "_demo_zero_ref", None)
        if z is None:
            try:
                with io.open(DEMO_ZERO_PATH, encoding="utf-8") as f:
                    z = dict((str(int(k)), int(v)) for k, v in json.load(f).items() if 0 <= int(v) < 16384)
            except (OSError, ValueError, TypeError, AttributeError):
                z = {}
            self._demo_zero_ref = z
        return z

    def demo_zero_save(self, refs):
        if not isinstance(refs, dict) or not refs:
            return {"ok": False, "err": "refs は {軸番号: 絶対角}"}
        z = {}
        for k, v in refs.items():
            n, e = num_int(k, "軸番号"), num_int(v, "絶対角")
            if not 0 <= e < 16384:
                return {"ok": False, "err": "軸%d の絶対角 %d が 0〜16383 の外" % (n, e)}
            z[str(n)] = e
        tmp = DEMO_ZERO_PATH + ".tmp"
        try:
            with io.open(tmp, "w", encoding="utf-8") as f:
                json.dump(z, f)
            os.replace(tmp, DEMO_ZERO_PATH)
        except OSError as e:
            return {"ok": False, "err": "0 点を保存できませんでした: %s" % e}
        self._demo_zero_ref = z
        return {"ok": True}

    def demo_tlm(self, div):
        div = num_int(div, "間引き")
        if not 0 <= div <= 255:
            return {"ok": False, "err": "間引きは 0〜255"}
        with self.lock:
            now = time.time()
            for n in self.order:
                self.pending.append((now, lambda k=n, d=div: self.bus._cfg(k, CFG_SET_TLM, d), True))
        return {"ok": True}

    def snapshot(self, since, cid=None):
        now = time.time()
        with self.lock:
            if cid == self.arm_cid:
                self.last_poll = time.monotonic()
            for j in list(self.offcal.values()):
                if j.cid == cid:
                    j.beat()
            try:
                since = int(since)
            except (TypeError, ValueError):
                since = 0
            if since < 0 or since > self.seq:
                since = 0
            stops = []
            for n in self.order:
                ahead = [x[0] for x in list(self.ax[n].hist) if x[0] > since]
                if len(ahead) > HANDOFF_N:
                    stops.append(ahead[HANDOFF_N - 1])
            stop = min(stops) if stops else None
            p = list(self.periods)

            drive = self.driven_nodes()
            if self.node is None:
                a = None
                mx = None
                pos = vel = iq = pt = None
                pos_why = "軸がありません（基板の電源・CAN を確かめて「再探索」）"
            else:
                a = self.bus.axes[self.node]
                mx = self.ax[self.node]
                pos, vel, iq = self._live(self.node, self.node in drive)
                pt = self._pos_true(self.node)
                _p, pos_why = self._gate(self.node)
            axes = []
            for n in self.order:
                lp, lv, li = self._live(n, n in drive)
                b = self._board(n)
                _q, why = self._gate(n)
                smp = [x for x in list(self.ax[n].hist)
                       if x[0] > since and (stop is None or x[0] <= stop)]
                kt_why = self._kt_why(n)
                pt_why = self._pt_why(n)
                arm_block = self._arm_block(n)
                axes.append(dict(
                    node=n, on=self.ax[n].on, mode=self.ax[n].mode,
                    kp=self.ax[n].kp, kd=self.ax[n].kd,
                    kp_max=self._kp_max(n),
                    kp_sent=self._gain_sent(n, "kp", self.ax[n].kp), kd_sent=self._gain_sent(n, "kd", self.ax[n].kd),
                    amp=self.ax[n].amp, freq=self.ax[n].freq,
                    v_target=self.ax[n].v_target,
                    t_target=self.ax[n].t_target,
                    center=self.ax[n].center,
                    board=b,
                    samples=smp,
                    seq_last=(smp[-1][0] if smp else since),
                    pos=lp, vel=lv, iq=li,
                    pos_true=self._pos_true(n),
                    p_target=self.ax[n].p_target,
                    p_cmd=self.ax[n].traj_p, glide=self.ax[n].glide_v,
                    stop_iq=self.ax[n].stop_iq, stop_hit=self.ax[n].stop_hit,
                    pos_ok=(why is None), pos_why=why, pos_why_en=en_of(why),
                    p_lo=self._p_bound(n)[0], p_hi=self._p_bound(n)[1],
                    tripped=trip_state(self.bus.is_tripped(n),
                                       b.get("trip")),
                    trip_label=trip_label(
                        trip_state(self.bus.is_tripped(n), b.get("trip")),
                        b.get("trip")),
                    trip_label_en=en_of(trip_label(
                        trip_state(self.bus.is_tripped(n), b.get("trip")),
                        b.get("trip"))),
                    alive=b.get("alive", False),
                    selftest=b.get("selftest"), cal=b.get("cal"),
                    therm_pct=b.get("therm_pct"), isr_max=b.get("isr_max"),
                    enc_err=b.get("enc_err"),
                    temp_c=b.get("temp_c"),
                    fb_count=self.bus.axes[n].fb_count,
                    fb_missing=self.bus.axes[n].fb_missing,
                    t_max=round(self._t_max(n), 3),
                    v_max=round(self._v_max(n), 1),
                    params=self._param_view(n)[0],
                    param_save=self._param_view(n)[1],
                    kt=self._kt(n),
                    kt_pulse=self._kt_pulse_view(n, now),
                    kt_why=kt_why, kt_why_en=en_of(kt_why),
                    pt=self._pt_view(n, now),
                    pt_why=pt_why, pt_why_en=en_of(pt_why),
                    therm_trip=self._therm(n)[0], therm_clear=self._therm(n)[1],
                    armed=(n in drive),
                    held=self._board_held(n, now),
                    arm_block=arm_block, arm_block_en=en_of(arm_block),
                    rezero=self._may_rezero(n)[0],
                    rezero_why=self._may_rezero(n)[1],
                    rezero_why_en=en_of(self._may_rezero(n)[1]),
                    offcal=self._offcal_view(n, now),
                    wcal=self._wcal_view(n, now),
                ))
            st = dict(
                node=self.node, nodes=list(self.order), axes=axes,
                rate=self.rate, armed=self.armed,
                mode=(mx.mode if mx else None), target=self.target,
                note=self.note, note_seq=self.note_seq, note_en=self.note_en,
                note_trip=(self.note_seq == self._estop_note_seq),
                stall=self.stall, wd_fired=self.wd_fired,
                jt=self._jt_snap(),
                wd_s=WATCHDOG_S,
                p_target=(mx.p_target if mx else None),
                v_target=(mx.v_target if mx else None),
                t_target=(mx.t_target if mx else None),
                kp=(mx.kp if mx else None), kd=(mx.kd if mx else None),
                amp=(mx.amp if mx else None), freq=(mx.freq if mx else None),
                center=(mx.center if mx else None),
                pos=pos, vel=vel, iq=iq,
                tripped=(self.bus.is_tripped(self.node) if mx else None),
                pos_true=pt,
                pos_ok=(pos_why is None),
                pos_why=pos_why,
                pos_safe=POS_SAFE_RAD,
                fb_count=(a.fb_count if a else 0), fb_missing=(a.fb_missing if a else 0),
                seq=self.seq,
            )
        if p:
            p = p[2:] or p
            m = sum(p) / len(p)
            st["loop_ms"] = m * 1000.0
            st["loop_sd_ms"] = math.sqrt(
                sum((x - m) ** 2 for x in p) / len(p)) * 1000.0
            st["loop_max_ms"] = max(p) * 1000.0
        st["board"] = self._board(self.node) if self.node is not None else {}
        st["tripped"] = trip_state(st["tripped"], st["board"].get("trip"))
        for x in axes:
            x["trip_unknown"] = (None if x["tripped"] is not None
                                 else (TRIP_UNKNOWN_TEXT if x["alive"] else TRIP_NOREPLY_TEXT))
            x["trip_unknown_en"] = en_of(x["trip_unknown"])
        unk = [x["trip_unknown"] for x in axes if x["trip_unknown"] is not None]
        st["trip_unknown"] = (TRIP_UNKNOWN_TEXT if TRIP_UNKNOWN_TEXT in unk
                              else (unk[0] if unk else None))
        st["arm_block"] = self._arm_block()
        busy = self.armed or self._arming()
        st["trip_action"] = (
            None if not any(x["tripped"] is True for x in axes)
            else (self._trip_action(self.live_nodes()) if self._any_tripped() is True
                  else (TRIP_OFF_AXIS_TEXT if (st["arm_block"] is None or busy)
                        else None)))
        st["trip_action_en"] = en_of(st["trip_action"])
        st["trip_unknown_en"] = en_of(st["trip_unknown"])
        st["arm_note"] = (
            None if (busy or st["arm_block"] is None
                     or (st["trip_action"]
                         and st["trip_action"] in st["arm_block"]))
            else st["arm_block"])
        st["arm_note_en"] = en_of(st["arm_note"])
        hn = self._held_note
        cut = None
        if hn is not None and hn[0] == self.note_seq and st["arm_note"] is not None:
            now_c = time.time()
            busy_n = [n for n in self.live_nodes() if self._board_busy(n, now_c)]
            if busy_n and str(st["arm_note"]) == str(self._board_busy_why(busy_n)):
                cut = hn[2]
        st["note_cut"] = cut
        st["note_cut_en"] = en_of(cut)
        st["trip_label"] = trip_label(st["tripped"], st["board"].get("trip"))
        st["trip_label_en"] = en_of(st["trip_label"])

        st["seq_last"] = (stop if stop is not None
                          else max([a["seq_last"] for a in axes] + [since]))
        st["limits"] = dict(kp=UI_KP_MAX, kd=UI_KD_MAX, t=UI_T_MAX,
                            v=UI_V_MAX, amp=UI_AMP_MAX, freq=UI_FREQ_MAX,
                            p=POS_SAFE_RAD,
                            kt_secs=[KT_SECS_DEF, KT_SECS_MIN, KT_SECS_MAX],
                            pt=dict(pts=PT_PTS_DEF, speed=PT_SPEED_DEF, dwell=PT_DWELL_DEF, dwell_max=PT_DWELL_MAX),
                            offcal_n=[offset_full.N_DEF, offset_full.N_MIN, offset_full.N_MAX],
                            wcal=[WCAL_AMPS_DEF, WCAL_AMPS_MIN, WCAL_SECS_DEF, WCAL_SECS_MIN, WCAL_SECS_MAX],
                            vel_view=[VEL_VIEW_K, VEL_VIEW_STEP],
                            temp_view=[TEMP_VIEW_LO_C, TEMP_VIEW_HI_C],
                            gear=GEAR_RATIO,
                            ocg=[round(math.degrees(OCG_SEEK_V) / GEAR_RATIO), OCG_BACK_REV,
                                 round(OCG_BACK_REV * 360.0 / GEAR_RATIO)])
        st["param_defs"] = [dict(id=pid, key=v[0], name=v[1], unit=v[2], lsb=v[3],
                                 lo=krcbus.param_value(pid, v[4]), hi=krcbus.param_value(pid, v[5]),
                                 d=krcbus.param_value(pid, v[6]))
                            for pid, v in sorted(krcbus.PARAMS.items()) if v[0] not in krcbus.PARAMS_HIDDEN]
        st["param_save_text"] = krcbus.PARAM_SAVE_TEXT
        for ax in st.get("axes") or []:
            ax["samples"] = [[round(x[1] - now, 4),
                              None if x[2] is None else round(x[2], 5),
                              None if x[3] is None else round(x[3], 5),
                              None if x[4] is None else round(x[4], 3),
                              None if x[5] is None else round(x[5], 4),
                              None if x[6] is None else round(x[6], 4)]
                             for x in ax["samples"]]
        return st


def _new_axis(node):
    from krcbus import Axis
    return Axis(node)


class Handler(BaseHTTPRequestHandler):
    ctrl = None
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass

    def _json(self, obj, code=200, close=False):
        try:
            body = json.dumps(obj, allow_nan=False).encode("utf-8")
        except ValueError:
            body = json.dumps(no_nan(obj), allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if close:
            self.send_header("Connection", "close")
            self.close_connection = True
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/state":
            since = 0
            cid = None
            if "?" in self.path:
                for kv in self.path.split("?", 1)[1].split("&"):
                    if kv.startswith("since="):
                        try:
                            since = int(kv[6:])
                        except ValueError:
                            since = 0
                    elif kv.startswith("cid="):
                        cid = kv[4:]
            self._json(self.ctrl.snapshot(since, cid))
            return
        if path == "/api/demo":
            cid = None
            if "?" in self.path:
                for kv in self.path.split("?", 1)[1].split("&"):
                    if kv.startswith("cid="):
                        cid = kv[4:]
            self._json(self.ctrl.demo_snapshot(cid))
            return
        if path == "/api/temps":
            self._json(self.ctrl.temps_snapshot())
            return
        if path == "/api/leg":
            self._json(self.ctrl.leg_snapshot())
            return
        if path == "/api/rec":
            self._json(self.ctrl.rec_snapshot())
            return
        if path.startswith("/rec/"):
            fn = urllib.parse.unquote(path[5:])
            if (not fn or "/" in fn or "\\" in fn or ".." in fn or not fn.endswith(".csv")
                    or any(ord(ch) < 0x20 for ch in fn)):
                self.send_error(404)
                return
            p = os.path.join(REC_DIR, fn)
            try:
                with io.open(p, "rb") as f:
                    body = f.read()
            except OSError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", "attachment; filename=\"%s\"; filename*=UTF-8''%s"
                             % (fn.encode("ascii", "replace").decode("ascii").replace('"', "_"),
                                urllib.parse.quote(fn)))
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path in ("/leg", "/demo"):
            body = (LEG_PAGE if path == "/leg" else DEMO_PAGE).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def do_POST(self):
        if self.path != "/api/cmd":
            self.send_error(404)
            return
        try:
            n = num_int(self.headers.get("Content-Length", 0), "Content-Length")
        except BadValue as e:
            self._json({"ok": False, "err": str(e)}, 400, close=True)
            return
        if n < 0 or n > MAX_BODY:
            self._json({"ok": False, "err":
                        "Content-Length が扱える範囲の外です: %r"
                        "（0〜%d バイト）" % (n, MAX_BODY)}, 400, close=True)
            return
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            self._json({"ok": False, "err": "JSONが壊れています"}, 400)
            return
        except RecursionError:
            self._json({"ok": False, "err": "JSONが壊れています（入れ子が深すぎる）"}, 400)
            return
        if not isinstance(req, dict):
            self._json({"ok": False,
                        "err": "本文が読めません（JSON のオブジェクトではありません）"},
                       400)
            return
        params = req.get("params") or {}
        if not isinstance(params, dict):
            self._json({"ok": False, "err": "params が読めません"}, 400)
            return
        try:
            res = self.ctrl.do(req.get("action", ""), params)
        except BadValue as e:
            res = {"ok": False, "err": str(e)}
        if isinstance(res, dict) and en_of(res.get("err")):
            res["err_en"] = en_of(res["err"])
        self._json(res)


PAGE = r"""<!doctype html>
<html lang="ja" class="light"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KRC アクチュエータ</title>
<style>
/*__COMMON_CSS__*/

html.light{--bg:#f6f8fa;--panel:#ffffff;--line:#c4ccd5;--fg:#1f2328;--dim:#424a53;
  --ok:#1a7f37;--warn:#bc4c00;--bad:#cf222e;--cyan:#0a7c8c;--pri:#0969da;--estop:#c4161c;
  --c1:#0969da;--c2:#1a7f37;--c3:#cf222e;--c4:#8250df;--c5:#8a5a44;--c6:#bf3989;--c7:#9a6700;
  --btn:#f6f8fa;--bad-bg:#ffebe9;--bad-fg:#82071e;--warn-bg:#fff1e5;--warn-fg:#953800;--info-bg:#ddf4ff;--info-fg:#0a3069}
body{margin:0;background:var(--bg);color:var(--fg);font-size:14px}
.wrap{max-width:1600px;margin:0 auto;padding:14px}
header{display:flex;align-items:center;gap:12px;flex-wrap:wrap;
  padding:10px 14px;background:var(--panel);border:1px solid var(--line);
  border-radius:8px;position:sticky;top:0;z-index:5}
h1{font-size:15px;margin:0;font-weight:600}
.spacer{flex:1}
.banner{margin:10px 0}


#cards{display:grid;gap:12px;margin-top:12px;
  grid-template-columns:repeat(auto-fit,minmax(330px,1fr))}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;
  padding:10px 12px;max-width:640px;container-type:inline-size}
.card.off{opacity:.55}
.card.view{border-color:var(--cyan)}
.card h2{font-size:14px;margin:0;display:flex;align-items:center;gap:8px}
.card h2 .sw{margin-left:auto;font-size:11px;color:var(--dim);font-weight:400}
.card h2 .sw button[data-gear]{margin-right:6px}
.nums{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;margin:8px 0 4px}
.nums > div{background:var(--bg);border:1px solid var(--line);border-radius:5px;
  padding:5px 6px}
.nums .k{font-size:11px;color:var(--dim)}
.nums .v{font-size:clamp(15px,4cqw,22px);font-weight:600;font-variant-numeric:tabular-nums;white-space:nowrap}

body.stale .nums{opacity:.45}

body.stale .gval,body.stale .templine{opacity:.45}
.modes{display:flex;flex-wrap:wrap;gap:4px;margin:8px 0}
.modes button{padding:4px 9px;font-size:12px;line-height:17px}
.modes button.locked{color:var(--warn);border-style:dashed}
.warnline button{font-size:11px;line-height:15px;padding:3px 8px;margin-top:4px}
.row{display:flex;align-items:center;gap:8px;margin:5px 0;font-size:12px}
.row label{width:62px;color:var(--dim)}
.row input[type=range]{flex:1;accent-color:var(--cyan)}
.row .n{width:62px;text-align:right;font-variant-numeric:tabular-nums}
.row input.kin{width:62px;box-sizing:border-box;background:var(--bg);color:var(--fg);border:1px solid var(--line);
  border-radius:5px;padding:2px 6px;font:inherit;font-size:12px;text-align:right;font-variant-numeric:tabular-nums}
.row .kq{color:var(--dim);font-size:11px;font-variant-numeric:tabular-nums;white-space:nowrap}
.row .kq:empty{display:none}
.ktrow{flex-wrap:wrap;gap:4px 6px}
.ktrow button{padding:4px 9px;font-size:12px;line-height:17px}
.ktrow input{width:64px;box-sizing:border-box;background:var(--bg);color:var(--fg);border:1px solid var(--line);
  border-radius:5px;padding:3px 6px;font:inherit;font-size:12px}
.ktrow .ku{color:var(--dim)}
.ktrow .ktst{color:var(--dim);overflow-wrap:anywhere;flex:1;font-size:11px}
.ktrow .ktst.on{color:var(--warn);font-weight:600;font-size:12px;font-variant-numeric:tabular-nums}

.ktrow input.ptpts{flex:1;min-width:150px;width:auto}
.ktrow label.ptl{width:auto;margin-left:8px}

.oc{margin-top:10px;font-size:12px;border-top:1px solid var(--line);padding-top:6px}
.oc:empty{display:none}
.oc .row{flex-wrap:wrap;gap:4px 8px}
.oc .row button{padding:4px 9px;font-size:12px;line-height:17px}
.oc input[type=number]{width:56px;box-sizing:border-box;background:var(--bg);color:var(--fg);border:1px solid var(--line);
  border-radius:5px;padding:3px 6px;font:inherit;font-size:12px}
.oc .ocmsg{font-size:15px;font-weight:600;line-height:1.5;margin:6px 0;overflow-wrap:anywhere}
.oc .ocmsg.act{color:var(--warn)}
.oc .ocsub{color:var(--dim);line-height:1.6;overflow-wrap:anywhere}
.oc .row label.occhk{width:auto;color:var(--fg);display:flex;align-items:center;gap:6px;cursor:pointer}
.oc .ocbar{height:6px;background:var(--line);border-radius:3px;overflow:hidden;margin:6px 0}
.oc .ocbar > div{height:100%;background:var(--cyan)}
.oc table{border-collapse:collapse;font-variant-numeric:tabular-nums;margin:4px 0}
.oc td{padding:1px 12px 1px 0}
.oc td.k{color:var(--dim)}
.oc .wcwarn{color:var(--warn);line-height:1.6;overflow-wrap:anywhere}
.oc details.wccond{margin:4px 0}
.oc details.wccond summary{cursor:pointer;color:var(--dim)}
.oc details.wccond .row label{width:auto;min-width:40px}
.oc .wcref{display:flex;flex-wrap:wrap;align-items:center;gap:4px 8px;margin-top:8px;padding-top:6px;border-top:1px dashed var(--line);
  color:var(--dim);font-variant-numeric:tabular-nums}
.oc .wcref span{flex:1;min-width:160px;overflow-wrap:anywhere}
canvas{width:100%;background:var(--panel);border:1px solid var(--line);
  border-radius:5px;display:block;margin-top:6px}

canvas[data-cvp]{aspect-ratio:300/100}

canvas[data-cvv],canvas[data-cvc],canvas[data-cvt]{aspect-ratio:300/80;min-height:112px}
.cvwrap{display:flex;gap:4px;align-items:stretch;margin-top:6px}
.cvwrap canvas{flex:1;min-width:0;width:auto;margin-top:0}
.vzoom{writing-mode:vertical-lr;direction:rtl;width:16px;margin:0;accent-color:var(--cyan);cursor:ns-resize}
.vzgap{width:16px;flex:none}
.prm{margin-top:10px;font-size:12px;border-top:1px solid var(--line);padding-top:6px}
.prm summary{cursor:pointer;color:var(--dim);user-select:none}
.prm summary .ps{margin-left:8px}
.prow{display:grid;grid-template-columns:minmax(0,1fr) 88px 64px minmax(0,1fr);gap:6px;align-items:center;margin-top:4px}
.prow label{color:var(--dim);overflow-wrap:anywhere}
.prow input{width:100%;box-sizing:border-box;background:var(--bg);color:var(--fg);border:1px solid var(--line);
  border-radius:5px;padding:3px 6px;font:inherit;font-size:12px}
.prow .pu{color:var(--dim)}
.prow .pv{color:var(--dim);font-variant-numeric:tabular-nums}
.prow .pv.miss{color:var(--warn)}
.prm .ps.bad{color:var(--bad)}
.prm .ps.warn{color:var(--warn)}
.wref{display:flex;flex-wrap:wrap;align-items:center;gap:4px 8px;margin-top:8px;padding-top:6px;border-top:1px dashed var(--line)}
.wref .wl{color:var(--dim)}
.wref .wv{color:var(--dim);font-variant-numeric:tabular-nums;flex:1;min-width:120px}
.wref .wh{color:var(--dim);font-size:11px}
.wref button{padding:4px 9px;font-size:12px;line-height:17px}
.note{color:var(--dim);font-size:11px;line-height:1.7;margin-top:10px;overflow-wrap:anywhere}
.warnline{color:var(--warn);font-size:11px;margin-top:4px;line-height:1.5}
.gtitle{font-size:13px;font-weight:600;color:var(--fg);margin:12px 0 -2px}
.gtitle .gval{margin-left:12px;font-variant-numeric:tabular-nums;font-weight:600}
.gtitle .gtau{margin-left:14px;font-variant-numeric:tabular-nums;font-weight:400;color:var(--dim)}

.templine{display:inline;font-size:13px;margin-left:12px;font-variant-numeric:tabular-nums}
.templine>span{display:inline-block;margin-right:12px}
.templine .tb{color:var(--dim)}
.templine .tw{color:var(--c5)}
.templine .tr{color:var(--dim);font-size:11px}
.templine .th{color:var(--dim);font-size:11px}

#loopSt{font-weight:700}
#btnLang span{color:var(--dim)}
#btnLang span.lon{color:var(--fg);font-weight:700}
</style></head><body><div class="wrap">

<header>
  <span class="dot" id="dot"></span>
  <h1 id="hTitle">KRC アクチュエータ</h1>
  <span class="chip" id="chipLoop"><span id="loopNum"></span><span id="loopSt"></span></span>
  <span class="spacer"></span>
  <button id="btnLang"></button>
  <button id="btnDemo" title="12 軸がそろっているときに、12 軸デモの画面を開く" disabled>12 軸デモ</button>
  <button id="btnRescan" title="バスに居る軸を探し直す（こちらからは何も送りません）">再探索</button>
  <button id="btnDisarm" title="通電している軸をすべて解除（通電は各軸のカードの「通電」から）">全軸解除</button>
  <button id="btnClear">遮断を解除</button>
  <button id="btnRecM" title="200 Hz の記録（CSV）。脚の画面に一覧"><span class="rdot"></span> 記録</button>
  <button id="btnEstop" class="estop">非常停止</button>
</header>

<div class="banner bad" id="bnBad"></div>
<div class="banner warn" id="bnWarn"></div>
<div class="banner info" id="bnInfo"></div>

<div id="cards"></div>



</div>
<script>

const MODE_LABEL={free:"脱力",damp:"ダンパ",hold:"位置保持",
                  sine:"正弦波",vel:"速度",torque:"トルク",kt_test:"トルクテスト",pos_test:"位置制御テスト",homing:"原点出し中",leg_sine:"脚の正弦波",leg_traj:"脚の軌道"};
const I18N={
 ja:{
  title:"KRC アクチュエータ", langTip:"表示の言語（日本語 / English）",
  rescan:"再探索", rescanTip:"バスに居る軸を探し直す（こちらからは何も送りません）",
  disarmAll:"全軸解除", disarmAllTip:"通電している軸をすべて解除（通電は各軸のカードの「通電」から）",
  clear:"遮断を解除", estop:"非常停止",
  loop:(m,s)=>"周期 "+m+" ms ± "+s, loopArmed:"　通電中", loopDisarmed:"　解除中", loopHeld:"　基板が保持中",
  recOnHtml:"<span class='rdot blink'></span> 記録中 <span class='secs'></span> s",
  recStartHtml:"<span class='rdot'></span> 記録 開始",
  recIdleHtml:"<span class='rdot'></span> 記録",
  recOnTip:"押すと記録を止める（記録は動いたまま。非常停止ではない）",
  recTip:"200 Hz の記録（CSV）。脚の画面に一覧",
  recErr:(f,n,e)=>`前回の記録 ${f} は途中で書けなくなりました（書けたのは ${n} 行）: ${e}`,
  axisH:(n)=>"軸 "+n, axisN:(n)=>"軸"+n,
  nPos:"位置 rad", nVel:"速度 rad/s", nCur:"電流 A", nTq:"トルク N·m",
  arm:"通電", disarm:"解除",
  stTrip:"遮断", stUnk:"状態不明", stArmed:"通電中", stIdle:"待機", stOcRun:"オフセットを測定中", stOcPush:"押し込み直し待ち",
  stHeld:"基板が保持中",
  stJt:(nm)=>"軌道を再生中（"+nm+"）", stJtGo:(nm)=>"軌道の始点へ（"+nm+"）", stJtStop:(nm)=>"軌道を止めています（"+nm+"）",
  mode:MODE_LABEL,
  pTarget:"目標 rad", pAmp:"振幅 rad", pFreq:"周波数 Hz", pCenter:"中心 rad", pVel:"速度 rad/s", pTq:"トルク N·m",
  posLocked:(w,r)=>"位置モードに入れません: "+w+"。"+(r?"（"+r+"）":""),
  vzTip:(nm)=>nm+"の図の縦の最小の幅（上げると広く・下げると細かく）。線がはみ出したら広げます",
  gPos:"位置", gTgt:"目標", gPosUnit:(r)=>"位置 rad（最小 ±"+r+"）", gCur:"電流 A", gVel:(r)=>"速度 rad/s（±"+r+"）",
  gtPos:"位置 [rad]", gtVel:"速度 [rad/s]", gtCur:"電流 [A]", gtTemp:"温度 [℃]",
  gear:(r)=>"減速機 "+r+":1",
  gearTip:(r)=>"押すと出力軸（減速機 "+r+":1 の先）の位置・速度で見て動かします（位置は °）。もう一度押すとモーター軸に戻ります。Kp・Kd・電流・トルクはモーター軸のまま",
  gtPosOut:"出力軸の位置 [°]", gtVelOut:"出力軸の速度 [rad/s]",
  pTargetOut:"目標 °", pAmpOut:"振幅 °", pCenterOut:"中心 °", pVelOut:"出力 rad/s", ptPtsOut:"出力 °",
  kinTip:(nm,hi)=>nm+"：数を打って Enter（0.01 刻み・0〜"+hi+"）。モーター軸の値です", kq:(v)=>"基板 "+v,
  kqTip:"MIT の 12 ビットに詰めて基板に届く値（目盛り ÷ 4095 の刻みで切り捨て）",
  degC:"℃",
  tauEst:(v,k)=>"トルク ≈ "+v+" N·m（Kt "+k+" N·m/A とした計算値）",
  prm:"基板の設定", prmTip:(nm,lo,hi,d)=>nm+"（範囲 "+lo+"〜"+hi+"・既定 "+d+"）",
  prmBoard:(v)=>"基板 "+v, prmDiff:"（画面が送った値と違う）", prmDiffV:(v)=>"（画面が送った "+v+" と違う）",
  ktCur:"電流 A", ktMax:(a)=>"上限 "+a, ktMaxTip:"この軸の電流の上限（基板の読み返し）", ktDir:"向き", ktFree:"自由入力",
  ktGo:"流す", ktSecs:"秒数", ktStop:"止める", ktPlus:"＋",
  ptPts:"角度 °", ptPtsTip:"モーター軸の角度（原点 0 から・度）を順に回り、最後まで行ったら最初へ戻って繰り返します。区切りはカンマ（この欄では Space も区切り。非常停止は Esc か赤いボタン）",
  ptStyle:"動き方", ptSmooth:"なめらか", ptStep:"段",
  ptSmoothTip:"点から点へ、速さの上限となめらかな加速・減速で動いて止まる（位置と速さを一緒に送る）",
  ptStepTip:"目標を次の点へ一気に切り替える（ステップ応答・速さは 0）。始めるときだけ最初の点へなめらかに寄る",
  ptSpeed:"速さ", ptSpeedTip:"速さの上限（なめらか・最初の点へ寄るとき）", ptDw:"止まる", ptDwTip:"各点で止まっている秒数（段のときは次へ切り替えるまでの秒数）",
  ptGo:"始める", ptStop:"止める", ptHold:"位置保持で待っています",
  ptApproach:(d)=>"最初の点 "+d+"° へ寄っています", ptMove:(k,n,d)=>k+"/"+n+" 点目 "+d+"° へ移動中",
  ptDwellAt:(k,n,d,l)=>k+"/"+n+" 点目 "+d+"° で停止中（あと "+l+" s）", ptStepAt:(k,n,d,l)=>k+"/"+n+" 点目 "+d+"°（次まで "+l+" s）",
  ptStopping:"止めています（その場で位置保持へ）",
  ktDown:(sg,A,l)=>"0 A へ戻しています（"+sg+A+" A から・残り "+l+" s）", ktOn:(sg,A,l)=>sg+A+" A を流しています（残り "+l+" s）",
  ktNoMax:(n)=>"軸"+n+" の電流の上限が読めていません", ktNeedNum:"流す電流（A）を数で入れてください", needNum:"数を入れてください",
  prmDropOne:(n,nm)=>"軸"+n+" の"+nm, prmDrop:(s)=>s+"は、打っている途中にカードを描き直したので送っていません。もう一度入れてください",
  ocOpen:"電気角オフセットを測る",
  ocgOpen:"ローター傾き補正",
  ocIdle:"腕・脚を外して、ローターが 1 回転まわれる状態にしてください（脚に組んだまま使わない）",
  ocChk:"外しました・ローターが 1 回転まわれます", ocRuns:"回数", ocRunsU:"回",
  ocRoundS:(s)=>"（1 回 約 "+s+" 秒。回のあいだにローターを手で押し込み直してもらいます）",
  ocChoose:"測り終えたら、補正（パラメータ 39〜42）も入れるか、オフセットだけかを選べます（同じ測定から両方出ます）。",
  ocStart:"始める", ocClose:"閉じる", ocCancel:"中止", ocNext:"次へ", ocAgain:"もう一度",
  ocRunning:(k,s)=>k+" 回目を回しています（約 "+s+" 秒・触らないでください）",
  ocKeepOpen:(n)=>"全 "+n+" 回。この画面を開いたままにしてください（裏に回す・閉じると止まります）。",
  ocPush:(k)=>"ローターを手で押し込み直して、『次へ』を押してください（"+k+" 回目の前）",
  ocStats:(m,sd,sp)=>"平均 "+m+"・ばらつき "+sd+"・幅 "+sp+"°", ocSdOne:"–（1 回だけ）",
  ocOff:(o,nw,d)=>"オフセット "+o+" → "+nw+"（"+d+" カウント）",
  ocApplying:"入れています（基板の読み返しを待っています）…",
  ocModeCorr:"補正も入れる（おすすめ）", ocModeOff:"オフセットだけ（補正 39〜42 は 0 に戻す）",
  ocNoSave:"保存はしません（入れたあとに「保存する」）。", ocApply:"入れる", ocApplyClose:"入れずに閉じる",
  ocApplied:(nw,w)=>"入れました（オフセット "+nw+"・"+w+"）。まだ保存していません",
  ocSave:"保存する", ocSaveClose:"保存せずに閉じる", ocSaved:(nw,w)=>"保存しました（オフセット "+nw+"・"+w+"）",
  ocFailed:(s)=>"測れませんでした（"+s+"）", ocChkTip:"確かめのチェックを入れると押せます",
  ocPrep:"基板の状態を読んでいます…", ocLeft:(s)=>"残り 約 "+s+" 秒", ocSoon:"まもなく終わります",
  ocSaving:(x)=>"保存しています: "+x, ocSaveAsked:"保存を頼みました",
  ocCorrK:(nm)=>"補正 "+nm,
  ocErr:(wb,rb,ra,ma)=>"ずれ（電気角）：補正なし 幅 "+wb+"°・RMS "+rb+"° → 補正後に残る RMS "+ra+"°（最大 "+ma+"°）の目安",
  ocErrNow:(w,r)=>"（いまの補正のまま：幅 "+w+"°・RMS "+r+"°）",
  ocRun:(i)=>i+" 回目", ocMean:(d)=>"1 周の平均のずれ "+d, ocLag:(x)=>"摩擦の遅れ "+x+"°",
  ocWhatCorr:"補正 ", ocWhatZero:"補正 39〜42 は 0",
  ocgIdle:"ローター傾き補正を測ります（減速機付き）。オフセットは裸で測った値のまま、補正（39〜42）だけを入れます",
  ocgHow:(v,r,d)=>"通電して、選んだ側のストッパへゆっくり動かし（出力 約 "+v+"°/s）、当たったらローター "+r+" 回転（出力 "+d+"°）戻ります。そこから整列モードでストッパの側へローター 1 回転強引き回して戻ります（ストッパの手前 約 0.4〜1.5 回転の範囲）",
  ocgBare:(b)=>"基板のオフセット "+b+"（裸で測った値として、このまま使います）",
  ocgHowFree:"ストッパに当てずに、いまの位置から整列モードでローター 1 回転強ずつ両向きに引き回して戻ります（自由に回る減速機用）",
  ocgSide:"ストッパ", ocgPlus:"＋側", ocgMinus:"−側", ocgNone:"なし", ocgRoundS:(s)=>"（1 回 約 "+s+" 秒・回のあいだは休むだけ）",
  ocgChk:"ストッパの側へ回して大丈夫（途中に当たる物が無い・脚は付いていない）",
  ocgChkFree:"その場から両向きに回して大丈夫（途中に当たる物が無い・脚は付いていない）",
  ocgPrep:"通電して位置保持に入れています…", ocgSeek:(s)=>"ストッパを探しています（"+s+"へゆっくり）",
  ocgBack:(r)=>"ストッパから "+r+" 回転戻っています", ocgRest:(s)=>"次の回まで休んでいます（あと "+s+" 秒）",
  ocgHit:(p)=>"ストッパ：モーター "+p+" rad",
  ocgMean:(d)=>"裸のオフセットからの平均のずれ "+d+"（オフセットは入れません）",
  ocgApply:"補正だけ入れる（オフセットは裸の値のまま）",
  ocgApplied:(o,w)=>"入れました（オフセットは裸の値 "+o+" のまま・"+w+"）。まだ保存していません",
  ocgSaved:(o,w)=>"保存しました（オフセットは裸の値 "+o+" のまま・"+w+"）",
  stOcG:"ローター傾き補正を測定中",
  wcOpen:"巻線温度の校正", stWcRun:"巻線温度を校正中",
  wcCool:"モーターを室温まで冷ました状態で行ってください",
  wcHow:"「始める」を押すと電流を流して見かけの抵抗を測り、流し始めの抵抗と、電流を流し始めた瞬間の基板温度を、この軸の巻線温度の基準として覚えます（室温まで冷えていれば巻線と基板は同じ温度とみなせるため）。",
  wcTorque:"電流を流すとトルクが出ます。腕が当たってローターが回らない状態にしてください。",
  wcChkCold:"モーターは室温まで冷えています", wcChkArm:"腕が当たっていて、ローターは回りません",
  wcChkTip:"確かめのチェックを 2 つとも入れると押せます",
  wcCond:(sg,A,s)=>"条件："+sg+A+" A・"+s+" 秒（変えるときはここを開く）", wcAmps:"電流", wcSecs:"秒数", wcDir:"向き",
  wcStart:"始める", wcCancel:"中止", wcClose:"閉じる", wcAgain:"もう一度", wcRefNow:"いまの基準：",
  wcRunning:(sg,A,l,n)=>sg+A+" A を流しています（残り "+l+" s・記録 "+n+" 点）",
  wcFitNow:(r,e)=>"いまの当てはめ：流し始めの R "+r+" Ω（±"+e+" ℃）",
  wcKeep:"終わるまでそのままにしてください（中止・モードのボタン・解除・非常停止で止まり、何も覚えません）。",
  wcSaving:"当てはめて覚えています…",
  wcDone:(f)=>"覚えました（記録："+f+"）", wcDoneNoLog:"覚えました（生の点の記録は書けませんでした）",
  wcFailed:"覚えられませんでした",
  wcR:"流し始めの R", wcT:"流し始めの基板温度", wcFit:"当てはめ", wcFitV:(n,s)=>n+" 点・"+s+" 秒", wcSlope:"傾き",
  wcSe:"ばらつき", wcIq:"電流", wcPrev:"前の基準：",
  wcDiff:(dr,pc,dt)=>"前の基準との差：R "+dr+" Ω（"+pc+" %）・温度 "+dt+" ℃",
  tBoard:(v)=>"基板 "+v+" ℃", tWind:(v)=>"巻線(推定) "+v+" ℃", lgBoard:"基板", lgWind:"巻線(推定)", lgVel:"速度",
  twNoTlm:"テレメトリが来ていない", twLowIq:(a)=>"電流が流れていない（"+a+" A 以上で始まる）",
  twSpin:"回転中は推定できない（負荷をかけて押さえると始まる）", twReading:"電流を読み始めた…",
  twNoBoard:"基板温度が読めないので基準が取れない",
  twBasis:(n,m)=>"基準を取っています "+n+" / "+m+"（電流を流し続けてください）",
  twMismatch:(a,b)=>"基準（"+a+" A）といまの電流（"+b+" A）が違いすぎて推定できない（同じ強さで押さえるか、押す力を変えて ΔV を当てはめる）",
  twStopped:"電流が止まった（最後の推定を保持していません）",
  twRange:"電圧と電流の関係が異常（vq が読めていないか、電流ループが不安定）",
  twOther:(w)=>"推定できない（"+w+"）",
  twSettling:(n,m)=>"抵抗を落ち着かせています "+n+" / "+m+"（電流を流し続けてください）",
  twRefErr:"基準ファイルが読めない（起動後の最初の抵抗を基準にしています）",
  refFile:(r,c,d)=>"巻線温度の基準：冷えた状態で覚えた値（R "+r+" Ω・"+c+" ℃・"+d+"）",
  refFirst:(r,c)=>"巻線温度の基準：サーバを起動して最初に R が落ち着いたときの基板温度（R "+r+" Ω・"+c+" ℃）。そのとき巻線が温まっていたなら低く出ます。モーターを室温まで冷まして「巻線温度の校正」から基準を覚えてください",
  refNone:"巻線温度の基準：まだありません",
  wref:"巻線温度の基準", wrefVal:(r,c,d)=>"R "+r+" Ω・"+c+" ℃・"+d, wrefNone:"なし（起動後の最初の抵抗を使用）",
  wrefFit:(n,s,e)=>"（流し始めの R：当てはめ "+n+" 点・"+s+" 秒・±"+e+" ℃）",
  wrefHint:"（覚える・消すはモードの並びの「巻線温度の校正」から）",
  wrefClear:"基準を消す", wrefClearAsk:"もう一度押すと消します",
  wrefClearTip:"覚えた基準を消します。次に R が落ち着いたときの基板温度を基準にします（2 度押し）",
  unreach:"画面がサーバに届いていません。ラズパイと LAN を確かめてください。",
  tripping:"遮断しています", wdFired:"画面からの応答が途切れたので停止しました。",
  noAxes:"軸が見つかりません（CAN に返事をする基板が居ません）。基板の電源と CAN の配線を確かめてください。電源を入れた軸は、解除中なら自動で並びます（「再探索」でも並びます）。"
 },
 en:{
  title:"KRC Actuator", langTip:"Display language (Japanese / English)",
  rescan:"Rescan", rescanTip:"Search the bus for axes again (sends nothing)",
  disarmAll:"Disarm all", disarmAllTip:"Disarm every armed axis (arm each axis from its card)",
  clear:"Clear trip", estop:"E-STOP",
  loop:(m,s)=>"Loop "+m+" ms ± "+s, loopArmed:"\u2003Armed", loopDisarmed:"\u2003Disarmed", loopHeld:"\u2003Board holding",
  recOnHtml:"<span class='rdot blink'></span> REC <span class='secs'></span> s",
  recStartHtml:"<span class='rdot'></span> Start rec",
  recIdleHtml:"<span class='rdot'></span> Rec",
  recOnTip:"Stop recording (the motors keep running; not an E-stop)",
  recTip:"200 Hz recording (CSV). Listed on the leg page",
  recErr:(f,n,e)=>`Last recording ${f} stopped early (${n} rows written): ${e}`,
  axisH:(n)=>"Axis "+n, axisN:(n)=>"Axis "+n,
  nPos:"Position rad", nVel:"Velocity rad/s", nCur:"Current A", nTq:"Torque N·m",
  arm:"Arm", disarm:"Disarm",
  stTrip:"Tripped", stUnk:"Unknown", stArmed:"Armed", stIdle:"Idle", stOcRun:"Measuring offset", stOcPush:"Waiting for re-seat",
  stHeld:"Board holding",
  stJt:(nm)=>"Playing trajectory ("+nm+")", stJtGo:(nm)=>"To trajectory start ("+nm+")", stJtStop:(nm)=>"Stopping trajectory ("+nm+")",
  mode:{free:"Free",damp:"Damper",hold:"Hold",sine:"Sine",vel:"Velocity",torque:"Torque",kt_test:"Torque test",pos_test:"Position test",
        homing:"Homing",leg_sine:"Leg sine",leg_traj:"Leg traj"},
  pTarget:"Target rad", pAmp:"Amp rad", pFreq:"Freq Hz", pCenter:"Center rad", pVel:"Vel rad/s", pTq:"Torque N·m",
  posLocked:(w,r)=>"Can't enter position mode: "+w+(r?" ("+r+")":""),
  vzTip:(nm)=>"Minimum vertical range of the "+nm.toLowerCase()+" plot (up = wider, down = finer). Widens if the line goes outside",
  gPos:"Position", gTgt:"Target", gPosUnit:(r)=>"Position rad (min ±"+r+")", gCur:"Current A", gVel:(r)=>"Velocity rad/s (±"+r+")",
  gtPos:"Position [rad]", gtVel:"Velocity [rad/s]", gtCur:"Current [A]", gtTemp:"Temperature [°C]",
  gear:(r)=>"Gear "+r+":1",
  gearTip:(r)=>"Show and command the output shaft (after the "+r+":1 gearbox; position in °). Press again for the motor shaft. Kp, Kd, current and torque stay on the motor shaft",
  gtPosOut:"Output position [°]", gtVelOut:"Output velocity [rad/s]",
  pTargetOut:"Target °", pAmpOut:"Amp °", pCenterOut:"Center °", pVelOut:"Out rad/s", ptPtsOut:"Out °",
  kinTip:(nm,hi)=>nm+": type a number and press Enter (0.01 steps, 0 to "+hi+"). Motor-shaft value", kq:(v)=>"board "+v,
  kqTip:"Value that reaches the board after packing into 12 bits (truncated to scale ÷ 4095 steps)",
  degC:"°C",
  tauEst:(v,k)=>"Torque ≈ "+v+" N·m (calc. with Kt "+k+" N·m/A)",
  prm:"Board settings", prmTip:(nm,lo,hi,d)=>nm+" (range "+lo+" to "+hi+", default "+d+")",
  prmBoard:(v)=>"Board "+v, prmDiff:" (differs from sent)", prmDiffV:(v)=>" (sent "+v+")",
  ktCur:"Current A", ktMax:(a)=>"Max "+a, ktMaxTip:"This axis's current limit (read back from the board)", ktDir:"Dir", ktFree:"Custom",
  ktGo:"Run", ktSecs:"Time", ktStop:"Stop", ktPlus:"+",
  ptPts:"Angles °", ptPtsTip:"Motor-shaft angles in degrees from origin 0. Visits them in order and repeats. Separate with commas (Space also separates in this box; E-stop is Esc or the red button)",
  ptStyle:"Motion", ptSmooth:"Smooth", ptStep:"Step",
  ptSmoothTip:"Moves point to point with the speed limit and smooth accel/decel, then stops (position and velocity sent together)",
  ptStepTip:"Switches the target to the next point at once (step response, velocity 0). Only the first approach is smooth",
  ptSpeed:"Speed", ptSpeedTip:"Speed limit (smooth motion and the first approach)", ptDw:"Dwell", ptDwTip:"Seconds to hold at each point (in Step: seconds until the next switch)",
  ptGo:"Start", ptStop:"Stop", ptHold:"Holding position",
  ptApproach:(d)=>"Approaching first point "+d+"°", ptMove:(k,n,d)=>"Point "+k+"/"+n+": moving to "+d+"°",
  ptDwellAt:(k,n,d,l)=>"Point "+k+"/"+n+": holding at "+d+"° ("+l+" s)", ptStepAt:(k,n,d,l)=>"Point "+k+"/"+n+": "+d+"° (next in "+l+" s)",
  ptStopping:"Stopping (holding in place)",
  ktDown:(sg,A,l)=>"Ramping to 0 A (from "+sg+A+" A, "+l+" s left)", ktOn:(sg,A,l)=>"Driving "+sg+A+" A ("+l+" s left)",
  ktNoMax:(n)=>"Axis "+n+": current limit not read yet", ktNeedNum:"Enter the current (A) as a number", needNum:"Enter a number",
  prmDropOne:(n,nm)=>"Axis "+n+" "+nm, prmDrop:(s)=>s+": not sent (the card was redrawn while you were typing). Enter it again",
  ocOpen:"Measure electrical offset",
  ocgOpen:"Rotor tilt correction",
  ocIdle:"Remove the arm/leg so the rotor can turn a full revolution (never with the leg attached)",
  ocChk:"Removed; the rotor can turn freely", ocRuns:"Runs", ocRunsU:"",
  ocRoundS:(s)=>"(about "+s+" s per run; you re-seat the rotor by hand between runs)",
  ocChoose:"When done, choose whether to also write the correction (params 39-42) or the offset only (both come from the same measurement).",
  ocStart:"Start", ocClose:"Close", ocCancel:"Cancel", ocNext:"Next", ocAgain:"Again",
  ocRunning:(k,s)=>"Run "+k+" in progress (about "+s+" s, do not touch)",
  ocKeepOpen:(n)=>n+" runs in total. Keep this page open (hiding or closing it stops the run).",
  ocPush:(k)=>"Re-seat the rotor by hand, then press Next (before run "+k+")",
  ocStats:(m,sd,sp)=>"Mean "+m+", spread "+sd+", range "+sp+"°", ocSdOne:"– (one run)",
  ocOff:(o,nw,d)=>"Offset "+o+" → "+nw+" ("+d+" counts)",
  ocApplying:"Writing (waiting for the board to read back)…",
  ocModeCorr:"Offset + correction (recommended)", ocModeOff:"Offset only (correction 39-42 reset to 0)",
  ocNoSave:"Not saved yet (press Save after writing).", ocApply:"Write", ocApplyClose:"Close without writing",
  ocApplied:(nw,w)=>"Written (offset "+nw+", "+w+"). Not saved yet",
  ocSave:"Save", ocSaveClose:"Close without saving", ocSaved:(nw,w)=>"Saved (offset "+nw+", "+w+")",
  ocFailed:(s)=>"Measurement failed ("+s+")", ocChkTip:"Tick the check box to enable",
  ocPrep:"Reading the board…", ocLeft:(s)=>"About "+s+" s left", ocSoon:"Almost done",
  ocSaving:(x)=>"Saving: "+x, ocSaveAsked:"save requested",
  ocCorrK:(nm)=>"Corr "+nm,
  ocErr:(wb,rb,ra,ma)=>"Error (electrical): uncorrected range "+wb+"°, RMS "+rb+"° → est. residual RMS "+ra+"° (max "+ma+"°) after correction",
  ocErrNow:(w,r)=>" (with the current correction: range "+w+"°, RMS "+r+"°)",
  ocRun:(i)=>"Run "+i, ocMean:(d)=>"mean error/rev "+d, ocLag:(x)=>"friction lag "+x+"°",
  ocWhatCorr:"corr ", ocWhatZero:"correction 39-42 = 0",
  ocgIdle:"Measures the rotor tilt correction (with the gearbox). The offset stays at the bare-motor value; only the correction (39-42) is written",
  ocgHow:(v,r,d)=>"Arms and moves slowly to the chosen stopper (output ~"+v+"°/s), backs off "+r+" rotor turns ("+d+"° output), then sweeps a bit over one rotor turn toward the stopper and back in align mode (about 0.4-1.5 turns before the stopper)",
  ocgBare:(b)=>"Board offset "+b+" (used as the bare-motor value, unchanged)",
  ocgHowFree:"Without touching a stopper, sweeps a bit over one rotor turn each way from where it is, in align mode, and back (for a freely turning gearbox)",
  ocgSide:"Stopper", ocgPlus:"+ side", ocgMinus:"- side", ocgNone:"None", ocgRoundS:(s)=>"(about "+s+" s per run; it just rests between runs)",
  ocgChk:"It may turn toward the stopper (nothing in the way, no leg)",
  ocgChkFree:"It may turn both ways from here (nothing in the way, no leg)",
  ocgPrep:"Arming and entering hold…", ocgSeek:(s)=>"Searching for the stopper (slowly to the "+s+")",
  ocgBack:(r)=>"Backing off "+r+" turns from the stopper", ocgRest:(s)=>"Resting before the next run ("+s+" s)",
  ocgHit:(p)=>"Stopper: motor "+p+" rad",
  ocgMean:(d)=>"Mean error from the bare offset "+d+" (offset not written)",
  ocgApply:"Write correction only (offset stays bare)",
  ocgApplied:(o,w)=>"Written (offset stays at bare "+o+", "+w+"). Not saved yet",
  ocgSaved:(o,w)=>"Saved (offset stays at bare "+o+", "+w+")",
  stOcG:"Measuring rotor tilt correction",
  wcOpen:"Winding temp calibration", stWcRun:"Calibrating winding",
  wcCool:"Run this with the motor cooled to room temperature.",
  wcHow:"Start drives a current and measures the apparent resistance, then stores the resistance at the start and the board temperature at the moment the current started as this axis's winding reference (at room temperature the winding and the board are taken to be at the same temperature).",
  wcTorque:"The current produces torque. Make sure the arm is blocked so the rotor can't turn.",
  wcChkCold:"The motor has cooled to room temperature", wcChkArm:"The arm is blocked; the rotor can't turn",
  wcChkTip:"Tick both check boxes to enable",
  wcCond:(sg,A,s)=>"Settings: "+sg+A+" A, "+s+" s (open to change)", wcAmps:"Current", wcSecs:"Time", wcDir:"Dir",
  wcStart:"Start", wcCancel:"Cancel", wcClose:"Close", wcAgain:"Again", wcRefNow:"Current reference: ",
  wcRunning:(sg,A,l,n)=>"Driving "+sg+A+" A ("+l+" s left, "+n+" points)",
  wcFitNow:(r,e)=>"Fit so far: R at start "+r+" Ω (±"+e+" °C)",
  wcKeep:"Leave it until it ends (Cancel, a mode button, Disarm or E-stop stops it and stores nothing).",
  wcSaving:"Fitting and storing…",
  wcDone:(f)=>"Stored (log: "+f+")", wcDoneNoLog:"Stored (couldn't write the raw points)",
  wcFailed:"Not stored",
  wcR:"R at start", wcT:"Board temp at start", wcFit:"Fit", wcFitV:(n,s)=>n+" pts, "+s+" s", wcSlope:"Slope",
  wcSe:"Uncertainty", wcIq:"Current", wcPrev:"Previous: ",
  wcDiff:(dr,pc,dt)=>"Change from previous: R "+dr+" Ω ("+pc+" %), temp "+dt+" °C",
  tBoard:(v)=>"Board "+v+" °C", tWind:(v)=>"Winding (est.) "+v+" °C", lgBoard:"Board", lgWind:"Winding (est.)", lgVel:"Velocity",
  twNoTlm:"no telemetry", twLowIq:(a)=>"no current (starts above "+a+" A)",
  twSpin:"can't estimate while spinning (hold it under load)", twReading:"reading current…",
  twNoBoard:"board temperature unreadable, no baseline",
  twBasis:(n,m)=>"taking baseline "+n+" / "+m+" (keep the current flowing)",
  twMismatch:(a,b)=>"baseline ("+a+" A) and current ("+b+" A) differ too much to estimate",
  twStopped:"current stopped (last estimate not kept)",
  twRange:"voltage/current relation abnormal (vq unreadable or unstable current loop)",
  twOther:(w)=>"can't estimate ("+w+")",
  twSettling:(n,m)=>"settling resistance "+n+" / "+m+" (keep the current flowing)",
  twRefErr:"reference file unreadable (using the first resistance after start-up)",
  refFile:(r,c,d)=>"Winding reference: stored cold value (R "+r+" Ω, "+c+" °C, "+d+")",
  refFirst:(r,c)=>"Winding reference: board temperature when R first settled after the server started (R "+r+" Ω, "+c+" °C). Reads low if the winding was warm then. Cool the motor to room temperature and store a reference with \"Winding temp calibration\"",
  refNone:"Winding reference: none yet",
  wref:"Winding reference", wrefVal:(r,c,d)=>"R "+r+" Ω, "+c+" °C, "+d, wrefNone:"none (using the first resistance after start-up)",
  wrefFit:(n,s,e)=>" (R at current start: fit of "+n+" pts, "+s+" s, ±"+e+" °C)",
  wrefHint:"(store / clear with \"Winding temp calibration\" next to the mode buttons)",
  wrefClear:"Clear reference", wrefClearAsk:"Press again to clear",
  wrefClearTip:"Clears the stored reference. The next settled R will use the board temperature (press twice)",
  unreach:"The page can't reach the server. Check the Pi and the LAN.",
  tripping:"tripped", wdFired:"Stopped: the page stopped responding.",
  noAxes:"No axes found (no board answers on CAN). Check board power and CAN wiring. Powered axes appear automatically while disarmed (or press Rescan).",
  w:{"基板":"Board","応答なし":"no reply","自己確認":"Self-test","読めない":"n/a","発生中":"active","磁石":"Magnet",
     "検出できず":"not detected","較正":"Calibration","あり":"OK","なし":"missing","巻線温度":"Winding","母線":"Bus",
     "ISR最大":"ISR max","エンコーダ異常":"Encoder errors","可動範囲":"Range","無効":"off","欠測":"Missed","異常":"fault"," 回":"",
     " 〜 ":" to ","・":", ","／":" / "},
  prmName:{imax:"Current limit",kt:"Kt (torque const.)",irated:"Rated current (thermal)",t_trip:"Thermal trip",t_clear:"Thermal clear",
     t_tau:"Thermal time const.",timeout:"Command timeout",vbus:"Bus voltage",lim_kp:"Wall stiffness",lim_kd:"Wall damping",
     harm:"Harmonic cancel",harm_mu:"Its learning rate",cur_kp:"Current PI Kp",cur_ki:"Current PI Ki",decoup:"Decoupling",
     vmax_frac:"Output voltage limit",hi_duty:"Duty to distrust current",extrap:"Max command extrapolation",
     acc_max:"Extrapolation accel. limit",harm_max:"Harmonic coeff. limit",harm_min:"Min speed for harmonic learning",
     l_uh:"Inductance",ke:"Back-EMF constant",ntc_b:"Thermistor B",align_mv:"Auto-align voltage",t_amb:"Thermal ambient",
     t_rise:"Rise at 100 %",t_tref:"Resistance ref. temp",lost_hold:"Hold after loss",lost_rel:"Then release after"},
  prmUnit:{"比":"ratio","カウント":"counts"},
  prmSave:{soon:"save pending",wait:"armed (saves when disarmed)",coast:"moving (saves when stopped)",
     tripped:"tripped (saves after clearing the trip)",saving:"saving…",saved:"saved to board",
     failed:"not saved (flash count did not increase)",unknown:"can't verify the save",few:"can't verify the save yet",
     unread:"can't verify the save",reset:"can't verify the save (board restarted?)",
     differs:"not saved (board value differs from what was sent)",verify:"verifying the save…"}
 }
};
let LANG="ja";
try{ const q=new URLSearchParams(location.search).get("lang");
     if((q==="en"||q==="ja"?q:localStorage.getItem("krc.lang"))==="en") LANG="en"; }catch(e){}
function t(k){ const d=I18N[LANG], v=(k in d)?d[k]:I18N.ja[k];
  return (typeof v==="function")?v.apply(null,Array.prototype.slice.call(arguments,1)):v; }
function sv(ja,en){ return (LANG==="en"&&en)?en:ja; }
function tw(k){ const m=(LANG==="en")?I18N.en.w:null; return (m&&(k in m))?m[k]:k; }
function modeLabel(m){ return t("mode")[m]||MODE_LABEL[m]||m; }
function saveText(ja){
  if(LANG!=="en"||!ja) return ja;
  const T=st.param_save_text||{};
  for(const k in T) if(T[k]===ja&&I18N.en.prmSave[k]) return I18N.en.prmSave[k];
  return ja;
}
const BUTTON_MODES=["free","damp","hold","sine","vel","torque","kt_test","pos_test"];
const POS_MODES=["hold","sine","leg_sine","leg_traj","pos_test"];

const VIEW={p:{lo:0.1, hi:300, unit:"rad",  name:"gPos"},
            q:{lo:1, hi:3600, def:360, unit:"deg", name:"gPos"}};
const viewVal={};

const gearOn={};
const DEG=180/Math.PI;
function gearR(n){
  const g=lim.gear; if(!(g>0)) return null;
  if(!(n in gearOn)){ let v=false; try{ v=localStorage.getItem("krc.gear."+n)==="1"; }catch(e){} gearOn[n]=v; }
  return gearOn[n]?g:null;
}
function gearSet(n,on){ gearOn[n]=!!on; try{ localStorage.setItem("krc.gear."+n, on?"1":"0"); }catch(e){} render(); }
function gearK(n,key){
  const R=gearR(n); if(!R) return 1;
  if(key==="p"||key==="center"||key==="amp") return R/DEG;
  if(key==="v") return R;
  return 1;
}
function vzRad(k,v){ const V=VIEW[k]; return V.lo*Math.pow(V.hi/V.lo, v/1000); }
function vzVal(k,r){ const V=VIEW[k]; return Math.round(1000*Math.log(r/V.lo)/Math.log(V.hi/V.lo)); }
function viewKey(k,n){ return k==="p" ? "krc.posview."+n : "krc.view."+k+"."+n; }
const CUR_VIEW_A=9;
function viewOf(k,n){
  const id=k+n;
  if(!(id in viewVal)){
    const V=VIEW[k]; let r=(V.def!==undefined)?V.def:V.hi;
    try{ const x=parseFloat(localStorage.getItem(viewKey(k,n))); if(x>=V.lo&&x<=V.hi) r=x; }catch(e){}
    viewVal[id]=r;
  }
  return viewVal[id];
}
function vzHtml(k,n,cv){
  if(k==="p"&&gearR(n)) k="q";
  const V=VIEW[k];
  return "<div class='cvwrap'><input type='range' class='vzoom' data-vz='"+k+":"+n+"' min='0' max='1000' step='1' value='"
    +vzVal(k,viewOf(k,n))+"' title='"+esc(t("vzTip",t(V.name)))+"'>"
    +cv+"</div>";
}

const prmOpen={};

let prmDrop=null;
const PRM_DROP_MS=10000;
function prmDropAdd(ks){
  const now=Date.now(), old=(prmDrop&&now<prmDrop.until)?prmDrop.ks:[];
  prmDrop={ks:old.concat(ks.filter(k=>old.indexOf(k)<0)), until:now+PRM_DROP_MS};
}
function prmDropDel(k){
  if(!prmDrop) return;
  prmDrop.ks=prmDrop.ks.filter(x=>x!==k);
  if(!prmDrop.ks.length) prmDrop=null;
}
function prmDropText(){
  if(!prmDrop||Date.now()>=prmDrop.until) return "";
  const en=(LANG==="en"), defs=st.param_defs||[];
  const one=k=>{ const i=k.indexOf(":"), n=k.slice(0,i), key=k.slice(i+1), d=defs.find(x=>x.key===key);
                 return t("prmDropOne",n,d?((en&&I18N.en.prmName[d.key])||d.name):({kp:"Kp",kd:"Kd"}[key]||key)); };
  return t("prmDrop",prmDrop.ks.map(one).join(en?", ":"・"));
}
function prmDigits(lsb){ return (lsb>0&&isFinite(lsb)) ? Math.min(6,Math.max(0,-Math.floor(Math.log10(lsb)+1e-9))) : 0; }
function prmHtml(a){
  const defs=st.param_defs||[];
  let h="<details class='prm' data-prm='"+a.node+"'"+(prmOpen[a.node]?" open":"")+"><summary>"+t("prm")
    +"<span class='ps' data-ps='"+a.node+"'></span></summary>";
  const en=(LANG==="en");
  for(const d of defs){
    const nm=(en&&I18N.en.prmName[d.key])||d.name, un=(en&&I18N.en.prmUnit[d.unit])||d.unit;
    h+="<div class='prow'><label title='"+esc(t("prmTip",nm,d.lo,d.hi,d.d))+"'>"+esc(nm)+"</label>"
      +"<input type='number' data-pk='"+d.key+"' data-pn='"+a.node+"' step='"+d.lsb+"' min='"+d.lo+"' max='"+d.hi+"'>"
      +"<span class='pu'>"+esc(un)+"</span><span class='pv' data-pv='"+a.node+"-"+d.key+"'></span></div>";
  }
  h+="<div class='wref'><span class='wl'>"+esc(t("wref"))+"</span><span class='wv' data-wrv='"+a.node+"'>–</span>"
    +"<span class='wh'>"+esc(t("wrefHint"))+"</span></div>";
  return h+"</details>";
}

function wrefTx(r){
  if(!r) return t("wrefNone");
  const f=r.fit, fs=(f&&has(f.n)&&has(f.secs)&&has(f.se_c))?t("wrefFit",f.n,f.secs.toFixed(1),f.se_c.toFixed(1)):"";
  return t("wrefVal",num(r.r,3),num(r.t,1),r.date||"–")+fs;
}
function wrefUpdate(a){ setText(document.querySelector("[data-wrv='"+a.node+"']"), wrefTx((a.wcal||{}).ref)); }

const WREF_ASK_MS=4000;
const wrefAsk={};
function wrefClear(n){
  if(wrefAsk[n]&&Date.now()<wrefAsk[n]){ delete wrefAsk[n]; post("wind_ref_clear",{node:n}).then(()=>pollTemps()); }
  else wrefAsk[n]=Date.now()+WREF_ASK_MS;
  render();
}
function tempRefTip(tp){
  const r0=has(tp.r0)?tp.r0.toFixed(3):"–", c0=has(tp.t0)?tp.t0.toFixed(1):"–";
  let s=(tp.ref_src==="file")?t("refFile",r0,c0,(tp.ref&&tp.ref.date)||"–")
       :(tp.ref_src==="first")?t("refFirst",r0,c0):t("refNone");
  if(has(tp.ref_err)) s+="\n"+sv(tp.ref_err,tp.ref_err_en);
  return s;
}

const PRM_WARN={unknown:1, few:1, unread:1, reset:1, tripped:1};
const PRM_BAD={failed:1, differs:1};
function prmUpdate(a){
  const ps=document.querySelector("[data-ps='"+a.node+"']");
  if(ps){ const tx=a.param_save?((LANG==="en"&&I18N.en.prmSave[a.param_save])||(st.param_save_text||{})[a.param_save]||a.param_save):"";
          if(ps.textContent!==tx) ps.textContent=tx;
          const c="ps"+(PRM_BAD[a.param_save]?" bad":(PRM_WARN[a.param_save]?" warn":"")); if(ps.className!==c) ps.className=c; }
  for(const d of (st.param_defs||[])){
    const pv=(a.params||{})[d.key]||{};
    const dg=prmDigits(d.lsb);
    const e=document.querySelector("[data-pv='"+a.node+"-"+d.key+"']");
    const ms=(pv.sent===null||pv.sent===undefined)?t("prmDiff"):t("prmDiffV",(+pv.sent).toFixed(dg));
    if(e){ const tx=(pv.v===null||pv.v===undefined)?t("prmBoard","–"):(t("prmBoard",(+pv.v).toFixed(dg))+(pv.miss?ms:""));
           if(e.textContent!==tx) e.textContent=tx;
           const c="pv"+(pv.miss?" miss":""); if(e.className!==c) e.className=c; }
    const inp=document.querySelector("input[data-pk='"+d.key+"'][data-pn='"+a.node+"']");
    if(inp&&document.activeElement!==inp&&pv.v!==null&&pv.v!==undefined){
      const bt=(+pv.v).toFixed(dg);
      if(inp.value===""||(inp._auto&&inp.value!==bt)){ inp.value=bt; inp._auto=true; }
    }
  }
}
function fmtRad(r){ return r>=10?r.toFixed(0):(r>=1?r.toFixed(1):r.toFixed(2)); }

const TOK={};
function tok(n){
  if(!(n in TOK)) TOK[n]=(typeof getComputedStyle==="function")?getComputedStyle(document.documentElement).getPropertyValue(n).trim():"";
  return TOK[n];
}
let st={}, lim={}, seq=0, hist={}, builtKey="", offline=0, dragging=null;
let temps={};

let lastOk=Date.now();
function wdMs(){ return (st.wd_s||0)*1000; }
function unreachable(){ return offline>3 || (wdMs()>0 && Date.now()-lastOk>wdMs()); }

function wdSignal(){ return wdMs()>0 ? AbortSignal.timeout(wdMs()) : undefined; }

const inflight={};

function setText(e,s){ if(e&&e.dataset.t!==s){ e.dataset.t=s; e.textContent=s; } }
function setHtml(e,h){ if(e&&e.dataset.h!==h){ e.dataset.h=h; e.innerHTML=h; } }

let flash=null, lastNoteSeq=-1;

const CID=Math.random().toString(36).slice(2)+Date.now().toString(36);


function has(v){ return v!==null&&v!==undefined; }
function num(v,d){ return has(v)?v.toFixed(d):"–"; }
function esc(x){ return String(x).replace(/&/g,"&amp;").replace(/</g,"&lt;")
                                 .replace(/>/g,"&gt;").replace(/"/g,"&quot;"); }
function chip(k,v,c){ if(hcSink) hcSink.push({k:k, v:v, c:c});
                      return "<span class='chip"+(c?" "+c:"")+"'>"+esc(tw(k))+" "
                             +esc(v)+"</span>"; }

async function post(action,params){
  try{
    const r=await fetch("/api/cmd",{method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({action:action,
                           params:Object.assign({cid:CID},params||{})})});
    const j=await r.json();
    if(j && j.ok===false && j.err) showInfo(sv(j.err,j.err_en));
    poll();
  }catch(e){   }
}

function showInfo(msg){
  flash={msg:msg, cls:"warn", until:Date.now()+6000};
  banners();
}

function niceStep(range,n){
  const raw=range/Math.max(1,n), e=Math.pow(10,Math.floor(Math.log10(raw))), f=raw/e;
  return (f<=1?1:f<=2?2:f<=5?5:10)*e;
}
function tickText(v,step){
  const d=Math.max(0,-Math.floor(Math.log10(step)+1e-9));
  const t=(Math.abs(v)<step*1e-6?0:v).toFixed(d);
  return t;
}

const CHART_FONT="12px sans-serif", CHART_LINE_W=2.4, CHART_GRID_A=0.9;

function strokeSeries(g,pts,get,X,Y,lo,hi,over){
  const clip=(lo!==undefined&&hi!==undefined);
  const put=(o)=>{ const L=over[over.length-1]; if(L&&L.side===o.side&&L.x1===o.x0) L.x1=o.x1; else over.push(o); };
  g.beginPath(); let pen=false, pv=null;
  const edge=(a,b,bd)=>a.x+(b.x-a.x)*(bd-a.v)/(b.v-a.v);
  for(const p of pts){
    const v=get(p);
    if(v===null||v===undefined){ pen=false; pv=null; continue; }
    const q={x:X(p[0]), v:v, i:!clip||(v>=lo&&v<=hi)};
    if(q.i){
      if(pv&&!pv.i){ const bd=pv.v>hi?hi:lo, xc=edge(pv,q,bd); g.moveTo(xc,Y(bd)); g.lineTo(q.x,Y(v));
                     put({side:bd===hi?1:-1, x0:pv.x, x1:xc}); }
      else if(!pen) g.moveTo(q.x,Y(v)); else g.lineTo(q.x,Y(v));
      pen=true;
    }else{
      const side=v>hi?1:-1;
      if(pv&&pv.i){ const bd=side>0?hi:lo, xc=edge(pv,q,bd); g.lineTo(xc,Y(bd)); put({side:side, x0:xc, x1:q.x}); }
      else if(pv&&!pv.i&&(pv.v>hi?1:-1)===side) put({side:side, x0:pv.x, x1:q.x});
      else put({side:side, x0:q.x, x1:q.x});
      pen=false;
    }
    pv=q;
  }
  g.stroke();
}
function strokeOver(g,over,color,lw,h){
  if(!over.length) return;
  const yt=lw/2+0.5, yb=h-lw/2-0.5;
  g.strokeStyle=color; g.lineWidth=lw; g.setLineDash([4,3]); g.beginPath();
  for(const o of over){ const y=o.side>0?yt:yb; g.moveTo(o.x0,y); g.lineTo(Math.max(o.x1,o.x0+1),y); }
  g.stroke(); g.setLineDash([]);
}
function draw(cv,series,ymin,ymax,zero,opt){
  if(!cv) return;
  const dpr=window.devicePixelRatio||1;
  const w=cv.clientWidth, h=cv.clientHeight;
  if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)){
    cv.width=Math.round(w*dpr); cv.height=Math.round(h*dpr);
  }
  const g=cv.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);
  const span=10, y0=ymin, y1=(ymax===ymin)?ymin+1:ymax;
  const X=t=>w*(1+t/span), Y=v=>h*(1-(v-y0)/(y1-y0));
  opt=opt||{};
  g.font=CHART_FONT; g.lineWidth=1;
  g.strokeStyle=tok("--line"); g.fillStyle=tok("--dim"); g.globalAlpha=CHART_GRID_A;
  g.textAlign="center"; g.textBaseline="bottom";
  for(let t=-2; t>-span; t-=2){
    const x=X(t);
    g.beginPath(); g.moveTo(x,0); g.lineTo(x,h); g.stroke();
    g.globalAlpha=1; g.fillText(t+"s",x,h-2); g.globalAlpha=CHART_GRID_A;
  }
  g.globalAlpha=1;
  let step=niceStep(y1-y0,h<80?2:4);
  if(step>0.35*(y1-y0)){ const e=Math.pow(10,Math.floor(Math.log10(step)+1e-9)), f=Math.round(step/e);
    step=(f===1?0.5:f===2?1:2)*e; }
  g.textBaseline="middle";
  for(let v=Math.ceil(y0/step)*step; v<=y1+1e-9; v+=step){
    const y=Y(v);
    if(y<18||y>h-14) continue;
    g.strokeStyle=tok("--line"); g.globalAlpha=CHART_GRID_A;
    g.beginPath(); g.moveTo(0,y); g.lineTo(w,y); g.stroke(); g.globalAlpha=1;
    g.fillStyle=tok("--dim"); g.textAlign="left";
    g.fillText(tickText(v,step),3,y);
    if(opt.right){
      g.fillStyle=opt.right.color||tok("--dim"); g.textAlign="right";
      g.fillText(tickText(v*opt.right.k,step*opt.right.k),w-3,y);
    }
  }
  g.strokeStyle=tok("--dim"); g.lineWidth=1.2;
  if(zero){ g.beginPath(); g.moveTo(0,Y(0)); g.lineTo(w,Y(0)); g.stroke(); }
  const overs=[];
  for(const s of series){
    g.strokeStyle=s.color; g.lineWidth=s.w||CHART_LINE_W;
    if(s.dash) g.setLineDash([4,3]); else g.setLineDash([]);
    const over=[];
    strokeSeries(g,s.pts,s.get,X,Y,s.lim?s.lim[0]:undefined,s.lim?s.lim[1]:undefined,over);
    if(over.length) overs.push({over:over, color:s.color, w:s.w||CHART_LINE_W});
  }
  g.setLineDash([]);
  g.textBaseline="top"; g.textAlign="left";
  const back=(x,wd)=>{ g.globalAlpha=.85; g.fillStyle=tok("--panel"); g.fillRect(x,1,wd,17); g.globalAlpha=1; };
  if(opt.unit){ back(1,g.measureText(opt.unit).width+5); g.fillStyle=tok("--dim"); g.fillText(opt.unit,3,3); }
  let xr=w-3;
  for(const L of (opt.legend||[]).slice().reverse()){
    const tw=g.measureText(L.label).width;
    back(xr-tw-21,tw+23);
    g.textAlign="right"; g.fillStyle=L.color; g.fillText(L.label,xr,3);
    g.strokeStyle=L.color; g.lineWidth=CHART_LINE_W; g.setLineDash(L.dash?[4,3]:[]);
    g.beginPath(); g.moveTo(xr-tw-18,10); g.lineTo(xr-tw-4,10); g.stroke();
    g.setLineDash([]);
    xr-=tw+26;
  }
  for(const o of overs) strokeOver(g,o.over,o.color,o.w,h);
  g.textAlign="left"; g.textBaseline="alphabetic";
}


const ktSign={}, ktSecs={}, ktFree={};
function ktImax(a){ return (has(a.t_max)&&has(a.kt)&&a.kt>0&&a.t_max>0)?a.t_max/a.kt:null; }
function ktSecsDef(){ const s=lim.kt_secs; return (s&&has(s[0]))?s[0]:""; }
function ktRows(a){
  const n=a.node, im=ktImax(a), sg=(ktSign[n]===-1)?-1:1;
  let h="<div class='row ktrow'><label>"+t("ktCur")+"</label>";
  for(const A of [1,2,3,4,5,6]) if(im!==null&&A<=im+1e-9) h+="<button data-kta='"+n+"' data-amps='"+A+"'>"+A+"</button>";
  if(im!==null) h+="<button data-kta='"+n+"' data-amps='max' title='"+esc(t("ktMaxTip"))+"'>"+t("ktMax",Math.floor(im*100+1e-9)/100)+"</button>";
  h+="</div>";
  h+="<div class='row ktrow'><label>"+t("ktDir")+"</label>"
    +"<button data-ktsg='"+n+"' data-sg='1'"+(sg>0?" class='on'":"")+">"+t("ktPlus")+"</button>"
    +"<button data-ktsg='"+n+"' data-sg='-1'"+(sg<0?" class='on'":"")+">−</button></div>";
  h+="<div class='row ktrow'><label>"+t("ktFree")+"</label>"
    +"<input type='number' data-ktfree='"+n+"' min='0' step='0.1' placeholder='A' value='"+esc(has(ktFree[n])?ktFree[n]:"")+"'>"
    +"<span class='ku'>A</span><button data-ktgo='"+n+"'>"+t("ktGo")+"</button></div>";
  h+="<div class='row ktrow'><label>"+t("ktSecs")+"</label>"
    +"<input type='number' data-ktsecs='"+n+"' step='0.5' value='"+esc(has(ktSecs[n])?ktSecs[n]:ktSecsDef())+"'>"
    +"<span class='ku'>s</span><button data-ktstop='"+n+"'>"+t("ktStop")+"</button></div>";
  h+="<div class='row ktrow'><label></label><span class='ktst' data-ktst='"+n+"'></span></div>";
  return h;
}
function ktSend(n,mag){
  const sg=(ktSign[n]===-1)?-1:1;
  let s=parseFloat(has(ktSecs[n])?ktSecs[n]:ktSecsDef());
  const p={node:n, amps:sg*mag};
  if(isFinite(s)) p.secs=s;
  post("tq_pulse",p);
}
function ktUpdate(a){
  const n=a.node, p=a.kt_pulse, on=!!(p&&p.on);
  const e=document.querySelector("[data-ktst='"+n+"']");
  if(e){
    let tx="";
    if(on){
      const A=has(p.amps)?Math.abs(p.amps).toFixed(1):"–", sg=(has(p.amps)&&p.amps<0)?"−":"";
      tx=(p.phase==="down") ? t("ktDown",sg,A,num(p.left,1)) : t("ktOn",sg,A,num(p.left,1));
    }else if(a.kt_why) tx=sv(a.kt_why,a.kt_why_en);
    setText(e,tx);
    const cl="ktst"+(on?" on":""); if(e.className!==cl) e.className=cl;
  }
  const why=sv(a.kt_why||"",a.kt_why_en);
  for(const sel of ["button[data-kta='"+n+"']","button[data-ktgo='"+n+"']"])
    for(const b of document.querySelectorAll(sel)){
      if(b.disabled!==!!why) b.disabled=!!why;
      if(b.title!==why&&b.dataset.amps!=="max") b.title=why;
    }
  const sb=document.querySelector("button[data-ktstop='"+n+"']");
  if(sb&&sb.disabled!==!on) sb.disabled=!on;
}


const ptPts={}, ptStyle={}, ptSpeed={}, ptDwell={};
function ptDef(k){ const p=lim.pt||{}; return has(p[k])?p[k]:""; }
function ptVal(m,n,k){ return has(m[n])?m[n]:ptDef(k); }
function ptRows(a){
  const n=a.node, step=(ptStyle[n]==="step");
  let h="<div class='row ktrow'><label>"+(gearR(n)?t("ptPtsOut"):t("ptPts"))+"</label>"
    +"<input type='text' class='ptpts' data-ptpts='"+n+"' spellcheck='false' autocomplete='off' placeholder='"+esc(ptDef("pts"))
    +"' title='"+esc(t("ptPtsTip"))+"' value='"+esc(ptVal(ptPts,n,"pts"))+"'></div>";
  h+="<div class='row ktrow'><label>"+t("ptStyle")+"</label>"
    +"<button data-ptsty='"+n+"' data-v='smooth' title='"+esc(t("ptSmoothTip"))+"'"+(step?"":" class='on'")+">"+t("ptSmooth")+"</button>"
    +"<button data-ptsty='"+n+"' data-v='step' title='"+esc(t("ptStepTip"))+"'"+(step?" class='on'":"")+">"+t("ptStep")+"</button></div>";
  h+="<div class='row ktrow'><label title='"+esc(t("ptSpeedTip"))+"'>"+t("ptSpeed")+"</label>"
    +"<input type='number' data-ptv='"+n+"' min='1' step='10' title='"+esc(t("ptSpeedTip"))+"' value='"+esc(ptVal(ptSpeed,n,"speed"))+"'>"
    +"<span class='ku'>°/s</span>"
    +"<label class='ptl' title='"+esc(t("ptDwTip"))+"'>"+t("ptDw")+"</label>"
    +"<input type='number' data-ptd='"+n+"' min='0' step='0.1' title='"+esc(t("ptDwTip"))+"' value='"+esc(ptVal(ptDwell,n,"dwell"))+"'>"
    +"<span class='ku'>s</span></div>";
  h+="<div class='row ktrow'><label></label><button class='pri' data-ptgo='"+n+"'>"+t("ptGo")+"</button>"
    +"<button data-ptstop='"+n+"'>"+t("ptStop")+"</button></div>";
  h+="<div class='row ktrow'><label></label><span class='ktst' data-ptst='"+n+"'></span></div>";
  return h;
}
function ptDeg(d){ return has(d)?String(Math.round(d*10)/10):"–"; }
function ptUpdate(a){
  const n=a.node, p=a.pt, on=!!(p&&p.on);
  let tx="";
  if(on){
    if(p.stage==="stop") tx=t("ptStopping");
    else if(p.stage==="approach") tx=t("ptApproach",ptDeg(p.deg));
    else if(p.stage==="move") tx=t("ptMove",p.k,p.n,ptDeg(p.deg));
    else tx=t(p.step?"ptStepAt":"ptDwellAt",p.k,p.n,ptDeg(p.deg),num(p.left,1));
  }else tx=a.pt_why?sv(a.pt_why,a.pt_why_en):(a.armed?t("ptHold"):"");
  const e=document.querySelector("[data-ptst='"+n+"']");
  if(e){ setText(e,tx); const cl="ktst"+(on?" on":""); if(e.className!==cl) e.className=cl; }
  const why=sv(a.pt_why||"",a.pt_why_en);
  const go=document.querySelector("button[data-ptgo='"+n+"']");
  if(go){ const d=!!why||on; if(go.disabled!==d) go.disabled=d; if(go.title!==why) go.title=why; }
  const sb=document.querySelector("button[data-ptstop='"+n+"']");
  if(sb&&sb.disabled!==!on) sb.disabled=!on;
  for(const el of document.querySelectorAll("[data-ptpts='"+n+"'],[data-ptv='"+n+"'],[data-ptd='"+n+"'],button[data-ptsty='"+n+"']"))
    if(el.disabled!==on) el.disabled=on;
  for(const b of document.querySelectorAll("button[data-ptsty='"+n+"']")){
    const c=((ptStyle[n]==="step")===(b.dataset.v==="step"))?"on":""; if(b.className!==c) b.className=c; }
}
function ptSend(n){
  const sp=parseFloat(ptVal(ptSpeed,n,"speed")), dw=parseFloat(ptVal(ptDwell,n,"dwell"));
  if(!isFinite(sp)||!isFinite(dw)){ showInfo(t("needNum")); return; }
  post("pt_start",{node:n, pts:String(ptVal(ptPts,n,"pts")), style:(ptStyle[n]==="step")?"step":"smooth", speed:sp, dwell:dw,
                   ratio:gearR(n)||1});
}


const ocChk={}, ocN={}, ocMode={};
const OC_CORR=["cos θ","sin θ","cos 2θ","sin 2θ"];
function ocNLim(i){ const s=lim.offcal_n; return (s&&has(s[i]))?s[i]:""; }
function ocDeg(v){ return has(v)?((v>=0?"+":"")+v.toFixed(2)+"°"):"–"; }
function ocC(v){ return has(v)?((v>=0?"+":"")+(v*0.1).toFixed(1)+"°"):"–"; }
function ocModeOf(n,o){ return (o.kind==="gear")?"gear":(ocMode[n]||(o.corr_why?"off":"corr")); }
const ocSide={};
function ocSd(n){ return (ocSide[n]===-1||ocSide[n]===0)?ocSide[n]:1; }
function ocgNums(){ const g=lim.ocg||[]; return [has(g[0])?g[0]:"–", has(g[1])?g[1]:"–", has(g[2])?g[2]:"–"]; }
function ocgBareTx(o){ return t("ocgBare",has(o.board_off)?o.board_off:"–"); }
function ocCorr(o){
  const a=o.corr_now||[], b=o.corr_new||[];
  if(!b.length) return "";
  let h="<table>";
  OC_CORR.forEach((nm,i)=>{ h+="<tr><td class='k'>"+t("ocCorrK",nm)+"</td><td>"+ocC(a[i])+" → "+ocC(b[i])+"</td></tr>"; });
  h+="</table><div class='ocsub'>"+t("ocErr",num(o.width_before,1),num(o.rms_before,1),num(o.rms_after,1),num(o.max_after,1))
    +(a.some(x=>x)?t("ocErrNow",num(o.width_now,1),num(o.rms_now,1)):"")+"</div>";
  return h;
}
function ocVals(o){
  const v=o.vals||[];
  if(!v.length) return "";
  let h="<table>";
  v.forEach((x,i)=>{ h+="<tr><td class='k'>"+t("ocRun",i+1)+"</td><td>"+t("ocMean",ocDeg(x[0]))+"</td>"
    +"<td class='k'>"+t("ocLag",num(x[1],2))+"</td></tr>"; });
  return h+"</table>";
}
function ocWhat(o){
  const b=o.corr_new||[];
  return (o.mode==="corr"||o.mode==="gear")?t("ocWhatCorr")+OC_CORR.map((nm,i)=>nm+" "+ocC(b[i])).join(" / "):t("ocWhatZero");
}

const tabSel={};
function tabBusy(a,k){ const g=(k==="oc"?a.offcal:a.wcal)||{}; return has(g.stage)&&g.stage!=="idle"; }
function tabOf(a){
  const sel=tabSel[a.node];
  if(sel!==undefined) return sel||null;
  return tabBusy(a,"oc")?"oc":(tabBusy(a,"wc")?"wc":null);
}

function tabSet(n,k){
  const a=(st.axes||[]).find(x=>x.node===n)||{node:n};
  if(k!=="oc"&&tabBusy(a,"oc")) post("offcal_cancel",{node:n});
  if(k!=="wc"&&tabBusy(a,"wc")) post("wcal_cancel",{node:n});
  tabSel[n]=k;
}
function ocHtml(a){
  const n=a.node, o=a.offcal||{}, s=o.stage||"idle";
  if(tabOf(a)!=="oc") return "";
  const msg=(t,act)=>"<div class='ocmsg"+(act?" act":"")+"'>"+esc(t)+"</div>";
  const btn=(at,label,cls)=>"<button "+at+"='"+n+"'"+(cls?" class='"+cls+"'":"")+">"+label+"</button>";
  const why="<div class='warnline' data-ocwhy='"+n+"'></div>";
  const newOff=has(o.new_off)?o.new_off:"–";
  const G=(o.kind==="gear")||(s==="idle"&&!!gearR(n));
  let h="<div class='ocsub'>"+(G?t("ocgOpen"):t("ocOpen"))+"</div>";
  const N=ocgNums(), sideTx=(o.side<0||(s==="idle"&&ocSd(n)===-1))?t("ocgMinus"):t("ocgPlus");
  if(G&&s==="idle"){
    const sd=ocSd(n);
    h+=msg(t("ocgIdle"),true);
    h+="<div class='ocsub'>"+esc(sd===0?t("ocgHowFree"):t("ocgHow",N[0],N[1],N[2]))+"</div>";
    h+="<div class='ocsub' data-ocbare='"+n+"'></div>";
    h+="<div class='row'><label class='occhk'>"+t("ocgSide")+"</label>"
      +"<button data-ocside='"+n+"' data-sg='1'"+(sd>0?" class='on'":"")+">"+t("ocgPlus")+"</button>"
      +"<button data-ocside='"+n+"' data-sg='-1'"+(sd<0?" class='on'":"")+">"+t("ocgMinus")+"</button>"
      +"<button data-ocside='"+n+"' data-sg='0'"+(sd===0?" class='on'":"")+">"+t("ocgNone")+"</button></div>";
    h+="<div class='row'><label class='occhk'><input type='checkbox' data-occhk='"+n+"'> "+t(sd===0?"ocgChkFree":"ocgChk")+"</label></div>";
    h+="<div class='row'>"+t("ocRuns")+" <input type='number' data-ocn='"+n+"' min='"+ocNLim(1)+"' max='"+ocNLim(2)+"' step='1' value='"+esc(ocNLim(0))+"'> "+t("ocRunsU")
      +(has(o.round_s)?"<span class='ocsub'>"+t("ocgRoundS",Math.round(o.round_s))+"</span>":"")+"</div>";
    h+="<div class='row'>"+btn("data-ocgo",t("ocStart"),"pri")+btn("data-occlose",t("ocClose"))+"</div>"+why;
    return h;
  }
  if(G&&(s==="prep"||s==="seek"||s==="back")){
    h+=msg(s==="prep"?t("ocgPrep"):(s==="seek"?t("ocgSeek",sideTx):t("ocgBack",N[1])),true);
    if(has(o.hit)) h+="<div class='ocsub'>"+t("ocgHit",o.hit.toFixed(2))+"</div>";
    h+="<div class='ocsub'>"+t("ocKeepOpen",o.n)+"</div>";
    h+="<div class='row'>"+btn("data-occancel",t("ocCancel"))+"</div>";
    return h;
  }
  if(G&&s==="running"&&has(o.rest_s)){
    h+=msg(t("ocgRest",Math.ceil(o.rest_s)),false);
    h+=ocVals(o)+"<div class='row'>"+btn("data-occancel",t("ocCancel"))+"</div>";
    return h;
  }
  if(G&&(s==="done"||s==="applying")){
    h+=ocVals(o);
    h+="<div class='ocsub'>"+t("ocgMean",ocDeg(o.mean))+"</div>";
    h+=ocCorr(o);
    if(s==="applying") h+="<div class='ocsub'>"+t("ocApplying")+"</div>";
    else{
      if(o.corr_why) h+="<div class='warnline'>"+esc(sv(o.corr_why,o.corr_why_en))+"</div>";
      h+="<div class='ocsub'>"+t("ocNoSave")+"</div>";
      h+="<div class='row'>"+btn("data-ocapply",t("ocgApply"),"pri")+btn("data-occlose",t("ocApplyClose"))+"</div>";
      if(o.apply_err) h+="<div class='warnline'>"+esc(sv(o.apply_err,o.apply_err_en))+"</div>";
      h+=why;
    }
    return h;
  }
  if(G&&(s==="applied"||s==="saving"||s==="saved")){
    h+=msg(s==="saved"?t("ocgSaved",has(o.off)?o.off:"–",ocWhat(o)):t("ocgApplied",has(o.off)?o.off:"–",ocWhat(o)),false);
    if(s==="saving") h+="<div class='ocsub' data-ocsavetx='"+n+"'></div>";
    else if(s==="applied"){
      h+="<div class='row'>"+btn("data-ocsave",t("ocSave"),"pri")+btn("data-occlose",t("ocSaveClose"))+"</div>";
      if(o.save_err) h+="<div class='warnline'>"+esc(sv(saveText(o.save_err),o.save_err_en))+"</div>";
    }else h+="<div class='row'>"+btn("data-occlose",t("ocClose"))+"</div>";
    return h;
  }
  if(s==="idle"){
    h+=msg(t("ocIdle"),true);
    h+="<div class='row'><label class='occhk'><input type='checkbox' data-occhk='"+n+"'> "+t("ocChk")+"</label></div>";
    h+="<div class='row'>"+t("ocRuns")+" <input type='number' data-ocn='"+n+"' min='"+ocNLim(1)+"' max='"+ocNLim(2)+"' step='1' value='"+esc(ocNLim(0))+"'> "+t("ocRunsU")
      +(has(o.round_s)?"<span class='ocsub'>"+t("ocRoundS",Math.round(o.round_s))+"</span>":"")+"</div>";
    h+="<div class='ocsub'>"+t("ocChoose")+"</div>";
    h+="<div class='row'>"+btn("data-ocgo",t("ocStart"),"pri")+btn("data-occlose",t("ocClose"))+"</div>"+why;
  }else if(s==="running"){
    h+=msg(t("ocRunning",o.k,(has(o.est_s)?Math.round(o.est_s):"–")),false);
    h+="<div class='ocbar'><div data-ocbar='"+n+"'></div></div><div class='ocsub' data-ocleft='"+n+"'></div>";
    h+="<div class='ocsub'>"+t("ocKeepOpen",o.n)+"</div>";
    h+=ocVals(o)+"<div class='row'>"+btn("data-occancel",t("ocCancel"))+"</div>";
  }else if(s==="wait_push"){
    h+=msg(t("ocPush",o.k),true);
    h+=ocVals(o)+"<div class='row'>"+btn("data-ocnext",t("ocNext"),"pri")+btn("data-occancel",t("ocCancel"))+"</div>"+why;
  }else if(s==="done"||s==="applying"){
    h+=ocVals(o);
    h+="<div class='ocsub'>"+t("ocStats",ocDeg(o.mean),(has(o.sd)?o.sd.toFixed(2)+"°":t("ocSdOne")),num(o.span,2))+"</div>";
    h+=msg(t("ocOff",(has(o.off)?o.off:"–"),newOff,(has(o.d_off)?(o.d_off>=0?"+":"")+o.d_off.toFixed(1):"–")),false);
    h+=ocCorr(o);
    if(s==="applying") h+="<div class='ocsub'>"+t("ocApplying")+"</div>";
    else{
      const rd=(v,label,dis)=>"<label class='occhk'><input type='radio' name='ocmode"+n+"' data-ocmode='"+n+"' value='"+v+"'"
        +(dis?" disabled":"")+"> "+label+"</label>";
      h+="<div class='row'>"+rd("corr",t("ocModeCorr"),!!o.corr_why)+rd("off",t("ocModeOff"),false)+"</div>";
      if(o.corr_why) h+="<div class='warnline'>"+esc(sv(o.corr_why,o.corr_why_en))+"</div>";
      h+="<div class='ocsub'>"+t("ocNoSave")+"</div>";
      h+="<div class='row'>"+btn("data-ocapply",t("ocApply"),"pri")+btn("data-occlose",t("ocApplyClose"))+"</div>";
      if(o.apply_err) h+="<div class='warnline'>"+esc(sv(o.apply_err,o.apply_err_en))+"</div>";
      h+=why;
    }
  }else if(s==="applied"||s==="saving"){
    h+=msg(t("ocApplied",newOff,ocWhat(o)),false);
    if(s==="saving") h+="<div class='ocsub' data-ocsavetx='"+n+"'></div>";
    else{
      h+="<div class='row'>"+btn("data-ocsave",t("ocSave"),"pri")+btn("data-occlose",t("ocSaveClose"))+"</div>";
      if(o.save_err) h+="<div class='warnline'>"+esc(sv(saveText(o.save_err),o.save_err_en))+"</div>";
    }
  }else if(s==="saved"){
    h+=msg(t("ocSaved",newOff,ocWhat(o)),false);
    h+="<div class='row'>"+btn("data-occlose",t("ocClose"))+"</div>";
  }else{
    h+=ocVals(o)+msg(o.why?sv(o.why,o.why_en):t("ocFailed",s),true);
    h+="<div class='row'>"+btn("data-ocagain",t("ocAgain"))+btn("data-occlose",t("ocClose"))+"</div>";
  }
  return h;
}
function ocUpdate(a){
  const n=a.node, o=a.offcal||{}, s=o.stage||"idle";
  setHtml(document.querySelector("[data-oc='"+n+"']"), ocHtml(a));
  const w=(s==="idle")?sv(o.start_why||"",o.start_why_en):(s==="wait_push")?sv(o.next_why||"",o.next_why_en)
         :(s==="done")?sv(o.apply_why||"",o.apply_why_en):"";
  if(!(s==="idle"&&o.kind!=="bare"&&gearR(n))) setText(document.querySelector("[data-ocwhy='"+n+"']"), w);
  const cb=document.querySelector("input[data-occhk='"+n+"']");
  if(cb&&cb.checked!==!!ocChk[n]) cb.checked=!!ocChk[n];
  const ni=document.querySelector("input[data-ocn='"+n+"']");
  if(ni&&document.activeElement!==ni&&has(ocN[n])&&ni.value!==String(ocN[n])) ni.value=ocN[n];
  const dis=(sel,d,tt)=>{ const b=document.querySelector(sel); if(!b) return;
    if(b.disabled!==d) b.disabled=d; if(b.title!==tt) b.title=tt; };
  const G=(o.kind==="gear")||(s==="idle"&&!!gearR(n));
  if(G&&s==="idle"){
    const gq=(ocSd(n)===0)?[o.gear_why_free,o.gear_why_free_en]:[o.gear_why,o.gear_why_en];
    const gw=o.start_why?sv(o.start_why,o.start_why_en):(gq[0]?sv(gq[0],gq[1]):"");
    setText(document.querySelector("[data-ocwhy='"+n+"']"), gw);
    setText(document.querySelector("[data-ocbare='"+n+"']"), ocgBareTx(o));
    document.querySelectorAll("button[data-ocside='"+n+"']").forEach(b=>{
      const c=(ocSd(n)===+b.dataset.sg)?"on":""; if(b.className!==c) b.className=c; });
    dis("button[data-ocgo='"+n+"']", !!gw||!ocChk[n], gw||(ocChk[n]?"":t("ocChkTip")));
  }else
  dis("button[data-ocgo='"+n+"']", !!o.start_why||!ocChk[n], o.start_why?sv(o.start_why,o.start_why_en):(ocChk[n]?"":t("ocChkTip")));
  dis("button[data-ocnext='"+n+"']", !!o.next_why, sv(o.next_why||"",o.next_why_en));
  const md=ocModeOf(n,o), aw=o.apply_why?sv(o.apply_why,o.apply_why_en):(md==="corr"?sv(o.corr_why||"",o.corr_why_en):"");
  dis("button[data-ocapply='"+n+"']", !!aw, aw);
  document.querySelectorAll("input[data-ocmode='"+n+"']").forEach(r=>{ const c=(r.value===md); if(r.checked!==c) r.checked=c; });
  const bar=document.querySelector("[data-ocbar='"+n+"']");
  if(bar){ const wd=(has(o.frac)?Math.round(o.frac*1000)/10:0)+"%"; if(bar.style.width!==wd) bar.style.width=wd; }
  setText(document.querySelector("[data-ocleft='"+n+"']"),
    o.prep?t("ocPrep"):(has(o.left_s)?(o.left_s>0.5?t("ocLeft",Math.round(o.left_s)):t("ocSoon")):""));
  setText(document.querySelector("[data-ocsavetx='"+n+"']"), t("ocSaving",o.save_text?saveText(o.save_text):t("ocSaveAsked")));
}
function ocWire(){
  const box=document.getElementById("cards");
  box.addEventListener("click",e=>{
    const t=e.target, d=(t&&t.dataset)||{};
    const st0=n=>(((st.axes||[]).find(x=>x.node===n)||{}).offcal||{}).stage||"idle";
    if(d.ocopen){ tabSet(+d.ocopen,"oc"); render(); }
    else if(d.ocgo){ const n=+d.ocgo, v=parseInt(has(ocN[n])?ocN[n]:ocNLim(0),10), p={node:n, free:!!ocChk[n]};
      if(isFinite(v)) p.n=v;
      if(gearR(n)){ p.gear=true; p.side=ocSd(n); }
      post("offcal_start",p); }
    else if(d.ocside){ const g=+d.sg; ocSide[+d.ocside]=(g<0)?-1:(g===0?0:1); render(); }
    else if(d.ocnext) post("offcal_next",{node:+d.ocnext});
    else if(d.occancel) post("offcal_cancel",{node:+d.occancel});
    else if(d.ocapply){ const n=+d.ocapply; post("offcal_apply",{node:n, mode:ocModeOf(n,((st.axes||[]).find(x=>x.node===n)||{}).offcal||{})}); }
    else if(d.ocsave) post("offcal_save",{node:+d.ocsave});
    else if(d.occlose||d.ocagain){ const n=+(d.occlose||d.ocagain);
      if(d.ocagain){ if(st0(n)!=="idle") post("offcal_cancel",{node:n}); tabSel[n]="oc"; } else tabSet(n,"");
      ocChk[n]=false; delete ocMode[n]; render(); }
  });
  box.addEventListener("change",e=>{ const t=e.target, d=(t&&t.dataset)||{};
    if(d.occhk){ ocChk[+d.occhk]=!!t.checked; render(); }
    else if(d.ocmode&&t.checked){ ocMode[+d.ocmode]=t.value; render(); } });
  box.addEventListener("input",e=>{ const t=e.target, d=(t&&t.dataset)||{};
    if(d.ocn) ocN[+d.ocn]=t.value; });
}


const wcChk={}, wcAmps={}, wcSecs={}, wcSign={}, wcCondOpen={};
function wcLim(i){ const s=lim.wcal; return (s&&has(s[i]))?s[i]:""; }
function wcHtml(a){
  const n=a.node, o=a.wcal||{}, s=o.stage||"idle";
  if(tabOf(a)!=="wc") return "";
  const msg=(tx,act)=>"<div class='ocmsg"+(act?" act":"")+"'>"+esc(tx)+"</div>";
  const sub=(tx)=>"<div class='ocsub'>"+esc(tx)+"</div>";
  const btn=(at,label,cls)=>"<button "+at+"='"+n+"'"+(cls?" class='"+cls+"'":"")+">"+label+"</button>";
  let h="<div class='ocsub'>"+t("wcOpen")+"</div>";
  if(s==="idle"){
    h+=msg(t("wcCool"),true)+sub(t("wcHow"))+"<div class='wcwarn'>"+esc(t("wcTorque"))+"</div>";
    h+="<div class='row'><label class='occhk'><input type='checkbox' data-wcchk='"+n+"' data-k='c'> "+t("wcChkCold")+"</label></div>";
    h+="<div class='row'><label class='occhk'><input type='checkbox' data-wcchk='"+n+"' data-k='b'> "+t("wcChkArm")+"</label></div>";
    const im=ktImax(a);
    h+="<details class='wccond' data-wccond='"+n+"'"+(wcCondOpen[n]?" open":"")+"><summary data-wcsum='"+n+"'></summary>";
    h+="<div class='row'><label>"+t("wcAmps")+"</label><input type='number' data-wca='"+n+"' min='"+wcLim(1)+"'"
      +(im!==null?" max='"+(Math.floor(im*100+1e-9)/100)+"'":"")+" step='0.5' value='"+esc(wcLim(0))+"'> A</div>";
    h+="<div class='row'><label>"+t("wcSecs")+"</label><input type='number' data-wcs='"+n+"' min='"+wcLim(3)+"' max='"+wcLim(4)
      +"' step='0.5' value='"+esc(wcLim(2))+"'> s</div>";
    h+="<div class='row'><label>"+t("wcDir")+"</label>"+"<button data-wcsg='"+n+"' data-sg='1'>"+t("ktPlus")+"</button>"
      +"<button data-wcsg='"+n+"' data-sg='-1'>−</button></div></details>";
    h+="<div class='row'>"+btn("data-wcgo",t("wcStart"),"pri")+btn("data-wcclose",t("wcClose"))+"</div>";
    h+="<div class='warnline' data-wcwhy='"+n+"'></div>";
    h+="<div class='wcref'><span data-wcref='"+n+"'></span>"+btn("data-wcclr",t("wrefClear"),"ghost")+"</div>";
  }else if(s==="running"){
    h+="<div class='ocmsg' data-wcrun='"+n+"'></div>";
    h+="<div class='ocbar'><div data-wcbar='"+n+"'></div></div><div class='ocsub' data-wcfit='"+n+"'></div>";
    h+=sub(t("wcKeep"));
    if(!o.saving) h+="<div class='row'>"+btn("data-wccancel",t("wcCancel"))+"</div>";
  }else if(s==="done"){
    const r=o.res||{}, f=r.fit||{};
    h+=msg(f.log?t("wcDone",f.log):t("wcDoneNoLog"),false);
    if(!f.log&&o.msg) h+=sub(sv(o.msg,o.msg_en));
    const row=(k,v)=>"<tr><td class='k'>"+t(k)+"</td><td>"+esc(v)+"</td></tr>";
    h+="<table>"+row("wcR",num(r.r,3)+" Ω")+row("wcT",num(r.t,1)+" "+t("degC"))
      +row("wcFit",t("wcFitV",has(f.n)?f.n:"–",num(f.secs,1)))
      +row("wcSlope",(has(f.slope)?(f.slope*1000).toFixed(2):"–")+" mΩ/s")
      +row("wcSe","±"+num(f.se_c,1)+" "+t("degC"))+row("wcIq",num(r.iq,2)+" A")+"</table>";
    const pv=o.prev;
    h+=sub(t("wcPrev")+wrefTx(pv));
    if(pv&&has(pv.r)&&has(r.r)){
      const dr=r.r-pv.r, dt=(has(r.t)&&has(pv.t))?r.t-pv.t:null;
      h+=sub(t("wcDiff",(dr>=0?"+":"")+dr.toFixed(3),(dr>=0?"+":"")+(100*dr/pv.r).toFixed(1),
                     has(dt)?(dt>=0?"+":"")+dt.toFixed(1):"–"));
    }
    h+="<div class='row'>"+btn("data-wcclose",t("wcClose"))+"</div>";
  }else{
    h+=msg(t("wcFailed"),true)+"<div class='warnline'>"+esc(o.why?sv(o.why,o.why_en):s)+"</div>";
    h+="<div class='row'>"+btn("data-wcagain",t("wcAgain"))+btn("data-wcclose",t("wcClose"))+"</div>";
  }
  return h;
}
function wcUpdate(a){
  const n=a.node, o=a.wcal||{}, s=o.stage||"idle";
  setHtml(document.querySelector("[data-wc='"+n+"']"), wcHtml(a));
  if(tabOf(a)!=="wc") return;
  const dis=(sel,d,tt)=>{ const b=document.querySelector(sel); if(!b) return;
    if(b.disabled!==d) b.disabled=d; if(b.title!==tt) b.title=tt; };
  const sg=(wcSign[n]===-1)?-1:1;
  if(s==="idle"){
    document.querySelectorAll("input[data-wcchk='"+n+"']").forEach(cb=>{ const c=!!wcChk[n+cb.dataset.k]; if(cb.checked!==c) cb.checked=c; });
    for(const [sel,v] of [["input[data-wca='"+n+"']",wcAmps[n]],["input[data-wcs='"+n+"']",wcSecs[n]]]){
      const e=document.querySelector(sel);
      if(e&&document.activeElement!==e&&has(v)&&e.value!==String(v)) e.value=v; }
    document.querySelectorAll("button[data-wcsg='"+n+"']").forEach(b=>{ const c=(+b.dataset.sg===sg)?"on":""; if(b.className!==c) b.className=c; });
    setText(document.querySelector("[data-wcsum='"+n+"']"),
            t("wcCond",sg<0?"−":"+",has(wcAmps[n])?wcAmps[n]:wcLim(0),has(wcSecs[n])?wcSecs[n]:wcLim(2)));
    const ok=!!wcChk[n+"c"]&&!!wcChk[n+"b"], why=o.start_why?sv(o.start_why,o.start_why_en):"";
    dis("button[data-wcgo='"+n+"']", !!why||!ok, why||(ok?"":t("wcChkTip")));
    setText(document.querySelector("[data-wcwhy='"+n+"']"), why);
    setText(document.querySelector("[data-wcref='"+n+"']"), t("wcRefNow")+wrefTx(o.ref));
    if(!o.ref) delete wrefAsk[n];
    const ask=!!wrefAsk[n]&&Date.now()<wrefAsk[n], cb=document.querySelector("button[data-wcclr='"+n+"']");
    if(cb){ const tx=t(ask?"wrefClearAsk":"wrefClear"), cl="ghost"+(ask?" warn":"");
      if(cb.textContent!==tx) cb.textContent=tx; if(cb.className!==cl) cb.className=cl; }
    dis("button[data-wcclr='"+n+"']", !o.ref, t("wrefClearTip"));
  }else if(s==="running"){
    const A=has(o.amps)?Math.abs(o.amps).toFixed(1):"–", g=(has(o.amps)&&o.amps<0)?"−":"+";
    setText(document.querySelector("[data-wcrun='"+n+"']"), o.saving?t("wcSaving"):t("wcRunning",g,A,num(o.left,1),has(o.pts)?o.pts:0));
    const bar=document.querySelector("[data-wcbar='"+n+"']");
    if(bar){ const fr=(has(o.left)&&o.total>0)?Math.min(1,Math.max(0,1-o.left/o.total)):(o.saving?1:0);
      const wd=Math.round(fr*1000)/10+"%"; if(bar.style.width!==wd) bar.style.width=wd; }
    setText(document.querySelector("[data-wcfit='"+n+"']"),
            o.fit?t("wcFitNow",num(o.fit.r,3),num(o.fit.se_c,1)):(o.fit_why?sv(o.fit_why,o.fit_why_en):""));
  }
}
function wcWire(){
  const box=document.getElementById("cards");
  box.addEventListener("click",e=>{
    const tg=e.target, d=(tg&&tg.dataset)||{};
    if(d.wcopen){ tabSet(+d.wcopen,"wc"); render(); }
    else if(d.wcgo){ const n=+d.wcgo, sg=(wcSign[n]===-1)?-1:1;
      const A=parseFloat(has(wcAmps[n])?wcAmps[n]:wcLim(0)), sc=parseFloat(has(wcSecs[n])?wcSecs[n]:wcLim(2));
      if(!isFinite(A)||!isFinite(sc)){ showInfo(t("needNum")); return; }
      post("wcal_start",{node:n, amps:sg*Math.abs(A), secs:sc, cold:!!wcChk[n+"c"], blocked:!!wcChk[n+"b"]}); }
    else if(d.wccancel) post("wcal_cancel",{node:+d.wccancel});
    else if(d.wcsg){ wcSign[+d.wcsg]=(+d.sg<0)?-1:1; render(); }
    else if(d.wcclr) wrefClear(+d.wcclr);
    else if(d.wcclose||d.wcagain){ const n=+(d.wcclose||d.wcagain);
      if(d.wcagain){ post("wcal_cancel",{node:n}); tabSel[n]="wc"; } else tabSet(n,"");
      wcChk[n+"c"]=wcChk[n+"b"]=false; render(); }
  });
  box.addEventListener("change",e=>{ const tg=e.target, d=(tg&&tg.dataset)||{};
    if(d.wcchk){ wcChk[d.wcchk+d.k]=!!tg.checked; render(); } });
  box.addEventListener("input",e=>{ const tg=e.target, d=(tg&&tg.dataset)||{};
    if(d.wca) wcAmps[+d.wca]=tg.value; else if(d.wcs) wcSecs[+d.wcs]=tg.value; });
  box.addEventListener("toggle",e=>{ const tg=e.target; if(tg&&tg.dataset&&tg.dataset.wccond) wcCondOpen[+tg.dataset.wccond]=tg.open; }, true);
}

function paramRows(a){
  const m=a.mode; let h="";
  if(m==="kt_test") return ktRows(a);
  const row=(key,label,min,max,step,val,fmt)=>
    "<div class='row'><label>"+label+"</label>"
    +"<input type='range' data-ax='"+a.node+"' data-key='"+key+"' min='"+min
    +"' max='"+max+"' step='"+step+"' value='"+(has(val)?val:min)+"'>"
    +((key==="kp"||key==="kd")
      ? "<input type='number' class='kin' data-kin='"+key+"' data-ax='"+a.node+"' min='0' max='"+max+"' step='0.01' value='"
        +(has(val)?(+val).toFixed(2):"")+"' title='"+esc(t("kinTip",label,max))+"'>"
        +"<span class='kq' data-kq='"+a.node+"-"+key+"' title='"+esc(t("kqTip"))+"'></span></div>"
      : "<span class='n' data-num='"+a.node+"-"+key+"'>"+num(val,fmt)+"</span></div>");
  if(m==="hold"||m==="sine"||m==="pos_test")
    h+=row("kp","Kp",0,(a.kp_max>0?a.kp_max:lim.kp),0.01,a.kp,2);
  if(m!=="free"&&m!=="torque")
    h+=row("kd","Kd",0,lim.kd,0.01,a.kd,2);
  if(m==="pos_test") return h+ptRows(a);
  const lP=t("pTarget"), lA=t("pAmp"), lF=t("pFreq"), lC=t("pCenter");
  const G=gearR(a.node), gK=(key)=>gearK(a.node,key), gV=(key,v)=>has(v)?v/gK(key):v;
  if(m==="hold")
    h+=G ? row("p",t("pTargetOut"),-lim.p/gK("p"),lim.p/gK("p"),0.1,gV("p",a.p_target),1)
         : row("p",lP,-lim.p,lim.p,0.005,a.p_target,3);
  if(m==="sine"){
    h+=G ? row("amp",t("pAmpOut"),0,lim.amp/gK("amp"),0.1,gV("amp",a.amp),1)
         : row("amp",lA,0,lim.amp,0.01,a.amp,2);
    h+=row("freq",lF,0.05,lim.freq,0.05,a.freq,2);
    h+=G ? row("center",t("pCenterOut"),-lim.p/gK("center"),lim.p/gK("center"),0.1,gV("center",a.center),1)
         : row("center",lC,-lim.p,lim.p,0.005,a.center,3);
  }
  if(m==="vel"&&G){ const vo=Math.floor(((a.v_max>0)?a.v_max:lim.v)/G/0.01+1e-9)/100; h+=row("v",t("pVelOut"),-vo,vo,0.01,gV("v",a.v_target),2); }
  else if(m==="vel"){ const vm=Math.round(Math.floor(((a.v_max>0)?a.v_max:lim.v)/0.1+1e-9))/10; h+=row("v",t("pVel"),-vm,vm,0.1,a.v_target,1); }
  if(m==="torque"){ const tm=Math.round(Math.floor(((a.t_max>0)?a.t_max:lim.t)/0.005+1e-9)*0.005*1000)/1000; h+=row("t",t("pTq"),-tm,tm,0.005,a.t_target,3); }
  return h;
}


function healthChips(a){
  const b=a.board||{}; let h="";
  if(!b.alive){ return chip("基板",tw("応答なし"),"bad"); }
  h+=chip("自己確認",has(b.selftest)
            ?(b.selftest?"OK":tw("異常")+" 0x"+(b.flags||0).toString(16))
            :tw("読めない"),
          has(b.selftest)?(b.selftest?"":"bad"):"");
  if(b.fault_now) h+=chip("nFAULT",tw("発生中"),"bad");
  if(b.nomag) h+=chip("磁石",tw("検出できず"),"bad");
  h+=chip("較正",has(b.cal)?(b.cal?tw("あり"):tw("なし")):tw("読めない"),
          has(b.cal)?(b.cal?"":"bad"):"");
  h+=chip("巻線温度",(has(b.therm_pct)?b.therm_pct+" %":"–"),
          b.therm_pct===undefined?"":
            (b.therm_pct>=a.therm_trip?"bad":
              (b.therm_pct>=a.therm_clear?"warn":"")));
  h+=chip("母線",(has(b.vbus_mv)?(b.vbus_mv/1000).toFixed(1):"–")+" V");
  {
    const isr=b.khz?Math.round(170000/b.khz):0;
    h+=chip("ISR最大",(has(b.isr_max)?b.isr_max+" cyc":"–")
              +(isr?" / "+isr:""),
            (!has(b.isr_max)||!isr)?"":
              (b.isr_max>isr?"bad":(b.isr_max>isr*0.8?"warn":"")));
  }
  if(b.enc_err) h+=chip("エンコーダ異常",b.enc_err+tw(" 回"),
                        b.enc_err>=10?"bad":"warn");
  h+=chip("可動範囲", has(b.lim_on)?(b.lim_on?
          (num(b.lim_lo,2)+tw(" 〜 ")+num(b.lim_hi,2)+" rad"):tw("無効")):tw("読めない"),
          has(b.lim_on)?(b.lim_on?"":"warn"):"");
  h+=chip("欠測",String(a.fb_missing||0),(a.fb_missing||0)>50?"warn":"");
  return h;
}

const HEALTH_QUIET={"可動範囲":1};
const HEALTH_COUNT={"エンコーダ異常":1, "欠測":1};
const HEALTH_RECENT_MS=60000;
const hcSeen={};
let hcSink=null;
function healthAlerts(a){
  hcSink=[]; let L=[];
  try{ healthChips(a); }finally{ L=hcSink; hcSink=null; }
  const b=a.board||{}, cnt={"エンコーダ異常":b.enc_err||0, "欠測":a.fb_missing||0}, now=Date.now();
  const fresh={};
  for(const k in HEALTH_COUNT){
    const id=a.node+":"+k, v=cnt[k], s=hcSeen[id];
    if(!s){ hcSeen[id]={v:v, t:-Infinity}; }
    else if(v>s.v){ s.v=v; s.t=now; }
    else if(v<s.v){ s.v=v; }
    fresh[k]=!!hcSeen[id]&&now-hcSeen[id].t<HEALTH_RECENT_MS;
  }
  let h="";
  for(const x of L){
    if(!x.c||HEALTH_QUIET[x.k]) continue;
    if(HEALTH_COUNT[x.k]&&!fresh[x.k]) continue;
    h+=chip(x.k,x.v,x.c);
  }
  return h;
}


function stateChip(a){
  let h;
  if(a.tripped===true) h="<span class='chip bad'>"+t("stTrip")+"</span>";
  else if(a.tripped===null) h="<span class='chip warn'>"+t("stUnk")+"</span>";
  else if(a.armed) h="<span class='chip"+(unreachable()?"":" warn")+"'>"+t("stArmed")+"</span>";
  else if(!a.held&&(a.offcal||{}).stage!=="running") h="<span class='chip'>"+t("stIdle")+"</span>";
  else h="";
  if(a.held&&!a.armed) h+="<span class='chip warn'>"+t("stHeld")+"</span>";
  if(a.mode&&BUTTON_MODES.indexOf(a.mode)<0) h+="<span class='chip"+(a.mode==="homing"?" warn":"")+"'>"+modeLabel(a.mode)+"</span>";
  { const jt=st.jt;
    if(jt&&(jt.nodes||[]).indexOf(a.node)>=0){
      const nm=esc(jt.name||"jtraj"), k=(jt.stage==="stopping")?"stJtStop":((jt.stage==="approach")?"stJtGo":"stJt");
      h+="<span class='chip'>"+t(k,nm)+"</span>"; } }
  const oc=(a.offcal||{}).stage;
  if((a.offcal||{}).kind==="gear"&&["prep","seek","back","running"].indexOf(oc)>=0) h+="<span class='chip warn'>"+t("stOcG")+"</span>";
  else if(oc==="running") h+="<span class='chip warn'>"+t("stOcRun")+"</span>";
  else if(oc==="wait_push") h+="<span class='chip warn'>"+t("stOcPush")+"</span>";
  if((a.wcal||{}).stage==="running") h+="<span class='chip warn'>"+t("stWcRun")+"</span>";
  return h;
}

function cardHtml(a){
  let h="<div class='card"+(a.on?"":" off")
       +"' data-card='"+a.node+"'>";
  h+="<h2>"+t("axisH",a.node)+" <span data-stt='"+a.node+"'></span>"
    +"<span class='sw'><button data-gear='"+a.node+"'></button><button data-arm='"+a.node+"'></button></span></h2>";
  h+="<div data-health='"+a.node+"'></div>";
  h+="<div class='modes'>";
  for(const m of BUTTON_MODES){
    h+="<button data-mode='"+m+"' data-ax='"+a.node+"'"
      +(a.mode===m?" class='on'":"")
      +">"+modeLabel(m)+"</button>";
  }
  h+="<button data-ocopen='"+a.node+"'>"+(gearR(a.node)?t("ocgOpen"):t("ocOpen"))+"</button>";
  h+="<button data-wcopen='"+a.node+"'>"+t("wcOpen")+"</button>";
  h+="</div>";
  h+="<div data-params='"+a.node+"'>"+paramRows(a)+"</div>";
  h+="<div class='warnline' data-why='"+a.node+"'></div>";
  h+="<div class='oc' data-oc='"+a.node+"'></div>";
  h+="<div class='oc' data-wc='"+a.node+"'></div>";
  h+="<div class='gtitle'>"+esc(gearR(a.node)?t("gtPosOut"):t("gtPos"))+"<span class='gval' data-v='"+a.node+"-pos'>–</span></div>";
  h+=vzHtml("p",a.node,"<canvas data-cvp='"+a.node+"' height='100'></canvas>");
  h+="<div class='gtitle'>"+esc(gearR(a.node)?t("gtVelOut"):t("gtVel"))+"<span class='gval' data-v='"+a.node+"-vel'>–</span></div>";
  h+="<div class='cvwrap'><div class='vzgap'></div><canvas data-cvv='"+a.node+"' height='80'></canvas></div>";
  h+="<div class='gtitle'>"+esc(t("gtCur"))+"<span class='gval' data-v='"+a.node+"-iq'>–</span></div>";
  h+="<div class='cvwrap'><div class='vzgap'></div><canvas data-cvc='"+a.node+"' height='80'></canvas></div>";
  h+="<div class='gtitle'>"+esc(t("gtTemp"))+"<span class='templine' data-tl='"+a.node+"'></span></div>";
  h+="<div class='cvwrap'><div class='vzgap'></div><canvas data-cvt='"+a.node+"' height='80'></canvas></div>";
  h+=prmHtml(a);
  h+="</div>";
  return h;
}

let tempsMeta={};
function tempLine(a){
  const tp=temps[a.node]||{}, d=tp.dbg||{};
  const bc=(tp.board_c===null||tp.board_c===undefined)?"–":tp.board_c.toFixed(1);
  const wc=(tp.wind_c===null||tp.wind_c===undefined)?"–":tp.wind_c.toFixed(0);
  let h="<span class='tb'>"+t("tBoard",bc)+"</span>";
  h+="<span class='tw'>"+t("tWind",wc)+"</span>";
  if(tp.r!==null&&tp.r!==undefined) h+="<span class='tr'>R "+tp.r.toFixed(3)+" Ω</span>";
  if(tp.wind_c===null||tp.wind_c===undefined){
    const why=tempWhy(tp,d);
    if(why) h+="<span class='th'>"+why+"</span>";
  }
  if(has(tp.ref_err)) h+="<span class='th'>"+esc(t("twRefErr"))+"</span>";
  return h;
}
function tempWhy(tp,d){
  const fx=v=>(v===null||v===undefined)?"–":Number(v).toFixed(1);
  switch(tp.why){
    case "no_tlm": return t("twNoTlm");
    case "low_iq": return t("twLowIq",fx(tempsMeta.iq_min));
    case "spinning": return t("twSpin");
    case "reading": return t("twReading");
    case "no_board_temp": return t("twNoBoard");
    case "basis": return t("twBasis",(d.n_ok||0),(tempsMeta.r0_after||0));
    case "settling": return t("twSettling",(d.n_ok||0),(tempsMeta.r0_after||0));
    case "iq_mismatch": return t("twMismatch",fx(tp.iq0),fx(tp.iq_now));
    case "stopped": return t("twStopped");
    case "range": return t("twRange");
    default: return tp.why?t("twOther",tp.why):"";
  }
}
function drawTemp(cv,pts,span,node){
  if(!cv) return;
  const dpr=window.devicePixelRatio||1;
  const w=cv.clientWidth, h=cv.clientHeight;
  if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)){
    cv.width=Math.round(w*dpr); cv.height=Math.round(h*dpr);
  }
  const g=cv.getContext("2d");
  g.setTransform(dpr,0,0,dpr,0,0);
  g.clearRect(0,0,w,h);
  if(!pts||!pts.length||!(span>0)) return;
  const tv=lim.temp_view;
  if(!tv||!has(tv[0])||!has(tv[1])||!(tv[1]>tv[0])) return;
  const lo=tv[0], hi=tv[1];
  const X=t=>w*(1+t/span), Y=v=>h*(1-(Math.min(hi,Math.max(lo,v))-lo)/(hi-lo));
  g.strokeStyle=tok("--line"); g.lineWidth=1; g.fillStyle=tok("--dim"); g.font=CHART_FONT;
  const xs=niceStep(span,3);
  g.globalAlpha=CHART_GRID_A; g.textAlign="center";
  for(let t=-xs; t>-span+xs*1e-6; t-=xs){
    const x=X(t);
    g.beginPath(); g.moveTo(x,0); g.lineTo(x,h); g.stroke();
    g.globalAlpha=1; g.fillText(tickText(t,xs)+"s",x,h-3); g.globalAlpha=CHART_GRID_A;
  }
  g.globalAlpha=1; g.textAlign="left";
  const ts=niceStep(hi-lo,4);
  for(let v=Math.ceil(lo/ts)*ts; v<hi; v+=ts){
    g.beginPath(); g.moveTo(0,Y(v)); g.lineTo(w,Y(v)); g.stroke();
    g.fillText(tickText(v,ts)+t("degC"),3,Y(v)<12?Y(v)+10:Y(v)-2);
  }
  const overs=[];
  for(const s of [{k:1,color:tok("--dim")},{k:2,color:tok("--c5")}]){
    g.strokeStyle=s.color; g.lineWidth=CHART_LINE_W;
    const over=[];
    strokeSeries(g,pts,p=>p[s.k],X,Y,lo,hi,over);
    if(over.length) overs.push({over:over, color:s.color});
  }
  g.font=CHART_FONT; g.textBaseline="top";
  let xr=w-3;
  for(const L of [{label:t("lgWind"),color:tok("--c5")},{label:t("lgBoard"),color:tok("--dim")}]){
    const tw2=g.measureText(L.label).width;
    g.globalAlpha=.85; g.fillStyle=tok("--panel"); g.fillRect(xr-tw2-21,1,tw2+23,17); g.globalAlpha=1;
    g.textAlign="right"; g.fillStyle=L.color; g.fillText(L.label,xr,3);
    g.strokeStyle=L.color; g.lineWidth=CHART_LINE_W; g.beginPath(); g.moveTo(xr-tw2-18,10); g.lineTo(xr-tw2-4,10); g.stroke();
    xr-=tw2+26;
  }
  for(const o of overs) strokeOver(g,o.over,o.color,CHART_LINE_W,h);
  g.textAlign="left"; g.textBaseline="alphabetic";
}
function drawTemps(){
  for(const a of (st.axes||[])){
    const t=temps[a.node];
    drawTemp(document.querySelector("[data-cvt='"+a.node+"']"), t?t.pts:null, tempsMeta.view_s, a.node);
  }
}
async function pollTemps(){
  if(inflight.temps) return;
  inflight.temps=true;
  try{
    const r=await fetch("/api/temps",{cache:"no-store", signal:wdSignal()});
    const j=await r.json();
    tempsMeta={r0_after:j.r0_after, iq_min:j.iq_min, view_s:j.view_s};
    const nx={};
    for(const a of (j.axes||[])) nx[a.node]=a;
    temps=nx;
    drawTemps();
  }catch(e){}finally{ inflight.temps=false; }
}

function wire(){
  const box=document.getElementById("cards");
  for(const b of box.querySelectorAll("button[data-arm]"))
    b.onclick=()=>post(b.dataset.state==="on"?"disarm":"arm",{node:+b.dataset.arm});
  for(const b of box.querySelectorAll("button[data-gear]"))
    b.onclick=()=>{ const n=+b.dataset.gear; gearSet(n,!gearR(n)); };
  for(const b of box.querySelectorAll("button[data-mode]"))
    b.onclick=()=>{
      const n=+b.dataset.ax;
      tabSet(n,"");
      post("mode",{mode:b.dataset.mode,node:n});
    };
  wireRanges();
  for(const b of box.querySelectorAll("button[data-kta]"))
    b.onclick=()=>{ const n=+b.dataset.kta;
      if(b.dataset.amps!=="max"){ ktSend(n,+b.dataset.amps); return; }
      const a=(st.axes||[]).find(x=>x.node===n), im=a?ktImax(a):null;
      if(im===null){ showInfo(t("ktNoMax",n)); return; }
      ktSend(n,im); };
  for(const b of box.querySelectorAll("button[data-ktsg]"))
    b.onclick=()=>{ const n=+b.dataset.ktsg; ktSign[n]=(+b.dataset.sg<0)?-1:1;
      for(const o of box.querySelectorAll("button[data-ktsg='"+n+"']")) o.className=(+o.dataset.sg===ktSign[n])?"on":""; };
  for(const inp of box.querySelectorAll("input[data-ktfree]")){
    const n=+inp.dataset.ktfree;
    inp.oninput=()=>{ ktFree[n]=inp.value; };
    inp.onkeydown=e=>{ if(e.key==="Enter"){ const go=box.querySelector("button[data-ktgo='"+n+"']"); if(go&&!go.disabled) go.onclick(); } };
  }
  for(const b of box.querySelectorAll("button[data-ktgo]"))
    b.onclick=()=>{ const n=+b.dataset.ktgo, inp=box.querySelector("input[data-ktfree='"+n+"']");
      const v=Math.abs(parseFloat(inp?inp.value:""));
      if(!isFinite(v)||v<=0){ showInfo(t("ktNeedNum")); return; }
      ktSend(n,v); };
  for(const inp of box.querySelectorAll("input[data-ktsecs]")){
    const n=+inp.dataset.ktsecs;
    inp.oninput=()=>{ ktSecs[n]=inp.value; };
  }
  for(const b of box.querySelectorAll("button[data-ktstop]"))
    b.onclick=()=>post("tq_stop",{node:+b.dataset.ktstop});
  for(const b of box.querySelectorAll("button[data-ptgo]"))
    b.onclick=()=>ptSend(+b.dataset.ptgo);
  for(const b of box.querySelectorAll("button[data-ptstop]"))
    b.onclick=()=>post("pt_stop",{node:+b.dataset.ptstop});
  for(const b of box.querySelectorAll("button[data-ptsty]"))
    b.onclick=()=>{ const n=+b.dataset.ptsty; ptStyle[n]=b.dataset.v;
      for(const o of box.querySelectorAll("button[data-ptsty='"+n+"']")) o.className=(o.dataset.v===ptStyle[n])?"on":""; };
  for(const [sel,store] of [["input[data-ptpts]",ptPts],["input[data-ptv]",ptSpeed],["input[data-ptd]",ptDwell]])
    for(const inp of box.querySelectorAll(sel)){
      const n=+(inp.dataset.ptpts||inp.dataset.ptv||inp.dataset.ptd);
      inp.oninput=()=>{ store[n]=inp.value; };
      inp.onkeydown=e=>{ if(e.key==="Enter"&&!e.isComposing&&e.keyCode!==229){ const go=box.querySelector("button[data-ptgo='"+n+"']"); if(go&&!go.disabled) ptSend(n); } };
    }
  for(const dt of box.querySelectorAll("details[data-prm]"))
    dt.ontoggle=()=>{ prmOpen[+dt.dataset.prm]=dt.open; };
  for(const inp of box.querySelectorAll("input[data-pk]")){
    const pk=inp.dataset.pn+":"+inp.dataset.pk;
    inp.onfocus=()=>{ if(!inp._typing) inp._base=inp.value; };
    inp.oninput=()=>{ inp._typing=true; inp._auto=false; inp._keySent=undefined; };
    const send=()=>{
      inp._typing=false; inp._base=inp.value;
      const v=parseFloat(inp.value);
      if(!isFinite(v)){ showInfo(t("needNum")); return; }
      prmDropDel(pk);
      post("param",{node:+inp.dataset.pn, key:inp.dataset.pk, value:v}); };
    inp.onchange=()=>{
      if(rebuilding){ if(rebuildDrop.indexOf(pk)<0) rebuildDrop.push(pk); return; }
      if(inp._keySent!==undefined&&inp._keySent===inp.value){ inp._keySent=undefined; return; }
      inp._keySent=undefined;
      send(); };
    inp.onkeydown=e=>{
      if(e.key!=="Enter"||e.isComposing||e.keyCode===229||e.repeat) return;
      inp._keySent=inp.value; send(); };
    inp.addEventListener("keydown",()=>{ if(inp.readOnly) inp.readOnly=false; },true);
    inp.onwheel=e=>{
      if(document.activeElement!==inp) return;
      if(!inp._typing){ inp.blur(); return; }
      if(!e.cancelable&&!inp.readOnly){ inp.readOnly=true; setTimeout(()=>{ inp.readOnly=false; },0); }
      e.preventDefault();
      if(!e.ctrlKey&&typeof window.scrollBy==="function"){
        const k=(e.deltaMode===1)?16:((e.deltaMode===2)?(window.innerHeight||0):1);
        window.scrollBy((e.deltaX||0)*k,(e.deltaY||0)*k);
      }
    };
  }
  for(const inp of box.querySelectorAll("input[data-kin]")){
    const n=+inp.dataset.ax, key=inp.dataset.kin, pk=n+":"+key;
    const send=()=>{
      inp._typing=false;
      const v=parseFloat(inp.value);
      if(!isFinite(v)){ showInfo(t("needNum")); return; }
      prmDropDel(pk);
      const p={node:n}; p[key]=v; post("set",p);
    };
    inp.oninput=()=>{ inp._typing=true; inp._keySent=undefined; };
    inp.onchange=()=>{
      if(rebuilding){ if(rebuildDrop.indexOf(pk)<0) rebuildDrop.push(pk); return; }
      if(inp._keySent!==undefined&&inp._keySent===inp.value){ inp._keySent=undefined; return; }
      inp._keySent=undefined;
      send();
    };
    inp.onblur=()=>{ inp._typing=false; };
    inp.onkeydown=e=>{
      if(e.key!=="Enter"||e.isComposing||e.keyCode===229||e.repeat) return;
      inp._keySent=inp.value; send();
    };
    inp.addEventListener("keydown",()=>{ if(inp.readOnly) inp.readOnly=false; },true);
    inp.onwheel=e=>{
      if(document.activeElement!==inp) return;
      if(!inp._typing){ inp.blur(); return; }
      if(!e.cancelable&&!inp.readOnly){ inp.readOnly=true; setTimeout(()=>{ inp.readOnly=false; },0); }
      e.preventDefault();
      if(!e.ctrlKey&&typeof window.scrollBy==="function"){
        const k=(e.deltaMode===1)?16:((e.deltaMode===2)?(window.innerHeight||0):1);
        window.scrollBy((e.deltaX||0)*k,(e.deltaY||0)*k);
      }
    };
  }
  for(const r of box.querySelectorAll("input[data-vz]")){
    r.oninput=()=>{
      const [k,ns]=r.dataset.vz.split(":"), n=+ns, v=vzRad(k,+r.value);
      viewVal[k+n]=v;
      try{ localStorage.setItem(viewKey(k,n), String(v)); }catch(e){}
      drawAxes();
    };
  }
}
function wireRanges(){
  for(const r of document.querySelectorAll("input[type=range][data-ax]")){
    r.oninput=()=>{
      dragging=r;
      const n=document.querySelector("[data-num='"+r.dataset.ax+"-"+r.dataset.key+"']");
      if(n) n.textContent=(+r.value).toFixed(+(r.step<0.01?3:(r.step<0.1?2:1)));
      const ki=document.querySelector("input[data-kin='"+r.dataset.key+"'][data-ax='"+r.dataset.ax+"']");
      if(ki){ ki.value=(+r.value).toFixed(2); ki._typing=false; }
      const p={node:+r.dataset.ax}; p[r.dataset.key]=+r.value*gearK(+r.dataset.ax,r.dataset.key);
      post("set",p);
    };
    r.onchange=()=>{ dragging=null; };
  }
}

function noAxesHtml(){ return "<div class='card'><p class='note'>"+t("noAxes")+"</p></div>"; }

function cardShape(a){
  const m=a.mode;
  return ((m==="torque"||m==="kt_test")?(a.t_max||0):"")+":"+(m==="vel"?(a.v_max||0):"")+":"+(m==="kt_test"?(a.kt||0):"");
}

let rebuilding=false, rebuildDrop=[];
function renderCards(){
  const axes=st.axes||[];
  const key=LANG+"#"+(axes.length ? axes.map(a=>a.node+":"+a.mode+":"+(a.on?1:0)+":"+(a.armed?1:0)+":"+cardShape(a)
                        +":"+(a.node===st.node?1:0)+":"+(gearR(a.node)?1:0)).join("|") : "none");
  if(key!==builtKey){
    builtKey=key;
    const box=document.getElementById("cards"), ae=document.activeElement;
    rebuildDrop=[];
    if(ae&&ae._typing&&ae.dataset&&ae.dataset.pk&&ae.value!==ae._base) rebuildDrop.push(ae.dataset.pn+":"+ae.dataset.pk);
    if(ae&&ae._typing&&ae.dataset&&ae.dataset.kin) rebuildDrop.push(ae.dataset.ax+":"+ae.dataset.kin);
    rebuilding=true;
    try{ box.innerHTML=axes.length ? axes.map(cardHtml).join("") : noAxesHtml(); }
    finally{ rebuilding=false; }
    wire();
    if(rebuildDrop.length) prmDropAdd(rebuildDrop);
  }
  for(const a of axes){
    const set=(k,v)=>{const e=document.querySelector("[data-v='"+a.node+"-"+k+"']");
                      if(e) e.textContent=v;};
    { const ab=document.querySelector("button[data-arm='"+a.node+"']");
      if(ab){ const on=!!a.armed, tx=on?t("disarm"):t("arm"), cl=on?"":"pri", sd=on?"on":"off";
              if(ab.textContent!==tx) ab.textContent=tx;
              if(ab.className!==cl) ab.className=cl;
              if(ab.dataset.state!==sd) ab.dataset.state=sd;
              const dis=on?false:!!a.arm_block; if(ab.disabled!==dis) ab.disabled=dis;
              const tt=on?"":sv(a.arm_block||"",a.arm_block_en); if(ab.title!==tt) ab.title=tt; } }
    { const gb=document.querySelector("button[data-gear='"+a.node+"']");
      if(gb){ const g=lim.gear, on=!!gearR(a.node), tx=has(g)?t("gear",g):"–", cl=on?"on":"";
              if(gb.textContent!==tx) gb.textContent=tx;
              if(gb.className!==cl) gb.className=cl;
              const dis=!(g>0); if(gb.disabled!==dis) gb.disabled=dis;
              const tt=has(g)?t("gearTip",g):""; if(gb.title!==tt) gb.title=tt; } }
    prmUpdate(a);
    wrefUpdate(a);
    ocUpdate(a);
    wcUpdate(a);
    if(a.mode==="kt_test") ktUpdate(a);
    if(a.mode==="pos_test") ptUpdate(a);
    { const R=gearR(a.node);
      set("pos",R?(has(a.pos)?(a.pos/R*DEG).toFixed(2):"–"):num(a.pos,4));
      set("vel",R?(has(a.vel)?(a.vel/R).toFixed(3):"–"):num(a.vel,2)); }
    set("iq",num(a.iq,3));
    set("tau",(!has(a.iq)||!has(a.kt))?""
              :t("tauEst",(a.iq*a.kt).toFixed(3),a.kt.toFixed(3)));
    setHtml(document.querySelector("[data-stt='"+a.node+"']"), stateChip(a));
    for(const m of POS_MODES){
      const mb=document.querySelector(
        "button[data-mode='"+m+"'][data-ax='"+a.node+"']");
      if(!mb) continue;
      const rz=(m==="hold"||m==="sine")&&a.rezero&&a.board&&a.board.alive;
      const on=(a.mode===m), lk=(a.pos_ok===false)&&!rz;
      const cl=on?"on":(lk?"locked":"");
      if(mb.className!==cl) mb.className=cl;
      const tt=lk?sv(a.pos_why||"",a.pos_why_en):"";
      if(mb.title!==tt) mb.title=tt;
      const tx=modeLabel(m)+(lk?" ⚠":"");
      if(mb.textContent!==tx) mb.textContent=tx;
    }
    { const tab=tabOf(a);
      for(const [at,k] of [["data-ocopen","oc"],["data-wcopen","wc"]]){
        const ob=document.querySelector("button["+at+"='"+a.node+"']");
        if(ob){ const c=(tab===k)?"on":""; if(ob.className!==c) ob.className=c; } }
      for(const mb of document.querySelectorAll("button[data-mode][data-ax='"+a.node+"']")){
        const want=!tab&&a.mode===mb.dataset.mode;
        if(mb.classList.contains("on")!==want) mb.classList.toggle("on",want); }
      for(const sel of ["[data-params='"+a.node+"']","[data-why='"+a.node+"']"]){
        const el=document.querySelector(sel); if(el){ const d=tab?"none":""; if(el.style.display!==d) el.style.display=d; } } }
    setHtml(document.querySelector("[data-health='"+a.node+"']"), healthAlerts(a));
    { const tl=document.querySelector("[data-tl='"+a.node+"']");
      setHtml(tl, tempLine(a));
      const tt=tempRefTip(temps[a.node]||{}); if(tl&&tl.title!==tt) tl.title=tt; }
    const wb=document.querySelector("[data-why='"+a.node+"']");
    if(wb){
      if(a.pos_ok===false&&a.pos_why&&!(a.rezero&&a.board&&a.board.alive)){
        const nb=t("posLocked",esc(sv(a.pos_why,a.pos_why_en)),a.rezero_why?esc(sv(a.rezero_why,a.rezero_why_en)):"");
        if(wb.dataset.h!==nb){ wb.dataset.h=nb; wb.innerHTML=nb; }
      }else if(wb.dataset.h!==""){ wb.dataset.h=""; wb.innerHTML=""; }
    }
    if(!dragging){
      for(const key of ["kp","kd","p","amp","freq","center","v","t"]){
        const r=document.querySelector(
          "input[type=range][data-ax='"+a.node+"'][data-key='"+key+"']");
        if(!r) continue;
        const gk=gearK(a.node,key);
        if((key==="p"||key==="center")&&has(a.p_lo)&&has(a.p_hi)){
          if(+r.min!==a.p_lo/gk) r.min=a.p_lo/gk;
          if(+r.max!==a.p_hi/gk) r.max=a.p_hi/gk;
        }
        if(key==="kp"&&a.kp_max>0&&+r.max!==a.kp_max) r.max=a.kp_max;
        const val0={kp:a.kp,kd:a.kd,p:a.p_target,amp:a.amp,freq:a.freq,
                    center:a.center,v:a.v_target,t:a.t_target}[key];
        if(val0===undefined) continue;
        const val=has(val0)?val0/gk:val0;
        if(Math.abs(+r.value-val)>(gk!==1?(+r.step)/2:1e-9)) r.value=val;
        const n=document.querySelector("[data-num='"+a.node+"-"+key+"']");
        if(n) n.textContent=(+r.value).toFixed(r.step<0.01?3:(r.step<0.1?2:1));
        const ki=document.querySelector("input[data-kin='"+key+"'][data-ax='"+a.node+"']");
        if(ki&&document.activeElement!==ki&&!ki._typing&&has(val0)){
          const tx=(+val0).toFixed(2); if(ki.value!==tx) ki.value=tx;
          if(key==="kp"&&a.kp_max>0&&+ki.max!==a.kp_max) ki.max=a.kp_max;
        }
      }
    }
    for(const [key,want,got] of [["kp",a.kp,a.kp_sent],["kd",a.kd,a.kd_sent]]){
      const q=document.querySelector("[data-kq='"+a.node+"-"+key+"']");
      if(!q) continue;
      setText(q,(has(want)&&has(got)&&(+got).toFixed(2)!==(+want).toFixed(2))?t("kq",(+got).toFixed(3)):"");
    }
  }
}


function velRange(a){
  const vv=lim.vel_view||[], k=vv[0], s=vv[1], vmax=((a.v_max>0)?a.v_max:lim.v)/(gearR(a.node)||1);
  if(!(k>0&&s>0&&vmax>0)) return null;
  return Math.ceil(vmax*k/s-1e-9)*s;
}
function drawAxes(){
  for(const a of (st.axes||[])){
    const pts=hist[a.node]||[];
    const tgt=POS_MODES.includes(a.mode);
    const GR=gearR(a.node), PK=GR?DEG/GR:1, VK=GR?1/GR:1;
    const sc=(v,k)=>(v===null||v===undefined)?v:v*k;
    let big=0;
    for(const p of pts){ for(const k of (tgt?[1,2]:[2])){ const v=p[k];
      if(v!==null&&v!==undefined) big=Math.max(big,Math.abs(v*PK)); } }
    const pv=viewOf(GR?"q":"p",a.node), half=Math.max(pv, big*1.1), mid=0;
    const ser=[{color:tok("--cyan"),pts:pts,get:p=>sc(p[2],PK)}], leg=[{label:t("gPos"),color:tok("--cyan")}];
    if(tgt){ ser.unshift({color:tok("--dim"),dash:true,pts:pts,get:p=>sc(p[1],PK)});
             leg.push({label:t("gTgt"),color:tok("--dim"),dash:true}); }
    draw(document.querySelector("[data-cvp='"+a.node+"']"), ser, mid-half, mid+half, true,
      {legend:leg});
    const cI=tok("--c6"), cD=tok("--c4");
    let c=0;
    for(const p of pts){ for(const k of [4,5]){ const v=p[k]; if(v!==null&&v!==undefined) c=Math.max(c,Math.abs(v)); } }
    c=Math.max(CUR_VIEW_A, c*1.05);
    draw(document.querySelector("[data-cvc='"+a.node+"']"),
      [{color:cD,pts:pts,get:p=>p[5]},
       {color:cI,pts:pts,get:p=>p[4]}], -c, c, true,
      {legend:[{label:"iq",color:cI},{label:"id",color:cD}]});
    let vm=velRange(a);
    if(vm===null){ vm=1; for(const p of pts){ const v=p[3]; if(v!==null&&v!==undefined) vm=Math.max(vm,Math.abs(v*VK)*1.1); } }
    draw(document.querySelector("[data-cvv='"+a.node+"']"),
      [{color:tok("--c1"),pts:pts,get:p=>sc(p[3],VK),lim:[-vm,vm]}],
      -vm, vm, true, {legend:[{label:t("lgVel"),color:tok("--c1")}]});
  }
}



const SEP="　／　";
function join2(a,b){ return a&&b ? a+(LANG==="en"?"  /  ":SEP)+b : (a||b||""); }

function banners(){
  const bad=document.getElementById("bnBad"), warn=document.getElementById("bnWarn");
  const info=document.getElementById("bnInfo");
  let b="", w="";
  const axes=st.axes||[];
  const trips=axes.filter(a=>a.tripped===true);
  const unks=axes.filter(a=>a.tripped===null);
  if(unreachable()) b=t("unreach");
  else if(trips.length) b=trips.map(a=>t("axisN",a.node)+" "
      +(a.trip_label?sv(a.trip_label,a.trip_label_en):t("tripping"))).join(tw("／"))
    +(st.trip_action?(LANG==="en"?". ":" ── ")+sv(st.trip_action,st.trip_action_en):"");
  if(!unreachable()&&unks.length){
    const g=[], by={};
    for(const a of unks){
      const ja=a.trip_unknown||st.trip_unknown, en=a.trip_unknown?a.trip_unknown_en:st.trip_unknown_en;
      if(!ja) continue;
      if(!(ja in by)){ by[ja]={en:en, ns:[]}; g.push(ja); }
      by[ja].ns.push(a.node);
    }
    for(const ja of g) w=join2(w, t("axisN",by[ja].ns.join(tw("・")))+": "+sv(ja,by[ja].en));
  }
  if(st.stall) w=join2(w,st.stall);
  if(st.wd_fired) w=join2(w,t("wdFired"));
  if(recLast&&recLast.err) w=join2(w,recErrText(recLast));

  if(st.note&&st.note_seq!==undefined&&st.note_seq!==lastNoteSeq){
    lastNoteSeq=st.note_seq;
    const nm=sv(st.note,st.note_en);
    if(!(flash&&flash.cls==="warn"&&Date.now()<flash.until&&flash.msg===nm))
      flash={msg:nm, cls:"info", until:Date.now()+6000, trip:!!st.note_trip, seq:st.note_seq};
  }

  const an=sv(st.arm_note||"",st.arm_note_en);
  const said=(m)=>b.indexOf(m)>=0||w.indexOf(m)>=0||an.indexOf(m)>=0;
  const tripSaid=!unreachable()&&trips.length>0;
  const fm=(flash&&flash.seq!==undefined&&flash.seq===st.note_seq&&st.note_cut)?sv(st.note_cut,st.note_cut_en):(flash&&flash.msg);
  const fl=(flash&&Date.now()<flash.until&&!said(fm)&&!(flash.trip&&tripSaid))?fm:"";
  const dw=prmDropText();
  const i=join2(join2(fl, dw), an);
  const show=(e,on)=>{ const d=on?"block":"none"; if(e.style.display!==d) e.style.display=d; };
  setText(bad,b); show(bad,b);
  setText(warn,w); show(warn,w);
  const icn="banner "+((an||dw||(fl&&flash.cls==="warn"))?"warn":"info");
  setText(info,i); if(info.className!==icn) info.className=icn;
  show(info,i);
  document.body.classList.toggle("stale", unreachable());
}

function render(){
  document.getElementById("dot").className="dot"
    +((st.axes||[]).some(a=>a.alive)&&!unreachable()?" on":"");
  const cl=document.getElementById("chipLoop");
  const held=!st.armed&&(st.axes||[]).some(a=>a.held);
  setText(document.getElementById("loopNum"), st.loop_ms===undefined?"":t("loop",num(st.loop_ms,3),num(st.loop_sd_ms,3)));
  setText(document.getElementById("loopSt"), st.armed?t("loopArmed"):(held?t("loopHeld"):t("loopDisarmed")));
  cl.className="chip"+((st.armed||held)&&!unreachable()?" warn":"");
  document.getElementById("btnDisarm").disabled=!st.armed;
  renderCards();
  banners();
}

async function poll(){
  if(inflight.st){ render(); return; }
  inflight.st=true;
  try{
    const r=await fetch("/api/state?since="+seq+"&cid="+CID,
                        {cache:"no-store", signal:wdSignal()});
    st=await r.json(); offline=0; lastOk=Date.now();
    lim=st.limits||lim;
    seq=st.seq_last||seq;
    document.getElementById("btnDemo").disabled=!demoReady();
    for(const a of (st.axes||[])){
      if(!hist[a.node]) hist[a.node]=[];
      for(const s of (a.samples||[])) hist[a.node].push(s);
      const t0=hist[a.node].length?hist[a.node][hist[a.node].length-1][0]:0;
      hist[a.node]=hist[a.node].filter(p=>p[0]>t0-11);
    }
    render();
  }catch(e){
    offline++;
    if(offline>3) document.getElementById("dot").className="dot";
    banners();
  }finally{ inflight.st=false; }
}

let last=performance.now();
function slide(now){
  const d=(now-last)/1000; last=now;
  for(const k in hist) for(const p of hist[k]) p[0]-=d;
  drawAxes();
  requestAnimationFrame(slide);
}

document.getElementById("btnDisarm").onclick=()=>post("disarm");
document.getElementById("btnEstop").onclick=()=>post("estop");
ocWire();
wcWire();
let recOn=false, recLast={}, recSecs=null;

function recErrText(l){ return t("recErr",l.file,l.rows,l.err); }
function recButton(b, on, secs, last, idle){
  b.className = on ? "rec" : ((last&&last.err) ? "warn" : (idle||""));
  const h = on ? t("recOnHtml") : (idle==="pri" ? t("recStartHtml") : t("recIdleHtml"));
  if(b.dataset.h!==h){ b.dataset.h=h; b.innerHTML=h; }
  if(on){ const s=b.querySelector(".secs"); const t=has(secs)?secs.toFixed(0):"0"; if(s&&s.textContent!==t) s.textContent=t; }
  b.title = on ? t("recOnTip")
             : ((last&&last.err) ? recErrText(last) : t("recTip"));
}
async function pollRecM(){ if(inflight.rec) return; inflight.rec=true;
  try{ const r=await fetch("/api/rec",{cache:"no-store", signal:wdSignal()}); const j=await r.json(); recOn=!!(j.rec&&j.rec.on); recSecs=j.rec&&j.rec.secs;
  recLast=(j.rec&&j.rec.last)||{};
  recButton(document.getElementById("btnRecM"), recOn, j.rec&&j.rec.secs, recLast, ""); banners(); }catch(e){}finally{ inflight.rec=false; } }
document.getElementById("btnRecM").onclick=()=>post(recOn?"rec_stop":"rec_start",{});
pollRecM(); setInterval(pollRecM,1000);
document.getElementById("btnClear").onclick=()=>post("clear");
document.getElementById("btnRescan").onclick=()=>post("rescan");
document.getElementById("btnDemo").onclick=()=>{ location.href="/demo"; };
function demoReady(){ const ax=(st&&st.axes)||[];
  for(let n=1;n<=12;n++){ const a=ax.find(x=>x.node===n); if(!a||!a.board||!a.board.alive) return false; } return true; }
function applyLang(){
  document.title=t("title");
  document.documentElement.lang=LANG;
  const T=(id,k,tip)=>{ const e=document.getElementById(id); if(!e) return; setText(e,t(k)); if(tip) e.title=t(tip); };
  T("hTitle","title"); T("btnRescan","rescan","rescanTip");
  setText(document.getElementById("btnDemo"), LANG==="en"?"12-axis demo":"12 軸デモ"); T("btnDisarm","disarmAll","disarmAllTip");
  T("btnClear","clear"); T("btnEstop","estop");
  const bl=document.getElementById("btnLang");
  setHtml(bl,"<span"+(LANG==="ja"?" class='lon'":"")+">JA</span> | <span"+(LANG==="en"?" class='lon'":"")+">EN</span>");
  bl.title=t("langTip");
  recButton(document.getElementById("btnRecM"), recOn, recSecs, recLast, "");
}
function setLang(l){
  LANG=(l==="en")?"en":"ja";
  try{ localStorage.setItem("krc.lang",LANG); }catch(e){}
  applyLang(); render();
}
document.getElementById("btnLang").onclick=()=>setLang(LANG==="en"?"ja":"en");
applyLang();
window.addEventListener("keydown",e=>{
  if(e.code==="Space"&&e.target&&e.target.dataset&&e.target.dataset.ptpts!==undefined) return;
  if(e.code==="Space"||e.code==="Escape"){ e.preventDefault(); post("estop"); }
});
document.addEventListener("visibilitychange",()=>{
  if(document.hidden) post("disarm");
});
window.addEventListener("pagehide",()=>{
  navigator.sendBeacon("/api/cmd",
    new Blob([JSON.stringify({action:"disarm",params:{cid:CID}})],
             {type:"application/json"}));
});

poll(); setInterval(poll,50); requestAnimationFrame(slide);
pollTemps(); setInterval(pollTemps,1000);
</script></body></html>
"""


LEG_PAGE = r"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Claude Leg — 実機</title>
<style>
/*__COMMON_CSS__*/
body{margin:0;background:var(--bg);color:var(--fg);font-size:14px;line-height:1.5}
header{display:flex;gap:10px;align-items:center;padding:10px 14px;border-bottom:1px solid var(--line);
  background:var(--panel);position:sticky;top:0;z-index:3;flex-wrap:wrap}
header b{font-size:15px}
select,input[type=number],input[type=text],textarea{background:var(--bg);color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:4px 8px;font-size:13px}

select,input[type=number],input[type=text]{font:inherit;font-size:13px;line-height:19px;padding:6px 8px}
select{height:33px}

.wrap{display:grid;grid-template-columns:minmax(420px,36%) 1fr;gap:14px;padding:14px;max-width:1500px;margin:0 auto}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.panel h2{font-size:13px;margin:0 0 8px;color:var(--dim);font-weight:600;letter-spacing:.03em}

.row{display:flex;align-items:center;gap:8px;margin:6px 0;flex-wrap:wrap}
.row>label{width:88px;flex:none;color:var(--dim);font-size:12px}
.row label.s{width:auto;margin-left:6px}
.row .f{display:inline-flex;align-items:center;gap:6px;white-space:nowrap;color:var(--dim)}
.row .f label{color:var(--dim);font-size:12px}
.row button{white-space:nowrap}
.row input[type=range]{flex:1;min-width:120px;accent-color:var(--cyan)}
.row .n{width:70px;text-align:right;font-variant-numeric:tabular-nums}
.big{font-size:26px;font-weight:700;font-variant-numeric:tabular-nums}

body.stale .big,body.stale #kv{opacity:.45}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-size:12px;color:var(--dim)}
.kv b{color:var(--fg);font-weight:500;font-variant-numeric:tabular-nums}
.banner{margin:10px 14px 0}
#cvWrap{position:relative}

@media(min-width:901px) and (min-height:700px){#cvWrap{position:sticky;top:calc(var(--hdr) + 14px);align-self:start}}
svg#cv{width:100%;height:calc(100vh - 140px);min-height:560px;background:var(--bg);border:1px solid var(--line);border-radius:10px;display:block}

.cvmsg{position:absolute;left:14px;top:10px;font-size:12px;font-weight:600;display:none;color:var(--dim)}
.cvmsg.warn{color:var(--warn)}
.cvmsg.bad{color:var(--bad)}

@media(max-width:900px){
  .wrap{grid-template-columns:1fr;gap:12px}
  .wrap>.col{display:contents}
  .panel{order:2;margin-top:0!important}
  .panel.p1,.panel.p2{order:0}
  #cvWrap{order:1}
  svg#cv{height:min(60vh,480px);min-height:0}
}
.lnk{stroke-width:3;stroke-linecap:round;fill:none}
.pt{fill:#c9d1d9}
.lbl{fill:var(--dim);font-size:12px}
.lbl.b{fill:var(--fg);font-weight:600}
.lbl.c7{fill:var(--c7)}
.ghost{stroke:var(--c7);stroke-dasharray:6 5;stroke-opacity:.6;stroke-width:2.5;fill:none}
.fan{fill:var(--c7);fill-opacity:.12;stroke:var(--dim);stroke-dasharray:4 4;stroke-width:1}
.fan.reach{fill:var(--cyan);fill-opacity:.10;stroke:none}
.arc{fill:none;stroke-width:1.5}
.note{color:var(--dim);font-size:12px;line-height:1.7;margin-top:8px;overflow-wrap:anywhere}
.trace{font-size:11px;color:var(--dim);white-space:pre-line;overflow-wrap:anywhere}
.trace a,.trace a:visited{color:var(--cyan)}
</style></head><body>
<header>
  <span class="dot" id="dot"></span><b>Claude Leg</b><span class="chip" id="chipArm">解除中</span>
  <label style="font-size:12px;color:var(--dim)">軸 <select id="node"></select></label>
  <span id="chipMode" class="chip"></span><span id="chipSync" class="chip"></span><span id="chipBoard" class="chip bad" style="display:none"></span>
  <span style="flex:1"></span>
  <button id="btnRec"><span class="rdot"></span> 記録</button>
  <button id="btnArm" class="pri">通電する</button>
  <button id="btnDisarm">解除</button>
  <button id="btnEstop" class="estop">非常停止</button>
</header>
<div class="banner bad" id="bnBad"></div>
<div class="banner warn" id="bnWarn"></div>
<div class="banner info" id="bnInfo"></div>
<p class="note" id="noAxes" style="display:none">軸が見つかりません（CAN に返事をする基板が居ません）。基板の電源と CAN の配線を確かめてください。電源を入れた軸は、解除中なら自動で並びます。</p>
<div class="wrap" id="legWrap">
<div class="col">
  <div class="panel p1">
    <h2>キャリブレーション</h2>
    <button id="btnHome" class="pri" style="width:100%">原点出し（畳み切りのストッパに当てて 0 点を取る）</button>
    <div class="note" id="homeNote">先に下の「畳み切りは＋側か−側か」を選んでから、通電して押します。<b>畳み切りのストッパにだけ</b>当てます（違う力で 4 回当て直し、力 0 へ外挿した位置をストッパにします）。そこで Θout = 156°。原点はそこから設計の半幅ぶん戻った所、壁は 畳み切り側 = ストッパ（156°）・伸ばし切り側 = 設計の 48°。壁は保存しません（電源を入れ直したらもう一度）。</div>
    <div class="trace" id="homeTrace"></div>
  </div>
  <div class="panel p2" style="margin-top:12px">
    <h2>膝の角 Θout（出力軸）</h2>
    <div class="big"><span id="toutNow">–</span><span style="font-size:14px;color:var(--dim)">°　いま</span></div>
    <div class="row"><label>目標 Θout</label><input type="range" id="tout" min="48" max="156" step="0.5" value="100"><span class="n" id="toutV">100.0</span></div>
    <div class="row"><label>硬さ Kp</label><input type="range" id="kp" min="0" max="80" step="0.5" value="12"><span class="n" id="kpV">12.0</span></div>
    <div class="row"><label>ダンパ Kd</label><input type="range" id="kd" min="0" max="1" step="0.01" value="0.15"><span class="n" id="kdV">0.15</span></div>
    <div class="row"><label>モード</label><button id="btnFree">脱力</button><button id="btnDamp">ダンパ</button><button id="btnHold">位置保持（いまの角）</button></div>
    <div class="note">逆駆動の試験は「脱力」か「ダンパ」で押す（記録は動いたままにする）</div>
    <div class="note" id="reachNote"></div>
    <div class="kv" id="kv"></div>
  </div>
  <div class="panel" style="margin-top:12px">
    <h2>正弦波（出力軸）</h2>
    <div class="row"><label>最小 Θout</label><span class="f"><input type="number" id="sLo" step="1" min="48" max="156" value="70" style="width:72px"><span>°</span></span>
      <span class="f"><label class="s">最大</label><input type="number" id="sHi" step="1" min="48" max="156" value="130" style="width:72px"><span>°</span></span></div>
    <div class="row"><label>周期</label><span class="f"><input type="number" id="sPeriod" step="0.1" min="0.1" max="60" value="2" style="width:72px"><span>s</span></span>
      <button id="btnSine" class="pri" style="margin-left:6px">開始</button><button id="btnSineStop">止める</button></div>
    <div class="note" id="sineNote">いまの膝の角から範囲の中へ滑って入り、着いたら走ります。走っている間は滑らせません（軌道の速度をそのまま出す）。止めるとその場で位置保持に戻ります。</div>
  </div>
  <div class="panel" style="margin-top:12px">
    <h2>重力補償</h2>
    <div class="row"><label>前置き</label><button id="btnGravOn">有効</button><button id="btnGravOff">無効</button></div>
    <div class="note" id="gravState" style="margin-top:0"></div>
    <div class="row"><label>A</label><span class="f"><input type="number" id="gravA" step="0.01" value="0" style="width:72px"><span>N·m</span></span>
      <span class="f"><label class="s">位相</label><input type="number" id="gravPh" step="1" value="0" style="width:72px"><span>°</span></span>
      <button id="btnGravSet" style="margin-left:6px">この値にする</button></div>
    <div class="row"><label>合わせる</label><button id="btnGravSample" class="pri">いまの角で測る</button><button id="btnGravFit">標本から合わせる</button><button id="btnGravClear" class="ghost">消す</button></div>
    <div class="trace" id="gravSamples"></div>
    <div class="note">膝を保持するのに要るトルクを τ = A·sin(Θout − 位相) と置き、位置保持で止めた状態の電流から <b>違う角で 2 回以上「測る」→「合わせる」</b>で A と位相が決まります（質量・重心・取り付けの向きを別々に知らなくてよい）。有効にすると、位置保持・正弦波・軌道のあいだ、この分を前置きトルクとして足します（±1 N·m の枠の内側）。自重を掛けたら測り直してください。</div>
  </div>
  <div class="panel" style="margin-top:12px">
    <h2>軌道の再生（出力軸）</h2>
    <textarea id="trajText" rows="5" style="width:100%;font-family:ui-monospace,monospace;font-size:12px" placeholder="t,Θout（1 行 1 点。t は 0 から秒で単調増加）&#10;0,100&#10;0.5,130&#10;1.0,100&#10;1.5,70&#10;2.0,100"></textarea>
    <div class="row"><label><input type="checkbox" id="trajLoop" checked> 繰り返す</label>
      <span class="f"><label class="s">速さ</label><input type="number" id="trajSpeed" step="0.1" value="1" style="width:64px"><span>×</span></span>
      <button id="btnTraj" class="pri" style="margin-left:6px">開始</button><button id="btnTrajStop">止める</button></div>
    <div class="note" id="trajNote">点列を線形につないで再生します。いまの膝の角から最初の点へ滑って入り、着いたら走ります（走行中は滑らせません）。壁の外・閉じない角があれば断ります。</div>
  </div>
  <div class="panel" style="margin-top:12px">
    <h2>記録（200 Hz・全軸・CSV）</h2>
    <div class="row"><label>名前</label><input type="text" id="recName" maxlength="40" placeholder="英数と _-・40 字まで" title="英数と _- だけ・40 字まで（それ以外はサーバが黙って落とす）" style="width:160px"><button id="btnRec2" class="pri"><span class="rdot"></span> 記録 開始</button></div>
    <div class="note" id="recNote"></div>
    <div class="trace" id="recFiles"></div>
  </div>
  <div class="panel" style="margin-top:12px">
    <h2>対応（畳み切りのストッパはどちら側か）</h2>
    <div class="row"><label>畳み切りは</label><button id="btnFold">＋側（モーターを＋に回すと畳む）</button><button id="btnExt">−側（−に回すと畳む）</button></div>
    <div class="kv" id="calkv"></div>
    <div class="note">基準は<b>畳み切りのストッパ 1 点</b>（そこで Θout = 156°・設計）。逆解で Θin が決まり、そのときのモーター角と結び付きます。あとは計算式だけ：目標 Θout → 逆解で Θin → 基準からの差分をモーターに回す。減速比は 9 で固定。伸ばし切り側は測らず、設計の 48° を壁にします。分からないのは「畳み切りが＋側か−側か」だけなので、<b>原点出しの前に</b>それを選びます（その向きにだけ当てます）。</div>
    <details style="margin-top:8px"><summary style="color:var(--dim);cursor:pointer">ストッパが 156° ちょうどでないとき（測った角で基準を置き換える）</summary>
    <div class="row"><span class="f"><span>いま膝は</span><input type="number" id="toutCal" step="0.5" min="48" max="156" value="100" style="width:72px"><span>°</span></span><button id="btnCal1">この角を基準にする</button><button id="btnShift0" class="ghost">ストッパ基準に戻す</button></div>
    </details>
  </div>
</div>
<div id="cvWrap"><svg id="cv" xmlns="http://www.w3.org/2000/svg"></svg><div class="cvmsg" id="cvMsg"></div></div>
</div>
<script>
const CID = Math.random().toString(36).slice(2, 10);
let st = {}, leg = {}, D = null, node = null, offline = 0, dragging = false, lastSend = 0, pendingTout = null;

let lastOk=Date.now();
function wdMs(){ return (st.wd_s||0)*1000; }
function unreachable(){ return offline>3 || (wdMs()>0 && Date.now()-lastOk>wdMs()); }
function wdSignal(){ return wdMs()>0 ? AbortSignal.timeout(wdMs()) : undefined; }

const inflight={};

function setText(e,s){ if(e&&e.dataset.t!==s){ e.dataset.t=s; e.textContent=s; } }
function setHtml(e,h){ if(e&&e.dataset.h!==h){ e.dataset.h=h; e.innerHTML=h; } }
let gravEdit = false;
const $ = (s) => document.querySelector(s);
const END_LABEL = {extend: "−側", fold: "＋側"};
const NS = "http://www.w3.org/2000/svg";
function esc(s){ return String(s).replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c])); }
function has(v){ return v !== null && v !== undefined; }
async function post(action, params){
  try{
    const r = await fetch("/api/cmd", {method:"POST", headers:{"Content-Type":"application/json"},
      body: JSON.stringify({action, params: Object.assign({cid: CID}, params||{})})});
    const j = await r.json();
    if (j && j.ok === false && j.err) showInfo(j.err, "warn");
    return j;
  }catch(e){ showInfo("サーバに届きませんでした", "warn"); return {ok:false}; }
}

const SEP="　／　";
function join2(a,b){ return a&&b ? a+SEP+b : (a||b||""); }
let flash=null, lastNoteSeq=-1, recOn=false, recLast={};
function showInfo(msg, cls){ flash={msg:msg, cls:(cls||"warn"), until:Date.now()+6000}; banners(); }
function recErrText(l){ return `前回の記録 ${l.file} は途中で書けなくなりました（書けたのは ${l.rows} 行）: ${l.err}`; }
function banners(){
  const bad=$("#bnBad"), warn=$("#bnWarn"), info=$("#bnInfo");
  let b="", w="";
  const axes=st.axes||[];
  const trips=axes.filter(a=>a.tripped===true);
  const unks=axes.filter(a=>a.tripped===null);
  if(unreachable()) b="画面がサーバに届いていません。ラズパイと LAN を確かめてください。";
  else if(trips.length) b=trips.map(a=>"軸"+a.node+" "+(a.trip_label||"遮断しています")).join("／")
    +(st.trip_action?" ── "+st.trip_action+"（→ アクチュエータの画面）":"");
  if(!unreachable()&&unks.length){
    const g=[], by={};
    for(const a of unks){ const s=a.trip_unknown||st.trip_unknown; if(!s) continue; if(!(s in by)){ by[s]=[]; g.push(s); } by[s].push(a.node); }
    for(const s of g) w=join2(w, "軸"+by[s].join("・")+": "+s);
  }
  if(st.stall) w=join2(w,st.stall);
  if(st.wd_fired) w=join2(w,"画面からの応答が途切れたので停止しました。");
  if(recLast&&recLast.err) w=join2(w,recErrText(recLast));
  if(st.note&&st.note_seq!==undefined&&st.note_seq!==lastNoteSeq){ lastNoteSeq=st.note_seq;
    if(!(flash&&flash.cls==="warn"&&Date.now()<flash.until&&flash.msg===st.note)) flash={msg:st.note, cls:"info", until:Date.now()+6000, trip:!!st.note_trip, seq:st.note_seq}; }
  const an=st.arm_note||"";
  const said=(m)=>b.indexOf(m)>=0||w.indexOf(m)>=0||an.indexOf(m)>=0;
  const fm=(flash&&flash.seq!==undefined&&flash.seq===st.note_seq&&st.note_cut)?st.note_cut:(flash&&flash.msg);
  const fl=(flash&&Date.now()<flash.until&&!said(fm)&&!(flash.trip&&!unreachable()&&trips.length>0))?fm:"";
  const i=join2(fl, an);
  const show=(e,on)=>{ const d=on?"block":"none"; if(e.style.display!==d) e.style.display=d; };
  setText(bad,b); show(bad,b);
  setText(warn,w); show(warn,w);
  const icn="banner "+((an||(fl&&flash.cls==="warn"))?"warn":"info");
  setText(info,i); if(info.className!==icn) info.className=icn; show(info,i);
  document.body.classList.toggle("stale", unreachable());
}

function recButton(b, on, secs, last, idle){
  b.className = on ? "rec" : ((last&&last.err) ? "warn" : (idle||""));
  const h = on ? "<span class='rdot blink'></span> 記録中 <span class='secs'></span> s"
               : (idle==="pri" ? "<span class='rdot'></span> 記録 開始" : "<span class='rdot'></span> 記録");
  if(b.dataset.h!==h){ b.dataset.h=h; b.innerHTML=h; }
  if(on){ const s=b.querySelector(".secs"); const t=has(secs)?secs.toFixed(0):"0"; if(s&&s.textContent!==t) s.textContent=t; }
  b.title = on ? "押すと記録を止める（記録は動いたまま。非常停止ではない）"
             : ((last&&last.err) ? recErrText(last) : "200 Hz の記録（CSV）");
}

function figState(a, sa){
  if(!a || !a.synced) return {ok:false, cls:"warn", msg:"図は実機と同期していません — "+((a&&a.sync_why)||"")};
  if(unreachable()) return {ok:false, cls:"bad", msg:"画面がサーバに届いていません — 実機の位置が読めていません"};
  if(sa && sa.alive===false) return {ok:false, cls:"bad", msg:"基板 応答なし — 実機の位置が読めていません"};
  if(!has(a.tin)) return {ok:false, cls:"warn", msg:"実機の位置がまだ読めていません"};
  return {ok:true, cls:"", msg:""};
}


const add=(p,q)=>[p[0]+q[0],p[1]+q[1]], sub=(p,q)=>[p[0]-q[0],p[1]-q[1]], mul=(p,s)=>[p[0]*s,p[1]*s];
const len=(p)=>Math.hypot(p[0],p[1]);
const rot=(v,a)=>[v[0]*Math.cos(a)-v[1]*Math.sin(a), v[0]*Math.sin(a)+v[1]*Math.cos(a)];
const wrap180=(d)=>((d+180)%360+360)%360-180;
function solve(tinDeg){
  const l1=D.L1, K1=l1/D.L4, K2=l1/D.L6, K3=(l1*l1+D.L4*D.L4+D.L6*D.L6-D.L5*D.L5)/(2*D.L4*D.L6);
  const t=D.SG*tinDeg*Math.PI/180, A=K1-Math.cos(t), B=-Math.sin(t), C=K2*Math.cos(t)-K3, R=Math.hypot(A,B);
  const ok=Math.abs(C)<=R; if(!ok) return {ok:false};
  const psi=Math.atan2(B,A)+Math.acos(Math.max(-1,Math.min(1,C/R)));
  return {ok:true, psi, tout: wrap180(D.SG*(psi*180/Math.PI+180+D.SG*D.ALPHA))};
}
function geometry(tinDeg){
  const t=D.TILT*Math.PI/180, A=[0,0], u=[Math.sin(t),-Math.cos(t)], n=[-u[1],u[0]], B=mul(u,D.L1);
  const al=Math.atan2(D.L3,D.L2), C=add(A,add(mul(u,D.L2*Math.cos(al)),mul(n,D.L2*Math.sin(al))));
  const X=add(B,mul(u,D.L1*0.5));
  const g={A,B,C,X,u,n,ok:false};
  if(!has(tinDeg)) return g;
  const sol=solve(tinDeg), tin=D.SG*tinDeg*Math.PI/180;
  g.Dp=add(A,add(mul(u,D.L4*Math.cos(tin)),mul(n,D.L4*Math.sin(tin))));
  if(!sol.ok) return g;
  const eDir=add(mul(u,Math.cos(sol.psi)),mul(n,Math.sin(sol.psi)));
  g.E=add(B,mul(eDir,D.L6)); const ext6=mul(eDir,-1); g.Y=add(B,mul(ext6,D.L6*0.5));
  g.F=add(B,mul(rot(ext6,D.SG*D.ALPHA*Math.PI/180),D.L7)); g.tout=sol.tout; g.ok=true; return g;
}
function footFor(toutDeg){
  const t=D.TILT*Math.PI/180, u=[Math.sin(t),-Math.cos(t)], B=mul(u,D.L1);
  return add(B, mul(rot(u,-toutDeg*Math.PI/180), D.L7));
}
const svg=$("#cv");
function el(tag,attrs,text){ const e=document.createElementNS(NS,tag); for(const k in attrs) e.setAttribute(k,attrs[k]); if(text!=null) e.textContent=text; return e; }
function render(){
  if(!D) return;
  const a = curAxis();
  const sa = (st.axes||[]).find(x=>x.node===node)||{};
  const fs = figState(a, sa);
  const synced = fs.ok;
  const tin = synced ? a.tin : null;
  svg.style.opacity = synced ? "1" : "0.45";
  const g = geometry(tin);
  const W = svg.clientWidth||900, H = svg.clientHeight||600;
  const pts=[g.A,g.B,g.C,g.X, footFor(D.PHI1), footFor(D.PHI2)]; if(g.F) pts.push(g.F); if(g.Dp) pts.push(g.Dp);
  let minx=Infinity,maxx=-Infinity,miny=Infinity,maxy=-Infinity;
  for(const p of pts){ minx=Math.min(minx,p[0]); maxx=Math.max(maxx,p[0]); miny=Math.min(miny,p[1]); maxy=Math.max(maxy,p[1]); }
  const pad=60, sc=Math.min((W-2*pad)/(maxx-minx||1),(H-2*pad)/(maxy-miny||1));
  const ox=(W-(maxx-minx)*sc)/2 - minx*sc, oy=(H+(maxy-miny)*sc)/2 + miny*sc;
  const T=(p)=>[ox+p[0]*sc, oy-p[1]*sc];
  const P=(p)=>T(p).map(v=>v.toFixed(1)).join(",");
  while(svg.firstChild) svg.removeChild(svg.firstChild);
  const arcPath=(c,r,a0,a1)=>{ const s=T(add(c,mul([Math.cos(a0),Math.sin(a0)],r))), e=T(add(c,mul([Math.cos(a1),Math.sin(a1)],r)));
    const big=Math.abs(a1-a0)>Math.PI?1:0; const sw=(a1<a0)?1:0;
    return `M${P(c)} L${s.map(v=>v.toFixed(1)).join(",")} A${(r*sc).toFixed(1)},${(r*sc).toFixed(1)} 0 ${big} ${sw} ${e.map(v=>v.toFixed(1)).join(",")} Z`; };
  const base=Math.atan2(g.u[1],g.u[0]);
  svg.appendChild(el("path",{class:"fan", d: arcPath(g.B, D.L7*0.55, base-D.PHI1*Math.PI/180, base-D.PHI2*Math.PI/180)}));
  if(a && has(a.reach_lo) && has(a.reach_hi))
    svg.appendChild(el("path",{class:"fan reach", d: arcPath(g.B, D.L7*0.55, base-a.reach_lo*Math.PI/180, base-a.reach_hi*Math.PI/180)}));
  svg.appendChild(el("line",{x1:T(g.B)[0],y1:T(g.B)[1],x2:T(g.X)[0],y2:T(g.X)[1],stroke:"var(--c1)","stroke-dasharray":"5 5","stroke-opacity":.5}));
  const line=(p,q,c,cls)=>svg.appendChild(el("line",{x1:T(p)[0],y1:T(p)[1],x2:T(q)[0],y2:T(q)[1],stroke:c,class:cls||"lnk"}));
  line(g.A,g.B,"var(--c1)"); line(g.A,g.C,"var(--c2)"); line(g.B,g.C,"var(--c3)");
  if(g.Dp){ line(g.A,g.Dp,"var(--c4)"); }
  if(g.ok){ line(g.Dp,g.E,"var(--c5)"); line(g.E,g.B,"var(--c6)"); line(g.B,g.F,"var(--c7)"); }
  if(a && has(a.target_tout) && a.mode==="hold"){ const Ft=footFor(a.target_tout); line(g.B,Ft,"","ghost"); }
  const dot=(p,name,dx,dy)=>{ svg.appendChild(el("circle",{cx:T(p)[0],cy:T(p)[1],r:4,class:"pt"})); svg.appendChild(el("text",{x:T(p)[0]+(dx||8),y:T(p)[1]+(dy||-6),class:"lbl"},name)); };
  dot(g.A,"A",-18,-8); dot(g.B,"B",10,16); dot(g.C,"C",10,-6); if(g.Dp) dot(g.Dp,"D",0,-10); if(g.E) dot(g.E,"E",10,10); if(g.F) dot(g.F,"F",-18,16);
  if(has(tin)){ const r=D.L4*1.6; const tl=D.SG*tin, a0=base, a1=base+tl*Math.PI/180;
    svg.appendChild(el("path",{class:"arc",stroke:"var(--c4)", d:`M${P(add(g.A,mul([Math.cos(a0),Math.sin(a0)],r)))} A${(r*sc).toFixed(1)},${(r*sc).toFixed(1)} 0 ${Math.abs(tl)>180?1:0} ${tl>=0?0:1} ${P(add(g.A,mul([Math.cos(a1),Math.sin(a1)],r)))}`}));
    svg.appendChild(el("text",{x:T(g.A)[0]+30,y:T(g.A)[1]+22,class:"lbl b"},"Θin "+tin.toFixed(1)+"°")); }
  if(g.ok){ const r=D.L6*1.8; const a0=base, a1=base-g.tout*Math.PI/180;
    svg.appendChild(el("path",{class:"arc",stroke:"var(--c7)", d:`M${P(add(g.B,mul([Math.cos(a0),Math.sin(a0)],r)))} A${(r*sc).toFixed(1)},${(r*sc).toFixed(1)} 0 0 1 ${P(add(g.B,mul([Math.cos(a1),Math.sin(a1)],r)))}`}));
    svg.appendChild(el("text",{x:T(g.B)[0]-10,y:T(g.B)[1]+60,class:"lbl b c7"},"Θout "+g.tout.toFixed(1)+"°")); }
  const mid=(p,q,name,val)=>{ const m=T(mul(add(p,q),0.5)); svg.appendChild(el("text",{x:m[0]+6,y:m[1]-6,class:"lbl"},name+" "+val.toFixed(1)+" mm")); };
  mid(g.A,g.B,"L1",D.L1); if(g.Dp) mid(g.A,g.Dp,"L4",D.L4); if(g.ok){ mid(g.Dp,g.E,"L5",D.L5); mid(g.B,g.F,"L7",D.L7); }
  { const m=$("#cvMsg");
    const txt = !synced ? fs.msg : (g.ok ? "" : "この Θin ではリンクが閉じません（較正の向き／原点を確かめてください）");
    const cls = !synced ? fs.cls : "bad";
    setText(m,txt); const mc="cvmsg"+(txt?" "+cls:""); if(m.className!==mc) m.className=mc;
    const md=txt?"block":"none"; if(m.style.display!==md) m.style.display=md; }
}
function curAxis(){ return (leg.axes||[]).find(a=>a.node===node) || null; }
function renderUI(){
  const a=curAxis();
  const sa=(st.axes||[]).find(x=>x.node===node)||{};
  $("#dot").className="dot"+((st.axes||[]).some(x=>x.alive)&&!unreachable()?" on":"");
  { const held=!leg.armed&&(st.axes||[]).some(x=>x.held);
    setText($("#chipArm"), leg.armed ? "通電中" : (held ? "基板が保持中" : "解除中"));
    $("#chipArm").className="chip"+((leg.armed||held)&&!unreachable()?" warn":""); }
  $("#btnArm").disabled = !!leg.arm_block; $("#btnDisarm").disabled = !leg.armed;
  const sel=$("#node"); const nodes=(leg.axes||[]).map(x=>x.node);
  if(sel.options.length!==nodes.length || [...sel.options].some((o,i)=>+o.value!==nodes[i])){
    sel.innerHTML=nodes.map(n=>`<option value="${n}">${n}</option>`).join(""); if(node===null&&nodes.length) node=nodes.includes(2)?2:nodes[0]; sel.value=node; }
  { const none=!nodes.length; $("#noAxes").style.display=none?"":"none"; $("#legWrap").style.display=none?"none":""; }
  if(!a){ setText($("#chipMode"), ""); return; }
  const ml={free:"脱力",damp:"ダンパ",hold:"位置保持",sine:"正弦波",vel:"速度",torque:"トルク",kt_test:"トルクテスト",pos_test:"位置制御テスト",homing:"原点出し中",leg_sine:"脚の正弦波",leg_traj:"脚の軌道"};
  const sn=a.sine||{};
  $("#btnSine").disabled = !a.synced || !leg.armed; $("#btnSineStop").disabled = !sn.on;
  setText($("#btnSine"), sn.on ? "範囲・周期を当て直す" : "開始");
  $("#btnSineStop").className = sn.on ? "pri" : ""; $("#btnSine").className = sn.on ? "" : "pri";
  { const el=$("#sineNote"); const tx = sn.on
      ? (sn.run ? `走行中: Θout ${sn.lo.toFixed(0)}〜${sn.hi.toFixed(0)}°・周期 ${sn.period.toFixed(2)} s・いまの目標 ${has(a.target_tout)?a.target_tout.toFixed(1):"–"}°・いま ${has(a.tout)?a.tout.toFixed(1):"–"}°・モーター最大 ${has(sn.vmax)?sn.vmax.toFixed(1):"–"} rad/s`
               : "始点へ寄っています…")
      : "いまの膝の角から範囲の中へ滑って入り、着いたら走ります。走っている間は滑らせません（軌道の速度をそのまま出す）。止めるとその場で位置保持に戻ります。";
    if(el.textContent!==tx) el.textContent=tx; }
  { const jt=st.jt, inJt=!!(jt&&(jt.nodes||[]).indexOf(a.node)>=0), jn=inJt?(jt.name||"jtraj"):"";
    const mt=!inJt ? (ml[a.mode]||a.mode)
           : (jt.stage==="stopping" ? "軌道を止めています（"+jn+"）" : (jt.stage==="approach" ? "軌道の始点へ（"+jn+"）" : "軌道を再生中（"+jn+"）"));
    setText($("#chipMode"), (a.tripped===true?"遮断 ":(a.tripped===null?"状態不明 ":""))+mt); }
  $("#chipMode").className="chip"+(a.tripped===true?" bad":((a.tripped===null||a.mode==="homing")?" warn":""));
  { const cb=$("#chipBoard"); const dead=(sa.alive===false); setText(cb, dead?"基板 応答なし":""); cb.style.display=dead?"":"none"; }
  $("#btnHome").disabled = !leg.armed || a.mode==="homing" || a.tripped===true || !(a.cal && a.cal.plus_end);
  $("#btnHome").className = a.synced ? "" : "pri";
  for(const [id,m] of [["#btnFree","free"],["#btnDamp","damp"],["#btnHold","hold"]]){ const b=$(id); b.disabled = !leg.armed || a.mode==="homing"; b.className = (a.mode===m) ? "on" : ""; }
  setText($("#toutNow"), (a.synced && has(a.tout))? a.tout.toFixed(1) : "–");
  $("#tout").disabled = !a.synced; $("#btnCal1").disabled = !a.synced || !!a.cal_block; $("#btnShift0").disabled = !(a.cal && a.cal.ref_src==="manual") || !!a.cal_block;
  $("#btnExt").disabled = !!a.cal_block; $("#btnFold").disabled = !!a.cal_block;
  const chipS=$("#chipSync"); if(chipS){ setText(chipS, a.synced ? "同期中" : ("未同期: "+(a.sync_why||""))); chipS.className="chip"+(a.synced?((sa.alive===false||unreachable())?"":" ok"):" warn"); }
  if(a.synced && has(a.reach_lo) && has(a.reach_hi)){
    for(const id of ["#tout","#sLo","#sHi"]){ const e=$(id); const lo=+a.reach_lo.toFixed(2), hi=+a.reach_hi.toFixed(2); if(+e.min!==lo) e.min=lo; if(+e.max!==hi) e.max=hi; } }
  { const tv = has(a.target_tout) ? a.target_tout : (has(a.tout) ? a.tout : null);
    if(!dragging && has(tv)){ $("#tout").value=tv.toFixed(1); setText($("#toutV"), (+$("#tout").value).toFixed(1)); } }
  if(!dragging && has(sa.kp)){ $("#kp").value=sa.kp; setText($("#kpV"), (+sa.kp).toFixed(1)); }
  if(!dragging && has(sa.kd)){ $("#kd").value=sa.kd; setText($("#kdV"), (+sa.kd).toFixed(2)); }
  setText($("#reachNote"), (has(a.reach_lo)&&has(a.reach_hi)) ? `いまの壁（${(a.p_lo||0).toFixed(2)}〜${(a.p_hi||0).toFixed(2)} rad）で届く膝の角: ${a.reach_lo.toFixed(1)}° 〜 ${a.reach_hi.toFixed(1)}°` : (a.p_hi && Math.abs(a.p_hi - 12) < 0.01 ? "壁がありません（原点出しをすると付きます）" : ""));
  { const g=a.grav||{}; const on=g.on===true;
    $("#btnGravOn").className = on ? "on" : ""; $("#btnGravOff").className = on ? "" : "on";
    $("#btnGravOn").disabled = !a.synced; $("#btnGravSample").disabled = g.can_sample!==true;
    $("#btnGravFit").disabled = !((g.samples||[]).length>=2);
    setText($("#gravState"), (on ? `有効: A ${(+g.a_nm).toFixed(3)} N·m・位相 ${(+g.phase_deg).toFixed(1)}°・いまの前置き ${has(g.ff_motor)?(+g.ff_motor).toFixed(3):"–"} N·m（モーター）`+(has(a.grav_out)?`＝膝 ${(+a.grav_out).toFixed(2)} N·m`:"") : "無効") + (g.sampling ? "・測っています…" : (g.sample_block ? `・測れない: ${g.sample_block}` : "")));
    if(!gravEdit){ $("#gravA").value=(+(g.a_nm||0)).toFixed(3); $("#gravPh").value=(+(g.phase_deg||0)).toFixed(1); }
    const sm=(g.samples||[]); const tx = sm.length ? ("標本:\n"+sm.map((x,i)=>`${i+1}. Θout ${(+x[0]).toFixed(1)}° → 膝 ${(+x[1]).toFixed(3)} N·m`).join("\n")) : "標本なし";
    if($("#gravSamples").textContent!==tx) $("#gravSamples").textContent=tx; }
  { const tj=a.traj||{};
    $("#btnTraj").disabled = !a.synced || !leg.armed; $("#btnTrajStop").disabled = !tj.on;
    setText($("#btnTraj"), tj.on ? "当て直す" : "開始");
    $("#btnTrajStop").className = tj.on ? "pri" : ""; $("#btnTraj").className = tj.on ? "" : "pri";
    const tx = tj.on ? (tj.run ? `走行中: ${tj.n_pts} 点・${has(tj.total_s)?tj.total_s.toFixed(2):"–"} s${tj.loop?"（繰り返し）":"（1 回）"}・速さ ${(+tj.speed).toFixed(2)}×・いまの目標 ${has(a.target_tout)?a.target_tout.toFixed(1):"–"}°・いま ${has(a.tout)?a.tout.toFixed(1):"–"}°` : "最初の点へ寄っています…")
                     : "点列を線形につないで再生します。いまの膝の角から最初の点へ滑って入り、着いたら走ります（走行中は滑らせません）。壁の外・閉じない角があれば断ります。";
    if($("#trajNote").textContent!==tx) $("#trajNote").textContent=tx; }
  { const r=a.rec||{}; const on=r.on===true;
    const last=r.last||{};
    const tx = on ? `記録中: ${r.file}・${r.rows} 行` : (last.err ? `前回の記録 ${last.file} は途中で書けなくなりました（書けたのは ${last.rows} 行）: ${last.err}` : "全軸の 指令・位置・速度・電流・Θout・外力・重力・温度 を 200 Hz で書きます（通電していない間は位置だけ）。");
    if($("#recNote").textContent!==tx) $("#recNote").textContent=tx; }
  const c=a.cal||{};
  setHtml($("#kv"), `<span>モーター</span><b>${has(a.motor_rad)?a.motor_rad.toFixed(3)+" rad":"–"}</b>`
    +`<span>Θin</span><b>${(a.synced&&has(a.tin))?a.tin.toFixed(1)+"°":"–"}</b>`
    +`<span>壁</span><b>${has(a.p_lo)?a.p_lo.toFixed(2)+" 〜 "+a.p_hi.toFixed(2)+" rad":"–"}</b>`
    +`<span>外力（推定）</span><b>${has(a.ext_nm)?(a.ext_nm>=0?"+":"")+a.ext_nm.toFixed(2)+" ± "+a.fric_nm.toFixed(2)+" N·m（膝）":"–"}</b>`
    +`<span title="保持中にモーターが出しているトルクの逆向き。摩擦の幅つき。脱力中は電流が流れないので測れず、動いている最中は慣性が乗る">　</span><b style="font-weight:normal;color:var(--dim)">${has(a.ext_nm)?"保持中だけ意味あり（重力も外力に含む）":""}</b>`);
  $("#btnExt").className = c.plus_end==="extend" ? "on" : ""; $("#btnFold").className = c.plus_end==="fold" ? "on" : "";
  setHtml($("#calkv"), `<span>畳み切りのストッパ</span><b>${END_LABEL[c.plus_end]||"未選択"}</b>`
    +`<span>基準</span><b>${has(c.tin0)?((c.ref_src==="manual"?"測った角（手入力）":"畳み切りのストッパ = "+(D?D.PHI2:156)+"°")+"・原点で Θin "+c.tin0.toFixed(1)+"°"):"–"}</b>`
    +`<span>両端の Θout（導出）</span><b>${(has(a.end_lo)&&has(a.end_hi))?a.end_lo.toFixed(1)+"° 〜 "+a.end_hi.toFixed(1)+"°":"–"}</b>`
    +`<span>半幅</span><b>${has(c.half)?c.half.toFixed(3)+" rad（原点出し済み）":"–（原点出し前）"}</b>`
    +`<span>減速比</span><b>${(c.ratio||9)}（固定）・向き ${has(a.slope)?(a.slope>0?"＋":"−"):"–"}</b>`);
  const h=a.homing, L=a.last;
  setText($("#homeTrace"), h ? ("段階: "+h.phase+"\n"+h.trace.map(t=>`${t[0]} 力 ${t[1].toFixed(2)} N·m → ${t[2].toFixed(3)} rad`).join("\n"))
    : (L ? (L.ok ? `前回: 可動範囲 ${L.span.toFixed(3)} rad・壁 ±${L.lim.toFixed(3)} rad\n`+L.trace.map(t=>`${t[0]} 力 ${t[1].toFixed(2)} N·m → ${t[2].toFixed(3)} rad`).join("\n") : "前回: 失敗 — "+L.why) : ""));
}
async function poll(){
  if(inflight.st){ renderUI(); render(); banners(); return; }
  inflight.st=true;
  try{ const r=await fetch("/api/state?since=0&cid="+CID,{cache:"no-store", signal:wdSignal()}); st=await r.json(); offline=0; lastOk=Date.now(); }
  catch(e){ offline++; if(offline>3){ $("#dot").className="dot"; } }
  finally{ inflight.st=false; }
  banners();
}
async function pollRec(){
  if(inflight.rec) return;
  inflight.rec=true;
  try{ const r=await fetch("/api/rec",{cache:"no-store", signal:wdSignal()}); const j=await r.json();
    recOn=!!(j.rec&&j.rec.on); recLast=(j.rec&&j.rec.last)||{};
    recButton($("#btnRec"), recOn, j.rec&&j.rec.secs, recLast, ""); recButton($("#btnRec2"), recOn, j.rec&&j.rec.secs, recLast, "pri"); banners();
    const shown=(j.files||[]).slice(0,12); const total=(typeof j.total==="number")?j.total:(j.files||[]).length; const more=total-shown.length;
    const rest = more>0 ? `ほか ${more} 件（置き場に ${total} 件）` : "";
    const tx = shown.length ? ("置き場: "+j.dir+"\n"+shown.map(f=>`${f.name}  ${(f.bytes/1024).toFixed(0)} KB`).join("\n")+(rest?"\n"+rest:"")) : ("置き場: "+j.dir+"（まだ無し）");
    const el=$("#recFiles"); if(el.dataset.tx!==tx){ el.dataset.tx=tx; el.innerHTML=shown.length ? ("置き場: "+esc(j.dir)+"<br>"+shown.map(f=>`<a href="/rec/${encodeURIComponent(f.name)}" download>${esc(f.name)}</a>  ${(f.bytes/1024).toFixed(0)} KB`).join("<br>")+(rest?"<br>"+esc(rest):"")) : ("置き場: "+esc(j.dir)+"（まだ無し）"); }
  }catch(e){}finally{ inflight.rec=false; }
}
async function pollLeg(){
  if(inflight.leg) return;
  inflight.leg=true;
  try{ const r=await fetch("/api/leg",{cache:"no-store", signal:wdSignal()}); leg=await r.json();
    if(!D){ D=leg.design;
      for(const id of ["#tout","#toutCal","#sLo","#sHi"]){ $(id).min=D.PHI1; $(id).max=D.PHI2; } }
    renderUI(); render(); }catch(e){}finally{ inflight.leg=false; }
}
function sendTout(){ const v=+$("#tout").value; const now=performance.now(); pendingTout={node, v};
  if(now-lastSend>40){ lastSend=now; post("leg_set",{node, tout:v}); pendingTout=null; } }
$("#tout").oninput=()=>{ dragging=true; setText($("#toutV"), (+$("#tout").value).toFixed(1)); sendTout(); };
$("#tout").onchange=()=>{ dragging=false; pendingTout=null; post("leg_set",{node, tout:+$("#tout").value}); };
$("#kp").oninput=()=>{ dragging=true; setText($("#kpV"), (+$("#kp").value).toFixed(1)); post("set",{node, kp:+$("#kp").value}); };
$("#kd").oninput=()=>{ dragging=true; setText($("#kdV"), (+$("#kd").value).toFixed(2)); post("set",{node, kd:+$("#kd").value}); };
$("#kp").onchange=$("#kd").onchange=()=>{ dragging=false; };
setInterval(()=>{ if(pendingTout!==null && performance.now()-lastSend>40){ lastSend=performance.now(); post("leg_set",{node:pendingTout.node, tout:pendingTout.v}); pendingTout=null; } }, 20);
$("#node").onchange=()=>{ node=+$("#node").value; gravEdit=false; renderUI(); render(); };
$("#btnArm").onclick=()=>post("arm");
$("#btnDisarm").onclick=()=>post("disarm");
$("#btnEstop").onclick=()=>post("estop");
$("#btnHome").onclick=()=>post("leg_home",{node});
$("#btnSine").onclick=()=>post("leg_sine",{node, lo:+$("#sLo").value, hi:+$("#sHi").value, period:+$("#sPeriod").value});
$("#btnSineStop").onclick=()=>post("mode",{mode:"hold", node});
$("#btnFree").onclick=()=>post("mode",{mode:"free", node});
$("#btnDamp").onclick=()=>post("mode",{mode:"damp", node});
$("#btnHold").onclick=()=>post("mode",{mode:"hold", node});
$("#btnGravOn").onclick=()=>post("grav_set",{node, on:true});
$("#btnGravOff").onclick=()=>post("grav_set",{node, on:false});
$("#gravA").oninput=$("#gravPh").oninput=()=>{ gravEdit=true; };
$("#btnGravSet").onclick=()=>{ const a_nm=+$("#gravA").value, phase_deg=+$("#gravPh").value; gravEdit=false; post("grav_set",{node, a_nm, phase_deg}); };
$("#btnGravSample").onclick=()=>post("grav_sample",{node});
$("#btnGravFit").onclick=()=>{ gravEdit=false; post("grav_fit",{node}); };
$("#btnGravClear").onclick=()=>{ gravEdit=false; post("grav_clear",{node}); };
$("#btnTraj").onclick=()=>post("leg_traj",{node, text:$("#trajText").value, loop:$("#trajLoop").checked, speed:+$("#trajSpeed").value});
$("#btnTrajStop").onclick=()=>post("mode",{mode:"hold", node});
async function toggleRec(){ const a=curAxis(); const on=a&&a.rec&&a.rec.on;
  if(on){ post("rec_stop",{}); return; }
  const name=$("#recName").value; const j=await post("rec_start",{name});
  if(j&&j.ok&&j.file&&name&&!String(j.file).endsWith("_"+name+".csv"))
    showInfo("名前は "+j.file+" になりました（英数と _- だけ・"+$("#recName").maxLength+" 字まで）", "warn");
}
$("#btnRec").onclick=toggleRec; $("#btnRec2").onclick=toggleRec;
$("#btnCal1").onclick=()=>post("leg_cal",{node, tout_now:+$("#toutCal").value});
$("#btnShift0").onclick=()=>post("leg_cal",{node, ref_clear:1});
$("#btnExt").onclick=()=>post("leg_cal",{node, plus_end:"extend"});
$("#btnFold").onclick=()=>post("leg_cal",{node, plus_end:"fold"});
window.addEventListener("keydown",e=>{ if(e.code==="Space"||e.code==="Escape"){ e.preventDefault(); post("estop"); } });
document.addEventListener("visibilitychange",()=>{ if(document.hidden) post("disarm"); });
window.addEventListener("pagehide",()=>{ navigator.sendBeacon("/api/cmd", new Blob([JSON.stringify({action:"disarm",params:{cid:CID}})],{type:"application/json"})); });
window.addEventListener("resize",render);
{ const hd=$("header"); const setHdr=()=>document.documentElement.style.setProperty("--hdr", hd.getBoundingClientRect().height+"px");
  setHdr(); new ResizeObserver(setHdr).observe(hd); }
poll(); setInterval(poll,50); pollLeg(); setInterval(pollLeg,100); pollRec(); setInterval(pollRec,1000);
</script></body></html>
"""

DEMO_PAGE = r"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>12 軸デモ — KRC アクチュエータ</title>
<style>
/*__COMMON_CSS__*/
body{margin:0;background:var(--bg);color:var(--fg);font-size:14px;line-height:1.5}
header{display:flex;gap:10px;align-items:center;padding:10px 14px;border-bottom:1px solid var(--line);
  background:var(--panel);position:sticky;top:0;z-index:3;flex-wrap:wrap}
header h1{font-size:16px;margin:0}
.spacer{flex:1}
.wrap{display:grid;grid-template-columns:1fr minmax(360px,420px);gap:14px;padding:14px;max-width:1500px;margin:0 auto}
@media(max-width:900px){.wrap{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.panel h2{font-size:13px;margin:0 0 8px;color:var(--dim);font-weight:600;letter-spacing:.03em}
.panel+.panel{margin-top:12px}
.row{display:flex;align-items:center;gap:8px;margin:6px 0;flex-wrap:wrap}
.row button{white-space:nowrap}
.row label{color:var(--dim);font-size:12px}
input[type=number],input[type=text]{background:var(--bg);color:var(--fg);border:1px solid var(--line);border-radius:6px;
  font:inherit;font-size:13px;line-height:19px;padding:6px 8px;width:80px;text-align:right}
input[type=text]{text-align:left}
select{background:var(--bg);color:var(--fg);border:1px solid var(--line);border-radius:6px;font:inherit;font-size:13px;height:33px;padding:0 8px}
.step{border:1px solid var(--line);border-radius:8px;padding:8px 10px;margin-bottom:10px}
.moves{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.moves button{text-align:left;padding:8px 12px}
.moves button small{display:block;color:var(--dim);font-size:11px;line-height:15px}
.moves button.on small{color:var(--bg)}
.hint{color:var(--dim);font-size:12px;line-height:1.6;margin:4px 0}
#grid{width:100%;height:auto;display:block}
.banner{margin:10px 14px 0}
#prog{height:6px;background:var(--line);border-radius:3px;overflow:hidden;margin:8px 0 2px}
#prog>div{height:100%;width:0;background:var(--cyan)}
</style></head><body>
<header>
  <span class="dot" id="dot"></span>
  <h1>12 軸デモ</h1>
  <button id="btnBack" title="主画面へ戻る（通電は切れます）">← 主画面</button>
  <span id="chips"></span>
  <span class="spacer"></span>
  <button id="btnDisarm" title="12 軸ともダンパーで止めてから解除">全軸解除</button>
  <button id="btnClear">遮断を解除</button>
  <button id="btnEstop" class="estop" title="Space か Esc でも止まります">非常停止</button>
</header>
<div class="banner bad" id="bnBad"></div>
<div class="banner info" id="bnInfo"></div>
<div class="wrap">
  <div class="panel">
    <h2>机の上の並び（手前から見た向き・上が奥）</h2>
    <div class="row"><label>並び</label>
      <button id="lay3" title="奥・真ん中・手前の 3 列。奥＝右から 1〜4／真ん中＝右から 5〜8／手前＝右から 9〜12">3 列（4 本 × 3）</button>
      <button id="lay2" title="奥・手前の 2 列。奥＝右から 1〜6／手前＝右から 7〜12">2 列（6 本 × 2）</button>
      <button id="lay1" title="横に 1 列。右から 1〜12">1 列（12 本）</button></div>
    <svg id="grid" viewBox="0 0 800 640"></svg>
    <div class="row"><label><input type="checkbox" id="flip"> 実機の矢印が画面と逆に回るなら入れる（画面の表示と、コンパス・集中のような向きの動きを実機に合わせる）</label></div>
  </div>
  <div>
    <div class="panel">
      <h2>0 点</h2>
      <div class="row">
        <button id="btnZero" class="pri" title="12 軸を通電してダンパーにする。手で矢印を真上に合わせてから「決定」">0 点合わせ（ダンパー）</button>
        <button id="btnZeroSet" title="いまの 12 軸の位置を、それぞれの 0 点にして位置保持に入る">決定</button>
      </div>
      <div class="row">
        <button id="btnHold" title="通電して、保存した 0 点（「決定」で保存した真上）へ 12 本をそろって戻し、位置保持">通電（保存した 0 点へ戻す）</button>
      </div>
      <div class="hint" id="zeroHint"></div>
    </div>
    <div class="panel">
      <h2>動き</h2>
      <div class="step">
        <div class="row"><b>角度の並び</b><span class="hint">並びの角度へ 1 ステップずつカチッと動いて止まる、を「止める」まで繰り返す。真上 0°・時計回りが正</span></div>
        <div class="row">
          <input type="text" id="stList" value="0, 45, 90, 135, 180, 135, 90, 45, 0, -45, -90, -135, -180, -135, -90, -45, 0" style="flex:1;min-width:220px" title="カンマか空白で区切る。最後が最初と同じ向き（差が 360° の倍数）なら、そこから次の巡へ続く（0, 45, … , 315, 360 で回り続ける）">
          <label>1 ステップ</label><input type="number" id="stStep" value="0.4" min="0.05" max="10" step="0.05"><label>s</label>
          <label title="ID 1 から順に、1 つ上の ID よりこれだけ遅れて同じ動きをなぞる（0 なら 12 本そろって動く）">ズレ</label><input type="number" id="stLag" value="0" min="0" max="5" step="0.01"><label>s</label>
          <button id="btnStep" class="pri">回し始める</button>
        </div>
        <div class="hint" id="stHint"></div>
      </div>
      <div class="step">
        <div class="row"><b>正弦波</b><span class="hint">真上を中心に揺れ続ける。「止める」で真上に戻る。揺れている間に数を変えると、そのままつながる</span></div>
        <div class="row">
          <label>振幅</label><input type="number" id="swAmp" value="90" min="1" max="360" step="1"><label>°</label>
          <label>周期</label><input type="number" id="swPer" value="2" min="0.2" max="20" step="0.1"><label>s</label>
          <label title="ID 1 から順に、1 つ上の ID よりこれだけ遅れて揺れ始める（波が 1 → 12 へ流れる）">ズレ</label><input type="number" id="swLag" value="0" min="0" max="5" step="0.01"><label>s</label>
          <button id="btnSine" class="pri">揺らし始める</button>
        </div>
        <div class="hint" id="swHint"></div>
      </div>
      <div class="step">
        <div class="row"><b>締め</b><span class="hint">少し逆へ溜めて → 一気に加速して回る → 真上で一斉にビタッと止まる</span></div>
        <div class="row">
          <label>回転数</label><input type="number" id="fnTurns" value="8" min="1" max="20" step="1">
          <select id="fnDir"><option value="1">右回り</option><option value="-1">左回り</option></select>
          <button id="btnFinale" class="pri">締める</button>
        </div>
      </div>
      <div class="step">
        <div class="row"><b>S 字ドミノ</b><span class="hint" id="sdHint"></span></div>
        <div class="row">
          <label>間隔</label><input type="number" id="sdGap" value="0.12" min="0" max="2" step="0.01"><label>s</label>
          <label>待ち</label><input type="number" id="sdHold" value="0.5" min="0" max="10" step="0.1"><label>s</label>
          <button id="btnSdom" class="pri">倒す</button>
        </div>
      </div>
      <div class="moves" id="moves"></div>
      <div class="row" style="margin-top:10px">
        <button id="btnAll" class="pri">通しで再生（全部）</button>
        <button id="btnStop">止める</button>
      </div>
      <div id="prog"><div></div></div>
      <div class="hint" id="playHint"></div>
      <div class="row"><label><input type="checkbox" id="preview"> 見本だけ（モーターは動かさず、画面の矢印だけ動かす）</label></div>
    </div>
    <div class="panel">
      <h2>硬さと速さ</h2>
      <div class="row"><label>Kp</label><input type="number" id="kp" value="10" min="0" max="80" step="0.5">
        <label>Kd</label><input type="number" id="kd" value="0.05" min="0" max="1" step="0.01">
        <label>ダンパーの Kd</label><input type="number" id="kdd" value="0.05" min="0" max="1" step="0.01"></div>
      <div class="row"><label>速さ</label><input type="number" id="tempo" value="1" min="0.5" max="1" step="0.05">
        <span class="hint">1 = 作った速さ・0.5 = 半分の速さ</span></div>
    </div>
  </div>
</div>
<script>
const CID=Math.random().toString(36).slice(2)+Date.now().toString(36);
const NODES=[1,2,3,4,5,6,7,8,9,10,11,12];
const TAU=2*Math.PI, D2R=Math.PI/180;
let st=null, seq=0, lastOk=0, inflight=false, flash=null;
let busy="";
let chain=null;
let ghost=null;
const $=id=>document.getElementById(id);
const esc=s=>String(s).replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/>/g,"&gt;");


const LAYOUTS={3:{rows:3,cols:4},2:{rows:2,cols:6},1:{rows:1,cols:12}};
let LAY=LAYOUTS[3];
try{ const k=localStorage.getItem("krc.demo.layout"); if(LAYOUTS[k]) LAY=LAYOUTS[k]; }catch(e){}
const rowOf=n=>Math.floor((n-1)/LAY.cols), colR=n=>(n-1)%LAY.cols, xOf=n=>LAY.cols-1-colR(n);

const smooth=u=>u<=0?0:u>=1?1:u*u*u*(10-15*u+6*u*u);
function bump(t,t0,D,A){ const u=(t-t0)/D; if(u<=0||u>=1) return 0; const s=Math.sin(Math.PI*u); return A*s*s; }
function pulse(t,t0,D,P,A){ const u=t-t0; if(u<=0||u>=D) return 0; const s=Math.sin(Math.PI*u/D); return A*s*s*Math.sin(TAU*u/P); }
function keys(t,K){ if(t<=K[0][0]) return K[0][1]*D2R;
  for(let i=1;i<K.length;i++){ if(t<=K[i][0]){ const a=K[i-1], b=K[i];
    return (a[1]+(b[1]-a[1])*smooth((t-a[0])/(b[0]-a[0])))*D2R; } }
  return K[K.length-1][1]*D2R; }
function trapS(u,a){ if(u<=0) return 0; if(u>=1) return 1; const tot=1-a;
  const ramp=x=>(x-(a/Math.PI)*Math.sin(Math.PI*x/a))/2;
  if(u<a) return ramp(u)/tot; if(u<=1-a) return (a/2+(u-a))/tot; return (tot-ramp(1-u))/tot; }
function spinS(u,a){ if(u<=0) return 0; if(u>=1) return 1; const tot=a/3+(1-a)/2;
  if(u<a) return (u*u*u/(3*a*a))/tot; const w=u-a; return (a/3+w-w*w/(2*(1-a)))/tot; }

function prof(u,a,b){ if(u<=0) return 0; if(u>=1) return 1; const area=a/2+(1-a-b)+b/2;
  if(u<a) return (u-(a/Math.PI)*Math.sin(Math.PI*u/a))/2/area;
  if(u<=1-b) return (a/2+(u-a))/area;
  const w=u-(1-b); return (a/2+(1-a-b)+(w+(b/Math.PI)*Math.sin(Math.PI*w/b))/2)/area; }
const tick=(t,t0)=>(Math.PI/4)*smooth((t-t0)/0.16);
function aim(n,tx,ty){ return Math.atan2(tx-xOf(n), -(ty-rowOf(n))); }
const CX=1.5, CY=1;

const MOVES=[
 {id:"tick", name:"カチカチ（倍々）", sub:"45° ずつ 8 拍で 1 周、折り返すたびに倍の速さ（4 段目は 3 段目の速さで逆回り）", T:8.0, same:true,
  f:(t)=>{ let v=0, t0=0.3; const IS=[0.45,0.225,0.1125,0.1125];
    for(let r=0;r<4;r++){ const I=IS[r], d=(r%2)?-1:1, tau=Math.max(0.1,Math.min(0.16,0.7*I));
      for(let i=0;i<8;i++) v+=d*(Math.PI/4)*smooth((t-(t0+i*I))/tau);
      t0+=8*I; }
    return v; }},
];

function ax(n){ return ((st&&st.axes)||[]).find(a=>a.node===n); }
function alive(n){ const a=ax(n); return !!(a&&a.board&&a.board.alive); }
function ready12(){ return !!st && NODES.every(alive); }
function allArmed(){ return ready12() && NODES.every(n=>ax(n).armed); }
function allMode(m){ return allArmed() && NODES.every(n=>ax(n).mode===m); }
function playing(){ return !!(st&&st.jt); }
function tripped(){ return !!(st&&(st.tripped||NODES.some(n=>ax(n)&&ax(n).tripped))); }
function sleep(ms){ return new Promise(r=>setTimeout(r,ms)); }
async function waitFor(cond,ms){ const t0=Date.now(); while(Date.now()-t0<ms){ if(cond()) return true; await sleep(60); } return cond(); }
function num(id,lo,hi,def){ const v=parseFloat($(id).value); return isFinite(v)?Math.min(hi,Math.max(lo,v)):def; }

async function post(action,params){
  try{
    const r=await fetch("/api/cmd",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({action:action,params:Object.assign({cid:CID},params||{})})});
    const j=await r.json();
    if(j&&j.ok===false&&j.err) say(j.err,true);
    return j||{};
  }catch(e){ say("送れませんでした: "+e,true); return {ok:false}; }
}
function say(msg,bad){ flash={msg:String(msg),bad:!!bad,until:Date.now()+(bad?8000:5000)}; banners(); }

async function prepAll(){
  const off=NODES.filter(n=>ax(n)&&ax(n).on===false);
  if(off.length){ say("主画面で軸 "+off.join("・")+" のチェックが外れています（動かす対象にしてから）",true); return false; }
  const r=await post("target",{target:"all"}); return r.ok!==false;
}
async function armAll(){
  if(allArmed()) return true;
  let r=null;
  for(let i=0;i<40;i++){
    r=await fetch("/api/cmd",{method:"POST",headers:{"Content-Type":"application/json"},
      body:JSON.stringify({action:"arm",params:{cid:CID}})}).then(x=>x.json()).catch(()=>({ok:false,err:"送れませんでした"}));
    if(r.ok!==false||!/目盛り/.test(String(r.err))) break;
    if(i===0) say("基板の設定を読み終えるのを待っています（立て直した直後は数秒〜20 秒）");
    await sleep(500);
  }
  if(r.ok===false){ say(r.err,true); return false; }
  if(!await waitFor(allArmed,2500)){ say("12 軸とも通電できませんでした（主画面で理由を確かめてください）",true); return false; }
  await post("demo_tlm",{div:10});
  return true;
}
async function zeroStart(){
  if(busy) return; busy="0 点合わせ";
  try{
    if(!await prepAll()) return;
    if((await post("set",{kd:num("kdd",0,1,0.05)})).ok===false) return;
    if(!await armAll()) return;
    if((await post("mode",{mode:"damp"})).ok===false) return;
    await waitFor(()=>allMode("damp"),1500);
    say("ダンパーにしました。手で矢印を真上に合わせて「決定」");
  }finally{ busy=""; }
}

const ENC_SIGN=-1, ENC_N=16384;
function encOf(n){ const a=ax(n); return a&&a.board?a.board.enc_raw:null; }
function toZero(n){ const z=st&&st.zero_ref||{}, ref=z[String(n)], e=encOf(n);
  if(ref==null||e==null) return null;
  const d=((ref-e+ENC_N/2)%ENC_N+ENC_N)%ENC_N-ENC_N/2; return ENC_SIGN*d*TAU/ENC_N; }
async function zeroSet(){
  if(busy) return; busy="決定";
  try{
    if((await post("set",{kp:num("kp",0,80,10),kd:num("kd",0,1,0.05)})).ok===false) return;
    if((await post("zero_mode",{mode:"hold"})).ok===false) return;
    if(!await waitFor(()=>allMode("hold"),2500)){ say("位置保持に入れなかった軸があります（主画面で理由を確かめてください）",true); return; }
    await sleep(1500);
    const refs={}; for(const n of NODES){ const e=encOf(n); if(e==null){ say("軸"+n+" の絶対角が読めず、0 点を保存できませんでした",true); return; } refs[n]=e; }
    if((await post("demo_zero_save",{refs:refs})).ok===false) return;
    say("いまの位置を 12 軸の 0 点にして保存しました（立て直しても、通電でここへ戻ります）");
  }finally{ busy=""; }
}
async function holdKeep(){
  if(busy) return; busy="通電";
  try{
    if(!await prepAll()) return;
    if((await post("set",{kp:num("kp",0,80,10),kd:num("kd",0,1,0.05)})).ok===false) return;
    const dz=NODES.map(toZero), have=dz.every(x=>x!==null);
    if(!await armAll()) return;
    if((await post("mode",{mode:"hold"})).ok===false) return;
    if(!await waitFor(()=>allMode("hold"),2500)) return;
    if(!have){ say("0 点が保存されていません（いまの 0 点のまま位置保持）。「0 点合わせ」→「決定」で保存できます",true); return; }
    const p0=NODES.map(n=>ax(n).pos), big=Math.max(...dz.map(Math.abs));
    if(big>0.002){
      const T=Math.max(1.0,big/2+0.6), N=60, pts=[];
      for(let i=0;i<N;i++){ const u=smooth(i/(N-1)); pts.push(NODES.map((n,k)=>+(p0[k]+dz[k]*u).toFixed(4))); }
      if((await post("jtraj",{nodes:NODES,period:T,points:pts,once:true,name:"0 点へ戻す"})).ok===false) return;
      await waitFor(()=>playing(),1500); await waitFor(()=>!playing(),(T+8)*1000);
    }
    if((await post("zero_mode",{mode:"hold"})).ok===false) return;
    if(await waitFor(()=>allMode("hold"),2500)) say("保存した 0 点（真上）へ戻して、位置保持に入りました");
  }finally{ busy=""; }
}
async function disarmAll(){
  chain=null;
  if(playing()) await post("jtraj_stop",{});
  if(st&&st.armed){ await post("mode",{mode:"damp"}); await sleep(400); }
  await post("disarm");
  await post("demo_tlm",{div:1});
}

function plan(m){
  const tempo=num("tempo",0.5,1,1);
  const T=m.T/tempo;
  const base=NODES.map(n=>{ const a=ax(n); const p=a&&isFinite(a.pos)?a.pos:0; return Math.round(p/TAU)*TAU; });
  const mean=base.reduce((x,y)=>x+y,0)/base.length;
  const flip=$("flip").checked?-1:1;
  const s=mean>Math.PI?-flip:(mean<-Math.PI?flip:1);
  const bs=base.map(b=>flip*b);
  const same=!!m.same && base.every(b=>b===base[0]);
  let N=Math.min(2000,Math.max(8,Math.ceil(T*(same?250:120)))), pts;
  for(;;){
    pts=[];
    for(let i=0;i<N;i++){ const t=i/(N-1)*m.T;
      pts.push(same?[+(base[0]+flip*m.f(t,1,s,bs[0])).toFixed(4)]
                   :NODES.map((n,k)=>+(base[k]+flip*m.f(t,n,s,bs[k])).toFixed(4))); }
    if(JSON.stringify(pts).length<56000||N<=60) break;
    N=Math.floor(N*0.8);
  }
  return {T,N,pts,base,s,bs,flip,same};
}
function showGhost(m,p){ ghost={m:m,t0:performance.now(),T:p.T,base:p.base,s:p.s,bs:p.bs,flip:p.flip}; }
async function playOne(m){
  const p=plan(m);
  if($("preview").checked){ showGhost(m,p); await sleep(p.T*1000+200); ghost=null; return true; }
  if(!allMode("hold")){ say("12 軸とも位置保持のときに動かせます（「決定」か「通電」から）",true); return false; }
  const kp=num("kp",0,80,10), kd=num("kd",0,1,0.05);
  const r=await post("jtraj",{nodes:NODES,period:p.T,points:p.pts,once:true,same:p.same,name:m.name,
                              kp:NODES.map(()=>kp),kd:NODES.map(()=>kd)});
  if(r.ok===false) return false;
  showGhost(m,p);
  await waitFor(()=>playing(),1500);
  const ok=await waitFor(()=>!playing(),(p.T+15)*1000);
  ghost=null;
  if(ok&&allMode("hold")&&NODES.some(n=>Math.abs(ax(n).pos||0)>2*TAU)){
    await post("zero_mode",{mode:"hold"}); await waitFor(()=>allMode("hold"),2500);
  }
  return ok;
}

function moveTime(d){ d=Math.abs(d); return d<1e-9?0:Math.max(0.05,Math.sqrt(5.77*d/450),1.875*d/50); }
function stepParams(){
  const q=String($("stList").value).split(/[\s,、，]+/).filter(x=>x!=="").map(Number);
  if(q.length<2||q.some(x=>!isFinite(x))) return {err:"角度を 2 つ以上、カンマか空白で区切って並べてください"};
  if(q.some(x=>Math.abs(x)>3600)) return {err:"角度は ±3600° までです"};
  const k=q.length, d=q[k-1]-q[0], closes=Math.abs(d/360-Math.round(d/360))<1e-9;
  const cyc=closes?q.slice(0,k-1):q.slice(), inc=closes?d:0;
  const tg=cyc.concat([cyc[0]+inc]).map(x=>x*D2R);
  const M=cyc.length, taus=[];
  for(let i=0;i<M;i++) taus.push(moveTime(tg[i+1]-tg[i]));
  if(taus.every(x=>x===0)) return {err:"動かない並びです（角度がすべて同じ）"};
  const Sin=num("stStep",0.05,10,0.4), S=Math.max(Sin,...taus,0.05), lag=num("stLag",0,5,0);
  return {q,tg,M,taus,S,Sin,T:M*S,inc:inc*D2R,incDeg:inc,lag};
}
function stepAt(p,t){ const i=Math.min(p.M-1,Math.floor(t/p.S)), u=p.taus[i]?(t-i*p.S)/p.taus[i]:1;
  return p.tg[i]+(p.tg[i+1]-p.tg[i])*smooth(u); }
function stepHint(){
  const p=stepParams();
  $("stHint").textContent=p.err?p.err:(p.M+" ステップで 1 巡（"+p.T.toFixed(2)+" s）"+(p.incDeg?"・巡るたびに "+(p.incDeg>0?"+":"")+p.incDeg+"°（回り続ける）":"・同じ所を繰り返す")
    +(p.S>p.Sin+1e-9?"。いちばん大きい動きに合わせて 1 ステップを "+p.S.toFixed(2)+" s に延ばします":"")
    +(p.lag>0?"。ID 1 から順に "+p.lag+" s ずつ遅れてなぞる（ID 12 は "+(11*p.lag).toFixed(2)+" s 遅れ）":""));
}
async function stepStart(){
  if(busy) return;
  if(!allMode("hold")){ say("12 軸とも位置保持のときに回せます（「決定」か「通電」から）",true); return; }
  const p=stepParams(); if(p.err){ say(p.err,true); return; }
  busy="角度の並び";
  try{
    const flip=$("flip").checked?-1:1;
    const base=NODES.map(n=>{ const a=ax(n), x=(a&&a.p_cmd!=null)?a.p_cmd:a.pos; return Math.round(x/TAU)*TAU; });
    const same=base.every(b=>Math.abs(b-base[0])<1e-6);
    let N=Math.min(2000,Math.max(8,Math.ceil(p.T*250),Math.ceil(12*p.T/Math.min(...p.taus.filter(x=>x>0))))), pts;
    for(;;){
      pts=[];
      for(let i=0;i<N;i++){ const g=flip*stepAt(p,i/N*p.T);
        pts.push(same?[+(base[0]+g).toFixed(4)]:base.map(b=>+(b+g).toFixed(4))); }
      if(JSON.stringify(pts).length<56000||N<=60) break;
      N=Math.floor(N*0.8);
    }
    const marks=[]; for(let i=1;i<p.M;i++) marks.push(i/p.M);
    const name="角度の並び（"+p.q.join(", ")+"・1 ステップ "+p.S.toFixed(2)+" s"+(p.lag>0?"・ズレ "+p.lag+" s":"")+"）";
    const inc=flip*p.inc;
    const delays=NODES.map((n,k)=>+(k*p.lag).toFixed(4));
    const r=await post("jtraj",{nodes:NODES,period:p.T,points:pts,same:same,inc:same?[inc]:NODES.map(()=>inc),marks:marks,delays:delays,name:name,
                                kp:NODES.map(()=>num("kp",0,80,10)),kd:NODES.map(()=>num("kd",0,1,0.05))});
    if(r.ok!==false){ await waitFor(()=>playing(),3000); say("回しています。「止める」で、いまのステップの終わりで止まります"); }
  }finally{ busy=""; }
}


let sineBase=null;
function sineParams(){
  const A=num("swAmp",1,360,90)*D2R, Tin=num("swPer",0.2,20,2);
  const Tmin=Math.max(2*Math.PI*Math.sqrt(A/450), 2*Math.PI*A/50, 0.2);
  return {A, T:Math.min(20,Math.max(Tin,Tmin)), Tin, Tmin};
}
function sinePlaying(){ return playing()&&/^正弦波/.test(st.jt.name||""); }
async function sineSend(start){
  const q=sineParams(), lag=num("swLag",0,5,0);
  const N=Math.min(2000,Math.max(64,Math.ceil(q.T*250))), pts=[];
  const same=sineBase.every(b=>Math.abs(b-sineBase[0])<1e-6);
  for(let i=0;i<N;i++){ const g=q.A*Math.sin(2*Math.PI*i/N);
    pts.push(same?[+(sineBase[0]+g).toFixed(4)]:sineBase.map(b=>+(b+g).toFixed(4))); }
  const name="正弦波（振幅 "+Math.round(q.A/D2R)+"°・周期 "+q.T.toFixed(2)+" s"+(lag>0?"・ズレ "+lag+" s":"")+"）";
  const g={nodes:NODES,period:q.T,points:pts,same:same,name:name,
           kp:NODES.map(()=>num("kp",0,80,10)),kd:NODES.map(()=>num("kd",0,1,0.05))};
  if(lag>0) Object.assign(g,{inc:same?[0]:NODES.map(()=>0),delays:NODES.map((n,k)=>+(k*lag).toFixed(4)),ramp:1.0});
  return post("jtraj",g);
}
async function sineStart(){
  if(busy) return;
  if(!allMode("hold")){ say("12 軸とも位置保持のときに揺らせます（「決定」か「通電」から）",true); return; }
  busy="正弦波";
  try{
    sineBase=NODES.map(n=>{ const a=ax(n); return (a&&a.p_cmd!=null)?a.p_cmd:a.pos; });
    const r=await sineSend(true);
    if(r.ok!==false){ await waitFor(()=>playing(),3000); say("揺らしています。数を変えるとそのままつながります。「止める」で真上に戻ります"); }
  }finally{ busy=""; }
}
let sineTimer=null;
function sineEdited(){
  const q=sineParams();
  const lag=num("swLag",0,5,0);
  $("swHint").textContent=(q.Tin<q.Tmin-1e-9?("この振幅では周期 "+q.Tmin.toFixed(2)+" s より速くできません（速さと加速度の上限）。"+q.Tmin.toFixed(2)+" s で揺らします。"):"")
    +(lag>0?"ID 12 は "+(11*lag).toFixed(2)+" s 遅れて揺れ始めます。ズレありは、揺れている間に数を変えても次に「揺らし始める」を押すまで効きません":"");
  if(!sinePlaying()||!sineBase||lag>0||(st.jt&&st.jt.ramp)) return;
  clearTimeout(sineTimer); sineTimer=setTimeout(()=>{ if(sinePlaying()) sineSend(false); },300);
}


function sdPath(){
  const P=[];
  for(let r=0;r<LAY.rows;r++){
    const row=NODES.filter(n=>rowOf(n)===r).sort((a,b)=>colR(a)-colR(b));
    P.push(...(r%2?row.reverse():row)); }
  return P;
}
const sdDir=n=>(rowOf(n)%2?1:-1);
function sdHintText(){
  const P=sdPath(), seg=[];
  for(let i=0;i<P.length;i+=LAY.cols) seg.push([P[i],P[i+LAY.cols-1],sdDir(P[i])]);
  const lr=d=>d>0?"右":"左";
  return "① "+seg.map(g=>g[0]+"→"+g[1]+" "+lr(g[2])).join("・")+"へ 1 本ずつ倒れる "
    +"② "+seg.slice().reverse().map(g=>g[1]+"→"+g[0]+" "+lr(-g[2])).join("・")+"へ逆に倒す "
    +"③ "+seg.map(g=>g[0]+"→"+g[1]).join("・")+" の順に縦へ戻す（ここまでで 1 セット）";
}
function sdMove(){
  const gap=num("sdGap",0,2,0.12), hold=num("sdHold",0,10,0.5), t90=moveTime(Math.PI/2), t180=moveTime(Math.PI);
  const tA=0.3, tB=tA+11*gap+t90+hold, tC=tB+11*gap+t180+hold, T=tC+11*gap+t90+0.3;
  const SD_A=sdPath(), SD_B=SD_A.slice().reverse(), SD_C=SD_A, dir={};
  for(const n of NODES) dir[n]=sdDir(n);
  return {id:"sdomino", name:"S 字ドミノ・"+LAY.rows+" 列（間隔 "+gap+" s・待ち "+hold+" s）", T:T,
    f:(t,n)=>{ const a=dir[n]*Math.PI/2;
      return a*smooth((t-(tA+SD_A.indexOf(n)*gap))/t90)
            -2*a*smooth((t-(tB+SD_B.indexOf(n)*gap))/t180)
            +a*smooth((t-(tC+SD_C.indexOf(n)*gap))/t90); }};
}


function finaleMove(){
  const turns=Math.round(num("fnTurns",1,20,8)), dir=+$("fnDir").value>0?1:-1;
  const back=30*D2R, dist=turns*TAU+back, ta=0.6, td=0.15;
  let vmax=36, D=dist/vmax+(ta+td)/2;
  if(D<ta+td){ D=ta+td; vmax=dist/((ta+td)/2); }
  const t0=1.0, T=t0+D+0.8;
  return {id:"finale", name:"締め（"+turns+" 回転・"+(dir>0?"右":"左")+"回り）", T:T, same:true,
    f:(t)=>dir*(-back*smooth((t-0.3)/0.5)+dist*prof((t-t0)/D,ta/D,td/D))};
}

async function play(m){
  if(busy) return; busy="再生";
  try{ await playOne(m); }finally{ busy=""; }
}
async function playAll(){
  if(busy) return; busy="通し"; chain={stop:false};
  try{
    for(const m of MOVES){
      if(!chain||chain.stop) break;
      if(!await playOne(m)) break;
      await sleep(700);
    }
  }finally{ chain=null; busy=""; }
}
async function stopAll(){
  if(chain) chain.stop=true;
  ghost=null;
  if(sinePlaying()&&sineBase){ await post("jtraj_stop",{to:sineBase}); return; }
  if(playing()) await post("jtraj_stop",{});
}

const CELL={w:200,h:200,ox:0,oy:20};
function cellXY(n){ return [CELL.ox+xOf(n)*CELL.w+CELL.w/2, CELL.oy+rowOf(n)*CELL.h+CELL.h/2]; }
function buildGrid(){
  const W=LAY.cols*CELL.w, H=CELL.oy+LAY.rows*CELL.h+20;
  $("grid").setAttribute("viewBox","0 0 "+W+" "+H);
  let h="";
  const fs=Math.round(13*Math.max(1,W/800));
  h+="<text x='"+W/2+"' y='"+(fs+1)+"' text-anchor='middle' fill='#8b949e' font-size='"+fs+"'>奥</text>";
  h+="<text x='"+W/2+"' y='"+(H-4)+"' text-anchor='middle' fill='#8b949e' font-size='"+fs+"'>手前</text>";
  for(const n of NODES){ const [x,y]=cellXY(n);
    h+="<g data-n='"+n+"' transform='translate("+x+","+y+")'>"
      +"<circle r='78' fill='#0d1117' stroke='#30363d' stroke-width='2' data-ring/>"
      +"<line x1='0' y1='-70' x2='0' y2='-62' stroke='#8b949e' stroke-width='2'/>"
      +"<g data-ghost style='display:none'><line x1='0' y1='10' x2='0' y2='-60' stroke='#d2a8ff' stroke-width='4' stroke-dasharray='6 5' stroke-linecap='round'/>"
      +"<path d='M0,-70 L-10,-52 L10,-52 Z' fill='none' stroke='#d2a8ff' stroke-width='2'/></g>"
      +"<g data-arrow><line x1='0' y1='14' x2='0' y2='-52' stroke='#e6edf3' stroke-width='7' stroke-linecap='round'/>"
      +"<path d='M0,-72 L-14,-48 L14,-48 Z' fill='#e6edf3'/></g>"
      +"<circle r='6' fill='#8b949e'/>"
      +"<text x='-70' y='-62' fill='#e6edf3' font-size='18' font-weight='700'>"+n+"</text>"
      +"<text x='0' y='60' text-anchor='middle' fill='#8b949e' font-size='13' data-txt></text>"
      +"</g>"; }
  $("grid").innerHTML=h;
}
const MODE_TX={free:"脱力",damp:"ダンパー",hold:"位置保持"};
function draw(){
  const flip=$("flip").checked?-1:1;
  const now=performance.now();
  let gt=null;
  if(ghost){ gt=(now-ghost.t0)/1000*(ghost.m.T/ghost.T); if(gt>ghost.m.T) gt=ghost.m.T; }
  for(const n of NODES){
    const g=document.querySelector("#grid g[data-n='"+n+"']"); if(!g) continue;
    const a=ax(n), on=alive(n);
    const ar=g.querySelector("[data-arrow]");
    const p=(a&&isFinite(a.pos))?a.pos:0;
    ar.setAttribute("transform","rotate("+(flip*p/D2R).toFixed(2)+")");
    ar.style.opacity=on?1:0.25;
    const ring=g.querySelector("[data-ring]");
    ring.setAttribute("stroke",!on?"#30363d":(a.tripped?"#f85149":(a.armed?(a.mode==="hold"?"#3fb950":"#d29922"):"#30363d")));
    const gh=g.querySelector("[data-ghost]");
    if(gt!==null){ const k=n-1; const v=ghost.flip*ghost.base[k]+ghost.m.f(gt,n,ghost.s,ghost.bs[k]);
      gh.style.display=""; gh.setAttribute("transform","rotate("+(v/D2R).toFixed(2)+")"); }
    else gh.style.display="none";
    const tx=g.querySelector("[data-txt]");
    const deg=p/D2R - Math.round(p/TAU)*360;
    const s=!on?"返事なし":((a.armed?(MODE_TX[a.mode]||a.mode):"解除")+"  "+(deg>=0?"+":"")+deg.toFixed(1)+"°"
             +(a.board&&isFinite(a.board.temp_c)?"  "+a.board.temp_c.toFixed(0)+"℃":""));
    if(tx.textContent!==s) tx.textContent=s;
  }
  let fr=0;
  if(ghost) fr=Math.min(1,(now-ghost.t0)/1000/ghost.T);
  $("prog").firstChild.style.width=(fr*100).toFixed(1)+"%";
  requestAnimationFrame(draw);
}
function banners(){
  const bad=$("bnBad"), info=$("bnInfo");
  let b="", i="";
  if(!st||Date.now()-lastOk>1500) b="画面のサーバから返事がありません";
  else if(tripped()) b="遮断中の軸があります（「遮断を解除」）";
  if(flash&&Date.now()<flash.until){ if(flash.bad) b=b||flash.msg; else i=flash.msg; }
  else if(st&&st.note) i=st.note;
  bad.style.display=b?"block":"none"; if(bad.textContent!==b) bad.textContent=b;
  info.style.display=i?"block":"none"; if(info.textContent!==i) info.textContent=i;
}
function render(){
  $("dot").className="dot"+(st&&Date.now()-lastOk<1500?" on":"");
  const n12=NODES.filter(alive).length;
  const ch="<span class='chip "+(n12===12?"ok":"warn")+"'>軸 "+n12+" / 12</span>"
    +(allArmed()?"<span class='chip ok'>通電中</span>":"")
    +(playing()?"<span class='chip ok'>再生中 "+esc(st.jt.name||"")+"</span>":"");
  if($("chips").innerHTML!==ch) $("chips").innerHTML=ch;
  const prev=$("preview").checked, idle=!busy&&!playing();
  $("btnZero").disabled=!ready12()||!idle||tripped();
  $("btnZeroSet").disabled=!allMode("damp")||!idle;
  $("btnHold").disabled=!ready12()||!idle||tripped();
  for(const b of document.querySelectorAll("#moves button")) b.disabled=!idle||!(prev||allMode("hold"));
  $("btnAll").disabled=!idle||!(prev||allMode("hold"));
  $("btnStep").disabled=!idle||!allMode("hold");
  $("btnSine").disabled=!idle||!allMode("hold");
  $("btnSdom").disabled=!idle||!(prev||allMode("hold"));
  $("btnFinale").disabled=!idle||!(prev||allMode("hold"));
  for(const k of [3,2,1]){ const b=$("lay"+k); b.disabled=!idle||!!ghost; b.classList.toggle("on",LAY===LAYOUTS[k]); }
  $("btnStop").disabled=!(playing()||chain||ghost);
  $("btnDisarm").disabled=!(st&&st.armed);
  let zh="";
  if(!ready12()) zh="12 軸そろっていません（返事がある軸 "+n12+" / 12）";
  else if(allMode("damp")) zh="ダンパー中：手で 12 本の矢印を真上に合わせて「決定」";
  else if(allMode("hold")) zh="位置保持中：動きのボタンで再生できます";
  else zh="ふだんは「通電」で保存した 0 点（真上）へ戻ります。0 点を合わせ直すなら「0 点合わせ」→ 手で真上に → 「決定」";
  if($("zeroHint").textContent!==zh) $("zeroHint").textContent=zh;
  let ph=busy?("進めています："+busy+(chain?"（止めるで止まる）":"")):(ghost&&prev?"見本を出しています（モーターは動きません）":"");
  if(!busy&&playing()&&st.jt.cycles!=null&&/^角度の並び/.test(st.jt.name||"")){
    ph=st.jt.name+"："+(st.jt.cycles+1)+" 巡目"+(st.jt.stop_at_wrap?"・止めています":"");
  }
  if($("playHint").textContent!==ph) $("playHint").textContent=ph;
  banners();
}
async function poll(){
  if(inflight){ render(); return; }
  inflight=true;
  try{
    const r=await fetch("/api/demo?cid="+CID,{cache:"no-store"});
    st=await r.json(); lastOk=Date.now();
  }catch(e){}finally{ inflight=false; }
  render();
}

buildGrid();
$("sdHint").textContent=sdHintText();
for(const k of [3,2,1]) $("lay"+k).onclick=()=>{
  LAY=LAYOUTS[k];
  try{ localStorage.setItem("krc.demo.layout",String(k)); }catch(e){}
  buildGrid(); $("sdHint").textContent=sdHintText(); render(); };
$("moves").innerHTML=MOVES.map((m,i)=>"<button data-i='"+i+"'>"+esc(m.name)+"<small>"+esc(m.sub)+"</small></button>").join("");
for(const b of document.querySelectorAll("#moves button")) b.onclick=()=>play(MOVES[+b.dataset.i]);
$("btnZero").onclick=zeroStart;
$("btnZeroSet").onclick=zeroSet;
$("btnHold").onclick=holdKeep;
$("btnAll").onclick=playAll;
$("btnStop").onclick=stopAll;
$("btnStep").onclick=stepStart;
for(const id of ["stList","stStep","stLag"]) $(id).addEventListener("input",stepHint);
stepHint();
$("btnSine").onclick=sineStart;
$("btnSdom").onclick=()=>play(sdMove());
$("btnFinale").onclick=()=>play(finaleMove());
$("swLag").addEventListener("input",sineEdited);
for(const id of ["swAmp","swPer"]) $(id).addEventListener("input",sineEdited);
$("btnDisarm").onclick=disarmAll;
$("btnClear").onclick=()=>post("clear");
$("btnEstop").onclick=()=>{ chain=null; ghost=null; post("estop"); };
$("btnBack").onclick=()=>{ location.href="/"; };
try{ $("flip").checked=localStorage.getItem("krc.demo.flip")==="1"; }catch(e){}
$("flip").onchange=()=>{ try{ localStorage.setItem("krc.demo.flip",$("flip").checked?"1":"0"); }catch(e){} };
window.addEventListener("keydown",e=>{
  if(e.target&&e.target.tagName==="INPUT"&&e.code==="Space") return;
  if(e.code==="Space"||e.code==="Escape"){ e.preventDefault(); chain=null; ghost=null; post("estop"); }
});
document.addEventListener("visibilitychange",()=>{ if(document.hidden){ chain=null; post("disarm"); post("demo_tlm",{div:1}); } });
window.addEventListener("pagehide",()=>{
  navigator.sendBeacon("/api/cmd",new Blob([JSON.stringify({action:"disarm",params:{cid:CID}})],{type:"application/json"}));
  navigator.sendBeacon("/api/cmd",new Blob([JSON.stringify({action:"demo_tlm",params:{div:1}})],{type:"application/json"}));
});
fetch("/api/demo?cid="+CID,{cache:"no-store"}).then(r=>r.json()).then(s=>{ if(!s.armed) post("demo_tlm",{div:1}); }).catch(()=>{});
poll(); setInterval(poll,50); requestAnimationFrame(draw);
</script></body></html>
"""


COMMON_CSS = r"""
:root{--bg:#0d1117;--panel:#161b22;--line:#30363d;--fg:#e6edf3;--dim:#8b949e;
  --ok:#3fb950;--warn:#d29922;--bad:#f85149;--cyan:#39c5cf;--pri:#1f6feb;--estop:#b62324;
  --c1:#58a6ff;--c2:#7ee787;--c3:#ff7b72;--c4:#d2a8ff;--c5:#ffa657;--c6:#ff9bce;--c7:#f2cc60;
  --btn:#21262d;--bad-bg:#2d1114;--bad-fg:#ffb4ae;--warn-bg:#2b2411;--warn-fg:#f0d38a;--info-bg:#0f2a3a;--info-fg:#bfe9ef}
*{box-sizing:border-box}

body{font-family:"Segoe UI",system-ui,sans-serif}

:focus-visible{outline:2px solid var(--c4);outline-offset:2px}
input[type=checkbox]{accent-color:var(--fg)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--dim);display:inline-block;flex:none}
.dot.on{background:var(--ok)}

button{font:inherit;font-size:13px;line-height:19px;padding:6px 12px;border-radius:6px;cursor:pointer;
  background:var(--btn);border:1px solid var(--line);color:var(--fg)}
button:hover:not(:disabled){border-color:var(--dim)}
button:disabled{opacity:.45;cursor:not-allowed}
button.pri{background:var(--pri);border-color:var(--pri);color:#fff}

button.on{background:var(--fg);border-color:var(--fg);color:var(--bg);font-weight:600}
button.estop{background:var(--estop);border-color:var(--estop);color:#fff;font-weight:700;font-size:15px;line-height:21px;padding:7px 16px}

button.rec{border-color:var(--cyan);color:var(--cyan)}
#btnRecM,#btnRec{min-width:130px}
button.rec .blink{animation:blink 1s step-end infinite}
@keyframes blink{50%{opacity:.2}}

.rdot{display:inline-block;width:8px;height:1em;vertical-align:-0.12em;background:radial-gradient(circle,currentColor 0 4px,transparent 4.5px)}
button.warn{border-color:var(--warn);color:var(--warn)}
button.ghost{background:transparent;color:var(--dim);border-style:dashed}
.chip{display:inline-block;padding:2px 8px;border-radius:999px;font-size:11px;line-height:16px;
  border:1px solid var(--line);color:var(--dim);margin:2px 4px 2px 0;white-space:nowrap}
.chip.warn{border-color:var(--warn);color:var(--warn)}
.chip.bad{border-color:var(--bad);color:var(--bad)}
.chip.ok{border-color:var(--ok);color:var(--ok)}
.banner{display:none;padding:8px 12px;border-radius:6px;font-size:13px;line-height:1.6}
.banner.bad{background:var(--bad-bg);border:1px solid var(--bad);color:var(--bad-fg)}
.banner.warn{background:var(--warn-bg);border:1px solid var(--warn);color:var(--warn-fg)}
.banner.info{background:var(--info-bg);border:1px solid var(--cyan);color:var(--info-fg)}
"""
assert (PAGE.count("/*__COMMON_CSS__*/") == 1 and LEG_PAGE.count("/*__COMMON_CSS__*/") == 1
        and DEMO_PAGE.count("/*__COMMON_CSS__*/") == 1)
PAGE = PAGE.replace("/*__COMMON_CSS__*/", COMMON_CSS)
LEG_PAGE = LEG_PAGE.replace("/*__COMMON_CSS__*/", COMMON_CSS)
DEMO_PAGE = DEMO_PAGE.replace("/*__COMMON_CSS__*/", COMMON_CSS)


def main():
    sys.setswitchinterval(0.0005)
    ap = argparse.ArgumentParser(description="KRC アクチュエータの画面")
    ap.add_argument("-n", "--node", default="auto",
                    help="動かす軸。'auto'（既定）はバスを2秒聞いて見つける。"
                         "'1,2,3' のように並べても指定できる")
    ap.add_argument("-r", "--rate", type=float, default=200.0)
    ap.add_argument("-p", "--port", type=int, default=8790)
    ap.add_argument("-i", "--iface", default="can0")
    ap.add_argument("--host", default="0.0.0.0")
    a = ap.parse_args()

    auto = str(a.node).strip().lower() in ("auto", "")
    if auto:
        print("バスを2秒聞いて、居る軸を探します（こちらからは何も送りません）…")
        nodes = discover(a.iface, 2.0)
        if not nodes:
            print("  見つかりませんでした。軸なしで立ち上げます"
                  "（電源を入れた軸は、解除中なら自動で並びます）")
        else:
            print("  見つかった軸: %s" % "・".join(str(n) for n in nodes))
    else:
        nodes = [x for x in str(a.node).replace(" ", "").split(",") if x]
        if not nodes:
            print("立ち上げられません: 軸の指定が読めません: %r（'auto' か '1,2' の形）" % a.node)
            return 2

    try:
        ctrl = Controller(nodes, a.rate, a.iface, auto_scan=auto)
    except BadValue as e:
        print("立ち上げられません: %s" % e)
        return 2
    ctrl.start()
    Handler.ctrl = ctrl

    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    srv.daemon_threads = True

    ip = "?"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except OSError:
        pass

    print("KRC アクチュエータの画面")
    print("  軸 %s / %.0f Hz / %s"
          % ("・".join(str(n) for n in ctrl.order), ctrl.rate, a.iface))
    print("  http://%s:%d/  （このPCから）" % (ip, a.port))
    print("  Ctrl-C で終了。終了時は必ずモーターモードを抜けます")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.armed = False
        try:
            for n in ctrl.order:
                ctrl._send_special(0xFD, n)
        except OSError:
            pass
        ctrl.stopping = True
        print("\n終了しました")
    return 0


if __name__ == "__main__":
    sys.exit(main())
