import io
import json
import math
import os
import shutil
import time

L1_MM = 250.0
_TRI_K = L1_MM / math.hypot(246.1, 43.7)
L2 = 246.1 * _TRI_K
L3 = 43.7 * _TRI_K
L4 = 35.0
L5 = 255.0
L6 = 35.0
L7 = 250.0
ALPHA_DEG = 13.0
SG = -1.0
CONV = 2
PHI1_DEG = 48.0
PHI2_DEG = 156.0
TILT_DEG = 45.0

RATIO_DEFAULT = 9.0
JOINT_FRICTION_MOTOR_NM = 0.04
DTOUT_H_DEG = 0.25
CAL_PATH = os.path.expanduser("~/krcleg.json")


def l1():
    return math.hypot(L2, L3)


def wrap180(dg):
    return ((dg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0


def _consts():
    a = l1()
    K1 = a / L4
    K2 = a / L6
    K3 = (a * a + L4 * L4 + L6 * L6 - L5 * L5) / (2.0 * L4 * L6)
    return K1, K2, K3


def solve(tin_deg):
    K1, K2, K3 = _consts()
    t = math.radians(SG * tin_deg)
    A = K1 - math.cos(t)
    B = -math.sin(t)
    C = K2 * math.cos(t) - K3
    R = math.hypot(A, B)
    ok = abs(C) <= R
    if not ok:
        return dict(ok=False, psi=None, tout=None, margin=R - abs(C))
    psi = math.atan2(B, A) + math.acos(max(-1.0, min(1.0, C / R)))
    tout = wrap180(SG * (math.degrees(psi) + 180.0 + SG * ALPHA_DEG))
    return dict(ok=True, psi=psi, tout=tout, margin=R - abs(C))


def solve_inv(tout_deg):
    K1, K2, K3 = _consts()
    psi = math.radians(SG * tout_deg - 180.0 - SG * ALPHA_DEG)
    A = K2 + math.cos(psi)
    B = math.sin(psi)
    C = K1 * math.cos(psi) + K3
    R = math.hypot(A, B)
    if abs(C) > R:
        return []
    base = math.atan2(B, A)
    h = math.acos(max(-1.0, min(1.0, C / R)))
    out = []
    for br in (+1.0, -1.0):
        tin = wrap180(SG * math.degrees(base + br * h))
        f = solve(tin)
        if f["ok"] and abs(wrap180(f["tout"] - tout_deg)) < 1e-6:
            out.append(round(tin, 6))
    return sorted(set(out))


def dtout_dtin(tin_deg, h=DTOUT_H_DEG):
    a = solve(tin_deg - h)
    b = solve(tin_deg + h)
    if not (a["ok"] and b["ok"]):
        return None
    return wrap180(b["tout"] - a["tout"]) / (2.0 * h)


def tin_for_tout(tout_deg, near_deg=None):
    cands = solve_inv(tout_deg)
    if not cands:
        return None
    if len(cands) == 1:
        return cands[0]
    if near_deg is not None:
        return min(cands, key=lambda x: abs(wrap180(x - near_deg)))
    inside = [x for x in cands if -176.0 <= x <= -45.0]
    return inside[0] if inside else cands[0]


class Grav(object):

    def __init__(self, a_nm=0.0, phase_deg=0.0, on=False, samples=None):
        self.a_nm = float(a_nm)
        self.phase_deg = float(phase_deg)
        self.on = bool(on)
        self.samples = [tuple(x) for x in (samples or [])]

    def tau_knee(self, tout_deg):
        return self.a_nm * math.sin(math.radians(tout_deg - self.phase_deg))

    def fit(self):
        if len(self.samples) < 2:
            return False
        s11 = s12 = s22 = b1 = b2 = 0.0
        for th, tau in self.samples:
            x1 = math.sin(math.radians(th))
            x2 = math.cos(math.radians(th))
            s11 += x1 * x1
            s12 += x1 * x2
            s22 += x2 * x2
            b1 += x1 * tau
            b2 += x2 * tau
        det = s11 * s22 - s12 * s12
        if abs(det) < 1e-9:
            return False
        c1 = (b1 * s22 - b2 * s12) / det
        c2 = (s11 * b2 - s12 * b1) / det
        self.a_nm = math.hypot(c1, c2)
        self.phase_deg = math.degrees(math.atan2(-c2, c1))
        return True

    def to_dict(self):
        return dict(a_nm=self.a_nm, phase_deg=self.phase_deg, on=self.on,
                    samples=[[float(a), float(b)] for a, b in self.samples])

    @classmethod
    def from_dict(cls, d):
        def fnum(v, dflt):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return dflt
            return float(v)
        samples = []
        for x in (d.get("samples") or []):
            if isinstance(x, (list, tuple)) and len(x) == 2:
                a = fnum(x[0], None)
                b = fnum(x[1], None)
                if a is not None and b is not None:
                    samples.append((a, b))
        return cls(a_nm=fnum(d.get("a_nm"), 0.0), phase_deg=fnum(d.get("phase_deg"), 0.0),
                   on=(d.get("on") is True), samples=samples)


class Calib(object):

    def __init__(self, plus_end=None, half=None, tin0=None, ref_src="stop"):
        self.plus_end = plus_end
        self.half = half
        self.tin0 = None if tin0 is None else float(tin0)
        self.ref_src = ref_src
        self.ratio = RATIO_DEFAULT
        self.grav = Grav()
        self._auto_ref()

    @property
    def sign(self):
        if self.plus_end == "extend":
            return -1.0
        if self.plus_end == "fold":
            return 1.0
        return None

    @property
    def valid(self):
        return (self.sign is not None and self.half is not None and self.half > 0
                and self.tin0 is not None)

    def why_invalid(self):
        if self.half is None:
            return "原点出しが済んでいません"
        if self.sign is None:
            return "畳み切りのストッパが＋側か−側か、まだ選ばれていません"
        if self.tin0 is None:
            return "基準が決まっていません"
        return None

    def _auto_ref(self):
        if self.ref_src == "manual" and self.tin0 is not None:
            return
        if self.half is None or self.sign is None:
            return
        m_fold = self.half if self.plus_end == "fold" else -self.half
        tin_fold = tin_for_tout(PHI2_DEG)
        self.tin0 = tin_fold - self.sign * math.degrees(m_fold) / self.ratio
        self.ref_src = "stop"

    def set_span(self, half):
        self.half = float(half)
        if self.ref_src == "stop":
            self.tin0 = None
        self._auto_ref()

    def set_plus_end(self, which):
        if which not in ("extend", "fold"):
            raise ValueError("plus_end は extend / fold")
        if which != self.plus_end:
            self.tin0 = None
            self.ref_src = "stop"
            if self.plus_end is not None:
                self.half = None
        self.plus_end = which
        self._auto_ref()

    def invalidate(self):
        self.half = None

    def clear_ref(self):
        self.tin0 = None
        self.ref_src = "stop"
        self._auto_ref()

    def set_from_tout(self, motor_rad, tout_now_deg):
        if self.sign is None or self.half is None:
            return False
        near = self.motor_to_tin(motor_rad) if self.valid else None
        want = tin_for_tout(tout_now_deg, near)
        if want is None:
            return False
        self.tin0 = want - self.sign * math.degrees(motor_rad) / self.ratio
        self.ref_src = "manual"
        return True

    def motor_to_tin(self, motor_rad):
        if not self.valid:
            return None
        return self.tin0 + self.sign * math.degrees(motor_rad) / self.ratio

    def tin_to_motor(self, tin_deg):
        if not self.valid:
            return None
        return math.radians((tin_deg - self.tin0) * self.sign * self.ratio)

    def motor_to_tout(self, motor_rad):
        t = self.motor_to_tin(motor_rad)
        if t is None:
            return None
        f = solve(t)
        return f["tout"] if f["ok"] else None

    def tout_to_motor(self, tout_deg, near_motor_rad=None):
        if not self.valid:
            return None
        near = None if near_motor_rad is None else self.motor_to_tin(near_motor_rad)
        tin = tin_for_tout(tout_deg, near)
        return None if tin is None else self.tin_to_motor(tin)

    def end_touts(self):
        if not self.valid:
            return (None, None)
        return (self.motor_to_tout(-self.half), self.motor_to_tout(self.half))

    def knee_from_motor_nm(self, motor_rad, tau_motor):
        if not self.valid:
            return None
        g = dtout_dtin(self.motor_to_tin(motor_rad))
        if g is None or abs(g) < 1e-6:
            return None
        return tau_motor * self.ratio * self.sign / g

    def motor_from_knee_nm(self, motor_rad, tau_knee):
        if not self.valid:
            return None
        g = dtout_dtin(self.motor_to_tin(motor_rad))
        if g is None:
            return None
        return tau_knee * g / (self.ratio * self.sign)

    def ext_torque_out(self, motor_rad, iq, kt):
        if not self.valid or iq is None or motor_rad is None:
            return (None, None)
        tm = self.knee_from_motor_nm(motor_rad, kt * iq)
        tf = self.knee_from_motor_nm(motor_rad, JOINT_FRICTION_MOTOR_NM)
        if tm is None or tf is None:
            return (None, None)
        return (-tm, abs(tf))

    def slope_deg_per_rad(self):
        if self.sign is None:
            return None
        return self.sign * (180.0 / math.pi) / self.ratio

    def to_dict(self):
        return dict(plus_end=self.plus_end, half=self.half, tin0=self.tin0, ref_src=self.ref_src,
                    ratio=self.ratio, grav=self.grav.to_dict(), conv=CONV)

    @staticmethod
    def _finite_or_none(v):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return None
        return float(v) if math.isfinite(v) else None

    @classmethod
    def from_dict(cls, d):
        plus_end = d.get("plus_end")
        if plus_end not in ("extend", "fold"):
            plus_end = None
        tin0 = cls._finite_or_none(d.get("tin0"))
        if tin0 is not None and d.get("conv") != CONV:
            tin0 = -tin0
        ref_src = d.get("ref_src", "stop")
        if ref_src != "manual" or tin0 is None:
            ref_src = "stop"
        c = cls(plus_end=plus_end, tin0=tin0, ref_src=ref_src)
        c.half = None
        c.grav = Grav.from_dict(d.get("grav") if isinstance(d.get("grav"), dict) else {})
        return c

    @classmethod
    def load(cls, path=None):
        path = path or CAL_PATH
        try:
            with io.open(path, encoding="utf-8-sig") as f:
                d = json.load(f)
            if not isinstance(d, dict):
                return cls()
            return cls.from_dict(d)
        except (OSError, ValueError):
            return cls()

    def save(self, path=None):
        path = path or CAL_PATH
        tmp = path + ".tmp"
        with io.open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=1)
        os.replace(tmp, path)


def _quarantine(path, what):
    bad = "%s.bad-%s" % (path, time.strftime("%Y%m%d-%H%M%S"))
    k = 0
    while os.path.exists(bad):
        k += 1
        bad = "%s.bad-%s-%d" % (path, time.strftime("%Y%m%d-%H%M%S"), k)
    try:
        shutil.copyfile(path, bad)
        kept = "原本は %s に残しました" % bad
    except OSError as e:
        kept = "原本を写せませんでした: %s" % e
    return "較正ファイル %s が読めなかったので既定で始めました（%s。%s）" % (path, what, kept)


def load_all_why(path=None):
    path = path or CAL_PATH
    try:
        with io.open(path, encoding="utf-8-sig") as f:
            d = json.load(f)
    except FileNotFoundError:
        return {}, None
    except OSError as e:
        return {}, "較正ファイル %s が読めなかったので既定で始めました（%s）" % (path, e)
    except ValueError as e:
        return {}, _quarantine(path, "JSON として読めない: %s" % e)
    if not isinstance(d, dict):
        return {}, _quarantine(path, "中身が軸ごとの表ではない: %s" % type(d).__name__)
    out = {}
    for k, v in d.items():
        try:
            n = int(k)
        except (TypeError, ValueError):
            continue
        if isinstance(v, dict):
            try:
                out[n] = Calib.from_dict(v)
            except (TypeError, ValueError):
                out[n] = Calib()
    return out, None


def load_all(path=None):
    return load_all_why(path)[0]


def save_all(cals, path=None):
    path = path or CAL_PATH
    tmp = path + ".tmp"
    with io.open(tmp, "w", encoding="utf-8") as f:
        json.dump(dict((str(int(n)), c.to_dict()) for n, c in cals.items()),
                  f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def design():
    return dict(L2=L2, L3=L3, L4=L4, L5=L5, L6=L6, L7=L7, ALPHA=ALPHA_DEG,
                PHI1=PHI1_DEG, PHI2=PHI2_DEG, TILT=TILT_DEG, L1=l1(), SG=SG)


def design_half_rad():
    return math.radians(abs(tin_for_tout(PHI1_DEG) - tin_for_tout(PHI2_DEG))) * RATIO_DEFAULT / 2.0


def reachable_tout(cal, p_lo, p_hi, n=200):
    if not cal.valid or p_hi <= p_lo:
        return None, None
    lo = hi = prev = None
    for k in range(n + 1):
        t = cal.motor_to_tout(p_lo + (p_hi - p_lo) * k / n)
        if t is None:
            continue
        if prev is not None:
            t += 360.0 * round((prev - t) / 360.0)
        prev = t
        lo = t if lo is None else min(lo, t)
        hi = t if hi is None else max(hi, t)
    return lo, hi
