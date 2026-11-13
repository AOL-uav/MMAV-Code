"""Tests for swoop_model_v1.py.   py -B rigid_glider/test_swoop_model_v1.py   (stdlib only; imports the old modules read-only for the comparison test)."""
import sys
sys.dont_write_bytecode = True

import copy
import hashlib
import math
import os
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import swoop_model_v1 as m
import swiftA_oscillation_study as so
import pullout_sim as ps

FROZEN_SHA256 = "4ffc89133b724cf974aeb9f822b6f9af83129a56f4fb0a21bf8f35eeb9d5965c"       # hash of the frozen README section, taken before any run
DX = 0.1337
GL = None
RESULTS = []


def gl():
    global GL
    if GL is None:
        GL = m.build_gliders(DX)
    return GL


def case(fn):
    def run():
        t0 = time.time()
        try:
            msg = fn()
            RESULTS.append((fn.__name__, True, msg or "", time.time() - t0))
        except Exception as e:
            RESULTS.append((fn.__name__, False, f"{e!r}\n{traceback.format_exc()}", time.time() - t0))
    run.__name__ = fn.__name__
    return run


# ------------------------------------------------------------------------------------------------ hash
@case
def t_frozen_hash():
    txt = m.frozen_text()
    h = hashlib.sha256(txt.encode("utf-8")).hexdigest()
    assert h == FROZEN_SHA256, f"frozen section changed: {h}"
    stored = (m.OUT_DEFAULT / "README_frozen.sha256").read_text().strip()
    assert stored == FROZEN_SHA256, "stored hash file differs from the constant in this test"
    assert txt.count(m.FROZEN_MARK) == 1 and "BEFORE any run" in txt
    return h[:16]


# ------------------------------------------------------------------------------------------------ lift curve vs cited data
def curve(s1_per_rad, ab=8.0, ld=6.0):
    g = gl()["b45"]
    sim = m.Sim(g, 66.0, 10.0, dict(s1=s1_per_rad / m.R2D, ab=ab, ld=ld, dx_default=DX))
    def cl(a):
        cn, ca, dx, c_l, c_d = sim.coefs(a, sim.fstat(a))
        return c_l, c_d
    return sim, cl


@case
def t_lift_curve_data_points():
    msgs = []
    # (planform, s1 /rad, alpha_CLmax, CLmax, tolerance on CL at that angle) from Okamoto & Jinba 2009 Table 1 (low_re_pitch.md).  The model anchors CL(35 deg) to 0.85, the mean of the
    # AR>=4 rows (spread 0.76-0.96), so the stated tolerance is +-0.15 (about the spread of the data).  Slope over 0-2 deg: within 2 %.  Best L/D 5.5-6.5 at 3.5-5.6 deg.
    pts = [("rect AR4", 3.78, 33.0, 0.81, 0.15), ("rect AR6", 4.47, 36.0, 0.96, 0.15), ("ellipse AR8", 4.98, 37.0, 0.85, 0.15), ("ellipse AR6", 4.07, 40.0, 0.80, 0.15), ("ellipse AR4", 3.32, 39.0, 0.76, 0.15)]
    for name, s1, a, clmax, tol in pts:
        sim, cl = curve(s1)
        lo = cl(2.0)[0] / 2.0 * m.R2D                                 # slope over 0-2 deg, /rad
        assert abs(lo - s1) / s1 < 0.02, (name, lo, s1)
        got = cl(a)[0]
        assert 0.76 <= max(cl(0.5 * k)[0] for k in range(0, 121)) <= 0.98, name
        assert abs(got - clmax) <= tol, (name, got, clmax)
        msgs.append(f"{name}: CL({a:g})={got:.2f} vs {clmax}")
        # slope break: mean slope 10-20 deg well below the attached slope, curve non-decreasing (within 0.004 in CL) to 35 deg
        assert (cl(20.0)[0] - cl(10.0)[0]) / 10.0 < 0.35 * sim.s1, name
        prev = -1
        for k in range(0, 71):
            v = cl(0.5 * k)[0]
            assert v >= prev - 0.004, (name, 0.5 * k)           # the smooth blend of the two slopes dips by up to 0.003 in CL when s2 is small (< 0.5 % of CL): known, harmless artefact
            prev = v
        # best L/D: Okamoto 5.5-6.5 at 4.0-5.4 deg
        best = max(((cl(0.1 * k)[0] / cl(0.1 * k)[1], 0.1 * k) for k in range(5, 100)))
        assert 5.5 <= best[0] <= 6.5 and 3.5 <= best[1] <= 5.6, (name, best)
    sim, _ = curve(3.78)
    cn_p, _, _, _, _ = sim.coefs(12.0, 1.0)
    cn_m, _, _, _, _ = sim.coefs(-12.0, 1.0)
    assert abs(cn_p + cn_m) < 1e-12
    return "; ".join(msgs[:3])


# ------------------------------------------------------------------------------------------------ energy / units
def free_state_run(sim, t_end, dt=0.004):
    s = [0.0, 0.0, -90.0 * m.D2R, 0.0, 0.0, 0.0, 0.0]
    E = lambda st: 0.5 * sim.mass * (st[3] ** 2 + st[4] ** 2) + 0.5 * sim.I * st[5] ** 2 + sim.mass * m.G * st[1]
    power = lambda st: (lambda d: sim.mass * (st[3] * d[3] + st[4] * (d[4] + m.G)) + sim.I * st[5] * d[5])(sim.deriv(st))      # aerodynamic power = F.v + M q
    E0, P0, t, I = E(s), power(s), 0.0, 0.0
    for _ in range(int(t_end / dt)):
        s2 = sim.rk4(s, dt, 0.0, 0.0, 0.0)
        P1 = power(s2)
        I += 0.5 * (P0 + P1) * dt
        s, P0 = s2, P1
    return E(s) - E0, I


@case
def t_energy_and_units():
    g = gl()["swift"]
    # 1. no aerodynamics (zero area): free fall z = -g t^2 / 2
    g0 = copy.copy(g)
    g0.S = 0.0
    sim0 = m.Sim(g0, 73.0, 6.0, dict(dx_default=DX))
    rec = m.simulate(sim0, t_max=1.0)
    zt = rec[-1][2]
    assert abs(zt + 0.5 * m.G * rec[-1][0] ** 2) < 1e-6, zt
    # 2. energy bookkeeping: change of mechanical energy equals the integral of the aerodynamic power (integrator and units consistent)
    sim = m.Sim(g, 73.0, 6.0, dict(dx_default=DX))
    dE, I = free_state_run(sim, 10.0)
    assert abs(dE - I) < 0.01 * max(abs(dE), 1e-3), (dE, I)
    # 3. trim: weight equals the resultant force, glide angle = atan(CD/CL)
    tr = m.find_trim(sim)
    H = 0.5 * m.RHO * tr["V"] ** 2 * sim.S * math.hypot(*sim.coefs(tr["a_eff"], sim.fstat(tr["a_eff"]))[:2])
    assert abs(H - sim.W) / sim.W < 1e-6
    # glide angle = -atan(CD/CL) minus the force-vector rotation tau x device that the angle-shift treatment of the device carries (inherited from the old sim)
    assert abs(tr["gamma"] - (-math.degrees(math.atan(tr["cd"] / tr["cl"])) - sim.tau_up)) < 0.2, (tr["gamma"], tr["cd"] / tr["cl"], sim.tau_up)
    # 4. magnitudes (mass g, speed m/s, density, gravity)
    assert 9.0 < sim.mass * 1e3 < 10.5 and 3.0 < tr["V"] < 8.0, (sim.mass, tr["V"])
    assert abs(m.G - 9.80665) < 1e-9 and 1.1 < m.RHO < 1.3
    sb = m.Sim(gl()["b45"], 66.0, 10.0, dict(dx_default=DX))
    assert 15e-3 < sb.mass < 20e-3 and 0.03 < sb.S < 0.04
    # 5. as-flown run: pulls out within 5-10 m and glides at the trim flight path
    a = m.run_case(g, 73.0, 6.0, "base", dx_max=DX)
    assert 3.0 < a["h_pullout_m"] < 12.0 and abs(a["gamma_final"] - tr["gamma"]) < 0.5, (a["h_pullout_m"], a["gamma_final"], tr["gamma"])
    return f"dE {dE:.4f} J vs integral {I:.4f} J; trim V {tr['V']:.2f} m/s; pull-out loss {a['h_pullout_m']:.1f} m"


# ------------------------------------------------------------------------------------------------ numerical convergence
@case
def t_step_convergence():
    g = gl()["b45"]
    sims = [m.Sim(g, 66.0, 10.0, dict(dx_default=DX)) for _ in range(2)]
    ra = m.analyze(m.simulate(sims[0], dt=0.004), sims[0])
    rb = m.analyze(m.simulate(sims[1], dt=0.002), sims[1])
    assert abs(ra["amp1"] - rb["amp1"]) < 0.03 * rb["amp1"], (ra["amp1"], rb["amp1"])
    assert abs(ra["period_s"] - rb["period_s"]) < 0.02 * rb["period_s"], (ra["period_s"], rb["period_s"])
    assert ra["class"] == rb["class"]
    return f"amp1 {ra['amp1']:.2f}/{rb['amp1']:.2f}, period {ra['period_s']:.3f}/{rb['period_s']:.3f}"


# ------------------------------------------------------------------------------------------------ linear model vs the old sim
@case
def t_linear_matches_old_phugoid():
    a = {0.0: m.parse_avl_log(m.SWIFT_AVL_DIR / "swA400_b0.log"), 5.0: m.parse_avl_log(m.SWIFT_AVL_DIR / "swA400_b5.log")}
    nm = so.numbers_from_avl(a)
    spec = so.make_spec(nm, so.mass_basis())
    plate = ps.PlateTable(spec["ar"], spec["taper"])
    old, _, _ = so.run_one(spec, plate, 73.0, 6.0)
    T_old = old["modes"]["phugoid"]["period"]
    g = gl()["swift"]
    res = {}
    # fully linear (no break, no CP shift); (i) with the old sim's own lift slope and L/D, (ii) with this model's base numbers
    for lab, kw in (("old-matched slope and L/D", dict(s1=plate.cl_alpha_deg, ld=8.1)), ("base numbers", {})):
        kw = dict(kw)
        kw["dx_default"] = DX
        sim = m.Sim(g, 73.0, 6.0, kw, linear=True)
        tr = m.find_trim(sim)
        lm = m.linear_modes(sim, tr)
        res[lab] = lm["phugoid_period"]
    d1 = abs(res["old-matched slope and L/D"] - T_old) / T_old
    d2 = abs(res["base numbers"] - T_old) / T_old
    assert d1 <= 0.05, f"linear (old-matched) phugoid {res['old-matched slope and L/D']:.3f} s vs old {T_old:.3f} s: {100 * d1:.1f} %"
    assert d2 <= 0.08, d2                           # sanity only: base numbers are not the old sim's numbers
    return f"old {T_old:.3f} s; linear old-matched {res['old-matched slope and L/D']:.3f} s ({100 * d1:.1f} %); linear base numbers {res['base numbers']:.3f} s ({100 * d2:.1f} %)"


# ------------------------------------------------------------------------------------------------ GIF decode (own LZW decoder)
def lzw_decode(data, min_size, npix):
    clear, eoi = 1 << min_size, (1 << min_size) + 1
    cs = min_size + 1
    table = [bytes([i]) for i in range(clear)] + [b"", b""]
    out = bytearray()
    prev = None
    acc = nb = pos = 0
    while len(out) < npix:
        while nb < cs:
            if pos >= len(data):
                raise ValueError("ran out of data")
            acc |= data[pos] << nb
            pos += 1
            nb += 8
        code = acc & ((1 << cs) - 1)
        acc >>= cs
        nb -= cs
        if code == clear:
            table = table[:clear + 2]
            cs = min_size + 1
            prev = None
            continue
        if code == eoi:
            break
        if prev is None:
            entry = table[code]
        else:
            entry = table[code] if code < len(table) else prev + prev[:1]
            table.append(prev + entry[:1])
            if len(table) == (1 << cs) and cs < 12:
                cs += 1
        out += entry
        prev = entry
    return bytes(out)


def gif_decode(path):
    b = Path(path).read_bytes()
    assert b[:6] == b"GIF89a" and b[-1] == 0x3B
    w, h = int.from_bytes(b[6:8], "little"), int.from_bytes(b[8:10], "little")
    flags = b[10]
    ncol = 2 ** ((flags & 7) + 1) if flags & 0x80 else 0
    pos = 13 + 3 * ncol
    frames = []
    while b[pos] != 0x3B:
        if b[pos] == 0x21:
            pos += 2
            while b[pos]:
                pos += b[pos] + 1
            pos += 1
        elif b[pos] == 0x2C:
            iw, ih = int.from_bytes(b[pos + 5:pos + 7], "little"), int.from_bytes(b[pos + 7:pos + 9], "little")
            assert (iw, ih) == (w, h) and b[pos + 9] == 0
            pos += 10
            mn = b[pos]
            pos += 1
            data = bytearray()
            while b[pos]:
                data += b[pos + 1:pos + 1 + b[pos]]
                pos += b[pos] + 1
            pos += 1
            frames.append(lzw_decode(bytes(data), mn, w * h))
        else:
            raise ValueError("bad block")
    return w, h, ncol, frames


@case
def t_gif_roundtrip_and_output():
    # (1) encoder/decoder round trip on synthetic data incl. table overflow (> 4096 codes)
    import random
    rng = random.Random(3)
    data = bytes(rng.choice([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]) if rng.random() < 0.3 else (i // 7) % 12 for i in range(60000))
    enc = m.lzw_encode(data, 4)
    assert lzw_decode(enc, 4, len(data)) == data
    # (2) the real output
    p = m.OUT_DEFAULT / "swoop_compare.gif"
    assert p.exists(), "swoop_compare.gif missing (run swoop_model_v1.py first)"
    w, h, ncol, frames = gif_decode(p)
    assert ncol == 16 and len(frames) >= 100 and all(len(f) == w * h for f in frames)
    assert max(max(f) for f in frames[::10]) < 16
    assert frames[0] != frames[len(frames) // 2] != frames[-1]
    lit = sum(1 for v in frames[-1] if v != 0)
    assert lit > 0.2 * w * h, lit
    return f"{len(frames)} frames {w}x{h}, {p.stat().st_size // 1024} KiB"


# ------------------------------------------------------------------------------------------------ outputs and scope
@case
def t_outputs_present_and_dark():
    d = m.OUT_DEFAULT
    for f in ("README.md", "cases.csv", "maps.csv", "index.html", "swoop_compare.gif", "README_frozen.sha256"):
        assert (d / f).exists(), f
    html = (d / "index.html").read_text(encoding="utf-8")
    assert "--bg:#14181d" in html and "swoop_compare.gif" in html and "color-scheme:dark" in html
    ok, got, exp = m.frozen_hash_ok()
    assert ok
    assert (d / "README.md").read_text(encoding="utf-8").count("## Results") == 1
    rd = (d / "README.md").read_text(encoding="utf-8")
    for need in ("Post-hoc additions", "TEST PROPOSAL", "Glide angle is unmeasured", "Dive edge not trusted", "VERIFIED", "Linear-test disclosure", "Next flight day"):
        assert need in rd, need
    rows = (d / "maps.csv").read_text(encoding="utf-8").strip().splitlines()
    assert len(rows) - 1 == 11 * (11 * 8 + 15 * 11), len(rows)
    return f"maps.csv {len(rows) - 1} rows"


@case
def t_writes_only_inside_owned_paths():
    """Run the whole pipeline on a tiny grid with every write hooked: all write targets must be inside the temporary output folder
    (so the code cannot touch other project files).  Then scan the project for files modified since the frozen README as information."""
    import builtins
    import shutil
    import tempfile
    targets = []
    real_open, real_wt, real_wb, real_mk, real_md, real_mds = builtins.open, Path.write_text, Path.write_bytes, Path.mkdir, os.mkdir, os.makedirs

    def rec(p):
        targets.append(os.path.abspath(str(p)))

    def hopen(file, mode="r", *a, **k):
        if any(c in str(mode) for c in "wax+") and not isinstance(file, int):
            rec(file)
        return real_open(file, mode, *a, **k)
    def hwt(self, *a, **k):
        rec(self)
        return real_wt(self, *a, **k)
    def hwb(self, *a, **k):
        rec(self)
        return real_wb(self, *a, **k)
    def hmk(self, *a, **k):
        rec(self)
        return real_mk(self, *a, **k)
    def hmd(path, *a, **k):
        rec(path)
        return real_md(path, *a, **k)
    def hmds(path, *a, **k):
        rec(path)
        return real_mds(path, *a, **k)
    with tempfile.TemporaryDirectory() as td:
        builtins.open, Path.write_text, Path.write_bytes, Path.mkdir, os.mkdir, os.makedirs = hopen, hwt, hwb, hmk, hmd, hmds
        old_argv, old_v = sys.argv, list(m.VNAMES)
        try:
            sys.argv = ["swoop_model_v1.py", "--tiny", "--jobs", "1", "--out", td]
            m.main()
        finally:
            builtins.open, Path.write_text, Path.write_bytes, Path.mkdir, os.mkdir, os.makedirs = real_open, real_wt, real_wb, real_mk, real_md, real_mds
            sys.argv = old_argv
        m.VNAMES = old_v
        tdn = os.path.abspath(td)
        outside = sorted({t for t in targets if not t.startswith(tdn)})
        assert targets and not outside, f"writes outside the output folder: {outside[:5]}"
        files = sorted(os.listdir(td))
        for need in ("README.md", "cases.csv", "maps.csv", "index.html", "swoop_compare.gif"):
            assert need in files, (need, files)
        w, h, ncol, frames = gif_decode(Path(td) / "swoop_compare.gif")
        assert len(frames) >= 100
        txt = (Path(td) / "README.md").read_text(encoding="utf-8")
        assert txt.startswith(m.frozen_text()), "frozen section not reproduced byte for byte in the generated README"
        n_writes = len(targets)
    t0 = (m.OUT_DEFAULT / "README_frozen.sha256").stat().st_mtime - 2.0
    owned = [HERE / "swoop_model_v1.py", HERE / "test_swoop_model_v1.py"]
    foreign = []
    for dp, dn, fn in os.walk(m.ROOT):
        for f in fn:
            pth = Path(dp) / f
            try:
                if pth.stat().st_mtime > t0:
                    pr = pth.resolve()
                    if pr in owned or m.OUT_DEFAULT in pr.parents:
                        continue
                    foreign.append(str(pth.relative_to(m.ROOT)))
            except OSError:
                pass
    note = f"; INFO: {len(foreign)} other project files were modified since the frozen README by something else (not this code): {foreign[:6]}" if foreign else "; no other project file changed"
    return f"{n_writes} write calls, all inside the temp output folder" + note


if __name__ == "__main__":
    for t in (t_frozen_hash, t_lift_curve_data_points, t_energy_and_units, t_step_convergence, t_linear_matches_old_phugoid, t_gif_roundtrip_and_output, t_outputs_present_and_dark, t_writes_only_inside_owned_paths):
        t()
    bad = 0
    for name, ok, msg, dt in RESULTS:
        print(f"{'PASS' if ok else 'FAIL'}  {name}  ({dt:.1f} s)  {msg}")
        bad += (not ok)
    print(f"\n{len(RESULTS) - bad}/{len(RESULTS)} passed")
    sys.exit(1 if bad else 0)
