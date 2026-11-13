// wingpolar_llt_dualmode.cpp
//
// Nonlinear LLT postprocess using megapolar.dat
// Adds:
//  (1) spanwise Reynolds sampling (Re(y) per station)
//  (2) simple stall-cap (clamped to cl_max/cl_min of the 2D polar)
//  (3) dual-mode run: GLIDER mode + ROTOR mode (Re(y) from Omega*r)
//  (4) 3D Pitching Moment Integration (Cm) with Sweep offset
//
// FIX 4.1 (OPTIMIZATION PASS, MINIMAL BEHAVIOR CHANGES):
//  - Big runtime optimizations for scaling to many airfoil combos:
//    (A) PolarDB: replace map<int,PolarCurve> with vector-based Re tables (binary-search + O(1) curve access)
//    (B) Precompute per-station invariants (wTip, ReLoc, alpha-window min/max) once per alpha solve
//        (since ReLoc doesn't depend on A) and reuse each iteration.
//    (C) Precompute LU factorization of M once per solve and use fast forward/back substitution
//        instead of Gauss-Jordan each iteration.
//    (D) Avoid repeated bracket_Re/get_interpolated_range work inside the iteration loop.
//
// BULK RUNNER (minimal additions):
//  - Iterate a fixed airfoil list (root=tip=single foil; no blending effect)
//  - Run at Re_ref=80000 (both modes)
//  - Write ONE summary CSV: one row per foil per mode (glider/rotor) with metrics
//      * best L/D over alpha sweep
//      * max CL over alpha sweep
//      * L/D at CL_target (interpolated by CL)
//      * composite score (normalized + weighted)
//      * 3D CM at target points, and Pitching Moment Slope (dCM/dAlpha) for stability
//  - Light parallelization over airfoils
//  - Dynamic progress bar with ETA
//
// PLUS: Stall speed sweep CSV on top glider foils:
//  - Pick unique foils from top-by-bestLD and top-by-maxCL (glider)
//  - Sweep b and cr candidates
//  - Vs = sqrt(2W/(rho*S*CLmax_used))
//  - Writes one row per (foil,b,cr)
//
// PLUS (for stallPlotter.py):
//  - Write results\runs\llt\glider_metrics.csv and rotor_metrics.csv
//    (one row per foil, per mode)
//
// Build: g++ wingpolar_llt_dualmode.cpp -O2 -std=c++17 -pthread -o wingpolar_llt_dualmode

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iomanip>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#if __has_include(<filesystem>)
  #include <filesystem>
  namespace fs = std::filesystem;
#endif

using namespace std;

// -------------------- data structs --------------------

struct Row {
    string airfoil;
    int Re = 0;
    int Ncrit = 0;
    int Iter = 0;
    double alpha = 0.0;
    double CL = 0.0;
    double CD = 0.0;
    double CDp = 0.0;
    double CM = 0.0;
    bool hasXtr = false;
    double topXtr = 0.0;
    double botXtr = 0.0;
    string note;
};

static bool is_finite(double x) { return std::isfinite(x) != 0; }

static bool parse_int_strict(const string& s, int& out) {
    try {
        size_t idx = 0;
        long long v = stoll(s, &idx);
        if (idx != s.size()) return false;
        if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
        out = (int)v;
        return true;
    } catch (...) { return false; }
}

static bool parse_double_strict(const string& s, double& out) {
    try {
        size_t idx = 0;
        double v = stod(s, &idx);
        if (idx != s.size()) return false;
        out = v;
        return true;
    } catch (...) { return false; }
}

static bool looks_like_int(const string& s) {
    int tmp = 0;
    return parse_int_strict(s, tmp);
}

static string normalize_airfoil_key(string s) {
    for (char& c : s) {
        if (c == ' ') c = '_';
        if (!(isalnum((unsigned char)c) || c=='_' || c=='-' || c=='.')) c = '_';
    }
    return s;
}

static bool parse_megapolar_line(const string& line, Row& r) {
    if (line.empty()) return false;

    istringstream iss(line);
    string airfoilS, ReS, NS, IterS, aS, clS, cdS, cdpS, cmS, txS, bxS, noteS;

    if (!(iss >> airfoilS >> ReS >> NS >> IterS >> aS >> clS >> cdS >> cdpS >> cmS >> txS >> bxS)) return false;
    if (!(iss >> noteS)) noteS = "BASE";

    int Re=0, Ncrit=0, Iter=0;
    double a=0, cl=0, cd=0, cdp=0, cm=0;

    if (!parse_int_strict(ReS, Re)) return false;
    if (!parse_int_strict(NS, Ncrit)) return false;
    if (!parse_int_strict(IterS, Iter)) return false;
    if (!parse_double_strict(aS, a)) return false;
    if (!parse_double_strict(clS, cl)) return false;
    if (!parse_double_strict(cdS, cd)) return false;
    if (!parse_double_strict(cdpS, cdp)) return false;
    if (!parse_double_strict(cmS, cm)) return false;

    r.airfoil = airfoilS;
    r.Re = Re;
    r.Ncrit = Ncrit;
    r.Iter = Iter;
    r.alpha = a;
    r.CL = cl;
    r.CD = cd;
    r.CDp = cdp;
    r.CM = cm;
    r.note = noteS;

    if (txS == "NA" || bxS == "NA") {
        r.hasXtr = false;
        r.topXtr = r.botXtr = 0.0;
    } else {
        double tx=0, bx=0;
        if (parse_double_strict(txS, tx) && parse_double_strict(bxS, bx) && is_finite(tx) && is_finite(bx)) {
            r.hasXtr = true;
            r.topXtr = tx;
            r.botXtr = bx;
        } else {
            r.hasXtr = false;
            r.topXtr = r.botXtr = 0.0;
        }
    }
    return true;
}

static bool passes_sanity_filters(const Row& r) {
    if (!is_finite(r.alpha) || !is_finite(r.CL) || !is_finite(r.CD) || !is_finite(r.CDp) || !is_finite(r.CM)) return false;
    if (r.CD <= 0.0) return false;
    if (r.CD > 5.0)  return false;
    if (fabs(r.CL) > 5.0) return false;
    if (fabs(r.alpha) > 90.0) return false;
    return true;
}

// -------------------- small math helpers --------------------

static double clamp01(double x) { return (x < 0.0) ? 0.0 : (x > 1.0) ? 1.0 : x; }
static double clampd(double x, double lo, double hi) { return (x < lo) ? lo : (x > hi) ? hi : x; }

static double smoothstep01(double t) {
    t = clamp01(t);
    return t*t*t*(t*(t*6.0 - 15.0) + 10.0);
}

static double smooth_clamp(double x, double lo, double hi, double width) {
    if (width <= 0.0) return clampd(x, lo, hi);

    double maxW = 0.5 * (hi - lo);
    if (width > maxW) width = maxW;

    if (x <= lo) return lo;
    if (x >= hi) return hi;

    if (x < lo + width) {
        double t = (x - lo) / width;
        return lo + width * (2.0*t*t - t*t*t);
    }

    if (x > hi - width) {
        double t = (x - (hi - width)) / width;
        double delta = width * (-t*t*t + t*t + t);
        return (hi - width) + delta;
    }

    return x;
}

// -------------------- polar curve --------------------

struct PolarCurve {
    vector<double> a, cl, cd, cm;

    bool hasExtrema = false;
    double clMax = 0.0, aAtClMax = 0.0;
    double clMin = 0.0, aAtClMin = 0.0;

    bool empty() const { return a.empty(); }

    void sort_by_alpha() {
        vector<size_t> idx(a.size());
        for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
        sort(idx.begin(), idx.end(), [&](size_t i, size_t j){ return a[i] < a[j]; });

        auto reorder = [&](vector<double>& v) {
            vector<double> out; out.reserve(v.size());
            for (size_t k : idx) out.push_back(v[k]);
            v.swap(out);
        };
        reorder(a); reorder(cl); reorder(cd); reorder(cm);
    }

    void compute_extrema() {
        if (a.empty()) { hasExtrema = false; return; }
        hasExtrema = true;
        clMax = cl[0]; aAtClMax = a[0];
        clMin = cl[0]; aAtClMin = a[0];
        for (size_t i = 1; i < cl.size(); ++i) {
            if (cl[i] > clMax) { clMax = cl[i]; aAtClMax = a[i]; }
            if (cl[i] < clMin) { clMin = cl[i]; aAtClMin = a[i]; }
        }
    }

    bool sample_linear(double alpha, double& outCL, double& outCD, double& outCM) const {
        if (a.empty()) return false;
        if (alpha < a.front() - 1e-12 || alpha > a.back() + 1e-12) return false;

        auto it = lower_bound(a.begin(), a.end(), alpha);
        if (it != a.end() && fabs(*it - alpha) < 1e-12) {
            size_t i = (size_t)(it - a.begin());
            outCL = cl[i]; outCD = cd[i]; outCM = cm[i];
            return true;
        }
        if (it == a.begin() || it == a.end()) return false;

        size_t hi = (size_t)(it - a.begin());
        size_t lo = hi - 1;

        double a0 = a[lo], a1 = a[hi];
        double t = (alpha - a0) / (a1 - a0);

        outCL = cl[lo] + t * (cl[hi] - cl[lo]);
        outCD = cd[lo] + t * (cd[hi] - cd[lo]);
        outCM = cm[lo] + t * (cm[hi] - cm[lo]);
        return true;
    }

    bool sample_with_stallcap(double alpha, double& outCL, double& outCD, double& outCM) const {
        if (!sample_linear(alpha, outCL, outCD, outCM)) return false;
        if (!hasExtrema) return true;
        if (alpha > aAtClMax && outCL > clMax) outCL = clMax;
        if (alpha < aAtClMin && outCL < clMin) outCL = clMin;
        return true;
    }

    double alpha_min() const { return a.empty() ? 0.0 : a.front(); }
    double alpha_max() const { return a.empty() ? 0.0 : a.back();  }

    bool sample_clamped(double alphaDeg, double& outCL, double& outCD, double& outCM, bool stallCap) const {
        if (a.empty()) return false;
        const double CLAMP_W_DEG = 0.0; // keep behavior identical
        double aMin = alpha_min(), aMax = alpha_max();
        double aUse = smooth_clamp(alphaDeg, aMin, aMax, CLAMP_W_DEG);
        if (stallCap) return sample_with_stallcap(aUse, outCL, outCD, outCM);
        else          return sample_linear(aUse, outCL, outCD, outCM);
    }
};

// -------------------- Polar DB --------------------

struct FoilPolars {
    vector<int> ReVals;
    vector<PolarCurve> curves;
    unordered_map<int, PolarCurve> buildMap;
};

struct PolarDB {
    unordered_map<string, FoilPolars> foils;

    void add_point(const string& foil, int Re, double alpha, double cl, double cd, double cm) {
        FoilPolars& fp = foils[foil];
        PolarCurve& pc = fp.buildMap[Re];
        pc.a.push_back(alpha);
        pc.cl.push_back(cl);
        pc.cd.push_back(cd);
        pc.cm.push_back(cm);
    }

    // HARDENED: build ReVals+curves in lockstep, no unordered_map operator[] during finalize
    void finalize_all() {
        for (auto& fk : foils) {
            FoilPolars& fp = fk.second;

            vector<int> res;
            res.reserve(fp.buildMap.size());
            for (auto& kv : fp.buildMap) {
                kv.second.sort_by_alpha();
                kv.second.compute_extrema();
                res.push_back(kv.first);
            }
            sort(res.begin(), res.end());

            fp.ReVals.clear();
            fp.curves.clear();
            fp.ReVals.reserve(res.size());
            fp.curves.reserve(res.size());

            for (int Re : res) {
                auto it = fp.buildMap.find(Re);
                if (it == fp.buildMap.end()) continue; // shouldn't happen, but stay safe
                fp.ReVals.push_back(Re);
                fp.curves.push_back(std::move(it->second));
            }

            fp.buildMap.clear();
            fp.buildMap.rehash(0);
        }
    }

    bool has_foil(const string& foil) const {
        return foils.find(foil) != foils.end();
    }

    static double interp_weight_logRe(double desiredRe, int ReLo, int ReHi) {
        if (ReLo == ReHi) return 0.0;
        double r  = max(1.0, desiredRe);
        double r0 = (double)max(1, ReLo);
        double r1 = (double)max(1, ReHi);
        double t = (log(r) - log(r0)) / (log(r1) - log(r0));
        return clamp01(t);
    }

    // HARDENED: use safeSz=min(ReVals,curves)
    bool bracket_Re_for_foil_idx(const string& foil, double desiredRe,
                                 int& idxLo, int& idxHi, int& ReLo, int& ReHi) const {
        auto itF = foils.find(foil);
        if (itF == foils.end()) return false;
        const FoilPolars& fp = itF->second;

        const int safeSz = (int)min(fp.ReVals.size(), fp.curves.size());
        if (safeSz <= 0) return false;

        const int dRe = (int)llround(desiredRe);

        auto beginIt = fp.ReVals.begin();
        auto endIt   = beginIt + safeSz; // only search the synced prefix
        auto it = lower_bound(beginIt, endIt, dRe);

        if (it == beginIt) {
            idxLo = idxHi = 0;
            ReLo = ReHi = fp.ReVals[0];
            return true;
        }
        if (it == endIt) {
            idxLo = idxHi = safeSz - 1;
            ReLo = ReHi = fp.ReVals[(size_t)(safeSz - 1)];
            return true;
        }

        idxHi = (int)(it - beginIt);
        idxLo = idxHi - 1;
        ReHi = fp.ReVals[(size_t)idxHi];
        ReLo = fp.ReVals[(size_t)idxLo];
        return true;
    }

    bool get_interpolated_range(const string& foil, double desiredRe, double& outMin, double& outMax) const {
        int idxLo=-1, idxHi=-1, ReLo=-1, ReHi=-1;
        if (!bracket_Re_for_foil_idx(foil, desiredRe, idxLo, idxHi, ReLo, ReHi)) return false;

        auto itF = foils.find(foil);
        if (itF == foils.end()) return false;
        const FoilPolars& fp = itF->second;

        const int safeSz = (int)min(fp.ReVals.size(), fp.curves.size());
        if (safeSz <= 0) return false;
        if (idxLo < 0 || idxHi < 0) return false;
        if (idxLo >= safeSz || idxHi >= safeSz) return false;

        const PolarCurve& pcLo = fp.curves[(size_t)idxLo];
        const PolarCurve& pcHi = fp.curves[(size_t)idxHi];

        double minLo = pcLo.alpha_min(); double maxLo = pcLo.alpha_max();
        double minHi = pcHi.alpha_min(); double maxHi = pcHi.alpha_max();
        double t = interp_weight_logRe(desiredRe, ReLo, ReHi);

        outMin = (1.0 - t) * minLo + t * minHi;
        outMax = (1.0 - t) * maxLo + t * maxHi;
        return true;
    }

    bool sample_bracketed_idx(const string& foil,
                              int idxLo, int idxHi, int ReLo, int ReHi, double t,
                              double alphaDeg, double& outCL, double& outCD, double& outCM, bool stallCap) const {
        auto itF = foils.find(foil);
        if (itF == foils.end()) return false;
        const FoilPolars& fp = itF->second;

        const int safeSz = (int)min(fp.ReVals.size(), fp.curves.size());
        if (safeSz <= 0) return false;

        if (idxLo < 0 || idxHi < 0) return false;
        if (idxLo >= safeSz || idxHi >= safeSz) return false;

        if (ReLo == ReHi || idxLo == idxHi) {
            return fp.curves[(size_t)idxLo].sample_clamped(alphaDeg, outCL, outCD, outCM, stallCap);
        }

        double cl0=0.0, cd0=0.0, cm0=0.0, cl1=0.0, cd1=0.0, cm1=0.0;
        if (!fp.curves[(size_t)idxLo].sample_clamped(alphaDeg, cl0, cd0, cm0, stallCap)) return false;
        if (!fp.curves[(size_t)idxHi].sample_clamped(alphaDeg, cl1, cd1, cm1, stallCap)) return false;

        outCL = (1.0 - t) * cl0 + t * cl1;
        outCD = (1.0 - t) * cd0 + t * cd1;
        outCM = (1.0 - t) * cm0 + t * cm1;
        return true;
    }
};

// -------------------- geometry helpers --------------------

static double chord_linear_taper(double y_abs, double b, double cr, double ct) {
    double yFrac = (2.0 * y_abs) / b;
    return cr + (ct - cr) * yFrac;
}

static void choose_planform_from_AR_taper(double AR, double taper, double b, double& cr, double& ct, double& S) {
    S = (b * b) / AR;
    cr = (2.0 * S) / (b * (1.0 + taper));
    ct = taper * cr;
}

static double tip_weight(double yFrac01, double blendStart, double blendEnd) {
    if (fabs(blendEnd - blendStart) < 1e-12) return (yFrac01 >= blendStart) ? 1.0 : 0.0;
    double t = (yFrac01 - blendStart) / (blendEnd - blendStart);
    return smoothstep01(t);
}

// -------------------- LLT solve --------------------

struct LLTResult {
    double alpha_deg = 0.0;
    double CL = 0.0;
    double CDi = 0.0;
    double CDp = 0.0;
    double CD = 0.0;
    double CM = 0.0;
    double LD = 0.0;
    int iters = 0;
    bool converged = false;
    string note;
};

struct StationDumpRow {
    int i = 0;
    double theta = 0.0;
    double yFrac = 0.0;
    double yAbs = 0.0;
    double chord = 0.0;
    double wTip = 0.0;
    double Re_local = 0.0;

    double alpha_i_deg = 0.0;
    double alpha_eff_raw_deg = 0.0;
    double alpha_eff_deg = 0.0;

    double minWindow = 0.0;
    double maxWindow = 0.0;
    double clamp_delta = 0.0;
    double identity_error = 0.0;

    int ReLoRoot = -1, ReHiRoot = -1;
    int ReLoTip  = -1, ReHiTip  = -1;

    double cl = 0.0;
    double cd = 0.0;
    double cm = 0.0;
};

enum class Mode { GLIDER, ROTOR };

struct ModeParams {
    Mode mode = Mode::GLIDER;
    double Re_ref = 80000.0;
    double omega_rad_s = 0.0;
    double hubFrac = 0.10;
    double rRefFrac = 0.75;
    bool stallCap = true;
};

// debugFlags bitmask:
// 1: disable window clamp
// 2: freeze rotor Re across span (use glider-like Re scaling)
// 4: disable stallCap

// ---------- LU factorization helpers ----------
static bool lu_decompose_inplace(vector<double>& A, int N, vector<int>& piv) {
    piv.resize(N);
    for (int i = 0; i < N; ++i) piv[i] = i;

    for (int k = 0; k < N; ++k) {
        int pivRow = k;
        double best = fabs(A[k*N + k]);
        for (int r = k + 1; r < N; ++r) {
            double v = fabs(A[r*N + k]);
            if (v > best) { best = v; pivRow = r; }
        }
        if (best < 1e-14) return false;

        if (pivRow != k) {
            for (int c = 0; c < N; ++c) swap(A[k*N + c], A[pivRow*N + c]);
            swap(piv[k], piv[pivRow]);
        }

        double Akk = A[k*N + k];
        for (int i = k + 1; i < N; ++i) {
            double f = A[i*N + k] / Akk;
            A[i*N + k] = f;
            if (fabs(f) < 1e-20) continue;
            for (int j = k + 1; j < N; ++j) {
                A[i*N + j] -= f * A[k*N + j];
            }
        }
    }
    return true;
}

static void lu_solve(const vector<double>& LU, int N, const vector<int>& piv,
                     const vector<double>& b, vector<double>& x) {
    x.assign(N, 0.0);
    vector<double> y(N, 0.0);
    for (int i = 0; i < N; ++i) y[i] = b[piv[i]];

    vector<double> z(N, 0.0);
    for (int i = 0; i < N; ++i) {
        double sum = y[i];
        for (int j = 0; j < i; ++j) sum -= LU[i*N + j] * z[j];
        z[i] = sum;
    }

    for (int i = N - 1; i >= 0; --i) {
        double sum = z[i];
        for (int j = i + 1; j < N; ++j) sum -= LU[i*N + j] * x[j];
        double diag = LU[i*N + i];
        x[i] = (fabs(diag) > 1e-18) ? (sum / diag) : 0.0;
    }
}

// ---------- Precompute theta/sin tables + LU(M) once ----------

struct LLTPrecomp {
    int N = 0;
    double b = 0.0;
    double R = 0.0;
    double AR = 0.0;
    double taper = 0.0;
    double sweep_deg = 0.0;
    double tan_sweep = 0.0;
    double cr = 0.0, ct = 0.0, S = 0.0;
    double cRef = 0.0;
    double dth = 0.0;

    vector<double> theta, sinT, yAbs, yFrac, c;
    vector<double> nSin;          // row-major NxN: n*sin(n*theta_i)
    vector<double> LU;            // LU(M)
    vector<int> piv;              // pivots for LU(M)
};

static bool build_precomp(LLTPrecomp& pc, double b_m, double AR, double taper, double sweep_deg, int Npoints) {
    const double PI = 3.14159265358979323846;
    const double TIP_EPS_FRACTION = 0.5 / (double)(Npoints + 1);

    if (!(b_m > 0.0) || !(AR > 0.0) || !(taper > 0.0) || Npoints < 3) return false;

    pc.N = Npoints;
    pc.b = b_m;
    pc.R = 0.5 * b_m;
    pc.AR = AR;
    pc.taper = taper;
    pc.sweep_deg = sweep_deg;
    pc.tan_sweep = tan(sweep_deg * PI / 180.0);

    choose_planform_from_AR_taper(AR, taper, b_m, pc.cr, pc.ct, pc.S);
    pc.cRef = pc.cr;

    pc.theta.assign(Npoints, 0.0);
    pc.sinT.assign(Npoints, 0.0);
    pc.yAbs.assign(Npoints, 0.0);
    pc.yFrac.assign(Npoints, 0.0);
    pc.c.assign(Npoints, 0.0);

    for (int i = 0; i < Npoints; ++i) {
        double th = ((i + 1.0) * PI) / (double)(Npoints + 1.0);
        th = (TIP_EPS_FRACTION * PI) + (1.0 - 2.0 * TIP_EPS_FRACTION) * th;
        pc.theta[i] = th;
        pc.sinT[i] = sin(th);
        double y = (pc.b * 0.5) * cos(th);
        pc.yAbs[i] = fabs(y);
        pc.yFrac[i] = (2.0 * pc.yAbs[i]) / pc.b;
        pc.c[i] = chord_linear_taper(pc.yAbs[i], pc.b, pc.cr, pc.ct);
    }

    vector<double> M(Npoints * Npoints, 0.0);
    pc.nSin.assign(Npoints * Npoints, 0.0);

    for (int i = 0; i < Npoints; ++i) {
        for (int j = 0; j < Npoints; ++j) {
            int n = j + 1;
            double s = sin((double)n * pc.theta[i]);
            M[i*Npoints + j] = s;
            pc.nSin[i*Npoints + j] = (double)n * s;
        }
    }

    pc.LU = M;
    if (!lu_decompose_inplace(pc.LU, Npoints, pc.piv)) return false;

    pc.dth = PI / (double)Npoints;
    return true;
}

// Precomputed per-station invariants for a single alpha solve

struct StationPre {
    double wTip = 0.0;
    double ReLoc = 0.0;
    double minWindow = -90.0;
    double maxWindow =  90.0;

    int idxLoR=-1, idxHiR=-1, ReLoR=-1, ReHiR=-1;
    double tR=0.0;

    int idxLoT=-1, idxHiT=-1, ReLoT=-1, ReHiT=-1;
    double tT=0.0;
};

static LLTResult solve_llt_nonlinear_one_alpha_twofoil_mode(
    const PolarDB& db,
    const LLTPrecomp& pc,
    const string& rootFoil,
    const string& tipFoil,
    const ModeParams& mp_in,
    double alpha_deg,
    double blendStart,
    double blendEnd,
    int maxIter,
    double relax,
    double tolA,
    int debugFlags,
    const vector<double>* Ainit = nullptr,
    vector<double>* AfinalOut = nullptr,
    vector<StationDumpRow>* dumpOut = nullptr
) {
    const double PI = 3.14159265358979323846;

    ModeParams mp = mp_in;
    if (debugFlags & 4) mp.stallCap = false;

    const bool DISABLE_WINDOW_CLAMP = (debugFlags & 1) != 0;
    const bool FREEZE_ROTOR_RE      = (debugFlags & 2) != 0;

    LLTResult res;
    res.alpha_deg = alpha_deg;

    if (!db.has_foil(rootFoil) || !db.has_foil(tipFoil)) { res.note = "NO_FOIL_IN_DB"; return res; }
    if (!(blendStart >= 0.0 && blendStart <= 1.0 && blendEnd >= 0.0 && blendEnd <= 1.0)) { res.note = "BAD_BLEND"; return res; }
    if (pc.N < 3) { res.note = "BAD_PRECOMP"; return res; }

    const int N = pc.N;

    auto Re_local_at_station = [&](int i) -> double {
        double chordRatio = (pc.cRef > 0.0) ? (pc.c[i] / pc.cRef) : 1.0;

        if (mp.mode == Mode::GLIDER) {
            return mp.Re_ref * chordRatio;
        }

        if (FREEZE_ROTOR_RE) {
            return mp.Re_ref * chordRatio;
        } else {
            double rEff = max(pc.yAbs[i], mp.hubFrac * pc.R);
            double rRef = max(1e-6, mp.rRefFrac * pc.R);
            return mp.Re_ref * (rEff / rRef) * chordRatio;
        }
    };

    vector<StationPre> pre(N);
    for (int i = 0; i < N; ++i) {
        pre[i].wTip = tip_weight(pc.yFrac[i], blendStart, blendEnd);
        pre[i].ReLoc = Re_local_at_station(i);

        if (!db.bracket_Re_for_foil_idx(rootFoil, pre[i].ReLoc, pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR)) {
            res.note = "ROOT_BRACKET_FAIL"; return res;
        }
        pre[i].tR = PolarDB::interp_weight_logRe(pre[i].ReLoc, pre[i].ReLoR, pre[i].ReHiR);

        if (!db.bracket_Re_for_foil_idx(tipFoil, pre[i].ReLoc, pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT)) {
            res.note = "TIP_BRACKET_FAIL"; return res;
        }
        pre[i].tT = PolarDB::interp_weight_logRe(pre[i].ReLoc, pre[i].ReLoT, pre[i].ReHiT);

        double minR=0, maxRngR=0, minT=0, maxRngT=0;
        bool rangeOkR = db.get_interpolated_range(rootFoil, pre[i].ReLoc, minR, maxRngR);
        bool rangeOkT = db.get_interpolated_range(tipFoil,  pre[i].ReLoc, minT, maxRngT);
        if (!rangeOkR) { minR = -90; maxRngR = 90; }
        if (!rangeOkT) { minT = -90; maxRngT = 90; }

        pre[i].minWindow = (1.0 - pre[i].wTip) * minR + pre[i].wTip * minT;
        pre[i].maxWindow = (1.0 - pre[i].wTip) * maxRngR + pre[i].wTip * maxRngT;
    }

    vector<double> A(N, 0.0), Anew(N, 0.0), RHS(N, 0.0);
    if (Ainit && (int)Ainit->size() == N) A = *Ainit;

    auto alpha_induced_rad = [&](int i, const vector<double>& Acoef) -> double {
        double denom = pc.sinT[i];
        double sum = 0.0;
        const double* row = &pc.nSin[i*N];
        for (int j = 0; j < N; ++j) sum += row[j] * Acoef[j];
        return sum / denom;
    };

    const double alpha_geom_rad = alpha_deg * PI / 180.0;
    const double alpha_geom_deg = alpha_deg;

    double relaxLocal = relax;
    double prevMaxDelta = 1e300;
    int stallCount = 0;

    vector<double> Asolve;

    for (int iter = 1; iter <= maxIter; ++iter) {
        for (int i = 0; i < N; ++i) {
            double ai = alpha_induced_rad(i, A);
            double alpha_eff_raw_deg = (alpha_geom_rad - ai) * 180.0 / PI;

            double alpha_eff_deg = alpha_eff_raw_deg;
            if (!DISABLE_WINDOW_CLAMP) {
                alpha_eff_deg = smooth_clamp(alpha_eff_deg, pre[i].minWindow, pre[i].maxWindow, 0.75);
            }

            double clR=0.0, cdR=0.0, cmR=0.0, clT=0.0, cdT=0.0, cmT=0.0;
            bool okR = db.sample_bracketed_idx(rootFoil,
                                              pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR, pre[i].tR,
                                              alpha_eff_deg, clR, cdR, cmR, mp.stallCap);
            bool okT = db.sample_bracketed_idx(tipFoil,
                                              pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT, pre[i].tT,
                                              alpha_eff_deg, clT, cdT, cmT, mp.stallCap);
            if (!okR) { res.note = "ROOT_SAMPLE_FAIL"; res.iters = iter; return res; }
            if (!okT) { res.note = "TIP_SAMPLE_FAIL";  res.iters = iter; return res; }

            double cl = (1.0 - pre[i].wTip) * clR + pre[i].wTip * clT;
            RHS[i] = (cl * pc.c[i]) / (4.0 * pc.b);
        }

        lu_solve(pc.LU, N, pc.piv, RHS, Asolve);

        double maxDelta = 0.0;
        for (int k = 0; k < N; ++k) {
            Anew[k] = (1.0 - relaxLocal) * A[k] + relaxLocal * Asolve[k];
            maxDelta = max(maxDelta, fabs(Anew[k] - A[k]));
        }
        A.swap(Anew);

        res.iters = iter;
        if (maxDelta < tolA) { res.converged = true; res.note = "OK"; break; }

        if (maxDelta > prevMaxDelta * 1.05) {
            relaxLocal *= 0.5;
            if (relaxLocal < 0.03) relaxLocal = 0.03;
            stallCount = 0;
        } else {
            if (fabs(prevMaxDelta - maxDelta) < 0.15 * tolA) stallCount++;
            else stallCount = 0;
        }
        if (stallCount >= 25 && maxDelta < 8.0 * tolA) {
            res.converged = true; res.note = "OK_STALL"; break;
        }
        prevMaxDelta = maxDelta;
    }

    if (!res.converged) res.note = "NO_CONVERGENCE";

    res.CL = PI * pc.AR * A[0];
    double sumInd = 0.0;
    for (int n = 1; n <= N; ++n) sumInd += (double)n * A[n-1] * A[n-1];
    res.CDi = PI * pc.AR * sumInd;

    double S_num = 0.0;
    double Dp_num = 0.0;
    double CM_num = 0.0;

    for (int i = 0; i < N; ++i) {
        double ai = alpha_induced_rad(i, A);
        double alpha_eff_raw_deg = (alpha_geom_rad - ai) * 180.0 / PI;

        double alpha_eff_deg = alpha_eff_raw_deg;
        if (!DISABLE_WINDOW_CLAMP) {
            alpha_eff_deg = smooth_clamp(alpha_eff_deg, pre[i].minWindow, pre[i].maxWindow, 0.75);
        }

        double clR=0.0, cdR=0.0, cmR=0.0, clT=0.0, cdT=0.0, cmT=0.0;
        if (!db.sample_bracketed_idx(rootFoil,
                                     pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR, pre[i].tR,
                                     alpha_eff_deg, clR, cdR, cmR, mp.stallCap)) { res.note="ROOT_SAMPLE_FAIL_2"; res.converged=false; return res; }
        if (!db.sample_bracketed_idx(tipFoil,
                                     pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT, pre[i].tT,
                                     alpha_eff_deg, clT, cdT, cmT, mp.stallCap)) { res.note="TIP_SAMPLE_FAIL_2";  res.converged=false; return res; }

        double cl = (1.0 - pre[i].wTip) * clR + pre[i].wTip * clT;
        double cd = (1.0 - pre[i].wTip) * cdR + pre[i].wTip * cdT;
        double cm = (1.0 - pre[i].wTip) * cmR + pre[i].wTip * cmT;
        
        double w = pc.c[i] * pc.sinT[i] * pc.dth;
        S_num += w;
        Dp_num += cd * w;
        
        // Pitching moment integration including sweep lever arm offset
        double x_lever = pc.yAbs[i] * pc.tan_sweep; 
        double dCm = (cm * pc.c[i]) - (cl * x_lever);
        CM_num += dCm * w;
    }

    const double S_calc = (pc.b * 0.5) * S_num;
    if (!(S_calc > 0.0)) { res.note = "BAD_AREA_INTEGRAL"; res.converged = false; return res; }

    res.CDp = ((pc.b * 0.5) / S_calc) * Dp_num;
    res.CD  = res.CDp + res.CDi;
    res.CM  = CM_num / (S_num * pc.cRef);
    res.LD  = (res.CD > 0.0) ? (res.CL / res.CD) : 0.0;

    if (AfinalOut) *AfinalOut = A;

    if (dumpOut) {
        dumpOut->clear(); dumpOut->reserve((size_t)N);
        for (int i = 0; i < N; ++i) {
            double ai_rad = alpha_induced_rad(i, A);
            double alpha_i_deg = ai_rad * 180.0 / PI;
            double alpha_eff_raw_deg = (alpha_geom_rad - ai_rad) * 180.0 / PI;

            double alpha_eff_deg = alpha_eff_raw_deg;
            if (!DISABLE_WINDOW_CLAMP) {
                alpha_eff_deg = smooth_clamp(alpha_eff_deg, pre[i].minWindow, pre[i].maxWindow, 0.75);
            }

            double clR=0.0, cdR=0.0, cmR=0.0, clT=0.0, cdT=0.0, cmT=0.0;
            bool okR = db.sample_bracketed_idx(rootFoil,
                                              pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR, pre[i].tR,
                                              alpha_eff_deg, clR, cdR, cmR, mp.stallCap);
            bool okT = db.sample_bracketed_idx(tipFoil,
                                              pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT, pre[i].tT,
                                              alpha_eff_deg, clT, cdT, cmT, mp.stallCap);

            StationDumpRow d;
            d.i = i;
            d.theta = pc.theta[i];
            d.yFrac = pc.yFrac[i];
            d.yAbs  = pc.yAbs[i];
            d.chord = pc.c[i];
            d.wTip  = pre[i].wTip;
            d.Re_local = pre[i].ReLoc;

            d.alpha_i_deg = alpha_i_deg;
            d.alpha_eff_raw_deg = alpha_eff_raw_deg;
            d.alpha_eff_deg = alpha_eff_deg;

            d.minWindow = pre[i].minWindow;
            d.maxWindow = pre[i].maxWindow;
            d.clamp_delta = (alpha_eff_deg - alpha_eff_raw_deg);
            d.identity_error = (alpha_eff_deg + alpha_i_deg) - alpha_geom_deg;

            d.ReLoRoot = pre[i].ReLoR; d.ReHiRoot = pre[i].ReHiR;
            d.ReLoTip  = pre[i].ReLoT; d.ReHiTip  = pre[i].ReHiT;

            if (okR && okT) {
                d.cl = (1.0 - pre[i].wTip) * clR + pre[i].wTip * clT;
                d.cd = (1.0 - pre[i].wTip) * cdR + pre[i].wTip * cdT;
                d.cm = (1.0 - pre[i].wTip) * cmR + pre[i].wTip * cmT;
            } else {
                d.cl = 0.0; d.cd = 0.0; d.cm = 0.0;
            }
            dumpOut->push_back(d);
        }
    }

    return res;
}

// -------------------- BULK runner helpers --------------------

static vector<string> build_airfoil_list() {
    return {
        "NACA 0004","NACA 0006","NACA 0008","NACA 0010","NACA 0012","NACA 0015","NACA 0018","NACA 0021","NACA 0024",
        "NACA 1208","NACA 1209","NACA 1212","NACA 1408","NACA 1409","NACA 1410","NACA 1412","NACA 1415","NACA 1512","NACA 1608","NACA 1609","NACA 1612",
        "NACA 2208","NACA 2209","NACA 2212","NACA 2408","NACA 2409","NACA 2410","NACA 2412","NACA 2415","NACA 2512","NACA 2608","NACA 2609","NACA 2612",
        "NACA 3208","NACA 3209","NACA 3212","NACA 3408","NACA 3409","NACA 3410","NACA 3412","NACA 3415","NACA 3512","NACA 3608","NACA 3609","NACA 3612",
        "NACA 4208","NACA 4209","NACA 4212","NACA 4408","NACA 4409","NACA 4410","NACA 4412","NACA 4415","NACA 4512","NACA 4608","NACA 4609","NACA 4612",
        "NACA 5208","NACA 5209","NACA 5212","NACA 5408","NACA 5409","NACA 5410","NACA 5412","NACA 5415","NACA 5512","NACA 5608","NACA 5609","NACA 5612",
        "NACA 6208","NACA 6209","NACA 6212","NACA 6408","NACA 6409","NACA 6410","NACA 6412","NACA 6415","NACA 6512","NACA 6608","NACA 6609","NACA 6612",
        "NACA 7208","NACA 7209","NACA 7212","NACA 7408","NACA 7409","NACA 7410","NACA 7412","NACA 7415","NACA 7512","NACA 7608","NACA 7609","NACA 7612",
        "NACA 8208","NACA 8209","NACA 8212","NACA 8408","NACA 8409","NACA 8410","NACA 8412","NACA 8415","NACA 8512","NACA 8608","NACA 8609","NACA 8612",
        "NACA 9208","NACA 9209","NACA 9212","NACA 9408","NACA 9409","NACA 9410","NACA 9412","NACA 9415","NACA 9512","NACA 9608","NACA 9609","NACA 9612"
    };
}

struct BulkMetric {
    string foil;

    double bestLD = -1e300;
    double alphaBestLD = 0.0;
    double CL_at_bestLD = 0.0;
    double CD_at_bestLD = 0.0;
    double CM_at_bestLD = 0.0;

    double maxCL = -1e300;
    double alphaMaxCL = 0.0;
    double CD_at_maxCL = 0.0;
    double CM_at_maxCL = 0.0;

    double LD_at_CL_target = -1e300;
    double CL_at_CL_target = 0.0;
    double alpha_at_CL_target = 0.0;
    double CM_at_CL_target = 0.0;
    double dCM_dAlpha = 0.0;

    double compositeScore = -1e300;

    int nConverged = 0;
    int nTotal = 0;

    string note;
};

static BulkMetric summarize_wing(const string& foilKey, const vector<LLTResult>& wing, double CL_target) {
    BulkMetric m;
    m.foil = foilKey;
    m.nTotal = (int)wing.size();

    bool haveBestLD = false;
    bool haveMaxCL = false;

    struct CLPoint { double CL; double LD; double a; double CM; };
    vector<CLPoint> pts;
    pts.reserve(wing.size());

    for (const auto& r : wing) {
        if (!r.converged) continue;
        m.nConverged++;

        if (!haveBestLD || r.LD > m.bestLD) {
            haveBestLD = true;
            m.bestLD = r.LD;
            m.alphaBestLD = r.alpha_deg;
            m.CL_at_bestLD = r.CL;
            m.CD_at_bestLD = r.CD;
            m.CM_at_bestLD = r.CM;
            m.note = r.note;
        }

        if (!haveMaxCL || r.CL > m.maxCL) {
            haveMaxCL = true;
            m.maxCL = r.CL;
            m.alphaMaxCL = r.alpha_deg;
            m.CD_at_maxCL = r.CD;
            m.CM_at_maxCL = r.CM;
        }

        pts.push_back({r.CL, r.LD, r.alpha_deg, r.CM});
    }

    if (!pts.empty()) {
        sort(pts.begin(), pts.end(), [](const CLPoint& a, const CLPoint& b){ return a.CL < b.CL; });

        vector<CLPoint> uniq;
        uniq.reserve(pts.size());
        const double EPS_CL = 1e-9;
        for (const auto& p : pts) {
            if (uniq.empty() || fabs(p.CL - uniq.back().CL) > EPS_CL) {
                uniq.push_back(p);
            } else {
                if (p.LD > uniq.back().LD) uniq.back() = p;
            }
        }

        if (!uniq.empty()) {
            if (CL_target <= uniq.front().CL) {
                m.LD_at_CL_target = uniq.front().LD;
                m.CL_at_CL_target = uniq.front().CL;
                m.alpha_at_CL_target = uniq.front().a;
                m.CM_at_CL_target = uniq.front().CM;
            } else if (CL_target >= uniq.back().CL) {
                m.LD_at_CL_target = uniq.back().LD;
                m.CL_at_CL_target = uniq.back().CL;
                m.alpha_at_CL_target = uniq.back().a;
                m.CM_at_CL_target = uniq.back().CM;
            } else {
                for (size_t i = 1; i < uniq.size(); ++i) {
                    double CLlo = uniq[i-1].CL, LDlo = uniq[i-1].LD, alo = uniq[i-1].a, CMlo = uniq[i-1].CM;
                    double CLhi = uniq[i].CL,   LDhi = uniq[i].LD,   ahi = uniq[i].a, CMhi = uniq[i].CM;
                    if (CL_target >= CLlo && CL_target <= CLhi && fabs(CLhi - CLlo) > 1e-12) {
                        double t = (CL_target - CLlo) / (CLhi - CLlo);
                        m.LD_at_CL_target = (1.0 - t) * LDlo + t * LDhi;
                        m.CL_at_CL_target = CL_target;
                        m.alpha_at_CL_target = (1.0 - t) * alo + t * ahi;
                        m.CM_at_CL_target = (1.0 - t) * CMlo + t * CMhi;
                        
                        double da = ahi - alo;
                        if (fabs(da) > 1e-6) m.dCM_dAlpha = (CMhi - CMlo) / da; 
                        
                        break;
                    }
                }
            }
        }
    }

    if (!haveBestLD) {
        m.bestLD = -1e300;
        m.maxCL = -1e300;
        m.LD_at_CL_target = -1e300;
        m.note = "NO_CONVERGED_POINTS";
    }
    return m;
}

// -------------------- progress bar helpers --------------------

static string format_hhmmss(double seconds) {
    if (seconds < 0) seconds = 0;
    long long s = (long long)llround(seconds);
    long long hh = s / 3600; s %= 3600;
    long long mm = s / 60;   s %= 60;
    long long ss = s;
    ostringstream oss;
    oss << setfill('0') << setw(2) << hh << ":" << setw(2) << mm << ":" << setw(2) << ss;
    return oss.str();
}

static void print_progress_bar(double frac, long long done, long long total,
                               double rate_per_s, double elapsed_s, double eta_s)
{
    const int W = 28;
    int filled = (int)llround(frac * W);
    if (filled < 0) filled = 0;
    if (filled > W) filled = W;

    ostringstream bar;
    bar << "[";
    for (int i = 0; i < W; ++i) bar << (i < filled ? "#" : "-");
    bar << "]";

    cout << "\r"
         << bar.str() << " "
         << fixed << setprecision(1) << (100.0 * frac) << "% "
         << " | " << done << "/" << total
         << " | " << fixed << setprecision(2) << rate_per_s << " cases/s"
         << " | ETA " << format_hhmmss(eta_s)
         << " | Elap " << format_hhmmss(elapsed_s)
         << flush;
}

// -------------------- CSV helpers --------------------

static string csv_escape(const string& s) {
    bool needQuotes = false;
    for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') { needQuotes = true; break; }
    }
    if (!needQuotes) return s;
    string out; out.reserve(s.size() + 2);
    out.push_back('"');
    for (char c : s) {
        if (c == '"') out.push_back('"');
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

// -------------------- bulk summary CSV writer --------------------

static bool write_bulk_summary_csv(const string& outPath,
                                  int Re_ref,
                                  double b_m, double AR, double taper, double sweep_deg, int Npoints,
                                  double aMin, double aMax, double aStep,
                                  double CL_target,
                                  double wLD, double wCL, double wLDCL,
                                  const vector<BulkMetric>& glider,
                                  const vector<BulkMetric>& rotor)
{
    ofstream out(outPath, ios::trunc);
    if (!out) return false;

    out << "mode,foil,Re_ref,b_m,AR,taper,sweep_deg,Npoints,aMin,aMax,aStep,CL_target,wLD,wCL,wLDCL,"
           "bestLD,alphaBestLD,CL_at_bestLD,CD_at_bestLD,CM_at_bestLD,"
           "maxCL,alphaMaxCL,CD_at_maxCL,CM_at_maxCL,"
           "LD_at_CL_target,CL_at_CL_target,alpha_at_CL_target,CM_at_CL_target,dCM_dAlpha,"
           "compositeScore,nConverged,nTotal,note\n";

    auto emit_row = [&](const char* mode, const BulkMetric& m) {
        out << mode << ","
            << csv_escape(m.foil) << ","
            << Re_ref << ","
            << fixed << setprecision(6)
            << b_m << "," << AR << "," << taper << "," << sweep_deg << "," << Npoints << ","
            << aMin << "," << aMax << "," << aStep << ","
            << CL_target << ","
            << wLD << "," << wCL << "," << wLDCL << ","
            << m.bestLD << "," << m.alphaBestLD << "," << m.CL_at_bestLD << "," << m.CD_at_bestLD << "," << m.CM_at_bestLD << ","
            << m.maxCL << "," << m.alphaMaxCL << "," << m.CD_at_maxCL << "," << m.CM_at_maxCL << ","
            << m.LD_at_CL_target << "," << m.CL_at_CL_target << "," << m.alpha_at_CL_target << "," << m.CM_at_CL_target << "," << m.dCM_dAlpha << ","
            << m.compositeScore << ","
            << m.nConverged << "," << m.nTotal << ","
            << csv_escape(m.note)
            << "\n";
    };

    for (const auto& m : glider) emit_row("GLIDER", m);
    for (const auto& m : rotor)  emit_row("ROTOR",  m);

    return true;
}

// -------------------- NEW: per-mode metrics CSV writer (for stallPlotter.py) --------------------

static bool write_mode_metrics_csv(const string& outPath,
                                  int Re_ref,
                                  double b_m, double AR, double taper, double sweep_deg, int Npoints,
                                  double aMin, double aMax, double aStep,
                                  double CL_target,
                                  const vector<BulkMetric>& v)
{
    ofstream out(outPath, ios::trunc);
    if (!out) return false;

    out << "foil,Re_ref,b_m,AR,taper,sweep_deg,Npoints,aMin,aMax,aStep,CL_target,"
           "bestLD,alphaBestLD,CL_at_bestLD,CD_at_bestLD,CM_at_bestLD,"
           "maxCL,alphaMaxCL,CD_at_maxCL,CM_at_maxCL,"
           "LD_at_CL_target,CL_at_CL_target,alpha_at_CL_target,CM_at_CL_target,dCM_dAlpha,"
           "compositeScore,nConverged,nTotal,note\n";

    out << fixed << setprecision(6);

    for (const auto& m : v) {
        out << csv_escape(m.foil) << ","
            << Re_ref << ","
            << b_m << "," << AR << "," << taper << "," << sweep_deg << "," << Npoints << ","
            << aMin << "," << aMax << "," << aStep << ","
            << CL_target << ","
            << m.bestLD << "," << m.alphaBestLD << "," << m.CL_at_bestLD << "," << m.CD_at_bestLD << "," << m.CM_at_bestLD << ","
            << m.maxCL << "," << m.alphaMaxCL << "," << m.CD_at_maxCL << "," << m.CM_at_maxCL << ","
            << m.LD_at_CL_target << "," << m.CL_at_CL_target << "," << m.alpha_at_CL_target << "," << m.CM_at_CL_target << "," << m.dCM_dAlpha << ","
            << m.compositeScore << ","
            << m.nConverged << "," << m.nTotal << ","
            << csv_escape(m.note) << "\n";
    }

    return true;
}

// -------------------- stall sweep helper (area + CSV writer) --------------------

static double wing_area_from_b_cr_taper(double b, double cr, double taper) {
    if (!(b > 0.0) || !(cr > 0.0) || !(taper > 0.0)) return 0.0;
    double ct = taper * cr;
    return 0.5 * b * (cr + ct);
}

static bool write_stall_sweep_for_top_glider_csv(
    const string& outPath,
    const vector<BulkMetric>& glider,
    int topByLD,
    int topByCL,
    double mass_kg,
    double rho,
    double g,
    double taper_for_area,
    const vector<double>& bCandidates,
    const vector<double>& cRootCandidates
) {
    ofstream out(outPath, ios::trunc);
    if (!out) return false;

    vector<BulkMetric> byLD = glider;
    vector<BulkMetric> byCL = glider;

    sort(byLD.begin(), byLD.end(), [](const BulkMetric& a, const BulkMetric& b){ return a.bestLD > b.bestLD; });
    sort(byCL.begin(), byCL.end(), [](const BulkMetric& a, const BulkMetric& b){ return a.maxCL  > b.maxCL;  });

    unordered_map<string, int> pickMask; // 1=byLD, 2=byCL
    vector<const BulkMetric*> chosen;
    chosen.reserve((size_t)(topByLD + topByCL));

    auto already_chosen = [&](const string& foil) {
        for (auto* pm : chosen) if (pm->foil == foil) return true;
        return false;
    };

    auto take_from = [&](const vector<BulkMetric>& v, int k, int bit) {
        int taken = 0;
        for (const auto& m : v) {
            if (taken >= k) break;
            if (m.nConverged <= 0) continue;
            if (!(m.maxCL > 0.0)) continue;

            pickMask[m.foil] |= bit;
            if (!already_chosen(m.foil)) chosen.push_back(&m);
            taken++;
        }
    };

    take_from(byLD, topByLD, 1);
    take_from(byCL, topByCL, 2);

    out << "foil,picked_from,mass_kg,rho,g,taper_for_area,"
           "b_m,cr_m,ct_m,S_m2,AR_implied,CLmax_used,Vs_mps,Vs_kts,"
           "glider_bestLD,glider_maxCL,nConverged,nTotal,note\n";

    out << fixed << setprecision(6);

    for (const auto* pm : chosen) {
        const BulkMetric& m = *pm;

        string pickedFrom = "UNKNOWN";
        int mask = pickMask[m.foil];
        if      (mask == 1) pickedFrom = "TOP_BESTLD";
        else if (mask == 2) pickedFrom = "TOP_MAXCL";
        else if (mask == 3) pickedFrom = "BOTH";

        for (double b : bCandidates) {
            for (double cr : cRootCandidates) {
                double ct = taper_for_area * cr;
                double S  = wing_area_from_b_cr_taper(b, cr, taper_for_area);
                if (!(S > 0.0)) continue;

                double ARi = (b*b) / S;
                double W   = mass_kg * g;
                double denom = rho * S * m.maxCL;
                double Vs  = (denom > 1e-12) ? sqrt(2.0 * W / denom) : 0.0;
                double Vsk = Vs * 1.9438444924406;

                out << csv_escape(m.foil) << ","
                    << pickedFrom << ","
                    << mass_kg << "," << rho << "," << g << "," << taper_for_area << ","
                    << b << "," << cr << "," << ct << ","
                    << S << "," << ARi << ","
                    << m.maxCL << ","
                    << Vs << "," << Vsk << ","
                    << m.bestLD << "," << m.maxCL << ","
                    << m.nConverged << "," << m.nTotal << ","
                    << csv_escape(m.note)
                    << "\n";
            }
        }
    }

    return true;
}
// -------------------- bulk summary DAT writer (Formatted for reading) --------------------

static bool write_summary_dat(const string& outPath,
                              int Re_ref, double b_m, double AR, double taper, double sweep_deg, int Npoints,
                              double aMin, double aMax, double aStep, int topK,
                              double CL_target, double wLD, double wCL, double wLDCL,
                              const vector<BulkMetric>& glider,
                              const vector<BulkMetric>& rotor)
{
    ofstream out(outPath, ios::trunc);
    if (!out) return false;

    out << "# BULK LLT SUMMARY (single-foil root=tip; Re_ref=" << Re_ref << ")\n";
    out << "# Geometry: b_m=" << b_m << " AR=" << AR << " taper=" << taper << " sweep_deg=" << sweep_deg << " Npoints=" << Npoints << "\n";
    out << "# Alpha sweep: aMin=" << aMin << " aMax=" << aMax << " aStep=" << aStep << "\n";
    out << "# Ranking: topK=" << topK << "\n";
    out << "# Balanced metrics:\n";
    out << "#   - LD@CL_target uses CL_target=" << CL_target << " (CL-space interpolation)\n";
    out << "#   - compositeScore = wLD*(bestLD/maxBestLD) + wCL*(maxCL/maxMaxCL) + wLDCL*(LD@CL_target/maxLD@CL)\n";
    out << "#     weights: wLD=" << wLD << " wCL=" << wCL << " wLDCL=" << wLDCL << "\n\n";

    auto print_section = [&](const string& modeName, const vector<BulkMetric>& metrics) {
        out << "==================== " << modeName << " MODE ====================\n\n";

        vector<BulkMetric> byLD = metrics;
        vector<BulkMetric> byCL = metrics;

        // Filter out non-converged data before sorting
        byLD.erase(remove_if(byLD.begin(), byLD.end(), [](const BulkMetric& m){ return m.nConverged == 0; }), byLD.end());
        byCL.erase(remove_if(byCL.begin(), byCL.end(), [](const BulkMetric& m){ return m.nConverged == 0; }), byCL.end());

        sort(byLD.begin(), byLD.end(), [](const BulkMetric& a, const BulkMetric& b){ return a.bestLD > b.bestLD; });
        sort(byCL.begin(), byCL.end(), [](const BulkMetric& a, const BulkMetric& b){ return a.maxCL > b.maxCL; });

        // TOP BY BEST L/D
        out << "## Top by best L/D (" << modeName << ")\n";
        out << left << setw(5) << "rank" << setw(15) << "airfoil"
            << right << setw(12) << "best_LD" << setw(18) << "alpha_at_bestLD"
            << setw(15) << "CL_at_bestLD" << setw(15) << "CD_at_bestLD" 
            << setw(15) << "CM_at_bestLD" << setw(15) << "dCM_dAlpha"
            << setw(14) << "conv/total" << "  note\n";

        int limit = min(topK, (int)byLD.size());
        for (int i = 0; i < limit; ++i) {
            const auto& m = byLD[i];
            out << left << setw(5) << (i+1) << setw(15) << m.foil << right << fixed << setprecision(6)
                << setw(12) << m.bestLD << setw(18) << m.alphaBestLD
                << setw(15) << m.CL_at_bestLD << setw(15) << m.CD_at_bestLD
                << setw(15) << m.CM_at_bestLD << setw(15) << m.dCM_dAlpha
                << setw(9) << m.nConverged << "/" << left << setw(4) << m.nTotal
                << " " << m.note << "\n";
        }
        out << "\n";

        // TOP BY MAX CL
        out << "## Top by max CL (" << modeName << ")\n";
        out << left << setw(5) << "rank" << setw(15) << "airfoil"
            << right << setw(12) << "maxCL" << setw(18) << "alpha_at_maxCL"
            << setw(15) << "CD_at_maxCL" << setw(15) << "CM_at_maxCL"
            << setw(15) << "dCM_dAlpha"
            << setw(14) << "conv/total" << "  note\n";

        limit = min(topK, (int)byCL.size());
        for (int i = 0; i < limit; ++i) {
            const auto& m = byCL[i];
            out << left << setw(5) << (i+1) << setw(15) << m.foil << right << fixed << setprecision(6)
                << setw(12) << m.maxCL << setw(18) << m.alphaMaxCL
                << setw(15) << m.CD_at_maxCL << setw(15) << m.CM_at_maxCL
                << setw(15) << m.dCM_dAlpha
                << setw(9) << m.nConverged << "/" << left << setw(4) << m.nTotal
                << " " << m.note << "\n";
        }
        out << "\n";
    };

    print_section("GLIDER", glider);
    print_section("ROTOR", rotor);

    return true;
}

// -------------------- main --------------------

int main(int argc, char** argv) {
    // Active executables live in tools/bin; resolve all project-relative paths
    // from the repository root, independent of the caller's working directory.
    const fs::path projectRoot = fs::absolute(argv[0]).parent_path().parent_path().parent_path();
    fs::current_path(projectRoot);
    const string DEFAULT_MEGA_PATH = "data\\polars\\Currentmegapolar.dat";
    const int    DEFAULT_NCRIT     = 5;
    const string OUT_DIR = "results\\runs\\llt";
    string megaPath = DEFAULT_MEGA_PATH;

    int Re_glider = 80000;
    int Re_rotor  = 80000;

    double b_m = .9;
    double AR = 6.0;
    double taper = 1;
    double sweep_deg = 0.0; // <-- Tweak to calculate proper 3D Cm
    int Npoints = 45;
    double blendStart = 0.3;
    double blendEnd   = 0.8;

    double aMin = -10.0;
    double aMax = 20.0;
    double aStep = 0.25;

    int TOP_K = 10;            // (kept, but CSV writes all foils; TOP_K still used for stall-top selection below)
    double CL_TARGET = 0.50;   // L/D at this CL (interpolated)
    double W_LD   = 0.50;      // composite weight for best L/D
    double W_CL   = 0.20;      // composite weight for max CL
    double W_LDCL = 0.30;      // composite weight for L/D@CL_TARGET

    const int    MAX_ITER = 250;
    const double RELAX    = 0.15;
    const double TOL_A    = 2e-5;

    double omega_rad_s = 300.0;
    double hubFrac = 0.10;
    double rRefFrac = 0.75;

    int debugFlags = 0;

    bool BULK_MODE = true;
    int userThreads = 0; // 0 = auto
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        if (a == "--single") BULK_MODE = false;
        if (a == "--bulk")   BULK_MODE = true;
        if (a == "--threads" && i + 1 < argc) {
            int t = 0;
            if (parse_int_strict(argv[i+1], t)) userThreads = t;
        }
    }

    // minimal file arg override (first non-flag)
    if (argc >= 2) {
        string a1 = argv[1];
        if (a1.rfind("--", 0) != 0 && !looks_like_int(a1)) megaPath = a1;
    }

    // -------------------- stall speed sweep tunables --------------------
    const double MASS_KG = 0.060;     // <-- set your aircraft mass here (kg)
    const double RHO     = 1.225;     // air density (kg/m^3)
    const double G0      = 9.80665;   // gravity (m/s^2)

    const int TOP_FOILS_BY_BESTLD = 5; // choose top-N foils by bestLD (glider)
    const int TOP_FOILS_BY_MAXCL  = 5; // choose top-N foils by maxCL (glider)

    const vector<double> bCandidates = { 0.6, 0.8, 1.0, 1.2, 1.4 };
    const vector<double> cRootCandidates = { 0.08, 0.10, 0.12, 0.14, 0.16 };

    const double TAPER_FOR_STALL_SWEEP = taper;
    // ---------------------------------------------------

#if __has_include(<filesystem>)
    {
        std::error_code ec;
        fs::create_directories(OUT_DIR, ec);
        if (ec) { cout << "ERROR: could not create output folder: " << OUT_DIR << "\n"; return 2; }
    }
#endif

    ifstream in(megaPath);
    if (!in) { cout << "ERROR: could not open megapolar file: " << megaPath << "\n"; return 2; }

    vector<string> bulkFoilsRaw = build_airfoil_list();
    vector<string> bulkFoilsKey;
    bulkFoilsKey.reserve(bulkFoilsRaw.size());
    for (const auto& s : bulkFoilsRaw) bulkFoilsKey.push_back(normalize_airfoil_key(s));

    unordered_set<string> neededFoils;
    if (BULK_MODE) {
        for (const auto& k : bulkFoilsKey) neededFoils.insert(k);
        Re_glider = 80000;
        Re_rotor  = 80000;
    } else {
        neededFoils.insert(normalize_airfoil_key("NACA 2208"));
    }

    PolarDB db;
    string line;
    while (getline(in, line)) {
        Row r;
        if (!parse_megapolar_line(line, r)) continue;
        if (!passes_sanity_filters(r)) continue;
        if (r.Ncrit != DEFAULT_NCRIT) continue;
        if (neededFoils.find(r.airfoil) == neededFoils.end()) continue;
        db.add_point(r.airfoil, r.Re, r.alpha, r.CL, r.CD, r.CM);
    }
    db.finalize_all();

    for (const auto& f : neededFoils) {
        if (!db.has_foil(f)) {
            cout << "ERROR: foil not found in megapolar (after Ncrit filter): " << f << "\n";
            return 2;
        }
    }

    LLTPrecomp pc;
    if (!build_precomp(pc, b_m, AR, taper, sweep_deg, Npoints)) {
        cout << "ERROR: failed to build LLT precompute tables.\n";
        return 2;
    }

    ModeParams mpG; mpG.mode = Mode::GLIDER; mpG.Re_ref = (double)Re_glider; mpG.stallCap = true;
    ModeParams mpR; mpR.mode = Mode::ROTOR;  mpR.Re_ref = (double)Re_rotor;  mpR.stallCap = true;
    mpR.omega_rad_s = omega_rad_s; mpR.hubFrac = hubFrac; mpR.rRefFrac = rRefFrac;

    const int nAlpha = (int)floor((aMax - aMin) / aStep + 0.5) + 1;
    const int nFoils = (int)bulkFoilsKey.size();

    unsigned hw = std::thread::hardware_concurrency();
    int nThreads = (userThreads > 0) ? userThreads : (hw ? (int)hw : 8);
    if (nThreads < 1) nThreads = 1;
    if (nThreads > 32) nThreads = 32;

    cout << "Loaded polars for " << neededFoils.size() << " foils at Ncrit=" << DEFAULT_NCRIT << "\n";
    cout << "Precompute: Npoints=" << Npoints << " (LU(M) built once)\n";
    cout << "Bulk: foils=" << nFoils << "  modes=2  alphaSteps=" << nAlpha << "  threads=" << nThreads << "\n";
    cout << "Ranking tunables: TOP_K=" << TOP_K << "  CL_TARGET=" << CL_TARGET
         << "  weights=(" << W_LD << "," << W_CL << "," << W_LDCL << ")\n";

    vector<BulkMetric> gliderMetrics(nFoils);
    vector<BulkMetric> rotorMetrics(nFoils);

    atomic<int> nextIdx(0);
    atomic<long long> doneCases(0);
    const long long totalCases = (long long)nFoils * 2LL * (long long)nAlpha;

    atomic<bool> stopProgress(false);
    mutex ioMutex;

    auto t0 = chrono::steady_clock::now();

    thread progressThread([&](){
        while (!stopProgress.load()) {
            auto now = chrono::steady_clock::now();
            double elapsed = chrono::duration<double>(now - t0).count();
            long long done = doneCases.load();
            double frac = (totalCases > 0) ? (double)done / (double)totalCases : 1.0;
            if (frac > 1.0) frac = 1.0;

            double rate = (elapsed > 1e-6) ? ((double)done / elapsed) : 0.0;
            double eta = (rate > 1e-9) ? ((double)(totalCases - done) / rate) : 0.0;

            {
                lock_guard<mutex> lk(ioMutex);
                print_progress_bar(frac, done, totalCases, rate, elapsed, eta);
            }

            if (done >= totalCases) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        {
            lock_guard<mutex> lk(ioMutex);
            cout << "\n";
        }
    });

    auto run_alpha_sweep = [&](const string& foilKey, const ModeParams& mpBase, BulkMetric& outMetric) {
        vector<LLTResult> wing;
        wing.reserve((size_t)nAlpha + 2);

        vector<double> Aprev;
        long long localDone = 0;

        for (int ia = 0; ia < nAlpha; ++ia) {
            double a = aMin + (double)ia * aStep;

            vector<double> Afinal;
            LLTResult r = solve_llt_nonlinear_one_alpha_twofoil_mode(
                db, pc,
                foilKey, foilKey,
                mpBase,
                a,
                blendStart, blendEnd,
                MAX_ITER, RELAX, TOL_A, debugFlags,
                (Aprev.empty() ? nullptr : &Aprev),
                &Afinal,
                nullptr
            );

            if (r.converged) Aprev = Afinal;
            wing.push_back(r);
            localDone++;
        }

        doneCases.fetch_add(localDone, std::memory_order_relaxed);
        outMetric = summarize_wing(foilKey, wing, CL_TARGET);
    };

    auto worker = [&](){
        while (true) {
            int k = nextIdx.fetch_add(1);
            if (k >= nFoils) break;

            const string& foilKey = bulkFoilsKey[(size_t)k];

            BulkMetric mg, mr;
            run_alpha_sweep(foilKey, mpG, mg);
            run_alpha_sweep(foilKey, mpR, mr);

            gliderMetrics[(size_t)k] = mg;
            rotorMetrics[(size_t)k]  = mr;
        }
    };

    vector<thread> threads;
    threads.reserve((size_t)nThreads);
    for (int i = 0; i < nThreads; ++i) threads.emplace_back(worker);
    for (auto& th : threads) th.join();

    doneCases.store(totalCases);
    stopProgress.store(true);
    if (progressThread.joinable()) progressThread.join();

    // -------------------- composite score (per mode, normalized) --------------------
    auto fill_composite = [&](vector<BulkMetric>& v) {
        double maxBestLD = 1e-12;
        double maxMaxCL  = 1e-12;
        double maxLDCL   = 1e-12;

        for (const auto& m : v) {
            if (m.nConverged <= 0) continue;
            if (m.bestLD > maxBestLD) maxBestLD = m.bestLD;
            if (m.maxCL  > maxMaxCL)  maxMaxCL  = m.maxCL;
            if (m.LD_at_CL_target > maxLDCL) maxLDCL = m.LD_at_CL_target;
        }

        for (auto& m : v) {
            if (m.nConverged <= 0) { m.compositeScore = -1e300; continue; }
            double nLD  = (m.bestLD > 0.0) ? (m.bestLD / maxBestLD) : 0.0;
            double nCL  = (m.maxCL  > 0.0) ? (m.maxCL  / maxMaxCL)  : 0.0;
            double nLDC = (m.LD_at_CL_target > 0.0) ? (m.LD_at_CL_target / maxLDCL) : 0.0;
            m.compositeScore = W_LD * nLD + W_CL * nCL + W_LDCL * nLDC;
        }
    };

    fill_composite(gliderMetrics);
    fill_composite(rotorMetrics);
    // -------------------------------------------------------------------------------

    // -------------------- WRITE: per-mode metrics CSVs (for stallPlotter.py) --------------------
    string OUT_GLIDER_METRICS = OUT_DIR + "\\glider_metrics.csv";
    if (!write_mode_metrics_csv(OUT_GLIDER_METRICS, 80000, b_m, AR, taper, sweep_deg, Npoints,
                                aMin, aMax, aStep,
                                CL_TARGET,
                                gliderMetrics))
    {
        cout << "ERROR: failed to write glider metrics CSV: " << OUT_GLIDER_METRICS << "\n";
        return 2;
    }
    cout << "Wrote: " << OUT_GLIDER_METRICS << "\n";

    string OUT_ROTOR_METRICS = OUT_DIR + "\\rotor_metrics.csv";
    if (!write_mode_metrics_csv(OUT_ROTOR_METRICS, 80000, b_m, AR, taper, sweep_deg, Npoints,
                                aMin, aMax, aStep,
                                CL_TARGET,
                                rotorMetrics))
    {
        cout << "ERROR: failed to write rotor metrics CSV: " << OUT_ROTOR_METRICS << "\n";
        return 2;
    }
    cout << "Wrote: " << OUT_ROTOR_METRICS << "\n";
    // ------------------------------------------------------------------------------------------

    // -------------------- WRITE: bulk summary CSV --------------------
    string OUT_SUM_CSV = OUT_DIR + "\\bulk_summary_Re80000.csv";
    if (!write_bulk_summary_csv(OUT_SUM_CSV, 80000, b_m, AR, taper, sweep_deg, Npoints,
                                aMin, aMax, aStep,
                                CL_TARGET, W_LD, W_CL, W_LDCL,
                                gliderMetrics, rotorMetrics))
    {
        cout << "ERROR: failed to write bulk summary CSV: " << OUT_SUM_CSV << "\n";
        return 2;
    }
    cout << "Wrote: " << OUT_SUM_CSV << "\n";

    // -------------------- WRITE: formatted .dat summary --------------------
    string OUT_DAT = OUT_DIR + "\\bulk_summary_Re80000.dat";
    if (!write_summary_dat(OUT_DAT, 80000, b_m, AR, taper, sweep_deg, Npoints,
                           aMin, aMax, aStep, TOP_K,
                           CL_TARGET, W_LD, W_CL, W_LDCL,
                           gliderMetrics, rotorMetrics))
    {
        cout << "ERROR: failed to write dat summary: " << OUT_DAT << "\n";
    } else {
        cout << "Wrote: " << OUT_DAT << "\n";
    }
    
    // -------------------- WRITE: stall sweep CSV --------------------
    string OUT_STALL_CSV = OUT_DIR + "\\stall_sweep_glider_topfoils.csv";
    if (!write_stall_sweep_for_top_glider_csv(
            OUT_STALL_CSV,
            gliderMetrics,
            TOP_FOILS_BY_BESTLD,
            TOP_FOILS_BY_MAXCL,
            MASS_KG,
            RHO,
            G0,
            TAPER_FOR_STALL_SWEEP,
            bCandidates,
            cRootCandidates))
    {
        cout << "ERROR: failed to write stall sweep CSV: " << OUT_STALL_CSV << "\n";
        return 2;
    }
    cout << "Wrote: " << OUT_STALL_CSV << "\n";

    return 0;
}
