// wingpolar_llt_dualmode.cpp
// Nonlinear LLT postprocess using megapolar.dat
// Adds:
//  (1) spanwise Reynolds sampling (Re(y) per station)
//  (2) simple stall-cap (clamped to cl_max/cl_min of the 2D polar)
//  (3) dual-mode run: GLIDER mode + ROTOR mode (Re(y) from Omega*r)
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
//  - Kept the overall solver logic and outputs the same (same inputs/outputs/flags).
//
// Build: g++ wingpolar_llt_dualmode.cpp -O2 -std=c++17 -o wingpolar_llt_dualmode

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

// Strictly smooth clamp.
// Ensures return value is strictly in [lo, hi].
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

// -------------------- polar curve (linear interp + stall cap data) --------------------

struct PolarCurve {
    vector<double> a, cl, cd;

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
        reorder(a); reorder(cl); reorder(cd);
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

    bool sample_linear(double alpha, double& outCL, double& outCD) const {
        if (a.empty()) return false;
        if (alpha < a.front() - 1e-12 || alpha > a.back() + 1e-12) return false;

        auto it = lower_bound(a.begin(), a.end(), alpha);
        if (it != a.end() && fabs(*it - alpha) < 1e-12) {
            size_t i = (size_t)(it - a.begin());
            outCL = cl[i]; outCD = cd[i];
            return true;
        }
        if (it == a.begin() || it == a.end()) return false;

        size_t hi = (size_t)(it - a.begin());
        size_t lo = hi - 1;

        double a0 = a[lo], a1 = a[hi];
        double t = (alpha - a0) / (a1 - a0);

        outCL = cl[lo] + t * (cl[hi] - cl[lo]);
        outCD = cd[lo] + t * (cd[hi] - cd[lo]);
        return true;
    }

    bool sample_with_stallcap(double alpha, double& outCL, double& outCD) const {
        if (!sample_linear(alpha, outCL, outCD)) return false;
        if (!hasExtrema) return true;
        if (alpha > aAtClMax && outCL > clMax) outCL = clMax;
        if (alpha < aAtClMin && outCL < clMin) outCL = clMin;
        return true;
    }

    double alpha_min() const { return a.empty() ? 0.0 : a.front(); }
    double alpha_max() const { return a.empty() ? 0.0 : a.back();  }

    // Fast helper used by optimized sampling (keeps your current internal clamp policy)
    bool sample_clamped(double alphaDeg, double& outCL, double& outCD, bool stallCap) const {
        if (a.empty()) return false;
        const double CLAMP_W_DEG = 0.0; // keep behavior identical
        double aMin = alpha_min(), aMax = alpha_max();
        double aUse = smooth_clamp(alphaDeg, aMin, aMax, CLAMP_W_DEG);
        if (stallCap) return sample_with_stallcap(aUse, outCL, outCD);
        else          return sample_linear(aUse, outCL, outCD);
    }
};

// -------------------- Polar DB (optimized Re indexing) --------------------

struct FoilPolars {
    vector<int> ReVals;             // sorted
    vector<PolarCurve> curves;      // same length as ReVals

    // build-time only (minimal changes: we store points in a hash first)
    unordered_map<int, PolarCurve> buildMap;
};

struct PolarDB {
    unordered_map<string, FoilPolars> foils;

    void add_point(const string& foil, int Re, double alpha, double cl, double cd) {
        FoilPolars& fp = foils[foil];
        PolarCurve& pc = fp.buildMap[Re];
        pc.a.push_back(alpha);
        pc.cl.push_back(cl);
        pc.cd.push_back(cd);
    }

    void finalize_all() {
        for (auto& fk : foils) {
            FoilPolars& fp = fk.second;

            // finalize curves in buildMap
            vector<int> res;
            res.reserve(fp.buildMap.size());
            for (auto& kv : fp.buildMap) {
                kv.second.sort_by_alpha();
                kv.second.compute_extrema();
                res.push_back(kv.first);
            }
            sort(res.begin(), res.end());

            fp.ReVals = res;
            fp.curves.clear();
            fp.curves.reserve(res.size());
            for (int Re : res) {
                fp.curves.push_back(std::move(fp.buildMap[Re]));
            }

            // release buildMap memory
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

    // Returns indices into ReVals/curves plus the actual Re values
    bool bracket_Re_for_foil_idx(const string& foil, double desiredRe,
                                 int& idxLo, int& idxHi, int& ReLo, int& ReHi) const {
        auto itF = foils.find(foil);
        if (itF == foils.end()) return false;
        const FoilPolars& fp = itF->second;
        if (fp.ReVals.empty()) return false;

        const int dRe = (int)llround(desiredRe);
        auto it = lower_bound(fp.ReVals.begin(), fp.ReVals.end(), dRe);
        if (it == fp.ReVals.begin()) {
            idxLo = idxHi = 0;
            ReLo = ReHi = fp.ReVals[0];
            return true;
        }
        if (it == fp.ReVals.end()) {
            idxLo = idxHi = (int)fp.ReVals.size() - 1;
            ReLo = ReHi = fp.ReVals.back();
            return true;
        }
        idxHi = (int)(it - fp.ReVals.begin());
        idxLo = idxHi - 1;
        ReHi = fp.ReVals[(size_t)idxHi];
        ReLo = fp.ReVals[(size_t)idxLo];
        return true;
    }

    bool get_interpolated_range(const string& foil, double desiredRe, double& outMin, double& outMax) const {
        int idxLo=-1, idxHi=-1, ReLo=-1, ReHi=-1;
        if (!bracket_Re_for_foil_idx(foil, desiredRe, idxLo, idxHi, ReLo, ReHi)) return false;

        const FoilPolars& fp = foils.at(foil);
        const PolarCurve& pcLo = fp.curves[(size_t)idxLo];
        const PolarCurve& pcHi = fp.curves[(size_t)idxHi];

        double minLo = pcLo.alpha_min(); double maxLo = pcLo.alpha_max();
        double minHi = pcHi.alpha_min(); double maxHi = pcHi.alpha_max();
        double t = interp_weight_logRe(desiredRe, ReLo, ReHi);

        outMin = (1.0 - t) * minLo + t * minHi;
        outMax = (1.0 - t) * maxLo + t * maxHi;
        return true;
    }

    // Optimized sampler using pre-bracketed indices (no map lookup / no bracket work)
    bool sample_bracketed_idx(const string& foil,
                              int idxLo, int idxHi, int ReLo, int ReHi, double t,
                              double alphaDeg, double& outCL, double& outCD, bool stallCap) const {
        auto itF = foils.find(foil);
        if (itF == foils.end()) return false;
        const FoilPolars& fp = itF->second;
        if (idxLo < 0 || idxHi < 0) return false;
        if ((size_t)idxLo >= fp.curves.size() || (size_t)idxHi >= fp.curves.size()) return false;

        if (ReLo == ReHi || idxLo == idxHi) {
            return fp.curves[(size_t)idxLo].sample_clamped(alphaDeg, outCL, outCD, stallCap);
        }

        double cl0=0.0, cd0=0.0, cl1=0.0, cd1=0.0;
        if (!fp.curves[(size_t)idxLo].sample_clamped(alphaDeg, cl0, cd0, stallCap)) return false;
        if (!fp.curves[(size_t)idxHi].sample_clamped(alphaDeg, cl1, cd1, stallCap)) return false;

        outCL = (1.0 - t) * cl0 + t * cl1;
        outCD = (1.0 - t) * cd0 + t * cd1;
        return true;
    }

    // Keep original API for any callers that still use it (now faster internally)
    bool sample(const string& foil, double desiredRe, double alphaDeg,
                double& outCL, double& outCD, bool stallCap) const {
        int idxLo=-1, idxHi=-1, ReLo=-1, ReHi=-1;
        if (!bracket_Re_for_foil_idx(foil, desiredRe, idxLo, idxHi, ReLo, ReHi)) return false;
        double t = interp_weight_logRe(desiredRe, ReLo, ReHi);
        return sample_bracketed_idx(foil, idxLo, idxHi, ReLo, ReHi, t, alphaDeg, outCL, outCD, stallCap);
    }

    // Compatibility helper for dump diagnostics (still used)
    bool bracket_Re_for_foil(const string& foil, double desiredRe, int& ReLo, int& ReHi) const {
        int idxLo=-1, idxHi=-1;
        return bracket_Re_for_foil_idx(foil, desiredRe, idxLo, idxHi, ReLo, ReHi);
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

    // angles
    double alpha_i_deg = 0.0;
    double alpha_eff_raw_deg = 0.0;
    double alpha_eff_deg = 0.0; // (possibly clamped)

    // window diagnostics
    double minWindow = 0.0;
    double maxWindow = 0.0;
    double clamp_delta = 0.0;   // alpha_eff_deg - alpha_eff_raw_deg
    double identity_error = 0.0; // (alpha_eff_deg + alpha_i_deg) - alpha_geom_deg

    // Re bracket diagnostics
    int ReLoRoot = -1, ReHiRoot = -1;
    int ReLoTip  = -1, ReHiTip  = -1;

    // aero
    double cl = 0.0;
    double cd = 0.0;
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

// ---------- LU factorization helpers (optimization) ----------
// In-place LU with partial pivoting for dense NxN matrix (row-major).
// Returns false if singular.
// Stores pivots in 'piv' such that rows are permuted accordingly.
static bool lu_decompose_inplace(vector<double>& A, int N, vector<int>& piv) {
    piv.resize(N);
    for (int i = 0; i < N; ++i) piv[i] = i;

    for (int k = 0; k < N; ++k) {
        // pivot
        int pivRow = k;
        double best = fabs(A[k*N + k]);
        for (int r = k + 1; r < N; ++r) {
            double v = fabs(A[r*N + k]);
            if (v > best) { best = v; pivRow = r; }
        }
        if (best < 1e-14) return false;

        if (pivRow != k) {
            // swap rows in A
            for (int c = 0; c < N; ++c) swap(A[k*N + c], A[pivRow*N + c]);
            swap(piv[k], piv[pivRow]);
        }

        double Akk = A[k*N + k];
        // eliminate
        for (int i = k + 1; i < N; ++i) {
            double f = A[i*N + k] / Akk;
            A[i*N + k] = f; // store L factor
            if (fabs(f) < 1e-20) continue;
            for (int j = k + 1; j < N; ++j) {
                A[i*N + j] -= f * A[k*N + j];
            }
        }
    }
    return true;
}

// Solve LU x = b (LU produced by lu_decompose_inplace), with pivot vector.
// A contains L (unit diag) and U.
// b is input, x is output.
static void lu_solve(const vector<double>& LU, int N, const vector<int>& piv,
                     const vector<double>& b, vector<double>& x) {
    x.assign(N, 0.0);
    // Apply pivot to b -> y
    vector<double> y(N, 0.0);
    // piv stores swapped indices; because we swapped piv entries during decomposition,
    // we can reconstruct the permuted RHS by tracking row swaps directly.
    // Easiest consistent approach: when we swapped rows in LU, we also swapped piv indices.
    // Then piv[k] indicates original row index now at row k.
    // So P*b in row k uses b[piv[k]].
    for (int i = 0; i < N; ++i) y[i] = b[piv[i]];

    // Forward solve L z = y (L has 1s on diag; lower stored in LU)
    vector<double> z(N, 0.0);
    for (int i = 0; i < N; ++i) {
        double sum = y[i];
        for (int j = 0; j < i; ++j) sum -= LU[i*N + j] * z[j];
        z[i] = sum;
    }

    // Back solve U x = z
    for (int i = N - 1; i >= 0; --i) {
        double sum = z[i];
        for (int j = i + 1; j < N; ++j) sum -= LU[i*N + j] * x[j];
        double diag = LU[i*N + i];
        x[i] = (fabs(diag) > 1e-18) ? (sum / diag) : 0.0;
    }
}

// Precomputed per-station invariants for a single alpha solve
struct StationPre {
    double wTip = 0.0;
    double ReLoc = 0.0;
    double minWindow = -90.0;
    double maxWindow =  90.0;

    // Root bracket
    int idxLoR=-1, idxHiR=-1, ReLoR=-1, ReHiR=-1;
    double tR=0.0;

    // Tip bracket
    int idxLoT=-1, idxHiT=-1, ReLoT=-1, ReHiT=-1;
    double tT=0.0;
};

static LLTResult solve_llt_nonlinear_one_alpha_twofoil_mode(
    const PolarDB& db,
    const string& rootFoil,
    const string& tipFoil,
    const ModeParams& mp_in,
    double alpha_deg,
    double b_m,
    double AR,
    double taper,
    int N,
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
    const double TIP_EPS_FRACTION = 0.5 / (double)(N + 1);

    ModeParams mp = mp_in;
    if (debugFlags & 4) mp.stallCap = false;

    const bool DISABLE_WINDOW_CLAMP = (debugFlags & 1) != 0;
    const bool FREEZE_ROTOR_RE      = (debugFlags & 2) != 0;

    LLTResult res;
    res.alpha_deg = alpha_deg;

    if (!db.has_foil(rootFoil) || !db.has_foil(tipFoil)) { res.note = "NO_FOIL_IN_DB"; return res; }
    if (!(b_m > 0.0) || !(AR > 0.0) || !(taper > 0.0) || N < 3) { res.note = "BAD_INPUTS"; return res; }
    if (!(blendStart >= 0.0 && blendStart <= 1.0 && blendEnd >= 0.0 && blendEnd <= 1.0)) { res.note = "BAD_BLEND"; return res; }

    const double b = b_m;
    const double R = 0.5 * b;

    double cr=0.0, ct=0.0, S=0.0;
    choose_planform_from_AR_taper(AR, taper, b, cr, ct, S);

    vector<double> theta(N), sinT(N), yAbs(N), yFrac(N), c(N);
    for (int i = 0; i < N; ++i) {
        double th = ((i + 1.0) * PI) / (double)(N + 1.0);
        th = (TIP_EPS_FRACTION * PI) + (1.0 - 2.0 * TIP_EPS_FRACTION) * th;
        theta[i] = th;
        sinT[i] = sin(th);
        double y = (b * 0.5) * cos(th);
        yAbs[i] = fabs(y);
        yFrac[i] = (2.0 * yAbs[i]) / b;
        c[i] = chord_linear_taper(yAbs[i], b, cr, ct);
    }
    const double cRef = cr;

    auto Re_local_at_station = [&](int i) -> double {
        double chordRatio = (cRef > 0.0) ? (c[i] / cRef) : 1.0;

        if (mp.mode == Mode::GLIDER) {
            return mp.Re_ref * chordRatio;
        }

        if (FREEZE_ROTOR_RE) {
            return mp.Re_ref * chordRatio;
        } else {
            double rEff = max(yAbs[i], mp.hubFrac * R);
            double rRef = max(1e-6, mp.rRefFrac * R);
            return mp.Re_ref * (rEff / rRef) * chordRatio;
        }
    };

    // ---- Precompute per-station invariants (optimization) ----
    vector<StationPre> pre(N);
    for (int i = 0; i < N; ++i) {
        pre[i].wTip = tip_weight(yFrac[i], blendStart, blendEnd);
        pre[i].ReLoc = Re_local_at_station(i);

        // Root brackets
        if (!db.bracket_Re_for_foil_idx(rootFoil, pre[i].ReLoc, pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR)) {
            res.note = "ROOT_BRACKET_FAIL"; return res;
        }
        pre[i].tR = PolarDB::interp_weight_logRe(pre[i].ReLoc, pre[i].ReLoR, pre[i].ReHiR);

        // Tip brackets
        if (!db.bracket_Re_for_foil_idx(tipFoil, pre[i].ReLoc, pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT)) {
            res.note = "TIP_BRACKET_FAIL"; return res;
        }
        pre[i].tT = PolarDB::interp_weight_logRe(pre[i].ReLoc, pre[i].ReLoT, pre[i].ReHiT);

        // Alpha window interpolation (same as your previous logic, but computed once per station)
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

    // Precompute sin(n*theta_i) matrix once (also used for LU M)
    vector<double> M(N * N, 0.0);
    vector<double> nSin(N * N, 0.0); // n * sin(n theta_i)
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            int n = j + 1;
            double s = sin((double)n * theta[i]);
            M[i*N + j] = s;
            nSin[i*N + j] = (double)n * s;
        }
    }

    // LU factorize M once (optimization)
    vector<double> LU = M;
    vector<int> piv;
    if (!lu_decompose_inplace(LU, N, piv)) { res.note = "LINEAR_SOLVE_FAIL"; return res; }

    auto alpha_induced_rad = [&](int i, const vector<double>& Acoef) -> double {
        double denom = sinT[i];
        double sum = 0.0;
        const double* row = &nSin[i*N];
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

            double clR=0.0, cdR=0.0, clT=0.0, cdT=0.0;
            bool okR = db.sample_bracketed_idx(rootFoil,
                                              pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR, pre[i].tR,
                                              alpha_eff_deg, clR, cdR, mp.stallCap);
            bool okT = db.sample_bracketed_idx(tipFoil,
                                              pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT, pre[i].tT,
                                              alpha_eff_deg, clT, cdT, mp.stallCap);
            if (!okR) { res.note = "ROOT_SAMPLE_FAIL"; res.iters = iter; return res; }
            if (!okT) { res.note = "TIP_SAMPLE_FAIL";  res.iters = iter; return res; }

            double cl = (1.0 - pre[i].wTip) * clR + pre[i].wTip * clT;
            RHS[i] = (cl * c[i]) / (4.0 * b);
        }

        // Fast solve using LU
        lu_solve(LU, N, piv, RHS, Asolve);

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

    res.CL = PI * AR * A[0];
    double sumInd = 0.0;
    for (int n = 1; n <= N; ++n) sumInd += (double)n * A[n-1] * A[n-1];
    res.CDi = PI * AR * sumInd;

    const double dth = PI / (double)N;
    double S_num = 0.0;
    double Dp_num = 0.0;

    for (int i = 0; i < N; ++i) {
        double ai = alpha_induced_rad(i, A);
        double alpha_eff_raw_deg = (alpha_geom_rad - ai) * 180.0 / PI;

        double alpha_eff_deg = alpha_eff_raw_deg;
        if (!DISABLE_WINDOW_CLAMP) {
            alpha_eff_deg = smooth_clamp(alpha_eff_deg, pre[i].minWindow, pre[i].maxWindow, 0.75);
        }

        double clR=0.0, cdR=0.0, clT=0.0, cdT=0.0;
        db.sample_bracketed_idx(rootFoil,
                                pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR, pre[i].tR,
                                alpha_eff_deg, clR, cdR, mp.stallCap);
        db.sample_bracketed_idx(tipFoil,
                                pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT, pre[i].tT,
                                alpha_eff_deg, clT, cdT, mp.stallCap);

        double cd = (1.0 - pre[i].wTip) * cdR + pre[i].wTip * cdT;
        double w = c[i] * sinT[i] * dth;
        S_num += w;
        Dp_num += cd * w;
    }

    const double S_calc = (b * 0.5) * S_num;
    if (!(S_calc > 0.0)) { res.note = "BAD_AREA_INTEGRAL"; res.converged = false; return res; }

    res.CDp = ((b * 0.5) / S_calc) * Dp_num;
    res.CD  = res.CDp + res.CDi;
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

            double clR=0.0, cdR=0.0, clT=0.0, cdT=0.0;
            bool okR = db.sample_bracketed_idx(rootFoil,
                                              pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR, pre[i].tR,
                                              alpha_eff_deg, clR, cdR, mp.stallCap);
            bool okT = db.sample_bracketed_idx(tipFoil,
                                              pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT, pre[i].tT,
                                              alpha_eff_deg, clT, cdT, mp.stallCap);

            StationDumpRow d;
            d.i = i;
            d.theta = theta[i];
            d.yFrac = yFrac[i];
            d.yAbs  = yAbs[i];
            d.chord = c[i];
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
            } else {
                d.cl = 0.0; d.cd = 0.0;
            }
            dumpOut->push_back(d);
        }
    }

    return res;
}

// -------------------- output writers --------------------

static bool write_wingpolar_llt(const string& outPath, const string& label, const string& rootFoil, const string& tipFoil,
    double Re_ref, int ncrit, double b_m, double AR, double taper, int N, double blendStart, double blendEnd,
    double aMin, double aMax, double aStep, const ModeParams& mp, const vector<LLTResult>& pol, int debugFlags)
{
    ofstream out(outPath, ios::trunc);
    if (!out) return false;
    out << "# Nonlinear LLT wing polar (" << label << ")\n";
    out << "# root=" << rootFoil << " tip=" << tipFoil << " Re_ref=" << Re_ref << " Ncrit=" << ncrit
        << " b_m=" << b_m << " AR=" << AR << " taper=" << taper << " Npoints=" << N
        << " blendStart=" << blendStart << " blendEnd=" << blendEnd
        << " aMin=" << aMin << " aMax=" << aMax << " aStep=" << aStep
        << " stallCap=" << (mp.stallCap ? 1 : 0)
        << " mode=" << ((mp.mode==Mode::GLIDER) ? "GLIDER" : "ROTOR")
        << " omega_rad_s=" << mp.omega_rad_s << " hubFrac=" << mp.hubFrac << " rRefFrac=" << mp.rRefFrac
        << " debugFlags=" << debugFlags << "\n";
    out << "alpha  CL  CD_profile  CDi  CD_total  L_over_D  iters  converged  Note\n";
    out << fixed << setprecision(6);
    for (const auto& p : pol) {
        out << setw(8) << p.alpha_deg << "  " << setw(10) << p.CL << "  " << setw(12) << p.CDp << "  "
            << setw(12) << p.CDi << "  " << setw(12) << p.CD << "  " << setw(12) << p.LD << "  "
            << setw(5) << p.iters << "  " << (p.converged ? 1 : 0) << "  " << p.note << "\n";
    }
    return true;
}

static bool write_station_dump(const string& outPath, const string& label, const string& rootFoil, const string& tipFoil,
    double Re_ref, int ncrit, double b_m, double AR, double taper, int N, double blendStart, double blendEnd,
    double alpha_deg, const ModeParams& mp, const LLTResult& res, const vector<StationDumpRow>& dump, int debugFlags)
{
    ofstream out(outPath, ios::trunc);
    if (!out) return false;
    out << "# LLT station dump (" << label << ")\n";
    out << "# root=" << rootFoil << " tip=" << tipFoil << " Re_ref=" << Re_ref << " Ncrit=" << ncrit
        << " b_m=" << b_m << " AR=" << AR << " taper=" << taper << " Npoints=" << N
        << " blendStart=" << blendStart << " blendEnd=" << blendEnd << " alpha_geom=" << alpha_deg
        << " mode=" << ((mp.mode==Mode::GLIDER) ? "GLIDER" : "ROTOR")
        << " omega_rad_s=" << mp.omega_rad_s << " hubFrac=" << mp.hubFrac << " rRefFrac=" << mp.rRefFrac
        << " stallCap=" << (mp.stallCap ? 1 : 0)
        << " debugFlags=" << debugFlags << "\n";
    out << "# Result: CL=" << res.CL << " CDp=" << res.CDp << " CDi=" << res.CDi
        << " CD=" << res.CD << " L/D=" << res.LD << " iters=" << res.iters << " converged=" << (res.converged ? 1 : 0)
        << " note=" << res.note << "\n";

    out << "i  theta_rad  yFracHalfSpan  yAbs_m  chord_m  wTip  Re_local  "
           "alpha_i_deg  alpha_eff_raw_deg  alpha_eff_deg  minWindow  maxWindow  clamp_delta  identity_error  "
           "ReLoRoot  ReHiRoot  ReLoTip  ReHiTip  cl_blend  cd_blend\n";

    out << fixed << setprecision(8);
    for (const auto& d : dump) {
        out << setw(2)  << d.i << "  "
            << setw(9)  << d.theta << "  "
            << setw(12) << d.yFrac << "  "
            << setw(10) << d.yAbs  << "  "
            << setw(10) << d.chord << "  "
            << setw(6)  << d.wTip  << "  "
            << setw(10) << d.Re_local << "  "
            << setw(12) << d.alpha_i_deg << "  "
            << setw(16) << d.alpha_eff_raw_deg << "  "
            << setw(12) << d.alpha_eff_deg << "  "
            << setw(10) << d.minWindow << "  "
            << setw(10) << d.maxWindow << "  "
            << setw(11) << d.clamp_delta << "  "
            << setw(14) << d.identity_error << "  "
            << setw(8)  << d.ReLoRoot << "  "
            << setw(8)  << d.ReHiRoot << "  "
            << setw(7)  << d.ReLoTip  << "  "
            << setw(7)  << d.ReHiTip  << "  "
            << setw(10) << d.cl << "  "
            << setw(10) << d.cd << "\n";
    }
    return true;
}

static string q(const string& s) {
    string out = "\"";
    for (char c : s) out += (c == '\"') ? '\'' : c;
    out += "\"";
    return out;
}

// -------------------- main --------------------

int main(int argc, char** argv) {
    //const string DEFAULT_MEGA_PATH = "outpolars\\Currentmegapolar.dat";
    const string DEFAULT_MEGA_PATH = "outpolars\\megapolar.dat";
    //const string DEFAULT_MEGA_PATH = "test_polar_output.dat";
    const int    DEFAULT_NCRIT     = 5;
    const string OUT_DIR = "wingAnalysis";
    string megaPath = DEFAULT_MEGA_PATH;

    // --- defaults ---
    int Re_glider = 100000;
    int Re_rotor  = 100000;
    string gliderRoot = "NACA_2208";
    string gliderTip  = "NACA_2208";
    string rotorRoot  = "NACA_2208";
    string rotorTip   = "NACA_2208";

    double b_m = 1.5;
    double AR = 6.0;
    double taper = 1;
    int Npoints = 45;
    double blendStart = 0.3;
    double blendEnd   = 0.8;

    double aMin = -10.0;
    double aMax = 20.0;
    double aStep = 0.25;

    const int    MAX_ITER = 250;
    const double RELAX    = 0.15;
    const double TOL_A    = 2e-5;

    double omega_rad_s = 300.0;
    double hubFrac = 0.10;
    double rRefFrac = 0.75;

    // debugFlags bitmask
    int debugFlags = 0;

    const string PY_CMD = "python";
    const string PY_SCRIPT = "llt_plot.py";

    // -------- parse args (minimal) --------
    int argi = 1;
    if (argc >= 2 && !looks_like_int(argv[1])) {
        megaPath = argv[1];
        argi = 2;
    }

    auto parse_next_int = [&](int& dst, const char* name) -> bool {
        if (argi >= argc) return true;
        int v = 0;
        if (!parse_int_strict(argv[argi], v)) { cout << "ERROR: " << name << " must be int.\n"; return false; }
        dst = v; argi++; return true;
    };
    auto parse_next_double = [&](double& dst, const char* name) -> bool {
        if (argi >= argc) return true;
        double v = 0.0;
        if (!parse_double_strict(argv[argi], v) || !is_finite(v)) { cout << "ERROR: " << name << " must be finite.\n"; return false; }
        dst = v; argi++; return true;
    };
    auto parse_next_string = [&](string& dst) -> bool {
        if (argi >= argc) return true;
        dst = argv[argi]; argi++; return true;
    };

    if (!parse_next_int(Re_glider, "Re_glider")) return 2;
    parse_next_string(gliderRoot);
    parse_next_string(gliderTip);

    if (!parse_next_int(Re_rotor, "Re_rotor")) return 2;
    parse_next_string(rotorRoot);
    parse_next_string(rotorTip);

    if (!parse_next_double(b_m, "b_m")) return 2;
    if (!parse_next_double(AR, "AR")) return 2;
    if (!parse_next_double(taper, "taper")) return 2;
    if (!parse_next_int(Npoints, "Npoints")) return 2;
    if (!parse_next_double(blendStart, "blendStart")) return 2;
    if (!parse_next_double(blendEnd, "blendEnd")) return 2;
    if (!parse_next_double(aMin, "aMin")) return 2;
    if (!parse_next_double(aMax, "aMax")) return 2;
    if (!parse_next_double(aStep, "aStep")) return 2;

    parse_next_double(omega_rad_s, "omega_rad_s");
    parse_next_double(hubFrac, "hubFrac");
    parse_next_double(rRefFrac, "rRefFrac");
    parse_next_int(debugFlags, "debugFlags");

    if (aMax < aMin) std::swap(aMin, aMax);

    gliderRoot = normalize_airfoil_key(gliderRoot);
    gliderTip  = normalize_airfoil_key(gliderTip);
    rotorRoot  = normalize_airfoil_key(rotorRoot);
    rotorTip   = normalize_airfoil_key(rotorTip);

    if (!(b_m > 0.01 && b_m < 1000.0)) { cout << "ERROR: b_m out of range.\n"; return 2; }
    if (!(AR > 0.2 && AR < 200.0)) { cout << "ERROR: AR out of range.\n"; return 2; }
    if (!(taper > 0.05 && taper <= 1.5)) { cout << "ERROR: taper out of range.\n"; return 2; }
    if (Npoints < 7 || Npoints > 120) { cout << "ERROR: Npoints out of range.\n"; return 2; }
    if (!(aStep > 0.0 && aStep <= 10.0)) { cout << "ERROR: aStep must be positive.\n"; return 2; }
    if (!(blendStart >= 0.0 && blendStart <= 1.0 && blendEnd >= 0.0 && blendEnd <= 1.0)) { cout << "ERROR: blend in [0,1].\n"; return 2; }
    hubFrac = clampd(hubFrac, 0.0, 0.95);
    rRefFrac = clampd(rRefFrac, 0.05, 1.0);

#if __has_include(<filesystem>)
    {
        std::error_code ec;
        fs::create_directories(OUT_DIR, ec);
        if (ec) { cout << "ERROR: could not create output folder: " << OUT_DIR << "\n"; return 2; }
    }
#endif

    ifstream in(megaPath);
    if (!in) { cout << "ERROR: could not open megapolar file: " << megaPath << "\n"; return 2; }

    unordered_set<string> neededFoils = { gliderRoot, gliderTip, rotorRoot, rotorTip };
    PolarDB db;
    string line;
    while (getline(in, line)) {
        Row r;
        if (!parse_megapolar_line(line, r)) continue;
        if (!passes_sanity_filters(r)) continue;
        if (r.Ncrit != DEFAULT_NCRIT) continue;
        if (neededFoils.find(r.airfoil) == neededFoils.end()) continue;
        db.add_point(r.airfoil, r.Re, r.alpha, r.CL, r.CD);
    }
    db.finalize_all();
    for (const auto& f : neededFoils) {
        if (!db.has_foil(f)) { cout << "ERROR: foil not found in megapolar (after Ncrit filter): " << f << "\n"; return 2; }
    }

    cout << "Loaded polars for 4 foils at Ncrit=" << DEFAULT_NCRIT << "\n";
    cout << "b_m=" << b_m << "  AR=" << AR << " taper=" << taper << "\n";
    cout << "Glider: root=" << gliderRoot << " tip=" << gliderTip << " Re_ref=" << Re_glider << "\n";
    cout << "Rotor : root=" << rotorRoot  << " tip=" << rotorTip  << " Re_ref=" << Re_rotor
         << " omega=" << omega_rad_s << " hubFrac=" << hubFrac << " rRefFrac=" << rRefFrac << "\n";
    cout << "debugFlags=" << debugFlags << " (1:disableClamp 2:freezeRotorRe 4:disableStallCap)\n";

    auto run_case = [&](const string& label, const string& rootFoil, const string& tipFoil, const ModeParams& mpBase) {
        vector<LLTResult> wing;
        wing.reserve((size_t)ceil((aMax - aMin) / aStep) + 2);

        // store Afinal for each alpha so best-alpha dump uses the matching solution
        vector<vector<double>> Afinal_list;
        vector<char> hasA;
        const int nSteps = (int)floor((aMax - aMin) / aStep + 0.5) + 3;
        Afinal_list.resize((size_t)nSteps);
        hasA.assign((size_t)nSteps, 0);

        vector<double> Aprev;

        int idx = 0;
        for (double a = aMin; a <= aMax + 1e-12; a += aStep, ++idx) {
            vector<double> Afinal;
            LLTResult r = solve_llt_nonlinear_one_alpha_twofoil_mode(
                db, rootFoil, tipFoil, mpBase, a, b_m, AR, taper, Npoints, blendStart, blendEnd,
                MAX_ITER, RELAX, TOL_A, debugFlags, (Aprev.empty() ? nullptr : &Aprev), &Afinal, nullptr
            );
            if (r.converged) {
                Aprev = Afinal;
                if ((size_t)idx < Afinal_list.size()) {
                    Afinal_list[(size_t)idx] = Afinal;
                    hasA[(size_t)idx] = 1;
                }
            }
            wing.push_back(r);
        }

        string OUT_POLAR = OUT_DIR + "\\" + label + "_wingpolar_llt.dat";
        if (!write_wingpolar_llt(OUT_POLAR, label, rootFoil, tipFoil, mpBase.Re_ref, DEFAULT_NCRIT,
                                 b_m, AR, taper, Npoints, blendStart, blendEnd, aMin, aMax, aStep, mpBase, wing, debugFlags)) {
            cout << "ERROR: failed to write: " << OUT_POLAR << "\n"; return;
        }

        double bestLD = -1e300; double bestA  = 0.0; int bestIdx = -1;
        for (int i = 0; i < (int)wing.size(); ++i) {
            if (!wing[i].converged) continue;
            if (wing[i].LD > bestLD) { bestLD = wing[i].LD; bestA = wing[i].alpha_deg; bestIdx = i; }
        }

        cout << label << " bestIdx=" << bestIdx
             << " bestA=" << bestA
             << " bestLD=" << bestLD
             << " converged=" << ((bestIdx>=0)? wing[bestIdx].converged : 0)
             << " note=" << ((bestIdx>=0)? wing[bestIdx].note : string("NONE")) << "\n";

        string OUT_DUMP = OUT_DIR + "\\" + label + "_llt_bestalpha_stations.dat";
        if (bestIdx >= 0) {
            vector<StationDumpRow> dump;
            vector<double> Afinal;

            const vector<double>* seedA = nullptr;
            if (bestIdx >= 0 && bestIdx < (int)hasA.size() && hasA[(size_t)bestIdx]) {
                seedA = &Afinal_list[(size_t)bestIdx];
            }

            LLTResult bestRes = solve_llt_nonlinear_one_alpha_twofoil_mode(
                db, rootFoil, tipFoil, mpBase, bestA, b_m, AR, taper, Npoints, blendStart, blendEnd,
                MAX_ITER, RELAX, TOL_A, debugFlags, seedA, &Afinal, &dump
            );

            if (!write_station_dump(OUT_DUMP, label, rootFoil, tipFoil, mpBase.Re_ref, DEFAULT_NCRIT,
                                    b_m, AR, taper, Npoints, blendStart, blendEnd, bestA, mpBase, bestRes, dump, debugFlags)) {
                cout << "WARNING: failed to write station dump: " << OUT_DUMP << "\n";
            }
        }

        cout << "Wrote: " << OUT_POLAR << "\n";
        if (bestIdx >= 0) {
            cout << "Wrote: " << OUT_DUMP << "\n";
            string OUT_PNG = OUT_DIR + "\\" + label + "_lltPlot.png";
            string cmd = PY_CMD + string(" ") + q(PY_SCRIPT) + " " + q(OUT_POLAR) + " " + q(OUT_DUMP) + " " + q(OUT_PNG);
            int rc = system(cmd.c_str());
            if (rc != 0) cout << "NOTE: python plotter failed (rc=" << rc << ").\n";
            else cout << "Wrote: " << OUT_PNG << "\n";
        } else {
            cout << label << ": No converged points found.\n";
        }
    };

    ModeParams mpG; mpG.mode = Mode::GLIDER; mpG.Re_ref = (double)Re_glider; mpG.stallCap = true;
    ModeParams mpR; mpR.mode = Mode::ROTOR;  mpR.Re_ref = (double)Re_rotor;  mpR.stallCap = true;
    mpR.omega_rad_s = omega_rad_s; mpR.hubFrac = hubFrac; mpR.rRefFrac = rRefFrac;

    run_case("glider", gliderRoot, gliderTip, mpG);
    run_case("rotor",  rotorRoot,  rotorTip,  mpR);

    return 0;
}
