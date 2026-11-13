// wingpolar_llt_dualmode.cpp
//
// Nonlinear LLT postprocess using megapolar.dat
// Adds:
//  (1) spanwise Reynolds sampling (Re(y) per station)
//  (2) simple stall-cap (clamped to cl_max/cl_min of the 2D polar)
//  (3) dual-mode run: GLIDER mode + ROTOR mode (Re(y) from Omega*r)
//
// FIX 4.2 ("MEGA SWEEP" DRIVER + KEEP LLT CORE SAME):
//  - Implements a "big mega" batch runner over a provided airfoil list.
//  - Runs BOTH modes (glider + rotor) and evaluates ALL (root, tip) combinations.
//  - Reads megapolar ONCE, loads polars for ALL foils in your list (filtered by Ncrit).
//  - Writes compact CSV summaries (best L/D, best alpha, etc.) for each mode.
//  - Optionally can write full per-combo polar+dump+png, but defaults OFF to avoid file explosion.
//
// NOTE (DNH): This is purely algorithmic batch-driving + data management; it does NOT try to
// force absolute agreement with VLM. It's meant for RELATIVE ranking.
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

// -------------------- airfoil list you provided --------------------

static vector<string> build_airfoil_list() {
    return {
        // "NACA 0004","NACA 0006","NACA 0008","NACA 0010","NACA 0012","NACA 0015","NACA 0018","NACA 0021","NACA 0024",

        // "NACA 1208","NACA 1209","NACA 1212","NACA 1408","NACA 1409","NACA 1410","NACA 1412","NACA 1415","NACA 1512","NACA 1608","NACA 1609","NACA 1612",
        // "NACA 2208","NACA 2209","NACA 2212","NACA 2408","NACA 2409","NACA 2410","NACA 2412","NACA 2415","NACA 2512","NACA 2608","NACA 2609","NACA 2612",
        // "NACA 3208","NACA 3209","NACA 3212","NACA 3408","NACA 3409","NACA 3410","NACA 3412","NACA 3415","NACA 3512","NACA 3608","NACA 3609","NACA 3612",
        "NACA 4208","NACA 4209","NACA 4212","NACA 4408","NACA 4409","NACA 4410","NACA 4412","NACA 4415","NACA 4512","NACA 4608","NACA 4609","NACA 4612",
        "NACA 5208","NACA 5209","NACA 5212","NACA 5408","NACA 5409","NACA 5410","NACA 5412","NACA 5415","NACA 5512","NACA 5608","NACA 5609","NACA 5612",
        "NACA 6208","NACA 6209","NACA 6212","NACA 6408","NACA 6409","NACA 6410","NACA 6412","NACA 6415","NACA 6512","NACA 6608","NACA 6609","NACA 6612",
        // "NACA 7208","NACA 7209","NACA 7212","NACA 7408","NACA 7409","NACA 7410","NACA 7412","NACA 7415","NACA 7512","NACA 7608","NACA 7609","NACA 7612",
        // "NACA 8208","NACA 8209","NACA 8212","NACA 8408","NACA 8409","NACA 8410","NACA 8412","NACA 8415","NACA 8512","NACA 8608","NACA 8609","NACA 8612",
        // "NACA 9208","NACA 9209","NACA 9212","NACA 9408","NACA 9409","NACA 9410","NACA 9412","NACA 9415","NACA 9512","NACA 9608","NACA 9609","NACA 9612"
    };
}

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
    vector<double> a, cl, cd;

    bool hasExtrema = false;
    double clMax = 0.0, aAtClMax = 0.0;
    double clMin = 0.0, aAtClMin = 0.0;

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

    bool sample_clamped(double alphaDeg, double& outCL, double& outCD, bool stallCap) const {
        if (a.empty()) return false;
        const double CLAMP_W_DEG = 0.0; // keep same behavior
        double aMin = alpha_min(), aMax = alpha_max();
        double aUse = smooth_clamp(alphaDeg, aMin, aMax, CLAMP_W_DEG);
        if (stallCap) return sample_with_stallcap(aUse, outCL, outCD);
        else          return sample_linear(aUse, outCL, outCD);
    }
};

// -------------------- Polar DB (vector-indexed Re tables) --------------------

struct FoilPolars {
    vector<int> ReVals;             // sorted
    vector<PolarCurve> curves;      // same length as ReVals
    unordered_map<int, PolarCurve> buildMap; // build-time only
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

    bool sample(const string& foil, double desiredRe, double alphaDeg,
                double& outCL, double& outCD, bool stallCap) const {
        int idxLo=-1, idxHi=-1, ReLo=-1, ReHi=-1;
        if (!bracket_Re_for_foil_idx(foil, desiredRe, idxLo, idxHi, ReLo, ReHi)) return false;
        double t = interp_weight_logRe(desiredRe, ReLo, ReHi);
        return sample_bracketed_idx(foil, idxLo, idxHi, ReLo, ReHi, t, alphaDeg, outCL, outCD, stallCap);
    }

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

// -------------------- LLT solve (same behavior, optimized internals) --------------------

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
            for (int j = k + 1; j < N; ++j) A[i*N + j] -= f * A[k*N + j];
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

        if (mp.mode == Mode::GLIDER) return mp.Re_ref * chordRatio;

        if (FREEZE_ROTOR_RE) {
            return mp.Re_ref * chordRatio;
        } else {
            double rEff = max(yAbs[i], mp.hubFrac * R);
            double rRef = max(1e-6, mp.rRefFrac * R);
            return mp.Re_ref * (rEff / rRef) * chordRatio;
        }
    };

    vector<StationPre> pre(N);
    for (int i = 0; i < N; ++i) {
        pre[i].wTip = tip_weight(yFrac[i], blendStart, blendEnd);
        pre[i].ReLoc = Re_local_at_station(i);

        if (!db.bracket_Re_for_foil_idx(rootFoil, pre[i].ReLoc, pre[i].idxLoR, pre[i].idxHiR, pre[i].ReLoR, pre[i].ReHiR)) {
            res.note = "ROOT_BRACKET_FAIL"; return res;
        }
        pre[i].tR = PolarDB::interp_weight_logRe(pre[i].ReLoc, pre[i].ReLoR, pre[i].ReHiR);

        if (!db.bracket_Re_for_foil_idx(tipFoil, pre[i].ReLoc, pre[i].idxLoT, pre[i].idxHiT, pre[i].ReLoT, pre[i].ReHiT)) {
            res.note = "TIP_BRACKET_FAIL"; return res;
        }
        pre[i].tT = PolarDB::interp_weight_logRe(pre[i].ReLoc, pre[i].ReLoT, pre[i].ReHiT);

        double minR=0, maxR=0, minT=0, maxT=0;
        bool okR = db.get_interpolated_range(rootFoil, pre[i].ReLoc, minR, maxR);
        bool okT = db.get_interpolated_range(tipFoil,  pre[i].ReLoc, minT, maxT);
        if (!okR) { minR = -90; maxR = 90; }
        if (!okT) { minT = -90; maxT = 90; }

        pre[i].minWindow = (1.0 - pre[i].wTip) * minR + pre[i].wTip * minT;
        pre[i].maxWindow = (1.0 - pre[i].wTip) * maxR + pre[i].wTip * maxT;
    }

    vector<double> A(N, 0.0), Anew(N, 0.0), RHS(N, 0.0);
    if (Ainit && (int)Ainit->size() == N) A = *Ainit;

    vector<double> M(N * N, 0.0);
    vector<double> nSin(N * N, 0.0);
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            int n = j + 1;
            double s = sin((double)n * theta[i]);
            M[i*N + j] = s;
            nSin[i*N + j] = (double)n * s;
        }
    }

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

static string safe_name(string s) {
    // for filenames (already normalized keys use underscores; keep it simple)
    for (char& c : s) {
        if (!(isalnum((unsigned char)c) || c=='_' || c=='-' || c=='.')) c = '_';
    }
    return s;
}

// -------------------- MEGA SWEEP helpers --------------------

struct ComboSummary {
    string rootFoil;
    string tipFoil;
    double bestLD = -1e300;
    double bestA  = 0.0;
    double bestCL = 0.0;
    double bestCD = 0.0;
    int bestIters = 0;
    int nConverged = 0;
    int nTotal = 0;
    string bestNote = "NONE";
};

// Runs alpha sweep for one (root, tip) pair.
// By default: no per-combo files (to avoid explosion).
static ComboSummary eval_combo(
    const PolarDB& db,
    const string& rootFoil,
    const string& tipFoil,
    const string& labelPrefix,
    const ModeParams& mpBase,
    double b_m, double AR, double taper, int Npoints,
    double blendStart, double blendEnd,
    double aMin, double aMax, double aStep,
    int MAX_ITER, double RELAX, double TOL_A,
    int debugFlags,
    bool writeComboFiles,
    const string& OUT_DIR,
    const string& PY_CMD,
    const string& PY_SCRIPT
) {
    ComboSummary s;
    s.rootFoil = rootFoil;
    s.tipFoil  = tipFoil;

    vector<LLTResult> wing;
    wing.reserve((size_t)ceil((aMax - aMin) / aStep) + 4);

    vector<vector<double>> Afinal_list;
    vector<char> hasA;
    const int nSteps = (int)floor((aMax - aMin) / aStep + 0.5) + 5;
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

        s.nTotal++;
        if (r.converged) {
            s.nConverged++;
            Aprev = Afinal;
            if ((size_t)idx < Afinal_list.size()) {
                Afinal_list[(size_t)idx] = Afinal;
                hasA[(size_t)idx] = 1;
            }
            if (r.LD > s.bestLD) {
                s.bestLD = r.LD;
                s.bestA  = r.alpha_deg;
                s.bestCL = r.CL;
                s.bestCD = r.CD;
                s.bestIters = r.iters;
                s.bestNote = r.note;
            }
        }
        wing.push_back(r);
    }

    if (!writeComboFiles) return s;

    // If enabled: write one polar + best-alpha dump + png for THIS combo.
    // WARNING: This will create a lot of files if enabled for all combos.
    string comboTag = safe_name(rootFoil) + "_to_" + safe_name(tipFoil);
    string label = labelPrefix + "_" + comboTag;

    string OUT_POLAR = OUT_DIR + "\\" + label + "_wingpolar_llt.dat";
    if (!write_wingpolar_llt(OUT_POLAR, label, rootFoil, tipFoil, mpBase.Re_ref, /*ncrit*/5,
                             b_m, AR, taper, Npoints, blendStart, blendEnd, aMin, aMax, aStep, mpBase, wing, debugFlags)) {
        // ignore write failures in mega mode; summary still returned
        return s;
    }

    if (s.nConverged <= 0) return s;

    int bestIdx = (int)llround((s.bestA - aMin) / aStep);
    bestIdx = max(0, min(bestIdx, (int)hasA.size()-1));

    vector<StationDumpRow> dump;
    vector<double> Afinal;
    const vector<double>* seedA = (hasA[(size_t)bestIdx] ? &Afinal_list[(size_t)bestIdx] : nullptr);

    LLTResult bestRes = solve_llt_nonlinear_one_alpha_twofoil_mode(
        db, rootFoil, tipFoil, mpBase, s.bestA, b_m, AR, taper, Npoints, blendStart, blendEnd,
        MAX_ITER, RELAX, TOL_A, debugFlags, seedA, &Afinal, &dump
    );

    string OUT_DUMP = OUT_DIR + "\\" + label + "_llt_bestalpha_stations.dat";
    write_station_dump(OUT_DUMP, label, rootFoil, tipFoil, mpBase.Re_ref, /*ncrit*/5,
                       b_m, AR, taper, Npoints, blendStart, blendEnd, s.bestA, mpBase, bestRes, dump, debugFlags);

    string OUT_PNG = OUT_DIR + "\\" + label + "_lltPlot.png";
    string cmd = PY_CMD + string(" ") + q(PY_SCRIPT) + " " + q(OUT_POLAR) + " " + q(OUT_DUMP) + " " + q(OUT_PNG);
    (void)system(cmd.c_str());

    return s;
}

static bool write_summary_csv(const string& path, const string& modeLabel,
                              double b_m, double AR, double taper, int Npoints,
                              double blendStart, double blendEnd,
                              double aMin, double aMax, double aStep,
                              const ModeParams& mp,
                              int debugFlags,
                              const vector<ComboSummary>& rows) {
    ofstream out(path, ios::trunc);
    if (!out) return false;

    out << "# mode=" << modeLabel
        << " b_m=" << b_m
        << " AR=" << AR
        << " taper=" << taper
        << " Npoints=" << Npoints
        << " blendStart=" << blendStart
        << " blendEnd=" << blendEnd
        << " aMin=" << aMin
        << " aMax=" << aMax
        << " aStep=" << aStep
        << " Re_ref=" << mp.Re_ref
        << " omega_rad_s=" << mp.omega_rad_s
        << " hubFrac=" << mp.hubFrac
        << " rRefFrac=" << mp.rRefFrac
        << " debugFlags=" << debugFlags
        << "\n";

    out << "rootFoil,tipFoil,bestLD,bestAlphaDeg,bestCL,bestCD,bestIters,nConverged,nTotal,convergedFrac,bestNote\n";
    out << fixed << setprecision(8);

    for (const auto& r : rows) {
        double frac = (r.nTotal > 0) ? ((double)r.nConverged / (double)r.nTotal) : 0.0;
        out << r.rootFoil << ","
            << r.tipFoil << ","
            << r.bestLD << ","
            << r.bestA << ","
            << r.bestCL << ","
            << r.bestCD << ","
            << r.bestIters << ","
            << r.nConverged << ","
            << r.nTotal << ","
            << frac << ","
            << r.bestNote
            << "\n";
    }
    return true;
}

// -------------------- main --------------------

int main(int argc, char** argv) {
    const string DEFAULT_MEGA_PATH = "outpolars\\Currentmegapolar.dat";
    const int    DEFAULT_NCRIT     = 5;
    const string OUT_DIR = "wingAnalysis";
    string megaPath = DEFAULT_MEGA_PATH;

    // --- defaults (same as before) ---
    int Re_glider = 80000;
    int Re_rotor  = 80000;

    double b_m = 0.60;
    double AR = 6.0;
    double taper = 0.7;
    int Npoints = 45;
    double blendStart = 0.3;
    double blendEnd   = 0.8;

    double aMin = -10.0;
    double aMax = 15.0;
    double aStep = 0.25;

    const int    MAX_ITER = 250;
    const double RELAX    = 0.15;
    const double TOL_A    = 2e-5;

    double omega_rad_s = 300.0;
    double hubFrac = 0.10;
    double rRefFrac = 0.75;

    int debugFlags = 0;

    // MEGA SWEEP toggles
    //  - writeComboFiles = 0 keeps outputs small (just summaries).
    //  - if set to 1, it will write polar+dump+png for EVERY combo (tons of files).
    int writeComboFiles = 0;

    // If you only want a subset for testing, you can cap the show/hard-work.
    // maxFoils <= 0 means use the full list.
    int maxFoils = 0;

    const string PY_CMD = "python";
    const string PY_SCRIPT = "llt_plot.py";

    // -------- parse args (minimal, backwards-friendly) --------
    // You can call:
    //   ./wingpolar_llt_dualmode [megapolarPath] Re_glider Re_rotor b_m AR taper Npoints blendStart blendEnd aMin aMax aStep omega hubFrac rRefFrac debugFlags writeComboFiles maxFoils
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

    if (!parse_next_int(Re_glider, "Re_glider")) return 2;
    if (!parse_next_int(Re_rotor,  "Re_rotor"))  return 2;

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
    parse_next_int(writeComboFiles, "writeComboFiles");
    parse_next_int(maxFoils, "maxFoils");

    if (aMax < aMin) std::swap(aMin, aMax);

    if (!(b_m > 0.01 && b_m < 1000.0)) { cout << "ERROR: b_m out of range.\n"; return 2; }
    if (!(AR > 0.2 && AR < 200.0)) { cout << "ERROR: AR out of range.\n"; return 2; }
    if (!(taper > 0.05 && taper <= 1.5)) { cout << "ERROR: taper out of range.\n"; return 2; }
    if (Npoints < 7 || Npoints > 160) { cout << "ERROR: Npoints out of range.\n"; return 2; }
    if (!(aStep > 0.0 && aStep <= 10.0)) { cout << "ERROR: aStep must be positive.\n"; return 2; }
    if (!(blendStart >= 0.0 && blendStart <= 1.0 && blendEnd >= 0.0 && blendEnd <= 1.0)) { cout << "ERROR: blend in [0,1].\n"; return 2; }

    hubFrac = clampd(hubFrac, 0.0, 0.95);
    rRefFrac = clampd(rRefFrac, 0.05, 1.0);
    writeComboFiles = (writeComboFiles != 0) ? 1 : 0;

#if __has_include(<filesystem>)
    {
        std::error_code ec;
        fs::create_directories(OUT_DIR, ec);
        if (ec) { cout << "ERROR: could not create output folder: " << OUT_DIR << "\n"; return 2; }
    }
#endif

    // Build normalized airfoil list and set
    vector<string> foilsRaw = build_airfoil_list();
    vector<string> foils;
    foils.reserve(foilsRaw.size());
    for (const string& f : foilsRaw) foils.push_back(normalize_airfoil_key(f));

    // Optionally limit for quick testing
    if (maxFoils > 0 && (size_t)maxFoils < foils.size()) foils.resize((size_t)maxFoils);

    unordered_set<string> neededFoils;
    neededFoils.reserve(foils.size() * 2);
    for (const auto& f : foils) neededFoils.insert(f);

    ifstream in(megaPath);
    if (!in) { cout << "ERROR: could not open megapolar file: " << megaPath << "\n"; return 2; }

    PolarDB db;

    cout << "Loading megapolar once: " << megaPath << "\n";
    cout << "Target foils: " << foils.size() << " (Ncrit=" << DEFAULT_NCRIT << ")\n";

    string line;
    size_t linesRead = 0;
    size_t linesUsed = 0;

    while (getline(in, line)) {
        ++linesRead;
        Row r;
        if (!parse_megapolar_line(line, r)) continue;
        if (!passes_sanity_filters(r)) continue;
        if (r.Ncrit != DEFAULT_NCRIT) continue;
        if (neededFoils.find(r.airfoil) == neededFoils.end()) continue;
        db.add_point(r.airfoil, r.Re, r.alpha, r.CL, r.CD);
        ++linesUsed;
    }

    db.finalize_all();

    // Validate all foils exist
    vector<string> missing;
    for (const auto& f : foils) {
        if (!db.has_foil(f)) missing.push_back(f);
    }
    if (!missing.empty()) {
        cout << "ERROR: some foils were not found in megapolar (after Ncrit filter):\n";
        for (const auto& m : missing) cout << "  " << m << "\n";
        cout << "Fix megapolar coverage or adjust list.\n";
        return 2;
    }

    cout << "Megapolar parse done. linesRead=" << linesRead << " linesUsed=" << linesUsed << "\n";
    cout << "Geometry: b_m=" << b_m << " AR=" << AR << " taper=" << taper
         << " Npoints=" << Npoints << " blend=[" << blendStart << "," << blendEnd << "]\n";
    cout << "Alpha sweep: [" << aMin << "," << aMax << "] step=" << aStep << "\n";
    cout << "Glider: Re_ref=" << Re_glider << "\n";
    cout << "Rotor : Re_ref=" << Re_rotor << " omega=" << omega_rad_s << " hubFrac=" << hubFrac << " rRefFrac=" << rRefFrac << "\n";
    cout << "debugFlags=" << debugFlags << " (1:disableClamp 2:freezeRotorRe 4:disableStallCap)\n";
    cout << "writeComboFiles=" << writeComboFiles << " (WARNING: if 1, many files)\n";

    ModeParams mpG; mpG.mode = Mode::GLIDER; mpG.Re_ref = (double)Re_glider; mpG.stallCap = true;
    ModeParams mpR; mpR.mode = Mode::ROTOR;  mpR.Re_ref = (double)Re_rotor;  mpR.stallCap = true;
    mpR.omega_rad_s = omega_rad_s; mpR.hubFrac = hubFrac; mpR.rRefFrac = rRefFrac;

    // --------- MEGA combo loops ---------
    // We do ALL ordered pairs (root, tip). If you later want only i<=j or only i!=j, change here.
    const int nF = (int)foils.size();
    const long long totalCombos = (long long)nF * (long long)nF;

    cout << "Running combos: " << nF << " x " << nF << " = " << totalCombos << " per mode\n";

    vector<ComboSummary> gliderRows;
    vector<ComboSummary> rotorRows;
    gliderRows.reserve((size_t)totalCombos);
    rotorRows.reserve((size_t)totalCombos);

    // Simple progress print
    auto print_progress = [&](const string& modeName, long long k, long long total) {
        if (k <= 0) return;
        // print every ~1% or at least every 200
        long long step = max(200LL, total / 100LL);
        if (k % step == 0 || k == total) {
            double frac = (total > 0) ? (100.0 * (double)k / (double)total) : 0.0;
            cout << modeName << " progress: " << k << "/" << total << " (" << fixed << setprecision(1) << frac << "%)\n";
            cout.unsetf(std::ios::floatfield);
        }
    };

    // GLIDER mode sweep
    {
        long long k = 0;
        for (int i = 0; i < nF; ++i) {
            for (int j = 0; j < nF; ++j) {
                const string& root = foils[(size_t)i];
                const string& tip  = foils[(size_t)j];

                ComboSummary s = eval_combo(
                    db, root, tip, "glider", mpG,
                    b_m, AR, taper, Npoints, blendStart, blendEnd,
                    aMin, aMax, aStep,
                    MAX_ITER, RELAX, TOL_A,
                    debugFlags,
                    writeComboFiles != 0,
                    OUT_DIR, PY_CMD, PY_SCRIPT
                );
                gliderRows.push_back(std::move(s));
                ++k;
                print_progress("glider", k, totalCombos);
            }
        }
    }

    // ROTOR mode sweep
    {
        long long k = 0;
        for (int i = 0; i < nF; ++i) {
            for (int j = 0; j < nF; ++j) {
                const string& root = foils[(size_t)i];
                const string& tip  = foils[(size_t)j];

                ComboSummary s = eval_combo(
                    db, root, tip, "rotor", mpR,
                    b_m, AR, taper, Npoints, blendStart, blendEnd,
                    aMin, aMax, aStep,
                    MAX_ITER, RELAX, TOL_A,
                    debugFlags,
                    writeComboFiles != 0,
                    OUT_DIR, PY_CMD, PY_SCRIPT
                );
                rotorRows.push_back(std::move(s));
                ++k;
                print_progress("rotor", k, totalCombos);
            }
        }
    }

    // Write summary CSVs
    const string GL_CSV = OUT_DIR + "\\glider_combo_summary.csv";
    const string RO_CSV = OUT_DIR + "\\rotor_combo_summary.csv";

    if (!write_summary_csv(GL_CSV, "GLIDER", b_m, AR, taper, Npoints, blendStart, blendEnd, aMin, aMax, aStep, mpG, debugFlags, gliderRows)) {
        cout << "ERROR: failed to write " << GL_CSV << "\n";
    } else {
        cout << "Wrote: " << GL_CSV << "\n";
    }

    if (!write_summary_csv(RO_CSV, "ROTOR", b_m, AR, taper, Npoints, blendStart, blendEnd, aMin, aMax, aStep, mpR, debugFlags, rotorRows)) {
        cout << "ERROR: failed to write " << RO_CSV << "\n";
    } else {
        cout << "Wrote: " << RO_CSV << "\n";
    }

    cout << "DONE.\n";
    return 0;
}
