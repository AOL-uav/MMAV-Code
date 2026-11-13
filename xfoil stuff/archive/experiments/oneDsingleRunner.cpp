#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <windows.h>
#include <cmath>
#include <cctype>
#include <sstream>
#include <map>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <limits>

using namespace std;
namespace fs = std::filesystem;

// -------------------- GLOBAL toggles (set in main) --------------------
static bool g_enable_xfoil_plots      = true;

// NEW behavior:
// - We ALWAYS detect + tag FLAG(CL) (for debugging / plotting).
// - We ONLY PATCH if we see a *cluster* of flags (not a one-off).
static bool g_enable_cluster_patch    = true;

// -------------------- tiny file-op retry helpers (Windows file lock race fix) --------------------

static bool remove_file_retry(const string& path, int tries = 40, DWORD sleep_ms = 50) {
    std::error_code ec;
    for (int i = 0; i < tries; ++i) {
        ec.clear();
        if (!fs::exists(path, ec)) return true; // already gone
        fs::remove(path, ec);
        if (!ec) return true;
        Sleep(sleep_ms);
    }
    return false;
}

static bool copy_file_retry(const string& src, const string& dst, int tries = 40, DWORD sleep_ms = 50) {
    std::error_code ec;
    for (int i = 0; i < tries; ++i) {
        ec.clear();
        fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
        if (!ec) return true;
        Sleep(sleep_ms);
    }
    return false;
}

// forward decls
static bool append_polar_rows_with_note_retry(const string& srcPolar, const string& dstPolar, const string& note,
                                             int tries = 40, DWORD sleep_ms = 50);

// -------------------- polar parsing helpers --------------------
//
// We support TWO formats now:
//
// (A) Raw XFOIL line (no note):
//   alpha CL CD CDp CM [Top_Xtr Bot_Xtr]
//
// (B) Our master/cleaned line (note at end):
//   alpha CL CD CDp CM [Top_Xtr Bot_Xtr] NOTE
//
// NOTE is treated as a single token (no spaces). If you want multiple tags, join with '|'
// e.g. "XTR_ASYM(top=0.02,bot=0.30)|SHRINK"
//

static bool parse_polar_row_7_with_note(
    const string& line,
    double& a, double& cl, double& cd, double& cdp, double& cm,
    double& topXtr, double& botXtr,
    bool& hasXtr,
    string& noteOut
) {
    noteOut.clear();

    size_t i = 0;
    while (i < line.size() && isspace((unsigned char)line[i])) i++;
    if (i >= line.size()) return false;

    char c = line[i];
    if (!(c=='-' || c=='+' || c=='.' || (c>='0' && c<='9'))) return false;

    std::istringstream iss(line);
    if (!(iss >> a >> cl >> cd >> cdp >> cm)) return false;

    if (!std::isfinite(a) || !std::isfinite(cl) || !std::isfinite(cd) || !std::isfinite(cdp) || !std::isfinite(cm))
        return false;

    // CD sanity
    if (cd <= 0.0 || cd > 5.0) return false;

    // CL sanity
    if (fabs(cl) > 5.0) return false;

    // Try to read Xtr columns if present
    hasXtr = false;
    topXtr = botXtr = 0.0;

    streampos posBefore = iss.tellg();
    double tx=0.0, bx=0.0;

    if (iss >> tx >> bx) {
        if (std::isfinite(tx) && std::isfinite(bx)) {
            hasXtr = true;
            topXtr = tx;
            botXtr = bx;

            // optional note
            string maybeNote;
            if (iss >> maybeNote) noteOut = maybeNote;
            return true;
        }
    }

    // Rewind and try reading note only.
    iss.clear();
    iss.seekg(posBefore);

    string maybeNote;
    if (iss >> maybeNote) {
        noteOut = maybeNote;
    }

    return true;
}

static bool parse_polar_row_7(
    const string& line,
    double& a, double& cl, double& cd, double& cdp, double& cm,
    double& topXtr, double& botXtr,
    bool& hasXtr
) {
    string dummy;
    return parse_polar_row_7_with_note(line, a, cl, cd, cdp, cm, topXtr, botXtr, hasXtr, dummy);
}

// Best alpha in [lo,hi] from a polar file.
static bool get_best_alpha_in_range(const string& polarPath, double lo, double hi, bool wantMax, double& bestOut) {
    ifstream in(polarPath);
    if (!in) return false;

    const double aLo = std::min(lo, hi);
    const double aHi = std::max(lo, hi);

    string line;
    bool found = false;
    double bestA = 0.0;

    while (getline(in, line)) {
        double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
        bool hasXtr=false;
        string note;
        if (!parse_polar_row_7_with_note(line, a, cl, cd, cdp, cm, tx, bx, hasXtr, note)) continue;

        if (a < aLo - 1e-9 || a > aHi + 1e-9) continue;

        if (!found) { bestA = a; found = true; }
        else        { bestA = wantMax ? std::max(bestA, a) : std::min(bestA, a); }
    }

    if (!found) return false;
    bestOut = bestA;
    return true;
}

static bool file_exists_nonempty(const string& p) {
    std::error_code ec;
    return fs::exists(p, ec) && fs::file_size(p, ec) > 0;
}

// -------------------- master append with NOTE --------------------

static void write_master_header_if_needed(ofstream& out) {
    out << "  alpha        CL          CD         CDp         CM      Top_Xtr  Bot_Xtr  Note\n";
}

// Append parsed rows from srcPolar into dstPolar, but add NOTE to each line.
static bool append_polar_rows_with_note(const string& srcPolar, const string& dstPolar, const string& note) {
    ifstream in(srcPolar);
    if (!in) return false;

    bool dstExists = false;
    {
        std::error_code ec;
        dstExists = fs::exists(dstPolar, ec) && (fs::file_size(dstPolar, ec) > 0);
    }

    ofstream out(dstPolar, ios::app);
    if (!out) return false;

    if (!dstExists) {
        write_master_header_if_needed(out);
    }

    string line;
    while (getline(in, line)) {
        double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
        bool hasXtr=false;
        string ignored;
        if (!parse_polar_row_7_with_note(line, a, cl, cd, cdp, cm, tx, bx, hasXtr, ignored)) continue;

        out << std::fixed << std::setprecision(3)
            << std::setw(7) << a << "  "
            << std::setw(8) << std::setprecision(4) << cl << "  "
            << std::setw(9) << std::setprecision(5) << cd  << "  "
            << std::setw(9) << std::setprecision(5) << cdp << "  "
            << std::setw(8) << std::setprecision(4) << cm  << "  ";

        if (hasXtr) {
            out << std::setw(8) << std::setprecision(4) << tx << "  "
                << std::setw(8) << std::setprecision(4) << bx;
        } else {
            out << std::setw(8) << "NA" << "  " << std::setw(8) << "NA";
        }

        out << "  " << note << "\n";
    }

    return true;
}

static bool append_polar_rows_with_note_retry(const string& srcPolar, const string& dstPolar, const string& note,
                                             int tries, DWORD sleep_ms) {
    for (int i = 0; i < tries; ++i) {
        if (append_polar_rows_with_note(srcPolar, dstPolar, note)) return true;
        Sleep(sleep_ms);
    }
    return false;
}

// -------------------- cleanup: grid + dedupe + sort (now carries NOTE) --------------------

struct PolarRow {
    double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
    bool hasXtr=false;
    string note;
};

static long long grid_key(double a, double step) {
    return (long long) llround(a / step);
}

static bool note_has_token(const string& note, const string& tok) {
    return (note.find(tok) != string::npos);
}

static void note_add_tag(string& note, const string& tag) {
    if (tag.empty()) return;
    if (note.empty()) { note = tag; return; }
    if (note_has_token(note, tag)) return;
    note += "|" + tag;
}

// -------------------- NEW: "realistic" dupe selection helpers --------------------

static bool is_rescue_note(const string& note) {
    // "Rescue" points are more likely to be odd branches. We don't ban them, just penalize slightly.
    return note_has_token(note, "XTR_ASYM") || note_has_token(note, "HOP(");
}

static double continuity_score_one_sided(
    const PolarRow& cand,
    const PolarRow* prevSel // may be null
) {
    // Weighted absolute differences + tiny drag preference + rescue penalty
    const double wCL   = 10.0;
    const double wCDd  = 2.0;
    const double wCMd  = 1.0;
    const double wDrag = 0.2;
    const double wRes  = 1.0;

    double s = 0.0;

    if (prevSel) {
        s += wCL  * fabs(cand.cl - prevSel->cl);
        s += wCDd * fabs(cand.cd - prevSel->cd);
        s += wCMd * fabs(cand.cm - prevSel->cm);
    }

    s += wDrag * cand.cd;
    if (is_rescue_note(cand.note)) s += wRes;

    return s;
}

// If we have a baseline "next" row (not necessarily final-chosen), use a linear prediction.
static double continuity_score_two_sided(
    const PolarRow& cand,
    const PolarRow* prevSel,
    const PolarRow* nextBase
) {
    if (!prevSel || !nextBase) return continuity_score_one_sided(cand, prevSel);

    // predict mid as average (since uniform grid); compare to predicted.
    const double predCL = 0.5 * (prevSel->cl + nextBase->cl);
    const double predCD = 0.5 * (prevSel->cd + nextBase->cd);
    const double predCM = 0.5 * (prevSel->cm + nextBase->cm);

    const double wCL   = 12.0;
    const double wCDd  = 3.0;
    const double wCMd  = 1.2;
    const double wDrag = 0.15;
    const double wRes  = 1.0;

    double s = 0.0;
    s += wCL  * fabs(cand.cl - predCL);
    s += wCDd * fabs(cand.cd - predCD);
    s += wCMd * fabs(cand.cm - predCM);
    s += wDrag * cand.cd;
    if (is_rescue_note(cand.note)) s += wRes;

    return s;
}

// Pick a "baseline" representative in a bin for lookahead prediction.
// Use min-CD as a cheap baseline, but we do NOT output this directly unless it also wins the continuity pick.
static PolarRow pick_baseline_min_cd(const vector<PolarRow>& cands) {
    int best = 0;
    for (int i = 1; i < (int)cands.size(); ++i) {
        if (cands[i].cd < cands[best].cd) best = i;
    }
    return cands[best];
}

// Build final rows by scanning keys in order and choosing the candidate that best matches continuity.
static vector<PolarRow> select_realistic_dupes(
    const map<long long, vector<PolarRow>>& candsByKey,
    double step
) {
    vector<PolarRow> out;
    out.reserve(candsByKey.size());

    // Precompute a baseline per key for lookahead (min-CD baseline only for prediction).
    map<long long, PolarRow> baseline;
    for (const auto& kv : candsByKey) {
        baseline[kv.first] = pick_baseline_min_cd(kv.second);
    }

    const PolarRow* prevSel = nullptr;

    for (auto it = candsByKey.begin(); it != candsByKey.end(); ++it) {
        long long k = it->first;
        const auto& cands = it->second;

        // Snap alpha
        vector<PolarRow> local = cands;
        for (auto& r : local) {
            r.a = k * step;
            if (r.note.empty()) r.note = "BASE";
        }

        // Lookahead baseline for k+1 (if present and contiguous)
        const PolarRow* nextBase = nullptr;
        auto itNext = candsByKey.find(k + 1);
        if (itNext != candsByKey.end()) {
            nextBase = &baseline[k + 1];
        }

        int bestIdx = 0;
        double bestScore = std::numeric_limits<double>::infinity();

        for (int i = 0; i < (int)local.size(); ++i) {
            double s = continuity_score_two_sided(local[i], prevSel, nextBase);

            // tiny extra penalty if this candidate has missing Xtr but others have it (prefer consistency)
            // (very small; won't override continuity)
            if (!local[i].hasXtr) s += 0.05;

            if (s < bestScore) {
                bestScore = s;
                bestIdx = i;
            }
        }

        PolarRow chosen = local[bestIdx];

        // If there were duplicates, tag the chosen row so you can see it in the output
        if (local.size() > 1) {
            note_add_tag(chosen.note, "DUPE_PICK(CONT)");
            // also note how many candidates existed
            {
                std::ostringstream oss;
                oss << "DUPE_N=" << local.size();
                note_add_tag(chosen.note, oss.str());
            }
        }

        out.push_back(chosen);
        prevSel = &out.back();
    }

    return out;
}

// -------------------- smoothing / flags (unchanged except for using selected rows) --------------------

static bool should_flag_point(
    double y, double yhat,
    double slopeL, double slopeR,
    double absTol, double relTol,
    double slopeJumpTol
) {
    const double err = fabs(y - yhat);
    const double scale = std::max(1.0, fabs(yhat));
    const bool badResidual = (err > absTol) && (err > relTol * scale);

    const double jump = fabs(slopeR - slopeL);
    const bool badSlopeJump = (jump > slopeJumpTol);

    return (badResidual || badSlopeJump);
}

struct ClusterStats {
    int flaggedPoints = 0;
    int clusters      = 0;
    int clustersPatched = 0;
    int pointsPatched = 0;
};

// Find nearest index on the left/right that is NOT flagged, within a limited search window.
// Returns -1 if not found.
static int find_anchor_left(const vector<char>& flagged, int startIdx, int maxSearch) {
    for (int d = 1; d <= maxSearch; ++d) {
        int i = startIdx - d;
        if (i < 0) break;
        if (!flagged[i]) return i;
    }
    return -1;
}
static int find_anchor_right(const vector<char>& flagged, int endIdx, int n, int maxSearch) {
    for (int d = 1; d <= maxSearch; ++d) {
        int i = endIdx + d;
        if (i >= n) break;
        if (!flagged[i]) return i;
    }
    return -1;
}

// ---- Hermite smoothing helpers (better than linear) ----

static double hermite_interp(double t, double y0, double y1, double m0, double m1, double dx) {
    // Cubic Hermite basis
    double t2 = t * t;
    double t3 = t2 * t;
    double h00 =  2*t3 - 3*t2 + 1;
    double h10 =      t3 - 2*t2 + t;
    double h01 = -2*t3 + 3*t2;
    double h11 =      t3 -    t2;
    return h00*y0 + h10*m0*dx + h01*y1 + h11*m1*dx;
}

static double slope_at_CL(const vector<PolarRow>& rows, int i, double step) {
    const int n = (int)rows.size();
    if (i-1 >= 0 && i+1 < n) {
        return (rows[i+1].cl - rows[i-1].cl) / (2.0 * step);
    }
    if (i+1 < n) return (rows[i+1].cl - rows[i].cl) / step;
    if (i-1 >= 0) return (rows[i].cl - rows[i-1].cl) / step;
    return 0.0;
}

// Fill CL smoothly between anchors [L..R] (exclusive interior points).
static void hermite_fill_CL(vector<PolarRow>& rows, int L, int R, int wS, int wE, double step, ClusterStats& st, const vector<char>& flagged) {
    if (R <= L + 1) return;

    const double aL = rows[L].a;
    const double aR = rows[R].a;
    const double dx = aR - aL;
    if (!(dx > 0)) return;

    const double yL = rows[L].cl;
    const double yR = rows[R].cl;

    double mL = slope_at_CL(rows, L, step);
    double mR = slope_at_CL(rows, R, step);

    // ---- small stability improvement: limit endpoint slopes to avoid overshoot ----
    const double sec = (yR - yL) / dx;
    auto limit = [&](double m) {
        if (sec == 0.0) return 0.0;
        if (m * sec < 0.0) return 0.0;                  // opposite sign -> flatten
        const double maxMag = 3.0 * fabs(sec);          // safe limiter
        if (fabs(m) > maxMag) m = (m > 0 ? 1 : -1) * maxMag;
        return m;
    };
    mL = limit(mL);
    mR = limit(mR);

    note_add_tag(rows[L].note, "ANCHOR_L");
    note_add_tag(rows[R].note, "ANCHOR_R");

    for (int j = L + 1; j < R; ++j) {
        if (j < wS || j > wE) continue; // only overwrite inside window

        double t = (rows[j].a - aL) / dx; // 0..1
        double y = hermite_interp(t, yL, yR, mL, mR, dx);

        rows[j].cl = y;
        st.pointsPatched++;

        note_add_tag(rows[j].note, "SMOOTH(CL)");
        note_add_tag(rows[j].note, "PATCH(HERMITE)");
        note_add_tag(rows[j].note, "PATCHv1");   // explicit "patched version" tag
        if (flagged[j]) note_add_tag(rows[j].note, "PATCH(CLUSTER)");
        else            note_add_tag(rows[j].note, "FILL(CL)");
    }
}

// ---- simple monotonicity flag pass (catches small "dents") ----
static int flag_monotonic_dips(
    vector<PolarRow>& rows,
    vector<char>& flagged,
    double monoTol,            // allowable dip before flagging
    ClusterStats& st
) {
    const int n = (int)rows.size();
    if (n <= 1) return 0;

    int added = 0;
    double runningMax = rows[0].cl;

    for (int i = 1; i < n; ++i) {
        if (rows[i].cl < runningMax - monoTol) {
            if (!flagged[i]) {
                flagged[i] = 1;
                st.flaggedPoints++;
                added++;
                note_add_tag(rows[i].note, "FLAG(MONO)");
            }
        } else {
            runningMax = std::max(runningMax, rows[i].cl);
        }
    }
    return added;
}

// Core: detect CL flags, then patch only *clusters* (not isolated flags).
// Patch method: cubic Hermite interpolation between nearby unflagged anchors.
// Fallback: local neighbor averaging inside the window.
static ClusterStats detect_and_cluster_patch_CL(
    vector<PolarRow>& rows,
    double alphaStep
) {
    ClusterStats st{};
    const int n = (int)rows.size();
    if (n < 3) return st;

    // Tolerances tuned for 0.25 deg grid
    const double CL_absTol       = 0.06;
    const double CL_relTol       = 0.10;
    const double CL_slopeJumpTol = 0.35;

    // ---- 1) detect flags (no modification yet) ----
    vector<char> flagged(n, 0);

    for (int i = 1; i < n - 1; ++i) {
        const double a0 = rows[i-1].a;
        const double a1 = rows[i].a;
        const double a2 = rows[i+1].a;

        // uniform grid check
        if (fabs((a1 - a0) - alphaStep) > 1e-6) continue;
        if (fabs((a2 - a1) - alphaStep) > 1e-6) continue;

        const double y0 = rows[i-1].cl;
        const double y1 = rows[i].cl;
        const double y2 = rows[i+1].cl;

        const double yhat   = 0.5 * (y0 + y2);
        const double slopeL = (y1 - y0) / alphaStep;
        const double slopeR = (y2 - y1) / alphaStep;

        if (should_flag_point(y1, yhat, slopeL, slopeR, CL_absTol, CL_relTol, CL_slopeJumpTol)) {
            flagged[i] = 1;
            st.flaggedPoints++;
            note_add_tag(rows[i].note, "FLAG(CL)");
        }
    }

    // ---- 1b) extra simple flagger: monotonicity dents ----
    {
        const double MONO_TOL = 0.02; // tune: 0.01–0.04
        (void)flag_monotonic_dips(rows, flagged, MONO_TOL, st);
    }

    // ---- print detection summary ----
    if (st.flaggedPoints > 0) {
        cout << "[SMOOTH] Detected FLAG(CL/MONO) points: " << st.flaggedPoints
             << "  (cluster patch " << (g_enable_cluster_patch ? "ON" : "OFF") << ")\n";
    } else {
        cout << "[SMOOTH] No flags detected.\n";
        return st;
    }

    if (!g_enable_cluster_patch) return st;

    // ---- 2) build clusters of flags by proximity ----
    const int MIN_FLAGS_IN_CLUSTER  = 2;
    const int MERGE_DIST_STEPS      = 4;  // merge flags within 4 grid points (1.0 deg at 0.25 step)
    const int ANCHOR_SEARCH_STEPS   = 8;

    struct Cluster { int s, e, count; };
    vector<Cluster> clusters;

    int first = -1, last = -1, cnt = 0;

    for (int idx = 0; idx < n; ++idx) {
        if (!flagged[idx]) continue;

        if (first < 0) {
            first = last = idx;
            cnt = 1;
            continue;
        }

        if ((idx - last) <= MERGE_DIST_STEPS) {
            last = idx;
            cnt++;
        } else {
            clusters.push_back({first, last, cnt});
            first = last = idx;
            cnt = 1;
        }
    }
    if (first >= 0) clusters.push_back({first, last, cnt});

    // ---- 3) patch qualifying clusters ----
    for (auto& c : clusters) {
        if (c.count < MIN_FLAGS_IN_CLUSTER) continue;

        // Expand patch window by 1 on each side to blend
        int wS = std::max(0, c.s - 1);
        int wE = std::min(n - 1, c.e + 1);

        // find anchors outside cluster window
        int leftA  = find_anchor_left(flagged, wS, ANCHOR_SEARCH_STEPS);
        int rightA = find_anchor_right(flagged, wE, n, ANCHOR_SEARCH_STEPS);

        st.clusters++;
        st.clustersPatched++;

        if (leftA >= 0 && rightA >= 0 && leftA < rightA) {
            // Hermite interpolation between anchors (smooth slopes)
            hermite_fill_CL(rows, leftA, rightA, wS, wE, alphaStep, st, flagged);
        } else {
            // Fallback: local neighbor averaging inside [wS,wE]
            for (int pass = 0; pass < 2; ++pass) {
                for (int j = wS + 1; j <= wE - 1; ++j) {
                    double yhat = 0.5 * (rows[j-1].cl + rows[j+1].cl);
                    rows[j].cl = yhat;
                    st.pointsPatched++;
                    note_add_tag(rows[j].note, "SMOOTH(CL)");
                    note_add_tag(rows[j].note, "PATCH(AVG)");
                    note_add_tag(rows[j].note, "PATCHv1"); // explicit "patched version" tag
                    if (flagged[j]) note_add_tag(rows[j].note, "PATCH(CLUSTER)");
                    else            note_add_tag(rows[j].note, "FILL(CL)");
                }
            }
        }

        // Mark the cluster endpoints too
        note_add_tag(rows[c.s].note, "CLUSTER_START");
        note_add_tag(rows[c.e].note, "CLUSTER_END");
    }

    if (st.clustersPatched > 0) {
        cout << "[SMOOTH] Cluster patch applied: clusters=" << st.clustersPatched
             << ", points_patched=" << st.pointsPatched << "\n";
    } else {
        cout << "[SMOOTH] No clusters qualified for patch (likely isolated flags only).\n";
    }

    return st;
}


// -------------------- cleanup_polar_file_to_grid (UPDATED: realistic dupe pick + CLmax) --------------------

static bool cleanup_polar_file_to_grid(
    const string& inPath,
    const string& outPath,
    double baseStep
) {
    ifstream in(inPath);
    if (!in) return false;

    const double step = fabs(baseStep);
    if (!(step > 0)) return false;

    // Collect all candidates per grid key (do NOT dedupe yet)
    std::map<long long, vector<PolarRow>> candsByKey;

    string line;
    while (getline(in, line)) {
        PolarRow r;
        if (!parse_polar_row_7_with_note(line, r.a, r.cl, r.cd, r.cdp, r.cm, r.tx, r.bx, r.hasXtr, r.note))
            continue;

        long long k = grid_key(r.a, step);
        double snapped = k * step;

        // only accept values that are actually on the grid (within tolerance)
        if (fabs(r.a - snapped) > 1e-3) continue;

        // normalize note early
        if (r.note.empty()) r.note = "BASE";

        // Store candidate; IMPORTANT: notes stay attached only to this candidate.
        candsByKey[k].push_back(r);
    }

    if (candsByKey.empty()) return false;

    // Select one "realistic" candidate per key using continuity (notes of discarded candidates disappear automatically)
    vector<PolarRow> rows = select_realistic_dupes(candsByKey, step);

    // ---- cluster-based smoothing over CL ----
    detect_and_cluster_patch_CL(rows, step);


    // Write cleaned output
    ofstream out(outPath, ios::trunc);
    if (!out) return false;

    out << "  alpha        CL          CD         CDp         CM      Top_Xtr  Bot_Xtr  Note\n";

    out << std::fixed << std::setprecision(3);
    for (const auto& r : rows) {
        out << std::setw(7) << r.a << "  "
            << std::setw(8) << std::setprecision(4) << r.cl << "  "
            << std::setw(9) << std::setprecision(5) << r.cd  << "  "
            << std::setw(9) << std::setprecision(5) << r.cdp << "  "
            << std::setw(8) << std::setprecision(4) << r.cm  << "  ";

        if (r.hasXtr) {
            out << std::setw(8) << std::setprecision(4) << r.tx << "  "
                << std::setw(8) << std::setprecision(4) << r.bx;
        } else {
            out << std::setw(8) << "NA" << "  " << std::setw(8) << "NA";
        }

        out << "  " << r.note << "\n";
        out << std::setprecision(3);
    }


    return true;
}

// -------------------- process runner (watchdog) --------------------

static int run_xfoil_with_progress_watchdog(
    const string& cmdLine,
    const string& progressFilePath,
    DWORD stall_timeout_ms,
    DWORD poll_interval_ms
) {
    STARTUPINFOA si{};
    PROCESS_INFORMATION pi{};
    si.cb = sizeof(si);

    HANDLE hJob = CreateJobObjectA(nullptr, nullptr);
    if (!hJob) return -1;

    JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
    info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    if (!SetInformationJobObject(hJob, JobObjectExtendedLimitInformation, &info, sizeof(info))) {
        CloseHandle(hJob);
        return -1;
    }

    string mutableCmd = cmdLine;
    mutableCmd.push_back('\0');

    BOOL ok = CreateProcessA(
        nullptr,
        &mutableCmd[0],
        nullptr, nullptr,
        FALSE,
        0,
        nullptr,
        nullptr,
        &si,
        &pi
    );

    if (!ok) {
        CloseHandle(hJob);
        return -1;
    }

    AssignProcessToJobObject(hJob, pi.hProcess);
    SetPriorityClass(pi.hProcess, HIGH_PRIORITY_CLASS);

    bool haveSeenProgress = false;
    uintmax_t lastSize = 0;
    fs::file_time_type lastWriteTime{};
    ULONGLONG lastProgressTick = GetTickCount64();
    ULONGLONG startTick = lastProgressTick;

    auto read_progress_state = [&](uintmax_t& sizeOut, fs::file_time_type& timeOut, bool& existsOut) {
        existsOut = false;
        sizeOut = 0;

        std::error_code ec;
        if (fs::exists(progressFilePath, ec)) {
            existsOut = true;
            sizeOut = fs::file_size(progressFilePath, ec);
            timeOut = fs::last_write_time(progressFilePath, ec);
        }
    };

    {
        bool existsNow = false;
        uintmax_t s = 0;
        fs::file_time_type t{};
        read_progress_state(s, t, existsNow);
        if (existsNow) {
            lastSize = s;
            lastWriteTime = t;
            if (s > 0) haveSeenProgress = true;
        }
    }

    while (true) {
        DWORD wait = WaitForSingleObject(pi.hProcess, poll_interval_ms);
        if (wait == WAIT_OBJECT_0) break;

        bool existsNow = false;
        uintmax_t sizeNow = 0;
        fs::file_time_type writeNow{};
        read_progress_state(sizeNow, writeNow, existsNow);

        bool progressed = false;
        if (existsNow) {
            if (!haveSeenProgress) {
                haveSeenProgress = true;
                progressed = (sizeNow > 0);
            } else {
                if (sizeNow > lastSize) progressed = true;
                else if (writeNow != lastWriteTime) progressed = true;
            }
        }

        if (progressed) {
            lastProgressTick = GetTickCount64();
            lastSize = sizeNow;
            lastWriteTime = writeNow;
        }

        ULONGLONG now = GetTickCount64();
        if (haveSeenProgress && (now - lastProgressTick) > stall_timeout_ms) {
            TerminateJobObject(hJob, 1);
            CloseHandle(pi.hThread);
            CloseHandle(pi.hProcess);
            CloseHandle(hJob);
            return 124;
        }

        if (!haveSeenProgress && (now - startTick) > 60000ULL) {
            TerminateJobObject(hJob, 1);
            CloseHandle(pi.hThread);
            CloseHandle(pi.hProcess);
            CloseHandle(hJob);
            return 124;
        }
    }

    DWORD exitCode = 0;
    GetExitCodeProcess(pi.hProcess, &exitCode);

    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    CloseHandle(hJob);

    return (int)exitCode;
}

// -------------------- XFOIL script --------------------

static bool write_xfoil_script_with_seeded_retry(
    const string& scriptPath,
    const string& airfoilCmd,
    int Re,
    int Ncrit,
    int iter,
    double seedAlpha,
    double sweepStart,
    double sweepEnd,
    double sweepStep,
    const string& polarOut,
    bool useXtr,
    bool enablePlots = true,
    double xtrTop = 0.10,
    double xtrBot = 0.10
) {
    ofstream file(scriptPath);
    if (!file) return false;

    file << "Y\n";
    file << airfoilCmd << "\n";

    if (!enablePlots) {
        file << "PLOP\n";
        file << "G\n";
        file << "\n";
    }

    file << ".PPAR\n";
    file << "N 220\n";
    file << "\n";
    file << "PANE\n";

    file << "OPER\n";
    file << "VISC " << Re << "\n";

    file << "VPAR\n";
    file << "N " << Ncrit << "\n";
    if (useXtr) file << "XTR " << xtrTop << " " << xtrBot << "\n";
    file << "\n";

    file << "ITER " << iter << "\n";

    file << "PACC\n";
    file << "..\\" << polarOut << "\n";
    file << "\n";

    file << "ALFA " << seedAlpha << "\n";
    file << "INIT\n";

    // Only do the sweep you actually want (avoid duplicate alphas / mixed histories)
    file << "ALFA " << sweepStart << "\n";
    file << "ASEQ " << sweepStart << " " << sweepEnd << " " << sweepStep << "\n";


    file << "PACC\n";
    file << "\n";
    file << "QUIT\n";

    return true;
}

// -------------------- single-hop script (single operating point) --------------------
static bool write_xfoil_script_single_alpha(
    const string& scriptPath,
    const string& airfoilCmd,
    int Re,
    int Ncrit,
    int iter,
    double seedAlpha,
    double targetAlpha,
    const string& polarOut,
    bool useXtr,
    bool enablePlots = true,
    double xtrTop = 0.10,
    double xtrBot = 0.10
) {
    ofstream file(scriptPath);
    if (!file) return false;

    file << "Y\n";
    file << airfoilCmd << "\n";

    if (!enablePlots) {
        file << "PLOP\n";
        file << "G\n";
        file << "\n";
    }

    file << ".PPAR\n";
    file << "N 220\n";
    file << "\n";
    file << "PANE\n";

    file << "OPER\n";
    file << "VISC " << Re << "\n";

    file << "VPAR\n";
    file << "N " << Ncrit << "\n";
    if (useXtr) file << "XTR " << xtrTop << " " << xtrBot << "\n";
    file << "\n";

    file << "ITER " << iter << "\n";

    file << "PACC\n";
    file << "..\\" << polarOut << "\n";
    file << "\n";

    file << "ALFA " << seedAlpha << "\n";
    file << "INIT\n";

    file << "ALFA " << targetAlpha << "\n";

    file << "PACC\n";
    file << "\n";
    file << "QUIT\n";

    return true;
}

// -------------------- sweep runner (adds NOTE tags per row) --------------------

static int run_sweep_try_once_then_stop(
    const string& cmd,
    const string& scriptPath,
    const string& airfoilCmd,
    int Re, int Ncrit, int iter,
    bool useXtr,
    double start,
    double end,
    double step,
    DWORD stall_ms,
    DWORD poll_ms,
    const string& masterRawPolar,
    const string& partPolar,
    const string& tag,
    double progressLo,
    double progressHi,
    bool wantMax,
    double& outBestA,
    bool verbose
) {
    const double EPS = 1e-9;
    const double tol = 1e-6;

    const double baseStepAbs = fabs(step);
    const double stepSign    = (step >= 0.0) ? 1.0 : -1.0;

    const int    BACKTRACK_EXTRA_STEPS = 2;
    const double minStepAbs            = std::max(0.01, baseStepAbs / 64.0);
    const int    GROW_AFTER_SUCCESSES  = 4;

    // ------------------------------------------------------------------
    // These can create "unrealistic estimates" by forcing transition or
    // hopping onto a different solution branch. Keep the code, but disable.
    const bool   ENABLE_XTR_RESCUE  = false;
    const double XTR_EARLY = 0.02;
    const double XTR_LATE  = 0.30;

    const bool   ENABLE_SINGLE_HOP  = false;
    const double HOP_SPAN_DEG      = 2.0;
    const int    HOP_MAX_TRIES     = 6;
    // ------------------------------------------------------------------

    double curStepAbs = baseStepAbs;
    int successStreak = 0;

    auto finished = [&](double x) {
        return (stepSign > 0) ? (x > end + tol) : (x < end - tol);
    };

    auto reached_end = [&](double bestA) {
        return wantMax ? (bestA >= end - 1e-3) : (bestA <= end + 1e-3);
    };

    auto get_master_best = [&](double& outA) -> bool {
        if (!file_exists_nonempty(masterRawPolar)) return false;
        return get_best_alpha_in_range(masterRawPolar, progressLo, progressHi, wantMax, outA);
    };

    auto clamp_to_start_side = [&](double x) {
        if (stepSign > 0) return std::max(x, start);
        else              return std::min(x, start);
    };

    auto clamp_to_end_side = [&](double x) {
        if (stepSign > 0) return std::min(x, end);
        else              return std::max(x, end);
    };

    double a = start;

    for (int attempt = 0; attempt < 200000 && !finished(a); ++attempt) {
        double bestBefore = 0.0;
        bool haveBefore = get_master_best(bestBefore);

        const double sweepStep = stepSign * curStepAbs;

        remove_file_retry(partPolar);

        const double seedAlpha = haveBefore ? bestBefore : 0.0;
        const string baseNote = "BASE";

        if (!write_xfoil_script_with_seeded_retry(
                scriptPath, airfoilCmd, Re, Ncrit, iter,
                seedAlpha,
                a, end, sweepStep, partPolar, useXtr,
                g_enable_xfoil_plots,
                0.10, 0.10)) {
            return -1;
        }

        if (verbose) {
            cout << "[" << tag << " #" << attempt << "] a=" << a
                 << " step=" << sweepStep
                 << " (|step|=" << curStepAbs << ")"
                 << " seed=" << seedAlpha
                 << "\n";
        }

        int ret = run_xfoil_with_progress_watchdog(cmd, partPolar, stall_ms, poll_ms);

        if (file_exists_nonempty(partPolar)) {
            append_polar_rows_with_note_retry(partPolar, masterRawPolar, baseNote);
        }

        double bestAfter = 0.0;
        bool haveAfter = get_master_best(bestAfter);

        if (haveAfter) outBestA = bestAfter;
        else if (haveBefore) outBestA = bestBefore;
        else outBestA = a;

        if (verbose) {
            cout << "    ret=" << ret
                 << " bestBefore=" << (haveBefore ? to_string(bestBefore) : string("NA"))
                 << " bestAfter="  << (haveAfter  ? to_string(bestAfter)  : string("NA"))
                 << "\n";
        }

        if (haveAfter && reached_end(bestAfter)) return 0;

        bool progressed = false;
        if (haveAfter) {
            if (!haveBefore) progressed = true;
            else progressed = (fabs(bestAfter - bestBefore) > EPS);
        }

        const double bestUsed = haveAfter ? bestAfter : (haveBefore ? bestBefore : a);
        const double nextA = bestUsed + sweepStep;
        const bool nextEqualsCurrent = (fabs(nextA - a) < EPS);

        const bool stuck = (!progressed) || nextEqualsCurrent;

        if (stuck) {

#if 0
            // Single-hop rescue (DISABLED: can jump solution branch)
            if (ENABLE_SINGLE_HOP && haveBefore) {
                double span = std::max(HOP_SPAN_DEG, 4.0 * baseStepAbs);
                bool hopSucceeded = false;

                for (int h = 0; h < HOP_MAX_TRIES; ++h) {
                    double delta = span * pow(0.5, h);
                    if (delta < baseStepAbs - 1e-12) break;

                    double targetA = clamp_to_end_side(bestBefore + stepSign * delta);

                    if (stepSign > 0 && targetA <= bestBefore + 1e-12) break;
                    if (stepSign < 0 && targetA >= bestBefore - 1e-12) break;

                    remove_file_retry(partPolar);

                    std::ostringstream oss;
                    oss << "HOP(a=" << std::fixed << std::setprecision(3) << targetA << ")";
                    const string hopNote = oss.str();

                    if (verbose) {
                        cout << "    STUCK -> single-hop attempt targetA=" << targetA << "\n";
                    }

                    bool okWrite = write_xfoil_script_single_alpha(
                        scriptPath, airfoilCmd, Re, Ncrit, iter,
                        seedAlpha,
                        targetA,
                        partPolar,
                        useXtr,
                        g_enable_xfoil_plots,
                        0.10, 0.10
                    );

                    if (!okWrite) continue;

                    int retH = run_xfoil_with_progress_watchdog(cmd, partPolar, stall_ms, poll_ms);

                    if (file_exists_nonempty(partPolar)) {
                        append_polar_rows_with_note_retry(partPolar, masterRawPolar, hopNote);
                    }

                    double bestAfterH = 0.0;
                    bool haveAfterH = get_master_best(bestAfterH);

                    bool progressedH = false;
                    if (haveAfterH) progressedH = (fabs(bestAfterH - bestBefore) > EPS);

                    if (verbose) {
                        cout << "    hop ret=" << retH
                             << " bestAfterH=" << (haveAfterH ? to_string(bestAfterH) : string("NA"))
                             << "\n";
                    }

                    if (haveAfterH && progressedH) {
                        outBestA = bestAfterH;
                        a = bestAfterH + sweepStep;
                        successStreak = 1;
                        hopSucceeded = true;
                        break;
                    }
                }

                if (hopSucceeded) continue;
            }
#endif

#if 0
            // XTR rescue (DISABLED: forces transition / can distort polars)
            if (ENABLE_XTR_RESCUE) {
                remove_file_retry(partPolar);

                double rescueTop = XTR_EARLY;
                double rescueBot = XTR_LATE;
                if (stepSign < 0) {
                    rescueTop = XTR_LATE;
                    rescueBot = XTR_EARLY;
                }

                std::ostringstream oss;
                oss << "XTR_ASYM(top=" << std::fixed << std::setprecision(2) << rescueTop
                    << ",bot=" << std::fixed << std::setprecision(2) << rescueBot << ")";
                const string rescueNote = oss.str();

                if (verbose) {
                    cout << "    STUCK -> XTR rescue (asymmetric) XTR "
                         << rescueTop << " " << rescueBot
                         << "  (top,bottom)\n";
                }

                bool okWrite = write_xfoil_script_with_seeded_retry(
                    scriptPath, airfoilCmd, Re, Ncrit, iter,
                    seedAlpha,
                    a, end, sweepStep, partPolar,
                    true,
                    g_enable_xfoil_plots,
                    rescueTop,
                    rescueBot
                );

                if (okWrite) {
                    int ret2 = run_xfoil_with_progress_watchdog(cmd, partPolar, stall_ms, poll_ms);

                    if (file_exists_nonempty(partPolar)) {
                        append_polar_rows_with_note_retry(partPolar, masterRawPolar, rescueNote);
                    }

                    double bestAfter2 = 0.0;
                    bool haveAfter2 = get_master_best(bestAfter2);

                    bool progressed2 = false;
                    if (haveAfter2) progressed2 = (fabs(bestAfter2 - bestBefore) > EPS);

                    if (verbose) {
                        cout << "    rescue ret=" << ret2
                             << " bestAfter2=" << (haveAfter2 ? to_string(bestAfter2) : string("NA"))
                             << "\n";
                    }

                    if (haveAfter2 && progressed2) {
                        outBestA = bestAfter2;
                        a = bestAfter2 + sweepStep;
                        successStreak = 1;
                        continue;
                    }
                }
            }
#endif

            successStreak = 0;

            if (haveBefore) {
                double backA = bestBefore - stepSign * (BACKTRACK_EXTRA_STEPS * baseStepAbs);
                a = clamp_to_start_side(backA);
            }

            if (verbose) {
                cout << "    STUCK -> backtrack to a=" << a
                     << " then shrink step\n";
            }

            if (curStepAbs > minStepAbs + 1e-12) {
                curStepAbs *= 0.5;
                if (curStepAbs < minStepAbs) curStepAbs = minStepAbs;

                double testNext = a + stepSign * curStepAbs;
                if (fabs(testNext - a) < EPS) return 124;

                continue;
            }

            return 124;
        }

        a = nextA;
        successStreak++;

        if (successStreak >= GROW_AFTER_SUCCESSES && curStepAbs < baseStepAbs - 1e-12) {
            curStepAbs *= 2.0;
            if (curStepAbs > baseStepAbs) curStepAbs = baseStepAbs;
            successStreak = 0;

            if (verbose) cout << "    step growth -> |step|=" << curStepAbs << "\n";
        }
    }

    return 124;
}

// -------------------- NEW: call python matplotlib plotter (FIXED QUOTING) --------------------

static bool contains_space_or_tab(const string& s) {
    for (char ch : s) if (ch == ' ' || ch == '\t') return true;
    return false;
}

static string quote_if_needed(const string& s) {
    if (s.empty()) return "\"\"";
    if (!contains_space_or_tab(s)) return s;
    return string("\"") + s + "\"";
}

static void try_run_python_plotter(const string& cleanedPolarPath) {
    if (!file_exists_nonempty(cleanedPolarPath)) {
        cout << "Plot skipped: missing/empty file: " << cleanedPolarPath << "\n";
        return;
    }

    const string arg = quote_if_needed(cleanedPolarPath);
    string cmd = "cmd /C python plot_polar.py " + arg;

    int ret = system(cmd.c_str());
    if (ret != 0) {
        cout << "NOTE: python plotter returned code " << ret << "\n";
        cout << "If it says matplotlib missing:  python -m pip install matplotlib\n";
        cout << "Or try explicit:  python plot_polar.py " << cleanedPolarPath << "\n";
    }
}

//main
int main() {
    int Re = 90000;
    int Ncrit = 5;
    int iter = 250;

    string airfoilCmd = "NACA 8510";

    double aMin = -20.0, aMax = 20.0, aStep = 0.25; //airfoil tools ish
    //double aMin = -20.0, aMax = 25.0, aStep = 0.25;

    const string FINAL_OUT = "test_polar_output.dat";

    const string TMP_MASTER_RAW = "__tmp_master_raw_polar.dat";
    const string TMP_PART_POLAR = "__tmp_part_polar.dat";
    const string TMP_SCRIPT     = "__tmp_xfoil_script.in";

    const bool VERBOSE_DIAGNOSTICS = true;

    g_enable_xfoil_plots = true;

    // cluster smoothing patch on/off
    g_enable_cluster_patch = false;

    cout << "Running case: Re=" << Re << " Ncrit=" << Ncrit << " iter=" << iter << "\n";
    cout << "Airfoil: " << airfoilCmd << "\n";
    cout << "Final output: " << FINAL_OUT << "\n";
    cout << "Cluster smoothing patch: " << (g_enable_cluster_patch ? "ON" : "OFF") << "\n";

    remove_file_retry(FINAL_OUT);

    remove_file_retry(TMP_MASTER_RAW);
    remove_file_retry(TMP_PART_POLAR);
    remove_file_retry(TMP_SCRIPT);

    string cmd = "cmd /C \"cd /d xfoil && xfoil.exe < ..\\" + TMP_SCRIPT + " > NUL 2>&1\"";

    DWORD poll_ms  = 250;
    DWORD stall_ms = 20000;

    const double step = fabs(aStep);

    double bestPos = 0.0;
    int retP = run_sweep_try_once_then_stop(
        cmd, TMP_SCRIPT, airfoilCmd,
        Re, Ncrit, iter,
        /*useXtr=*/false, // disable forced transition for "realistic" polars
        0.0, aMax, +step,
        stall_ms, poll_ms,
        TMP_MASTER_RAW,
        TMP_PART_POLAR,
        "pos",
        0.0, aMax,
        true,
        bestPos,
        VERBOSE_DIAGNOSTICS
    );

    cout << "Positive sweep ret=" << retP << " bestPos=" << bestPos << "\n";
    if (retP == 124) cout << "WARNING: positive sweep stalled near next alpha ~" << (bestPos + step) << "\n";

    double bestNeg = 0.0;
    int retN = run_sweep_try_once_then_stop(
        cmd, TMP_SCRIPT, airfoilCmd,
        Re, Ncrit, iter,
        /*useXtr=*/false, // disable forced transition for "realistic" polars
        0.0, aMin, -step,
        stall_ms, poll_ms,
        TMP_MASTER_RAW,
        TMP_PART_POLAR,
        "neg",
        aMin, 0.0,
        false,
        bestNeg,
        VERBOSE_DIAGNOSTICS
    );

    cout << "Negative sweep ret=" << retN << " bestNeg=" << bestNeg << "\n";
    if (retN == 124) cout << "WARNING: negative sweep stalled near next alpha ~-29.25\n";

    bool haveFinal = false;

    if (fs::exists(TMP_MASTER_RAW)) {
        bool ok = cleanup_polar_file_to_grid(TMP_MASTER_RAW, FINAL_OUT, aStep);
        if (ok) {
            cout << "Wrote cleaned polar -> " << FINAL_OUT << "\n";
            haveFinal = true;
        } else {
            cout << "WARNING: cleanup failed; no final output produced\n";
        }
    } else {
        cout << "WARNING: no raw polar produced (TMP_MASTER_RAW missing)\n";
    }

    // ---- python matplotlib plot pop-up ----
    if (haveFinal) {
        cout << "Attempting to open matplotlib plot window via: python plot_polar.py " << FINAL_OUT << "\n";
        cout << "If nothing appears, verify Python + matplotlib and that plot_polar.py is beside this .exe.\n";
        try_run_python_plotter(FINAL_OUT);
    }
    // --------------------------------------

    remove_file_retry(TMP_PART_POLAR);
    remove_file_retry(TMP_SCRIPT);
    remove_file_retry(TMP_MASTER_RAW);

    if (retP == 124 || retN == 124) return 124;
    if (retP != 0) return retP;
    return retN;
}
