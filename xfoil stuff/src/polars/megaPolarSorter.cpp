// megaPolarSorter.cpp
// If run with no arguments, uses hardcoded defaults.
// Otherwise supports optional overrides.
//
// Arg patterns supported:
//   0 args: use defaults
//   1 arg : if numeric -> targetRe, else -> megaPath
//   2 args: megaPath targetRe   OR  targetRe CLmin (if first is numeric)
//   3 args: megaPath targetRe CLmin   OR  targetRe CLmin topN
//   4 args: megaPath targetRe CLmin topN
//
// Outputs TOP-N *UNIQUE AIRFOILS* (default 5) for several objectives at target Re (or nearest),
// filtering to a single Ncrit (DEFAULT_NCRIT):
//   1) max CL/CD
//   2) max CL^(3/2)/CD
//   3) max sqrt(CL)/CD
//   4) min CD with CL >= CLmin_for_minCD
//   5) min CD unconstrained (reference)
//
// NOTE: "Unique airfoils" means: per metric, each airfoil contributes ONLY its single best alpha row.

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

using namespace std;

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

struct ScoredRow {
    Row row;
    double score = 0.0; // max metrics: higher better; min-CD lists: lower better (score = CD)
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
    } catch (...) {
        return false;
    }
}

static bool parse_double_strict(const string& s, double& out) {
    try {
        size_t idx = 0;
        double v = stod(s, &idx);
        if (idx != s.size()) return false;
        out = v;
        return true;
    } catch (...) {
        return false;
    }
}

static bool looks_like_int(const string& s) {
    int tmp = 0;
    return parse_int_strict(s, tmp);
}

// Parses a megapolar data line:
// Airfoil  Re  Ncrit  Iter  alpha  CL  CD  CDp  CM  Top_Xtr  Bot_Xtr  Note
// (Airfoil + Note are single tokens; Top/Bot may be "NA")
static bool parse_megapolar_line(const string& line, Row& r) {
    if (line.empty()) return false;

    std::istringstream iss(line);
    string airfoilS, ReS, NS, IterS, aS, clS, cdS, cdpS, cmS, txS, bxS, noteS;

    if (!(iss >> airfoilS >> ReS >> NS >> IterS >> aS >> clS >> cdS >> cdpS >> cmS >> txS >> bxS)) {
        return false;
    }

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
    if (!is_finite(r.alpha) || !is_finite(r.CL) || !is_finite(r.CD) || !is_finite(r.CDp) || !is_finite(r.CM))
        return false;

    if (r.CD <= 0.0) return false;
    if (r.CD > 5.0)  return false;
    if (std::fabs(r.CL) > 5.0) return false;
    if (std::fabs(r.alpha) > 90.0) return false;

    return true;
}

static void sort_and_trim(vector<ScoredRow>& v, bool ascending, int topN) {
    if (ascending) {
        sort(v.begin(), v.end(), [](const ScoredRow& a, const ScoredRow& b) { return a.score < b.score; });
    } else {
        sort(v.begin(), v.end(), [](const ScoredRow& a, const ScoredRow& b) { return a.score > b.score; });
    }
    if ((int)v.size() > topN) v.resize(topN);
}

static void print_topN(const string& title, const vector<ScoredRow>& v, bool isMin) {
    cout << "---- " << title << " ----\n";
    if (v.empty()) {
        cout << "No valid points.\n\n";
        return;
    }

    for (size_t i = 0; i < v.size(); ++i) {
        const Row& r = v[i].row;
        cout << "#" << (i + 1) << "  "
             << r.airfoil
             << "  alpha=" << r.alpha << " deg"
             << "  CL=" << r.CL
             << "  CD=" << r.CD;

        if (isMin) cout << "  CD=" << v[i].score;
        else       cout << "  score=" << v[i].score;

        cout << "\n";
    }
    cout << "\n";
}

// -------------------- NEW: helpers for UNIQUE-AIRFOIL ranking --------------------

static void update_best_max(unordered_map<string, ScoredRow>& bestByFoil, const Row& r, double score) {
    auto it = bestByFoil.find(r.airfoil);
    if (it == bestByFoil.end()) {
        bestByFoil.emplace(r.airfoil, ScoredRow{r, score});
        return;
    }
    if (score > it->second.score) {
        it->second = ScoredRow{r, score};
    }
}

static void update_best_min(unordered_map<string, ScoredRow>& bestByFoil, const Row& r, double score) {
    auto it = bestByFoil.find(r.airfoil);
    if (it == bestByFoil.end()) {
        bestByFoil.emplace(r.airfoil, ScoredRow{r, score});
        return;
    }
    if (score < it->second.score) {
        it->second = ScoredRow{r, score};
    }
}

static vector<ScoredRow> map_values_to_vector(const unordered_map<string, ScoredRow>& m) {
    vector<ScoredRow> v;
    v.reserve(m.size());
    for (const auto& kv : m) v.push_back(kv.second);
    return v;
}

int main(int argc, char** argv) {
    // fun things to edit
    const string DEFAULT_MEGA_PATH = "data\\polars\\megapolar.dat";
    const int    DEFAULT_TARGET_RE = 80000;
    const double DEFAULT_CLMIN     = 0.2;
    const int    DEFAULT_TOPN      = 5;

    // NEW: lock to a single Ncrit (you said you want it the same)
    const int    DEFAULT_NCRIT     = 5;
    // ================================================================

    // Resolved runtime settings (start with defaults)
    string megaPath = DEFAULT_MEGA_PATH;
    int targetRe = DEFAULT_TARGET_RE;
    double CLmin_for_minCD = DEFAULT_CLMIN;
    int TOP_N = DEFAULT_TOPN;

    // ---------------- parse args (flexible) ----------------
    // 0 args => use defaults
    // 1 arg  => if numeric => targetRe; else => megaPath
    // 2 args => if first numeric => targetRe, CLmin ; else => megaPath, targetRe
    // 3 args => if first numeric => targetRe, CLmin, topN ; else => megaPath, targetRe, CLmin
    // 4 args => megaPath, targetRe, CLmin, topN
    if (argc >= 2) {
        string a1 = argv[1];

        if (looks_like_int(a1)) {
            // First arg is targetRe
            int tmpRe = 0;
            if (!parse_int_strict(a1, tmpRe) || tmpRe <= 0) {
                cout << "ERROR: targetRe must be a positive integer.\n";
                return 2;
            }
            targetRe = tmpRe;

            if (argc >= 3) {
                double tmpCL = 0.0;
                if (!parse_double_strict(argv[2], tmpCL) || !is_finite(tmpCL) || tmpCL < 0.0) {
                    cout << "ERROR: CLmin_for_minCD must be a nonnegative number.\n";
                    return 2;
                }
                CLmin_for_minCD = tmpCL;
            }
            if (argc >= 4) {
                int tmpN = 0;
                if (!parse_int_strict(argv[3], tmpN) || tmpN <= 0 || tmpN > 1000) {
                    cout << "ERROR: topN must be a positive integer (<=1000).\n";
                    return 2;
                }
                TOP_N = tmpN;
            }
            if (argc >= 5) {
                cout << "ERROR: too many args for numeric-first mode.\n";
                cout << "Try: " << argv[0] << " <megapolar.dat> <targetRe> [CLmin] [topN]\n";
                return 2;
            }
        } else {
            // First arg is megapolar path
            megaPath = a1;

            if (argc >= 3) {
                int tmpRe = 0;
                if (!parse_int_strict(argv[2], tmpRe) || tmpRe <= 0) {
                    cout << "ERROR: targetRe must be a positive integer.\n";
                    return 2;
                }
                targetRe = tmpRe;
            }
            if (argc >= 4) {
                double tmpCL = 0.0;
                if (!parse_double_strict(argv[3], tmpCL) || !is_finite(tmpCL) || tmpCL < 0.0) {
                    cout << "ERROR: CLmin_for_minCD must be a nonnegative number.\n";
                    return 2;
                }
                CLmin_for_minCD = tmpCL;
            }
            if (argc >= 5) {
                int tmpN = 0;
                if (!parse_int_strict(argv[4], tmpN) || tmpN <= 0 || tmpN > 1000) {
                    cout << "ERROR: topN must be a positive integer (<=1000).\n";
                    return 2;
                }
                TOP_N = tmpN;
            }
            if (argc >= 6) {
                cout << "ERROR: too many arguments.\n";
                cout << "Try: " << argv[0] << " <megapolar.dat> <targetRe> [CLmin] [topN]\n";
                return 2;
            }
        }
    }

    // ---------------- read file ----------------
    ifstream in(megaPath);
    if (!in) {
        cout << "ERROR: could not open megapolar file: " << megaPath << "\n";
        cout << "DEFAULT_MEGA_PATH is currently: " << DEFAULT_MEGA_PATH << "\n";
        cout << "Run from the project root, or pass a full path.\n";
        return 2;
    }

    vector<Row> allRows;
    allRows.reserve(200000);

    unordered_set<int> reSet;

    string line;
    while (getline(in, line)) {
        Row r;
        if (!parse_megapolar_line(line, r)) continue;
        if (!passes_sanity_filters(r)) continue;

        // NEW: only keep the chosen Ncrit
        if (r.Ncrit != DEFAULT_NCRIT) continue;

        allRows.push_back(r);
        reSet.insert(r.Re);
    }

    if (allRows.empty()) {
        cout << "ERROR: no valid data rows found in file (after filters).\n";
        cout << "Note: this build filters to Ncrit == " << DEFAULT_NCRIT << "\n";
        return 2;
    }

    // Resolve Re: exact if available, else nearest available
    int resolvedRe = targetRe;
    if (reSet.find(targetRe) == reSet.end()) {
        vector<int> reVals;
        reVals.reserve(reSet.size());
        for (int v : reSet) reVals.push_back(v);
        sort(reVals.begin(), reVals.end());

        int best = reVals[0];
        long long bestDist = llabs((long long)best - (long long)targetRe);
        for (int v : reVals) {
            long long d = llabs((long long)v - (long long)targetRe);
            if (d < bestDist) { bestDist = d; best = v; }
        }
        resolvedRe = best;
        cout << "WARNING: exact Re=" << targetRe << " not found. Using nearest available Re=" << resolvedRe << ".\n";
    }

    // Filter to resolvedRe
    vector<Row> rows;
    rows.reserve(allRows.size());
    for (const auto& r : allRows) {
        if (r.Re == resolvedRe) rows.push_back(r);
    }

    if (rows.empty()) {
        cout << "ERROR: no rows found for resolvedRe=" << resolvedRe << "\n";
        return 2;
    }

    // ---------------- UNIQUE-AIRFOIL bests per metric ----------------
    unordered_map<string, ScoredRow> best_CL_over_CD_byFoil;
    unordered_map<string, ScoredRow> best_CL32_over_CD_byFoil;
    unordered_map<string, ScoredRow> best_CL12_over_CD_byFoil;
    unordered_map<string, ScoredRow> best_min_CD_constrained_byFoil;
    unordered_map<string, ScoredRow> best_min_CD_unconstrained_byFoil;

    best_CL_over_CD_byFoil.reserve(4096);
    best_CL32_over_CD_byFoil.reserve(4096);
    best_CL12_over_CD_byFoil.reserve(4096);
    best_min_CD_constrained_byFoil.reserve(4096);
    best_min_CD_unconstrained_byFoil.reserve(4096);

    for (const auto& r : rows) {
        // Unconstrained min CD reference (score = CD, minimize)
        update_best_min(best_min_CD_unconstrained_byFoil, r, r.CD);

        // Constrained min CD: require CL >= threshold (minimize CD)
        if (r.CL >= CLmin_for_minCD) {
            update_best_min(best_min_CD_constrained_byFoil, r, r.CD);
        }

        // Max metrics: require CL > 0
        if (r.CL > 0.0) {
            {
                double s = r.CL / r.CD;
                if (is_finite(s)) update_best_max(best_CL_over_CD_byFoil, r, s);
            }
            {
                double s = pow(r.CL, 1.5) / r.CD;
                if (is_finite(s)) update_best_max(best_CL32_over_CD_byFoil, r, s);
            }
            {
                double s = sqrt(r.CL) / r.CD;
                if (is_finite(s)) update_best_max(best_CL12_over_CD_byFoil, r, s);
            }
        }
    }

    // Convert to vectors (one row per airfoil), then sort + trim
    vector<ScoredRow> v_CL_over_CD          = map_values_to_vector(best_CL_over_CD_byFoil);
    vector<ScoredRow> v_CL32_over_CD        = map_values_to_vector(best_CL32_over_CD_byFoil);
    vector<ScoredRow> v_CL12_over_CD        = map_values_to_vector(best_CL12_over_CD_byFoil);
    vector<ScoredRow> v_min_CD_constrained  = map_values_to_vector(best_min_CD_constrained_byFoil);
    vector<ScoredRow> v_min_CD_unconstrained= map_values_to_vector(best_min_CD_unconstrained_byFoil);

    sort_and_trim(v_CL_over_CD,           false, TOP_N);
    sort_and_trim(v_CL32_over_CD,         false, TOP_N);
    sort_and_trim(v_CL12_over_CD,         false, TOP_N);
    sort_and_trim(v_min_CD_constrained,   true,  TOP_N);
    sort_and_trim(v_min_CD_unconstrained, true,  TOP_N);

    cout << "\nMegapolar: " << megaPath << "\n";
    cout << "Target Re: " << targetRe << "   Resolved Re used: " << resolvedRe << "\n";
    cout << "Rows at Re=" << resolvedRe << ": " << rows.size() << "\n";
    cout << "Ncrit filter: Ncrit == " << DEFAULT_NCRIT << "\n";
    cout << "Min-CD constraint: CL >= " << CLmin_for_minCD << "\n";
    cout << "TopN (unique airfoils): " << TOP_N << "\n\n";

    print_topN("1) Max CL/CD (L/D)", v_CL_over_CD, false);
    print_topN("2) Max CL^(3/2)/CD (endurance-ish)", v_CL32_over_CD, false);
    print_topN("3) Max sqrt(CL)/CD (range-ish)", v_CL12_over_CD, false);

    if (!v_min_CD_constrained.empty()) {
        print_topN("4) Min CD (with CL constraint)", v_min_CD_constrained, true);
    } else {
        cout << "---- 4) Min CD (with CL constraint) ----\n";
        cout << "No valid points found with CL >= " << CLmin_for_minCD << ". Try lowering it (e.g. 0.1).\n\n";
    }

    print_topN("5) Min CD (unconstrained, reference)", v_min_CD_unconstrained, true);

    return 0;
}
