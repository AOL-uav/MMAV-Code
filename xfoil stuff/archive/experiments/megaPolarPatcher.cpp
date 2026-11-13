// megapolar_patch_parallel.cpp
// Parallel low-Re patch runner that produces ONE megapolar_patch.dat.
//
// Modes:
//   - Launcher mode (default): megaPatch
//       Spawns N workers, waits, verifies completion.
//   - Worker mode: megaPatch --worker <id> <N>
//
// Output:
//   outpolars\megapolar_patch.dat   (single file)
// Coordination:
//   outpolars\patch_state\locks\    (job locks)
//   outpolars\patch_state\done\     (done markers)
//   outpolars\patch_state\append_lock\ (single-writer lock for megapolar append)
//
// IMPORTANT:
// - Temp files are unique per PID + workerId to avoid collisions with other runs.

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
#include <chrono>

using namespace std;
namespace fs = std::filesystem;

// -------------------- GLOBAL toggles (set in main) --------------------
static bool g_enable_xfoil_plots      = false;
static bool g_enable_cluster_patch    = false;

// -------------------- tiny file-op retry helpers --------------------

static bool remove_file_retry(const string& path, int tries = 40, DWORD sleep_ms = 50) {
    std::error_code ec;
    for (int i = 0; i < tries; ++i) {
        ec.clear();
        if (!fs::exists(path, ec)) return true;
        fs::remove(path, ec);
        if (!ec) return true;
        Sleep(sleep_ms);
    }
    return false;
}

static bool file_exists_nonempty(const string& p) {
    std::error_code ec;
    return fs::exists(p, ec) && fs::file_size(p, ec) > 0;
}

static bool ensure_dir(const string& dir) {
    std::error_code ec;
    if (fs::exists(dir, ec)) return true;
    return fs::create_directories(dir, ec);
}

static string join_path(const string& a, const string& b) {
    if (a.empty()) return b;
    if (a.back()=='\\' || a.back()=='/') return a + b;
    return a + "\\" + b;
}

static string sanitize_filename(string s) {
    for (char& c : s) {
        if (c == ' ') c = '_';
        if (!(isalnum((unsigned char)c) || c=='_' || c=='-' || c=='.')) c = '_';
    }
    return s;
}

// -------------------- polar parsing helpers --------------------

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

    if (cd <= 0.0 || cd > 5.0) return false;
    if (fabs(cl) > 5.0) return false;

    hasXtr = false;
    topXtr = botXtr = 0.0;

    streampos posBefore = iss.tellg();
    double tx=0.0, bx=0.0;

    if (iss >> tx >> bx) {
        if (std::isfinite(tx) && std::isfinite(bx)) {
            hasXtr = true;
            topXtr = tx;
            botXtr = bx;

            string maybeNote;
            if (iss >> maybeNote) noteOut = maybeNote;
            return true;
        }
    }

    iss.clear();
    iss.seekg(posBefore);

    string maybeNote;
    if (iss >> maybeNote) noteOut = maybeNote;

    return true;
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

// -------------------- cleanup: grid + dedupe + sort (kept minimal) --------------------

struct PolarRow {
    double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
    bool hasXtr=false;
    string note;
};

static long long grid_key(double a, double step) {
    return (long long) llround(a / step);
}

static PolarRow pick_baseline_min_cd(const vector<PolarRow>& cands) {
    int best = 0;
    for (int i = 1; i < (int)cands.size(); ++i) {
        if (cands[i].cd < cands[best].cd) best = i;
    }
    return cands[best];
}

static vector<PolarRow> select_realistic_dupes(
    const map<long long, vector<PolarRow>>& candsByKey,
    double step
) {
    vector<PolarRow> out;
    out.reserve(candsByKey.size());

    for (const auto& kv : candsByKey) {
        long long k = kv.first;
        auto chosen = pick_baseline_min_cd(kv.second);
        chosen.a = k * step;
        if (chosen.note.empty()) chosen.note = "BASE";
        if (kv.second.size() > 1) chosen.note += "|DUPE_PICK(CD)";
        out.push_back(chosen);
    }
    return out;
}

static bool cleanup_polar_file_to_grid(
    const string& inPath,
    const string& outPath,
    double baseStep
) {
    ifstream in(inPath);
    if (!in) return false;

    const double step = fabs(baseStep);
    if (!(step > 0)) return false;

    std::map<long long, vector<PolarRow>> candsByKey;

    string line;
    while (getline(in, line)) {
        PolarRow r;
        if (!parse_polar_row_7_with_note(line, r.a, r.cl, r.cd, r.cdp, r.cm, r.tx, r.bx, r.hasXtr, r.note))
            continue;

        long long k = grid_key(r.a, step);
        double snapped = k * step;
        if (fabs(r.a - snapped) > 1e-3) continue;

        if (r.note.empty()) r.note = "BASE";
        candsByKey[k].push_back(r);
    }

    if (candsByKey.empty()) return false;

    vector<PolarRow> rows = select_realistic_dupes(candsByKey, step);

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

// -------------------- XFOIL watchdog runner --------------------

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

// -------------------- XFOIL script writer --------------------

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
    bool enablePlots
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
    file << "\n";

    file << "ITER " << iter << "\n";

    file << "PACC\n";
    file << "..\\" << polarOut << "\n";
    file << "\n";

    file << "ALFA " << seedAlpha << "\n";
    file << "INIT\n";

    file << "ALFA " << sweepStart << "\n";
    file << "ASEQ " << sweepStart << " " << sweepEnd << " " << sweepStep << "\n";

    file << "PACC\n";
    file << "\n";
    file << "QUIT\n";

    return true;
}

// -------------------- single-sweep runner (your logic kept) --------------------

static int run_sweep_try_once_then_stop(
    const string& cmd,
    const string& scriptPath,
    const string& airfoilCmd,
    int Re, int Ncrit, int iter,
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

    double a = start;

    for (int attempt = 0; attempt < 200000 && !finished(a); ++attempt) {
        double bestBefore = 0.0;
        bool haveBefore = get_master_best(bestBefore);

        const double sweepStep = stepSign * curStepAbs;

        remove_file_retry(partPolar);

        const double seedAlpha = haveBefore ? bestBefore : 0.0;

        if (!write_xfoil_script_with_seeded_retry(
                scriptPath, airfoilCmd, Re, Ncrit, iter,
                seedAlpha,
                a, end, sweepStep, partPolar,
                g_enable_xfoil_plots)) {
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

        // Append raw rows (no note needed here; we just need the raw master to grid-clean)
        if (file_exists_nonempty(partPolar)) {
            // naive append: just copy lines to masterRawPolar
            ifstream in(partPolar);
            ofstream out(masterRawPolar, ios::app);
            if (in && out) {
                string line;
                while (getline(in, line)) out << line << "\n";
            }
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

// -------------------- Job planner + locks + single-file append lock --------------------

struct Job {
    int airfoilIdx;
    int Re;
};

static string job_key(const Job& j) {
    // stable id string
    std::ostringstream oss;
    oss << "a" << std::setw(4) << std::setfill('0') << j.airfoilIdx
        << "_Re" << std::setw(6) << std::setfill('0') << j.Re;
    return oss.str();
}

static bool try_claim_job_lock(const string& lockDir, const string& key) {
    std::error_code ec;
    return fs::create_directory(join_path(lockDir, key), ec);
}

static void release_job_lock(const string& lockDir, const string& key) {
    std::error_code ec;
    fs::remove_all(join_path(lockDir, key), ec);
}

static bool is_done(const string& doneDir, const string& key) {
    std::error_code ec;
    return fs::exists(join_path(doneDir, key + ".done"), ec);
}

static void mark_done(const string& doneDir, const string& key, const string& msg) {
    ofstream out(join_path(doneDir, key + ".done"), ios::trunc);
    if (out) out << msg << "\n";
}

// Append lock: serialize writes to ONE megapolar_patch.dat
static bool acquire_append_lock(const string& appendLockDir, int tries=6000, DWORD sleep_ms=50) {
    std::error_code ec;
    const string p = join_path(appendLockDir, "LOCK");
    for (int i=0;i<tries;i++) {
        ec.clear();
        if (fs::create_directory(p, ec)) return true;
        Sleep(sleep_ms);
    }
    return false;
}
static void release_append_lock(const string& appendLockDir) {
    std::error_code ec;
    fs::remove_all(join_path(appendLockDir, "LOCK"), ec);
}

static bool append_cleaned_to_megapolar_locked(
    const string& megaPath,
    const string& cleanedPath,
    const string& airfoilCmd,
    int Re,
    int Ncrit,
    int iter,
    const string& appendLockDir
) {
    if (!file_exists_nonempty(cleanedPath)) return false;

    if (!acquire_append_lock(appendLockDir)) {
        cout << "ERROR: could not acquire append lock.\n";
        return false;
    }

    // Inside lock: open & append
    bool ok = false;
    {
        ifstream in(cleanedPath);
        if (!in) { release_append_lock(appendLockDir); return false; }

        bool megaExists = false;
        {
            std::error_code ec;
            megaExists = fs::exists(megaPath, ec) && fs::file_size(megaPath, ec) > 0;
        }

        ofstream out(megaPath, ios::app);
        if (!out) { release_append_lock(appendLockDir); return false; }

        if (!megaExists) {
            out << "Airfoil  Re  Ncrit  Iter  "
                << "alpha  CL  CD  CDp  CM  Top_Xtr  Bot_Xtr  Note\n";
        }

        string line;
        while (getline(in, line)) {
            double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
            bool hasXtr=false;
            string note;
            if (!parse_polar_row_7_with_note(line, a, cl, cd, cdp, cm, tx, bx, hasXtr, note))
                continue;

            out << sanitize_filename(airfoilCmd) << "  "
                << Re << "  "
                << Ncrit << "  "
                << iter << "  "
                << std::fixed << std::setprecision(3) << a << "  "
                << std::setprecision(4) << cl << "  "
                << std::setprecision(5) << cd << "  "
                << std::setprecision(5) << cdp << "  "
                << std::setprecision(4) << cm << "  ";

            if (hasXtr) out << std::setprecision(4) << tx << "  " << std::setprecision(4) << bx;
            else        out << "NA  NA";

            out << "  " << (note.empty() ? "BASE" : note) << "\n";
        }

        ok = true;
    }

    release_append_lock(appendLockDir);
    return ok;
}

// -------------------- Worker routine --------------------

static int run_worker(
    int workerId,
    int nWorkers
) {
    // -------------------- PATCH SETTINGS --------------------
    const int Ncrit = 5;
    const int iter  = 250;

    const int Re_start = 8000;
    const int Re_end   = 19000;
    const int Re_step  = 250;

    const double aMin  = -20.0;
    const double aMax  =  20.0;
    const double aStep =  0.25;

   vector<string> airfoils = {
        // Symmetric (00xx)
        "NACA 0004",
        "NACA 0006",
        "NACA 0008",
        "NACA 0010",
        "NACA 0012",
        "NACA 0015",
        "NACA 0018",
        "NACA 0021",
        "NACA 0024",

        // 1% camber (1xxx) — expanded: 12xx, 14xx, 15xx, 16xx
        "NACA 1208", "NACA 1209", "NACA 1212",
        "NACA 1408", "NACA 1409", "NACA 1410", "NACA 1412", "NACA 1415",
        "NACA 1512",
        "NACA 1608", "NACA 1609", "NACA 1612",

        // 2% camber (2xxx) — expanded: 22xx, 24xx, 25xx, 26xx
        "NACA 2208", "NACA 2209", "NACA 2212",
        "NACA 2408", "NACA 2409", "NACA 2410", "NACA 2412", "NACA 2415",
        "NACA 2512",
        "NACA 2608", "NACA 2609", "NACA 2612",

        // 3% camber (3xxx) — now as thorough as 6% set
        "NACA 3208", "NACA 3209", "NACA 3212",
        "NACA 3408", "NACA 3409", "NACA 3410", "NACA 3412", "NACA 3415",
        "NACA 3512",
        "NACA 3608", "NACA 3609", "NACA 3612",

        // 4% camber (4xxx)
        "NACA 4208", "NACA 4209", "NACA 4212",
        "NACA 4408", "NACA 4409", "NACA 4410", "NACA 4412", "NACA 4415",
        "NACA 4512",
        "NACA 4608", "NACA 4609", "NACA 4612",

        // 5% camber (5xxx)
        "NACA 5208", "NACA 5209", "NACA 5212",
        "NACA 5408", "NACA 5409", "NACA 5410", "NACA 5412", "NACA 5415",
        "NACA 5512",
        "NACA 5608", "NACA 5609", "NACA 5612",

        // 6% camber (6xxx) — kept the same “thorough” pattern, plus 62xx for wider 2nd digit range
        "NACA 6208", "NACA 6209", "NACA 6212",
        "NACA 6408", "NACA 6409", "NACA 6410", "NACA 6412", "NACA 6415",
        "NACA 6512",
        "NACA 6608", "NACA 6609", "NACA 6612",

        // 7% camber (7xxx)
        "NACA 7208", "NACA 7209", "NACA 7212",
        "NACA 7408", "NACA 7409", "NACA 7410", "NACA 7412", "NACA 7415",
        "NACA 7512",
        "NACA 7608", "NACA 7609", "NACA 7612",

        // 8% camber (8xxx)
        "NACA 8208", "NACA 8209", "NACA 8212",
        "NACA 8408", "NACA 8409", "NACA 8410", "NACA 8412", "NACA 8415",
        "NACA 8512",
        "NACA 8608", "NACA 8609", "NACA 8612",

        // 9% camber (9xxx)
        "NACA 9208", "NACA 9209", "NACA 9212",
        "NACA 9408", "NACA 9409", "NACA 9410", "NACA 9412", "NACA 9415",
        "NACA 9512",
        "NACA 9608", "NACA 9609", "NACA 9612"
    };

    const string OUT_DIR   = "outpolars";
    const string MEGA_PATH = OUT_DIR + string("\\megapolar_patch.dat");

    const string STATE_DIR = OUT_DIR + string("\\patch_state");
    const string LOCK_DIR  = STATE_DIR + string("\\locks");
    const string DONE_DIR  = STATE_DIR + string("\\done");
    const string APP_LOCK  = STATE_DIR + string("\\append_lock");

    const bool VERBOSE_DIAGNOSTICS = true;

    if (!ensure_dir(OUT_DIR) || !ensure_dir(STATE_DIR) || !ensure_dir(LOCK_DIR) || !ensure_dir(DONE_DIR) || !ensure_dir(APP_LOCK)) {
        cout << "ERROR: could not create state dirs.\n";
        return 2;
    }

    // Unique temp tag (prevents collisions with other running exe)
    DWORD pid = GetCurrentProcessId();
    std::ostringstream ut;
    ut << "_pid" << pid << "_w" << workerId;

    const string TMP_MASTER_RAW = "__tmp_master_raw_polar" + ut.str() + ".dat";
    const string TMP_PART_POLAR = "__tmp_part_polar" + ut.str() + ".dat";
    const string TMP_SCRIPT     = "__tmp_xfoil_script" + ut.str() + ".in";
    const string TMP_CLEANED    = "__tmp_cleaned_polar" + ut.str() + ".dat";

    // Keep your XFOIL invocation unchanged
    string cmd = "cmd /C \"cd /d xfoil && xfoil.exe < ..\\" + TMP_SCRIPT + " > NUL 2>&1\"";

    DWORD poll_ms  = 250;
    DWORD stall_ms = 20000;

    // Build all jobs
    vector<Job> jobs;
    for (int ai = 0; ai < (int)airfoils.size(); ++ai) {
        for (int Re = Re_start; Re <= Re_end; Re += Re_step) {
            jobs.push_back({ai, Re});
        }
    }

    cout << "Worker " << workerId << "/" << nWorkers
         << " PID=" << pid
         << " jobs=" << jobs.size() << "\n";

    int doneCount = 0;
    int failCount = 0;

    // Greedy scan: each worker claims the next available job via lock dir
    for (size_t pass = 0; pass < jobs.size() * 2; ++pass) {
        bool didWorkThisPass = false;

        for (const auto& j : jobs) {
            const string key = job_key(j);

            // already done?
            if (is_done(DONE_DIR, key)) continue;

            // claim?
            if (!try_claim_job_lock(LOCK_DIR, key)) continue;

            // inside claim:
            didWorkThisPass = true;
            const string& airfoilCmd = airfoils[j.airfoilIdx];
            const int Re = j.Re;

            cout << "------------------------------------------------------------\n";
            cout << "Worker " << workerId << " running: " << airfoilCmd << "  Re=" << Re
                 << " (job " << key << ")\n";

            // clean temp files
            remove_file_retry(TMP_CLEANED);
            remove_file_retry(TMP_MASTER_RAW);
            remove_file_retry(TMP_PART_POLAR);
            remove_file_retry(TMP_SCRIPT);

            const double step = fabs(aStep);

            // Positive sweep
            double bestPos = 0.0;
            int retP = run_sweep_try_once_then_stop(
                cmd, TMP_SCRIPT, airfoilCmd,
                Re, Ncrit, iter,
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

            // Negative sweep
            double bestNeg = 0.0;
            int retN = run_sweep_try_once_then_stop(
                cmd, TMP_SCRIPT, airfoilCmd,
                Re, Ncrit, iter,
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

            bool haveFinal = false;
            if (fs::exists(TMP_MASTER_RAW)) {
                if (cleanup_polar_file_to_grid(TMP_MASTER_RAW, TMP_CLEANED, aStep)) {
                    haveFinal = true;
                }
            }

            bool okAppend = false;
            if (haveFinal) {
                okAppend = append_cleaned_to_megapolar_locked(
                    MEGA_PATH, TMP_CLEANED, airfoilCmd, Re, Ncrit, iter, APP_LOCK
                );
            }

            // cleanup temps
            remove_file_retry(TMP_CLEANED);
            remove_file_retry(TMP_PART_POLAR);
            remove_file_retry(TMP_SCRIPT);
            remove_file_retry(TMP_MASTER_RAW);

            // mark done
            std::ostringstream msg;
            msg << "retP=" << retP << " retN=" << retN
                << " bestPos=" << bestPos << " bestNeg=" << bestNeg
                << " haveFinal=" << (haveFinal?1:0)
                << " appended=" << (okAppend?1:0);

            mark_done(DONE_DIR, key, msg.str());
            release_job_lock(LOCK_DIR, key);

            if (retP==124 || retN==124 || !haveFinal || !okAppend) failCount++;
            else doneCount++;

            // keep scanning for more jobs
        }

        if (!didWorkThisPass) break;
    }

    cout << "Worker " << workerId << " finished. ok=" << doneCount << " fails=" << failCount << "\n";
    return (failCount>0) ? 124 : 0;
}

// -------------------- Launcher routine --------------------

static int run_launcher(const string& exePath, int nWorkers) {
    cout << "Launcher spawning " << nWorkers << " workers...\n";

    vector<PROCESS_INFORMATION> pis;
    pis.reserve(nWorkers);

    for (int i = 0; i < nWorkers; ++i) {
        STARTUPINFOA si{};
        PROCESS_INFORMATION pi{};
        si.cb = sizeof(si);

        std::ostringstream cmd;
        cmd << "\"" << exePath << "\" --worker " << i << " " << nWorkers;

        string cmdStr = cmd.str();
        cmdStr.push_back('\0');

        BOOL ok = CreateProcessA(
            nullptr,
            &cmdStr[0],
            nullptr, nullptr,
            FALSE,
            CREATE_NEW_CONSOLE,
            nullptr,
            nullptr,
            &si,
            &pi
        );

        if (!ok) {
            cout << "ERROR: failed to spawn worker " << i << "\n";
            return 2;
        }

        pis.push_back(pi);
    }

    // Wait for all
    int worst = 0;
    for (auto& pi : pis) {
        WaitForSingleObject(pi.hProcess, INFINITE);
        DWORD exitCode = 0;
        GetExitCodeProcess(pi.hProcess, &exitCode);
        worst = max(worst, (int)exitCode);
        CloseHandle(pi.hThread);
        CloseHandle(pi.hProcess);
    }

    cout << "Launcher done. worstExit=" << worst << "\n";
    return worst;
}

// -------------------- main --------------------

int main(int argc, char** argv) {
    // If you’re also running your big megapolar job: this patch runner is safe
    // as long as temp names differ. We made them unique by PID+workerId.

    // Decide mode
    if (argc >= 2 && string(argv[1]) == "--worker") {
        if (argc < 4) {
            cout << "Usage: megaPatch --worker <id> <N>\n";
            return 2;
        }
        int wid = atoi(argv[2]);
        int nW  = max(1, atoi(argv[3]));
        if (wid < 0 || wid >= nW) {
            cout << "ERROR: worker id out of range.\n";
            return 2;
        }
        return run_worker(wid, nW);
    }

    // Launcher mode: default 4 workers
    int nWorkers = 4;

    // Determine exe path (best-effort)
    char exeBuf[MAX_PATH]{0};
    GetModuleFileNameA(nullptr, exeBuf, MAX_PATH);
    string exePath = exeBuf;
    if (exePath.empty()) exePath = "megaPatch.exe";

    // Reset the output megapolar_patch.dat ONCE (launcher only)
    const string OUT_DIR   = "outpolars";
    const string MEGA_PATH = OUT_DIR + string("\\megapolar_patch.dat");
    ensure_dir(OUT_DIR);
    remove_file_retry(MEGA_PATH);

    return run_launcher(exePath, nWorkers);
}
