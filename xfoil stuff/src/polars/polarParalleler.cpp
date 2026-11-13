// polarParalleler.cpp
//
// Parallel megapolar generator producing:
//   - ONE data\polars\generated\megapolar.dat
//   - ONE PNG per airfoil for Re=80000 in data\polars\generated\
//
// Robustness strategy (OneDrive-safe):
//   - ALL scratch/state/locks/temp live in %TEMP%
//   - Only final outputs remain in project: generated\megapolar.dat + *_4plots.png
//
// Key XFOIL robustness fix (ret=2 / no output):
//   - XFOIL script uses PACC with ONLY a filename (no absolute paths).
//   - We launch XFOIL with worker TEMP_DIR as the process working directory,
//     so "part_polar.dat" is created there reliably.
//
// Modes:
//   - Launcher (default): spawns N workers
//   - Worker: --worker <id> <N> --tag <runTag>
//
// Build:
//   g++ polarParalleler.cpp -O2 -std=c++17 -o polarParalleler.exe

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <windows.h>

using namespace std;
namespace fs = std::filesystem;

// -------------------- GLOBAL toggles --------------------
static bool g_enable_xfoil_plots   = false; // keep OFF for batch (XFOIL GUI)
static bool g_enable_cluster_patch = false; // keep OFF

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

static bool remove_all_retry(const string& path, int tries = 40, DWORD sleep_ms = 50) {
    std::error_code ec;
    for (int i = 0; i < tries; ++i) {
        ec.clear();
        if (!fs::exists(path, ec)) return true;
        fs::remove_all(path, ec);
        if (!ec) return true;
        Sleep(sleep_ms);
    }
    return false;
}

static bool file_exists_nonempty(const string& p) {
    std::error_code ec;
    return fs::exists(p, ec) && fs::file_size(p, ec) > 0;
}

static bool path_exists(const string& p) {
    std::error_code ec;
    return fs::exists(p, ec);
}

static bool ensure_dir(const string& dir) {
    std::error_code ec;
    if (fs::exists(dir, ec)) return true;
    return fs::create_directories(dir, ec);
}

static string join_path(const string& a, const string& b) {
    if (a.empty()) return b;
    if (a.back() == '\\' || a.back() == '/') return a + b;
    return a + "\\" + b;
}

static string sanitize_filename(string s) {
    for (char& c : s) {
        if (c == ' ') c = '_';
        if (!(isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.')) c = '_';
    }
    return s;
}

static bool contains_space_or_tab(const string& s) {
    for (char ch : s)
        if (ch == ' ' || ch == '\t') return true;
    return false;
}

static string quote_if_needed(const string& s) {
    if (s.empty()) return "\"\"";
    if (!contains_space_or_tab(s)) return s;
    return string("\"") + s + "\"";
}

static string win_path(string p) {
    for (char& c : p)
        if (c == '/') c = '\\';
    return p;
}

static string get_exe_dir() {
    char buf[MAX_PATH]{0};
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    string full = buf;
    size_t p1 = full.find_last_of('\\');
    size_t p2 = full.find_last_of('/');
    size_t p  = (p1 == string::npos) ? p2 : (p2 == string::npos ? p1 : max(p1, p2));
    if (p == string::npos) return ".";
    return full.substr(0, p);
}

static string get_temp_dir() {
    char buf[MAX_PATH]{0};
    DWORD n = GetTempPathA(MAX_PATH, buf);
    if (n == 0 || n >= MAX_PATH) return ".";
    string t = buf;
    while (!t.empty() && (t.back() == '\\' || t.back() == '/')) t.pop_back();
    if (t.empty()) t = ".";
    return t;
}

static string timestamp_YYYYMMDD_HHMMSS() {
    SYSTEMTIME st;
    GetLocalTime(&st);
    std::ostringstream oss;
    oss << st.wYear << std::setw(2) << std::setfill('0') << st.wMonth << std::setw(2) << std::setfill('0')
        << st.wDay << "_" << std::setw(2) << std::setfill('0') << st.wHour << std::setw(2) << std::setfill('0')
        << st.wMinute << std::setw(2) << std::setfill('0') << st.wSecond;
    return oss.str();
}

// -------------------- launcher progress helpers --------------------
static string format_hms(double seconds) {
    if (!(seconds >= 0.0) || !std::isfinite(seconds)) return "NA";
    long long s = (long long)llround(seconds);
    if (s < 0) s = 0;
    long long h   = s / 3600;
    long long m   = (s % 3600) / 60;
    long long sec = s % 60;
    std::ostringstream oss;
    oss << h << ":" << std::setw(2) << std::setfill('0') << m << ":" << std::setw(2) << std::setfill('0') << sec;
    return oss.str();
}

static string make_bar(double frac, int width = 30) {
    if (!std::isfinite(frac)) frac = 0.0;
    if (frac < 0.0) frac = 0.0;
    if (frac > 1.0) frac = 1.0;
    int filled = (int)llround(frac * width);
    if (filled < 0) filled = 0;
    if (filled > width) filled = width;
    string bar;
    bar.reserve(width);
    for (int i = 0; i < width; ++i) bar.push_back(i < filled ? '#' : '-');
    return bar;
}

// -------------------- polar parsing helpers --------------------
static bool parse_polar_row_7_with_note(const string& line,
                                       double& a,
                                       double& cl,
                                       double& cd,
                                       double& cdp,
                                       double& cm,
                                       double& topXtr,
                                       double& botXtr,
                                       bool& hasXtr,
                                       string& noteOut) {
    noteOut.clear();

    size_t i = 0;
    while (i < line.size() && isspace((unsigned char)line[i])) i++;
    if (i >= line.size()) return false;

    char c = line[i];
    if (!(c == '-' || c == '+' || c == '.' || (c >= '0' && c <= '9'))) return false;

    std::istringstream iss(line);
    if (!(iss >> a >> cl >> cd >> cdp >> cm)) return false;

    if (!std::isfinite(a) || !std::isfinite(cl) || !std::isfinite(cd) || !std::isfinite(cdp) || !std::isfinite(cm))
        return false;

    if (cd <= 0.0 || cd > 5.0) return false;
    if (fabs(cl) > 5.0) return false;

    hasXtr       = false;
    topXtr = botXtr = 0.0;

    streampos posBefore = iss.tellg();
    double tx = 0.0, bx = 0.0;

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

static bool get_best_alpha_in_range(const string& polarPath, double lo, double hi, bool wantMax, double& bestOut) {
    ifstream in(polarPath);
    if (!in) return false;

    const double aLo = std::min(lo, hi);
    const double aHi = std::max(lo, hi);

    string line;
    bool found  = false;
    double bestA = 0.0;

    while (getline(in, line)) {
        double a = 0, cl = 0, cd = 0, cdp = 0, cm = 0, tx = 0, bx = 0;
        bool hasXtr = false;
        string note;
        if (!parse_polar_row_7_with_note(line, a, cl, cd, cdp, cm, tx, bx, hasXtr, note)) continue;

        if (a < aLo - 1e-9 || a > aHi + 1e-9) continue;

        if (!found) {
            bestA = a;
            found = true;
        } else {
            bestA = wantMax ? std::max(bestA, a) : std::min(bestA, a);
        }
    }

    if (!found) return false;
    bestOut = bestA;
    return true;
}

// -------------------- cleanup: grid + dedupe + sort (minimal) --------------------
struct PolarRow {
    double a = 0, cl = 0, cd = 0, cdp = 0, cm = 0, tx = 0, bx = 0;
    bool hasXtr = false;
    string note;
};

static long long grid_key(double a, double step) { return (long long)llround(a / step); }

static PolarRow pick_baseline_min_cd(const vector<PolarRow>& cands) {
    int best = 0;
    for (int i = 1; i < (int)cands.size(); ++i) {
        if (cands[i].cd < cands[best].cd) best = i;
    }
    return cands[best];
}

static vector<PolarRow> select_realistic_dupes(const map<long long, vector<PolarRow>>& candsByKey, double step) {
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

static bool cleanup_polar_file_to_grid(const string& inPath, const string& outPath, double baseStep) {
    ifstream in(inPath);
    if (!in) return false;

    const double step = fabs(baseStep);
    if (!(step > 0)) return false;

    std::map<long long, vector<PolarRow>> candsByKey;

    string line;
    while (getline(in, line)) {
        PolarRow r;
        if (!parse_polar_row_7_with_note(line, r.a, r.cl, r.cd, r.cdp, r.cm, r.tx, r.bx, r.hasXtr, r.note)) continue;

        long long k    = grid_key(r.a, step);
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
        out << std::setw(7) << r.a << "  " << std::setw(8) << std::setprecision(4) << r.cl << "  "
            << std::setw(9) << std::setprecision(5) << r.cd << "  " << std::setw(9) << std::setprecision(5) << r.cdp
            << "  " << std::setw(8) << std::setprecision(4) << r.cm << "  ";

        if (r.hasXtr) {
            out << std::setw(8) << std::setprecision(4) << r.tx << "  " << std::setw(8) << std::setprecision(4) << r.bx;
        } else {
            out << std::setw(8) << "NA" << "  " << std::setw(8) << "NA";
        }

        out << "  " << r.note << "\n";
        out << std::setprecision(3);
    }

    return true;
}

// -------------------- XFOIL watchdog runner (with explicit working directory) --------------------
static int run_xfoil_with_progress_watchdog(const string& cmdLine,
                                            const string& workingDir,
                                            const string& progressFilePath,
                                            DWORD stall_timeout_ms,
                                            DWORD poll_interval_ms) {
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

    BOOL ok = CreateProcessA(nullptr,
                            &mutableCmd[0],
                            nullptr,
                            nullptr,
                            FALSE,
                            0,
                            nullptr,
                            workingDir.empty() ? nullptr : workingDir.c_str(), // <-- critical
                            &si,
                            &pi);

    if (!ok) {
        CloseHandle(hJob);
        return -1;
    }

    AssignProcessToJobObject(hJob, pi.hProcess);
    SetPriorityClass(pi.hProcess, NORMAL_PRIORITY_CLASS);

    bool haveSeenProgress = false;
    uintmax_t lastSize = 0;
    fs::file_time_type lastWriteTime{};
    ULONGLONG lastProgressTick = GetTickCount64();
    ULONGLONG startTick        = lastProgressTick;

    auto read_progress_state = [&](uintmax_t& sizeOut, fs::file_time_type& timeOut, bool& existsOut) {
        existsOut = false;
        sizeOut   = 0;

        std::error_code ec;
        if (fs::exists(progressFilePath, ec)) {
            existsOut = true;
            sizeOut   = fs::file_size(progressFilePath, ec);
            timeOut   = fs::last_write_time(progressFilePath, ec);
        }
    };

    {
        bool existsNow = false;
        uintmax_t s    = 0;
        fs::file_time_type t{};
        read_progress_state(s, t, existsNow);
        if (existsNow) {
            lastSize      = s;
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
                progressed       = (sizeNow > 0);
            } else {
                if (sizeNow > lastSize) progressed = true;
                else if (writeNow != lastWriteTime) progressed = true;
            }
        }

        if (progressed) {
            lastProgressTick = GetTickCount64();
            lastSize         = sizeNow;
            lastWriteTime    = writeNow;
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

// -------------------- XFOIL script writer (PACC uses only filename) --------------------
static bool write_xfoil_script_with_seeded_retry(const string& scriptPath,
                                                 const string& airfoilCmd,
                                                 int Re,
                                                 int Ncrit,
                                                 int iter,
                                                 double seedAlpha,
                                                 double sweepStart,
                                                 double sweepEnd,
                                                 double sweepStep,
                                                 const string& polarOutFilenameOnly, // e.g., "part_polar.dat"
                                                 bool enablePlots) {
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
    file << polarOutFilenameOnly << "\n";
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

// -------------------- single-sweep runner --------------------
static int run_sweep_try_once_then_stop(const string& cmd,
                                        const string& cmd_dbg,
                                        const string& workingDir,
                                        const string& xfoilLogPath,
                                        const string& scriptPath,
                                        const string& airfoilCmd,
                                        int Re,
                                        int Ncrit,
                                        int iter,
                                        double start,
                                        double end,
                                        double step,
                                        DWORD stall_ms,
                                        DWORD poll_ms,
                                        const string& masterRawPolar,
                                        const string& partPolarAbsPath,
                                        const string& tag,
                                        double progressLo,
                                        double progressHi,
                                        bool wantMax,
                                        double& outBestA,
                                        bool verbose) {
    const double EPS = 1e-9;
    const double tol = 1e-6;

    const double baseStepAbs = fabs(step);
    const double stepSign    = (step >= 0.0) ? 1.0 : -1.0;

    const int    BACKTRACK_EXTRA_STEPS = 2;
    const double minStepAbs            = std::max(0.01, baseStepAbs / 64.0);
    const int    GROW_AFTER_SUCCESSES  = 4;

    double curStepAbs = baseStepAbs;
    int successStreak = 0;

    auto finished = [&](double x) { return (stepSign > 0) ? (x > end + tol) : (x < end - tol); };

    auto reached_end = [&](double bestA) { return wantMax ? (bestA >= end - 1e-3) : (bestA <= end + 1e-3); };

    auto get_master_best = [&](double& outA) -> bool {
        if (!file_exists_nonempty(masterRawPolar)) return false;
        return get_best_alpha_in_range(masterRawPolar, progressLo, progressHi, wantMax, outA);
    };

    auto clamp_to_start_side = [&](double x) { return (stepSign > 0) ? std::max(x, start) : std::min(x, start); };

    double a = start;

    for (int attempt = 0; attempt < 200000 && !finished(a); ++attempt) {
        double bestBefore = 0.0;
        bool haveBefore   = get_master_best(bestBefore);

        const double sweepStep = stepSign * curStepAbs;

        remove_file_retry(partPolarAbsPath);
        remove_file_retry(xfoilLogPath);

        const double seedAlpha = haveBefore ? bestBefore : 0.0;

        // write script every attempt
        if (!write_xfoil_script_with_seeded_retry(scriptPath,
                                                  airfoilCmd,
                                                  Re,
                                                  Ncrit,
                                                  iter,
                                                  seedAlpha,
                                                  a,
                                                  end,
                                                  sweepStep,
                                                  "part_polar.dat", // filename only!
                                                  g_enable_xfoil_plots)) {
            return -1;
        }

        if (verbose) {
            cout << "[" << tag << " #" << attempt << "] a=" << a << " step=" << sweepStep << " (|step|=" << curStepAbs
                 << ")" << " seed=" << seedAlpha << "\n";
        }

        int ret = run_xfoil_with_progress_watchdog(cmd, workingDir, partPolarAbsPath, stall_ms, poll_ms);

        if (!file_exists_nonempty(partPolarAbsPath)) {
            cout << "NOTE: XFOIL produced no output; writing debug log -> " << xfoilLogPath << "\n";
            (void)system(cmd_dbg.c_str());
        }

        if (file_exists_nonempty(partPolarAbsPath)) {
            ifstream in(partPolarAbsPath);
            ofstream out(masterRawPolar, ios::app);
            if (in && out) {
                string line;
                while (getline(in, line)) out << line << "\n";
            }
        }

        double bestAfter = 0.0;
        bool haveAfter   = get_master_best(bestAfter);

        if (haveAfter) outBestA = bestAfter;
        else if (haveBefore) outBestA = bestBefore;
        else outBestA = a;

        if (verbose) {
            cout << "    ret=" << ret << " bestBefore=" << (haveBefore ? to_string(bestBefore) : string("NA"))
                 << " bestAfter=" << (haveAfter ? to_string(bestAfter) : string("NA")) << "\n";
        }

        if (haveAfter && reached_end(bestAfter)) return 0;

        bool progressed = false;
        if (haveAfter) {
            if (!haveBefore) progressed = true;
            else progressed = (fabs(bestAfter - bestBefore) > EPS);
        }

        const double bestUsed = haveAfter ? bestAfter : (haveBefore ? bestBefore : a);
        const double nextA    = bestUsed + sweepStep;
        const bool nextEqualsCurrent = (fabs(nextA - a) < EPS);

        const bool stuck = (!progressed) || nextEqualsCurrent;

        if (stuck) {
            successStreak = 0;

            if (haveBefore) {
                double backA = bestBefore - stepSign * (BACKTRACK_EXTRA_STEPS * baseStepAbs);
                a = clamp_to_start_side(backA);
            }

            if (verbose) {
                cout << "    STUCK -> backtrack to a=" << a << " then shrink step\n";
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
    std::ostringstream oss;
    oss << "a" << std::setw(4) << std::setfill('0') << j.airfoilIdx << "_Re" << std::setw(6) << std::setfill('0')
        << j.Re;
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

static int count_done_files(const string& doneDir) {
    std::error_code ec;
    if (!fs::exists(doneDir, ec)) return 0;
    int c = 0;
    for (auto& p : fs::directory_iterator(doneDir, ec)) {
        if (ec) break;
        if (p.is_regular_file()) c++;
    }
    return c;
}

static bool acquire_append_lock(const string& appendLockDir, int tries = 6000, DWORD sleep_ms = 50) {
    std::error_code ec;
    const string p = join_path(appendLockDir, "LOCK");
    for (int i = 0; i < tries; i++) {
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

static bool append_cleaned_to_megapolar_locked(const string& megaPath,
                                               const string& cleanedPath,
                                               const string& airfoilCmd,
                                               int Re,
                                               int Ncrit,
                                               int iter,
                                               const string& appendLockDir) {
    if (!file_exists_nonempty(cleanedPath)) return false;

    if (!acquire_append_lock(appendLockDir)) {
        cout << "ERROR: could not acquire append lock.\n";
        return false;
    }

    bool ok = false;
    {
        ifstream in(cleanedPath);
        if (!in) {
            release_append_lock(appendLockDir);
            return false;
        }

        bool megaExists = false;
        {
            std::error_code ec;
            megaExists = fs::exists(megaPath, ec) && fs::file_size(megaPath, ec) > 0;
        }

        ofstream out(megaPath, ios::app);
        if (!out) {
            release_append_lock(appendLockDir);
            return false;
        }

        if (!megaExists) {
            out << "Airfoil  Re  Ncrit  Iter  "
                << "alpha  CL  CD  CDp  CM  Top_Xtr  Bot_Xtr  Note\n";
        }

        string line;
        while (getline(in, line)) {
            double a = 0, cl = 0, cd = 0, cdp = 0, cm = 0, tx = 0, bx = 0;
            bool hasXtr = false;
            string note;
            if (!parse_polar_row_7_with_note(line, a, cl, cd, cdp, cm, tx, bx, hasXtr, note)) continue;

            out << sanitize_filename(airfoilCmd) << "  " << Re << "  " << Ncrit << "  " << iter << "  "
                << std::fixed << std::setprecision(3) << a << "  " << std::setprecision(4) << cl << "  "
                << std::setprecision(5) << cd << "  " << std::setprecision(5) << cdp << "  " << std::setprecision(4)
                << cm << "  ";

            if (hasXtr) out << std::setprecision(4) << tx << "  " << std::setprecision(4) << bx;
            else out << "NA  NA";

            out << "  " << (note.empty() ? "BASE" : note) << "\n";
        }

        ok = true;
    }

    release_append_lock(appendLockDir);
    return ok;
}

// -------------------- Plot saver --------------------
static void try_run_python_plot_saver(const string& cleanedPolarPath, const string& outPngPath, const string& title) {
    if (!file_exists_nonempty(cleanedPolarPath)) {
        cout << "Plot skipped: missing/empty file: " << cleanedPolarPath << "\n";
        return;
    }

    const string a0 = quote_if_needed(cleanedPolarPath);
    const string a1 = quote_if_needed(outPngPath);
    const string a2 = quote_if_needed(title);

    string cmd = "cmd /C py -3 scripts\\plots\\plot_polar.py " + a0 + " " + a1 + " --title " + a2;
    int ret    = system(cmd.c_str());
    if (ret != 0) {
        cout << "NOTE: python plot saver returned code " << ret << "\n";
        cout << "If plotting packages are missing: py -m pip install -r scripts\\plots\\requirements.txt\n";
    }
}

static bool try_claim_plot_lock(const string& plotLockDir, const string& airfoilSanitized) {
    std::error_code ec;
    return fs::create_directory(join_path(plotLockDir, "PLOT_" + airfoilSanitized), ec);
}

// -------------------- Shared airfoil list --------------------
static vector<string> build_airfoil_list() {
    return {
        "NACA 0006","NACA 0008","NACA 0010",
        //"NACA 0004","NACA 0006","NACA 0008","NACA 0010","NACA 0012","NACA 0015","NACA 0018","NACA 0021","NACA 0024",
        // "NACA 1208","NACA 1209","NACA 1212","NACA 1408","NACA 1409","NACA 1410","NACA 1412","NACA 1415","NACA 1512","NACA 1608","NACA 1609","NACA 1612",
        // "NACA 2208","NACA 2209","NACA 2212","NACA 2408","NACA 2409","NACA 2410","NACA 2412","NACA 2415","NACA 2512","NACA 2608","NACA 2609","NACA 2612",
        // "NACA 3208","NACA 3209","NACA 3212","NACA 3408","NACA 3409","NACA 3410","NACA 3412","NACA 3415","NACA 3512","NACA 3608","NACA 3609","NACA 3612",
        // "NACA 4208","NACA 4209","NACA 4212","NACA 4408","NACA 4409","NACA 4410","NACA 4412","NACA 4415","NACA 4512","NACA 4608","NACA 4609","NACA 4612",
        // "NACA 5208","NACA 5209","NACA 5212","NACA 5408","NACA 5409","NACA 5410","NACA 5412","NACA 5415","NACA 5512","NACA 5608","NACA 5609","NACA 5612",
        // "NACA 6208","NACA 6209","NACA 6212","NACA 6408","NACA 6409","NACA 6410","NACA 6412","NACA 6415","NACA 6512","NACA 6608","NACA 6609","NACA 6612",
        // "NACA 7208","NACA 7209","NACA 7212","NACA 7408","NACA 7409","NACA 7410","NACA 7412","NACA 7415","NACA 7512","NACA 7608","NACA 7609","NACA 7612",
        // "NACA 8208","NACA 8209","NACA 8212","NACA 8408","NACA 8409","NACA 8410","NACA 8412","NACA 8415","NACA 8512","NACA 8608","NACA 8609","NACA 8612",
        // "NACA 9208","NACA 9209","NACA 9212","NACA 9408","NACA 9409","NACA 9410","NACA 9412","NACA 9415","NACA 9512","NACA 9608","NACA 9609","NACA 9612"
    };
}

// -------------------- Dynamic total job count (worker -> launcher) --------------------
static string total_jobs_file_path(const string& stateDir) { return join_path(stateDir, "total_jobs.txt"); }

static void write_total_jobs_if_missing(const string& stateDir, int totalJobs) {
    if (totalJobs <= 0) return;

    const string path = total_jobs_file_path(stateDir);
    std::error_code ec;
    if (fs::exists(path, ec)) return;

    // Write to temp then rename for best-effort atomicity
    const string tmp = path + ".tmp";
    {
        ofstream out(tmp, ios::trunc);
        if (!out) return;
        out << totalJobs << "\n";
    }

    ec.clear();
    fs::rename(tmp, path, ec);
    if (ec) {
        // fallback: overwrite directly
        remove_file_retry(path);
        std::error_code ec2;
        fs::rename(tmp, path, ec2);
        if (ec2) {
            // last resort copy
            std::error_code ec3;
            fs::copy_file(tmp, path, fs::copy_options::overwrite_existing, ec3);
            fs::remove(tmp, ec3);
        }
    }
}

static bool read_total_jobs_if_available(const string& stateDir, int& outTotalJobs) {
    const string path = total_jobs_file_path(stateDir);
    ifstream in(path);
    if (!in) return false;
    int v = 0;
    if (!(in >> v)) return false;
    if (v <= 0) return false;
    outTotalJobs = v;
    return true;
}

// -------------------- FINAL: sort megapolar.dat once at end --------------------
struct MegaRow {
    string airfoil;
    int Re = 0, Ncrit = 0, Iter = 0;
    double alpha = 0.0, CL = 0.0, CD = 0.0, CDp = 0.0, CM = 0.0;
    bool hasXtr = false;
    double topXtr = 0.0, botXtr = 0.0;
    string note;
};

static bool is_token_na(const string& s) {
    if (s.size() != 2) return false;
    return (tolower((unsigned char)s[0]) == 'n' && tolower((unsigned char)s[1]) == 'a');
}

static bool parse_megapolar_data_line(const string& line, MegaRow& r) {
    // skip blank / header
    size_t i = 0;
    while (i < line.size() && isspace((unsigned char)line[i])) i++;
    if (i >= line.size()) return false;
    if (line.compare(i, 6, "Airfoil") == 0) return false;

    std::istringstream iss(line);
    r = MegaRow{};

    if (!(iss >> r.airfoil >> r.Re >> r.Ncrit >> r.Iter >> r.alpha >> r.CL >> r.CD >> r.CDp >> r.CM)) return false;

    string t1, t2;
    if (iss >> t1 >> t2) {
        if (!is_token_na(t1) && !is_token_na(t2)) {
            try {
                r.topXtr = stod(t1);
                r.botXtr = stod(t2);
                r.hasXtr = std::isfinite(r.topXtr) && std::isfinite(r.botXtr);
            } catch (...) {
                r.hasXtr = false;
            }
        } else {
            r.hasXtr = false;
        }
    } else {
        r.hasXtr = false;
    }

    string noteTok;
    if (iss >> noteTok) r.note = noteTok;
    else r.note.clear();

    return true;
}

static bool sort_megapolar_file_inplace(const string& megaPath) {
    if (!file_exists_nonempty(megaPath)) return false;

    ifstream in(megaPath);
    if (!in) return false;

    vector<MegaRow> rows;
    rows.reserve(200000);

    string line;
    while (getline(in, line)) {
        MegaRow r;
        if (!parse_megapolar_data_line(line, r)) continue;
        rows.push_back(std::move(r));
    }
    in.close();

    if (rows.empty()) return false;

    std::sort(rows.begin(), rows.end(), [](const MegaRow& a, const MegaRow& b) {
        // Sort key: Airfoil, then Re, then alpha (then Ncrit/Iter just in case)
        if (a.airfoil != b.airfoil) return a.airfoil < b.airfoil;
        if (a.Re != b.Re) return a.Re < b.Re;
        if (a.Ncrit != b.Ncrit) return a.Ncrit < b.Ncrit;
        if (a.Iter != b.Iter) return a.Iter < b.Iter;
        return a.alpha < b.alpha;
    });

    const string tmpPath = megaPath + ".tmp_sorted";
    ofstream out(tmpPath, ios::trunc);
    if (!out) return false;

    out << "Airfoil  Re  Ncrit  Iter  "
        << "alpha  CL  CD  CDp  CM  Top_Xtr  Bot_Xtr  Note\n";

    for (const auto& r : rows) {
        out << r.airfoil << "  " << r.Re << "  " << r.Ncrit << "  " << r.Iter << "  " << std::fixed
            << std::setprecision(3) << r.alpha << "  " << std::setprecision(4) << r.CL << "  " << std::setprecision(5)
            << r.CD << "  " << std::setprecision(5) << r.CDp << "  " << std::setprecision(4) << r.CM << "  ";

        if (r.hasXtr) out << std::setprecision(4) << r.topXtr << "  " << std::setprecision(4) << r.botXtr;
        else out << "NA  NA";

        out << "  " << (r.note.empty() ? "BASE" : r.note) << "\n";
    }
    out.close();

    // swap in
    remove_file_retry(megaPath);
    std::error_code ec;
    fs::rename(tmpPath, megaPath, ec);
    if (ec) {
        // fallback copy if rename fails (OneDrive weirdness)
        std::error_code ec2;
        fs::copy_file(tmpPath, megaPath, fs::copy_options::overwrite_existing, ec2);
        fs::remove(tmpPath, ec2);
        return !ec2;
    }
    return true;
}

// -------------------- Worker routine --------------------
static int run_worker(int workerId, int nWorkers, const string& effectiveTag) {
    const string exeDir = get_exe_dir();
    const string projectRoot = fs::path(exeDir).parent_path().parent_path().string();
    if (!SetCurrentDirectoryA(projectRoot.c_str())) {
        cout << "ERROR: could not set project working directory: " << projectRoot << "\n";
        return 2;
    }

    const string xfoilExe = join_path(join_path(projectRoot, "tools\\xfoil"), "xfoil.exe");
    if (!path_exists(xfoilExe)) {
        cout << "ERROR: could not find xfoil.exe at: " << xfoilExe << "\n";
        cout << "Fix: make sure tools\\xfoil\\xfoil.exe exists in the project.\n";
        return 2;
    }

    struct Config {
        int Ncrit = 5;
        int iter  = 250;

        int Re_start = 1000;
        int Re_end   = 160000;
        int Re_step  = 500;

        double aMin  = -20.0;
        double aMax  = 20.0;
        double aStep = 0.25;

        bool verbose = true;

        string out_dir       = "data\\polars\\generated";
        string mega_filename = "megapolar.dat";

        int plot_re_only = 80000;

        DWORD poll_ms  = 250;
        DWORD stall_ms = 20000;

        vector<string> airfoils = build_airfoil_list();
    } cfg;

    const string OUT_DIR = cfg.out_dir;

    const string STATE_ROOT = join_path(get_temp_dir(), "xfoil_megastate");
    const string STATE_DIR  = join_path(STATE_ROOT, "mega_state_" + sanitize_filename(effectiveTag));
    const string LOCK_DIR   = join_path(STATE_DIR, "locks");
    const string DONE_DIR   = join_path(STATE_DIR, "done");
    const string APP_LOCK   = join_path(STATE_DIR, "append_lock");
    const string PLOT_LOCKS = join_path(STATE_DIR, "plot_locks");
    const string TEMP_DIR   = join_path(join_path(STATE_DIR, "temp"), string("w") + to_string(workerId));

    // ---- OneDrive workaround: write megapolar.dat to TEMP state dir during run ----
    const string MEGA_PATH = join_path(STATE_DIR, cfg.mega_filename);

    if (!ensure_dir(OUT_DIR) || !ensure_dir(STATE_ROOT) || !ensure_dir(STATE_DIR) || !ensure_dir(LOCK_DIR) ||
        !ensure_dir(DONE_DIR) || !ensure_dir(APP_LOCK) || !ensure_dir(PLOT_LOCKS) || !ensure_dir(TEMP_DIR)) {
        cout << "ERROR: could not create state dirs.\n";
        return 2;
    }

    remove_all_retry(TEMP_DIR);
    ensure_dir(TEMP_DIR);

    DWORD pid = GetCurrentProcessId();

    const string TMP_MASTER_RAW = join_path(TEMP_DIR, "master_raw.dat");
    const string TMP_PART_POLAR = join_path(TEMP_DIR, "part_polar.dat");
    const string TMP_SCRIPT     = join_path(TEMP_DIR, "xfoil_script.in");
    const string TMP_CLEANED    = join_path(TEMP_DIR, "cleaned_polar.dat");
    const string XFOIL_LOG      = join_path(TEMP_DIR, "xfoil_last.log");

    // Run xfoil.exe directly; set working dir to TEMP_DIR so outputs are plain filenames.
    const string xfoilExeQ = quote_if_needed(win_path(xfoilExe));
    const string scriptQ   = quote_if_needed(win_path(TMP_SCRIPT));
    const string logQ      = quote_if_needed(win_path(XFOIL_LOG));

    string cmd     = "cmd /C " + xfoilExeQ + " < " + scriptQ + " > " + logQ + " 2>&1";
    string cmd_dbg = cmd;

    vector<int> Re_list;
    for (int Re = cfg.Re_start; Re <= cfg.Re_end; Re += cfg.Re_step) Re_list.push_back(Re);

    vector<Job> jobs;
    for (int ai = 0; ai < (int)cfg.airfoils.size(); ++ai)
        for (int Re : Re_list) jobs.push_back({ai, Re});

    // Publish the true total job count (computed from real worker job list),
    // so launcher progress never goes out-of-sync with config changes.
    if (workerId == 0) {
        write_total_jobs_if_missing(STATE_DIR, (int)jobs.size());
    }

    cout << "Worker " << workerId << "/" << nWorkers << " PID=" << pid << " tag=" << effectiveTag
         << " jobs=" << jobs.size() << "\n";

    int okCount   = 0;
    int failCount = 0;

    for (size_t pass = 0; pass < jobs.size() * 3; ++pass) {
        bool didWorkThisPass = false;

        for (const auto& j : jobs) {
            const string key = job_key(j);

            if (is_done(DONE_DIR, key)) continue;
            if (!try_claim_job_lock(LOCK_DIR, key)) continue;

            didWorkThisPass = true;

            const string& airfoilCmd = cfg.airfoils[j.airfoilIdx];
            const int Re             = j.Re;

            cout << "------------------------------------------------------------\n";
            cout << "Worker " << workerId << " running: " << airfoilCmd << "  Re=" << Re << " (job " << key << ")\n";

            remove_file_retry(TMP_CLEANED);
            remove_file_retry(TMP_MASTER_RAW);
            remove_file_retry(TMP_PART_POLAR);
            remove_file_retry(TMP_SCRIPT);
            remove_file_retry(XFOIL_LOG);

            const double step = fabs(cfg.aStep);

            double bestPos = 0.0;
            int retP = run_sweep_try_once_then_stop(cmd,
                                                   cmd_dbg,
                                                   TEMP_DIR,
                                                   XFOIL_LOG,
                                                   TMP_SCRIPT,
                                                   airfoilCmd,
                                                   Re,
                                                   cfg.Ncrit,
                                                   cfg.iter,
                                                   0.0,
                                                   cfg.aMax,
                                                   +step,
                                                   cfg.stall_ms,
                                                   cfg.poll_ms,
                                                   TMP_MASTER_RAW,
                                                   TMP_PART_POLAR,
                                                   "pos",
                                                   0.0,
                                                   cfg.aMax,
                                                   true,
                                                   bestPos,
                                                   cfg.verbose);

            double bestNeg = 0.0;
            int retN = run_sweep_try_once_then_stop(cmd,
                                                   cmd_dbg,
                                                   TEMP_DIR,
                                                   XFOIL_LOG,
                                                   TMP_SCRIPT,
                                                   airfoilCmd,
                                                   Re,
                                                   cfg.Ncrit,
                                                   cfg.iter,
                                                   0.0,
                                                   cfg.aMin,
                                                   -step,
                                                   cfg.stall_ms,
                                                   cfg.poll_ms,
                                                   TMP_MASTER_RAW,
                                                   TMP_PART_POLAR,
                                                   "neg",
                                                   cfg.aMin,
                                                   0.0,
                                                   false,
                                                   bestNeg,
                                                   cfg.verbose);

            bool haveFinal = false;
            if (file_exists_nonempty(TMP_MASTER_RAW)) {
                if (cleanup_polar_file_to_grid(TMP_MASTER_RAW, TMP_CLEANED, cfg.aStep)) {
                    haveFinal = true;
                }
            }

            bool okAppend = false;
            if (haveFinal) {
                okAppend = append_cleaned_to_megapolar_locked(
                    MEGA_PATH, TMP_CLEANED, airfoilCmd, Re, cfg.Ncrit, cfg.iter, APP_LOCK);
            }

            bool plotMade = false;
            if (haveFinal && (Re == cfg.plot_re_only)) {
                const string airfoilSafe = sanitize_filename(airfoilCmd);
                if (try_claim_plot_lock(PLOT_LOCKS, airfoilSafe)) {
                    std::ostringstream caseTag;
                    caseTag << airfoilSafe << "_Re" << Re << "_N" << cfg.Ncrit;
                    const string outPng = join_path(OUT_DIR, caseTag.str() + "_4plots.png");
                    cout << "Saving PNG -> " << outPng << "\n";
                    try_run_python_plot_saver(TMP_CLEANED, outPng, caseTag.str());
                    plotMade = true;
                }
            }

            // cleanup scratch files immediately
            remove_file_retry(TMP_CLEANED);
            remove_file_retry(TMP_PART_POLAR);
            remove_file_retry(TMP_SCRIPT);
            remove_file_retry(TMP_MASTER_RAW);

            bool jobFailed = (retP == 124 || retN == 124 || !haveFinal || !okAppend);
            if (!jobFailed) remove_file_retry(XFOIL_LOG);

            std::ostringstream msg;
            msg << "retP=" << retP << " retN=" << retN << " bestPos=" << bestPos << " bestNeg=" << bestNeg
                << " haveFinal=" << (haveFinal ? 1 : 0) << " appended=" << (okAppend ? 1 : 0)
                << " plot=" << (plotMade ? 1 : 0) << " worker=" << workerId << " pid=" << pid;

            mark_done(DONE_DIR, key, msg.str());
            release_job_lock(LOCK_DIR, key);

            if (jobFailed) failCount++;
            else okCount++;
        }

        if (!didWorkThisPass) break;
    }

    cout << "Worker " << workerId << " finished. ok=" << okCount << " fails=" << failCount << "\n";
    return (failCount > 0) ? 124 : 0;
}

// -------------------- Launcher routine --------------------
static int run_launcher(const string& exePath, int nWorkers, const string& runTag) {
    string effectiveTag = sanitize_filename(runTag) + "_" + timestamp_YYYYMMDD_HHMMSS();

    cout << "Launcher spawning " << nWorkers << " workers... tag=" << runTag << "\n";
    cout << "Effective run tag: " << effectiveTag << "\n";

    const string OUT_DIR   = "data\\polars\\generated";
    const string MEGA_PATH = join_path(OUT_DIR, "megapolar.dat");

    const string STATE_ROOT = join_path(get_temp_dir(), "xfoil_megastate");
    const string STATE_DIR  = join_path(STATE_ROOT, "mega_state_" + effectiveTag);
    const string DONE_DIR   = join_path(STATE_DIR, "done");

    // ---- OneDrive workaround: workers write mega here during run ----
    const string MEGA_TMP_PATH = join_path(STATE_DIR, "megapolar.dat");

    cout << "Scratch/state dir (auto-deleted): " << STATE_DIR << "\n";

    ensure_dir(OUT_DIR);
    ensure_dir(STATE_ROOT);

    // Final output file in OneDrive/project folder (do not write hot during run)
    remove_file_retry(MEGA_PATH);

    // Ensure temp mega is clean too (belt + suspenders)
    remove_file_retry(MEGA_TMP_PATH);

    // remove old plots in the generated-output directory
    {
        std::error_code ec;
        if (fs::exists(OUT_DIR, ec)) {
            for (auto& p : fs::directory_iterator(OUT_DIR, ec)) {
                if (ec) break;
                if (!p.is_regular_file()) continue;
                const string fn = p.path().filename().string();
                if (fn.find("_4plots.png") != string::npos) remove_file_retry(p.path().string());
            }
        }
    }

    ensure_dir(join_path(STATE_DIR, "done"));
    ensure_dir(join_path(STATE_DIR, "locks"));
    ensure_dir(join_path(STATE_DIR, "append_lock"));
    ensure_dir(join_path(STATE_DIR, "plot_locks"));
    ensure_dir(join_path(STATE_DIR, "temp"));

    vector<PROCESS_INFORMATION> pis;
    pis.reserve(nWorkers);

    for (int i = 0; i < nWorkers; ++i) {
        STARTUPINFOA si{};
        PROCESS_INFORMATION pi{};
        si.cb = sizeof(si);

        std::ostringstream cmd;
        cmd << "\"" << exePath << "\""
            << " --worker " << i << " " << nWorkers
            << " --tag " << effectiveTag;

        string cmdStr = cmd.str();
        cmdStr.push_back('\0');

        BOOL ok = CreateProcessA(nullptr,
                                 &cmdStr[0],
                                 nullptr,
                                 nullptr,
                                 FALSE,
                                 CREATE_NEW_CONSOLE,
                                 nullptr,
                                 nullptr,
                                 &si,
                                 &pi);

        if (!ok) {
            cout << "ERROR: failed to spawn worker " << i << " (GetLastError=" << GetLastError() << ")\n";
            return 2;
        }

        pis.push_back(pi);
    }

    auto t0 = std::chrono::steady_clock::now();
    int lastDone = -1;

    // totalJobs is now dynamic: published by worker0 (or any worker) to STATE_DIR/total_jobs.txt
    int totalJobs = -1;

    // For slightly smoother updates (bar/ETA), poll more often than 30s.
    // This does NOT touch worker logic; it's just the launcher's display.
    const DWORD POLL_MS = 5000;

    while (true) {
        bool allExited = true;
        for (auto& pi : pis) {
            DWORD rc = WaitForSingleObject(pi.hProcess, 0);
            if (rc != WAIT_OBJECT_0) {
                allExited = false;
                break;
            }
        }

        int doneNow = count_done_files(DONE_DIR);

        // Refresh totalJobs if available (becomes "known" as soon as worker0 writes it)
        if (totalJobs <= 0) {
            int tmp = -1;
            if (read_total_jobs_if_available(STATE_DIR, tmp)) totalJobs = tmp;
        }

        if (doneNow != lastDone || allExited) {
            auto tNow = std::chrono::steady_clock::now();
            double elapsedSec = std::chrono::duration<double>(tNow - t0).count();
            if (elapsedSec < 1e-9) elapsedSec = 1e-9;

            double rate_per_sec = doneNow / elapsedSec;
            double cph = rate_per_sec * 3600.0;

            double frac = 0.0;
            int remaining = 0;
            double etaSec = std::numeric_limits<double>::infinity();

            bool haveTotal = (totalJobs > 0);
            if (haveTotal) {
                frac = (double)doneNow / (double)totalJobs;
                if (frac < 0.0) frac = 0.0;
                if (frac > 1.0) frac = 1.0;

                remaining = totalJobs - doneNow;
                if (remaining < 0) remaining = 0;

                etaSec = (rate_per_sec > 1e-12) ? (remaining / rate_per_sec) : std::numeric_limits<double>::infinity();
            } else {
                // total unknown yet: show done/? and no ETA
                frac = 0.0;
                remaining = 0;
                etaSec = std::numeric_limits<double>::infinity();
            }

            const int BAR_W = 32;
            string bar = make_bar(frac, BAR_W);

            std::ostringstream line;
            if (haveTotal) {
                line << "\r[Progress] " << std::setw(5) << doneNow << "/" << totalJobs << "  " << std::fixed
                     << std::setprecision(2) << (100.0 * frac) << "% "
                     << "[" << bar << "] ";
            } else {
                line << "\r[Progress] " << std::setw(5) << doneNow << "/?"
                     << "  "
                     << "??.?% "
                     << "[" << bar << "] ";
            }

            line << "  " << std::fixed << std::setprecision(1) << cph << " cases/hr"
                 << "  ETA " << (haveTotal ? format_hms(etaSec) : string("NA"))
                 << "  elapsed " << format_hms(elapsedSec) << "   ";

            cout << line.str() << std::flush;

            lastDone = doneNow;

            if (allExited) cout << "\n";
        }

        if (allExited) break;
        Sleep(POLL_MS);
    }

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

    // ---- OneDrive workaround: copy temp megapolar.dat into generated outputs at end ----
    if (file_exists_nonempty(MEGA_TMP_PATH)) {
        cout << "Finalizing megapolar.dat -> " << MEGA_PATH << "\n";
        remove_file_retry(MEGA_PATH);
        std::error_code ec;
        fs::copy_file(MEGA_TMP_PATH, MEGA_PATH, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            cout << "NOTE: final copy to generated outputs failed: " << ec.message() << "\n";
        }
    } else {
        cout << "NOTE: temp megapolar missing/empty: " << MEGA_TMP_PATH << "\n";
    }

    // ---- FINAL SORT ONCE (per your request) ----
    if (file_exists_nonempty(MEGA_PATH)) {
        cout << "Sorting megapolar.dat...\n";
        if (!sort_megapolar_file_inplace(MEGA_PATH)) {
            cout << "NOTE: sort failed (empty file or parse issue).\n";
        }
    }

    cout << "Cleaning scratch/state dir...\n";
    remove_all_retry(STATE_DIR);

    return worst;
}

// -------------------- CLI helpers --------------------
static bool arg_equals(const char* a, const char* b) { return _stricmp(a, b) == 0; }
static int clamp_int(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi ? hi : v); }

// -------------------- main --------------------
int main(int argc, char** argv) {
    const string projectRoot = fs::path(get_exe_dir()).parent_path().parent_path().string();
    if (!SetCurrentDirectoryA(projectRoot.c_str())) {
        cout << "ERROR: could not set project working directory: " << projectRoot << "\n";
        return 2;
    }
    int nWorkers = 20;
    string runTag = "main";

    if (argc >= 2 && arg_equals(argv[1], "--worker")) {
        if (argc < 4) {
            cout << "Usage: polarParalleler --worker <id> <N> --tag <runTag>\n";
            return 2;
        }

        int wid = atoi(argv[2]);
        int nW  = max(1, atoi(argv[3]));

        string tag = "main";
        for (int i = 4; i < argc; ++i) {
            if (arg_equals(argv[i], "--tag") && i + 1 < argc) {
                tag = argv[i + 1];
                i++;
            }
        }

        if (wid < 0 || wid >= nW) {
            cout << "ERROR: worker id out of range.\n";
            return 2;
        }
        return run_worker(wid, nW, tag);
    }

    for (int i = 1; i < argc; ++i) {
        if (arg_equals(argv[i], "--workers") && i + 1 < argc) {
            nWorkers = atoi(argv[i + 1]);
            i++;
        } else if (arg_equals(argv[i], "--tag") && i + 1 < argc) {
            runTag = argv[i + 1];
            i++;
        }
    }

    nWorkers = clamp_int(nWorkers, 1, 32);

    char exeBuf[MAX_PATH]{0};
    GetModuleFileNameA(nullptr, exeBuf, MAX_PATH);
    string exePath = exeBuf;
    if (exePath.empty()) exePath = "polarParalleler.exe";

    return run_launcher(exePath, nWorkers, runTag);
}
