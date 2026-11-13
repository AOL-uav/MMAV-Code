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

using namespace std;
namespace fs = std::filesystem;

// -------------------- GLOBAL toggle (set in main) --------------------
static bool g_enable_xfoil_plots = true;

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

static bool append_polar_rows_retry(const string& srcPolar, const string& dstPolar, int tries = 40, DWORD sleep_ms = 50);

// -------------------- NEW: filename helpers --------------------

static string sanitize_for_filename(const string& s) {
    // Keep only [A-Za-z0-9_-], convert whitespace to '_', drop everything else.
    // Also collapse multiple '_' into one.
    string out;
    out.reserve(s.size());

    auto is_ok = [](unsigned char c) {
        return std::isalnum(c) || c == '_' || c == '-';
    };

    bool lastUnderscore = false;
    for (unsigned char c : s) {
        if (std::isspace(c)) {
            if (!lastUnderscore) {
                out.push_back('_');
                lastUnderscore = true;
            }
            continue;
        }

        if (is_ok(c)) {
            out.push_back((char)c);
            lastUnderscore = (c == '_');
        } else {
            // replace other punctuation with '_' (but collapse repeats)
            if (!lastUnderscore) {
                out.push_back('_');
                lastUnderscore = true;
            }
        }
    }

    // trim leading/trailing underscores
    while (!out.empty() && out.front() == '_') out.erase(out.begin());
    while (!out.empty() && out.back() == '_') out.pop_back();

    if (out.empty()) out = "case";
    return out;
}

static string make_case_prefix(int Re, int Ncrit, const string& airfoilCmd) {
    // Example: Re200000_N9_NACA_0001  -> sanitized to Re200000_N9_NACA_0001 (then underscores trimmed/collapsed)
    // You can tweak formatting here if you want shorter names.
    std::ostringstream oss;
    oss << "Re" << Re << "_N" << Ncrit << "_" << airfoilCmd;
    return sanitize_for_filename(oss.str());
}

// -------------------- polar parsing helpers --------------------

static bool parse_polar_row_7(
    const string& line,
    double& a, double& cl, double& cd, double& cdp, double& cm,
    double& topXtr, double& botXtr,
    bool& hasXtr
) {
    size_t i = 0;
    while (i < line.size() && isspace((unsigned char)line[i])) i++;
    if (i >= line.size()) return false;

    char c = line[i];
    if (!(c=='-' || c=='+' || c=='.' || (c>='0' && c<='9'))) return false;

    std::istringstream iss(line);
    if (!(iss >> a >> cl >> cd >> cdp >> cm)) return false;

    if (!std::isfinite(a) || !std::isfinite(cl) || !std::isfinite(cd) || !std::isfinite(cdp) || !std::isfinite(cm))
        return false;

    // CD sanity: reject inf/garbage / completely blown solutions
    if (cd <= 0.0 || cd > 5.0) return false;

    // CL sanity (keeps totally bogus "CL=11" rows from counting as progress)
    if (fabs(cl) > 5.0) return false;

    // Try to read Xtr columns if present
    hasXtr = false;
    topXtr = botXtr = 0.0;
    if (iss >> topXtr >> botXtr) {
        if (std::isfinite(topXtr) && std::isfinite(botXtr)) hasXtr = true;
    }

    return true;
}

// Best alpha in [lo,hi] from a polar file.
static bool get_best_alpha_in_range(const string& polarPath, double lo, double hi, bool wantMax, double& bestOut) {
    ifstream in(polarPath);
    if (!in) return false;

    const double aLo = min(lo, hi);
    const double aHi = max(lo, hi);

    string line;
    bool found = false;
    double bestA = 0.0;

    while (getline(in, line)) {
        double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
        bool hasXtr=false;
        if (!parse_polar_row_7(line, a, cl, cd, cdp, cm, tx, bx, hasXtr)) continue;

        if (a < aLo - 1e-9 || a > aHi + 1e-9) continue;

        if (!found) { bestA = a; found = true; }
        else        { bestA = wantMax ? max(bestA, a) : min(bestA, a); }
    }

    if (!found) return false;
    bestOut = bestA;
    return true;
}

static bool append_polar_rows(const string& srcPolar, const string& dstPolar) {
    ifstream in(srcPolar);
    if (!in) return false;

    ofstream out(dstPolar, ios::app);
    if (!out) return false;

    string line;
    while (getline(in, line)) {
        double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
        bool hasXtr=false;
        if (!parse_polar_row_7(line, a, cl, cd, cdp, cm, tx, bx, hasXtr)) continue;
        out << line << "\n";
    }
    return true;
}

static bool append_polar_rows_retry(const string& srcPolar, const string& dstPolar, int tries, DWORD sleep_ms) {
    for (int i = 0; i < tries; ++i) {
        if (append_polar_rows(srcPolar, dstPolar)) return true;
        Sleep(sleep_ms);
    }
    return false;
}

static bool file_exists_nonempty(const string& p) {
    std::error_code ec;
    return fs::exists(p, ec) && fs::file_size(p, ec) > 0;
}

// -------------------- cleanup: grid + dedupe + sort --------------------

struct PolarRow {
    double a=0, cl=0, cd=0, cdp=0, cm=0, tx=0, bx=0;
    bool hasXtr=false;
};

static long long grid_key(double a, double step) {
    return (long long) llround(a / step);
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

    std::map<long long, PolarRow> bestByKey;
    std::map<long long, double>   bestCdByKey;

    string line;
    while (getline(in, line)) {
        PolarRow r;
        if (!parse_polar_row_7(line, r.a, r.cl, r.cd, r.cdp, r.cm, r.tx, r.bx, r.hasXtr)) continue;

        long long k = grid_key(r.a, step);
        double snapped = k * step;

        if (fabs(r.a - snapped) > 1e-3) continue;

        auto it = bestCdByKey.find(k);
        if (it == bestCdByKey.end() || r.cd < it->second) {
            bestCdByKey[k] = r.cd;
            bestByKey[k]   = r;
        }
    }

    ofstream out(outPath, ios::trunc);
    if (!out) return false;

    out << "  alpha        CL          CD         CDp         CM      Top_Xtr  Bot_Xtr\n";

    out << std::fixed << std::setprecision(3);
    for (const auto& kv : bestByKey) {
        const PolarRow& r = kv.second;
        const double aSnap = kv.first * step;

        out << std::setw(7) << aSnap << "  "
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
        out << "\n";
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
//
// Implemented changes:
//  1) actually USE seedAlpha by starting ASEQ from seedAlpha (continuity).
//  2) remove ALFA 0 + INIT reset that destroys the seed.
//

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
    bool enablePlots = true
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
    if (useXtr) file << "XTR 0.10 0.10\n";
    file << "\n";

    file << "ITER " << iter << "\n";

    file << "PACC\n";
    file << "..\\" << polarOut << "\n";
    file << "\n";

    // Seed at last known good alpha
    file << "ALFA " << seedAlpha << "\n";

    // Attempt #1: keep continuity, init at seed, and sweep from seedAlpha
    file << "INIT\n";
    file << "ASEQ " << seedAlpha << " " << sweepEnd << " " << sweepStep << "\n";

    // Attempt #2: try starting exactly at requested sweepStart (no INIT)
    file << "ALFA " << sweepStart << "\n";
    file << "ASEQ " << sweepStart << " " << sweepEnd << " " << sweepStep << "\n";

    file << "PACC\n";
    file << "\n";
    file << "QUIT\n";

    return true;
}

// -------------------- sweep runner: adaptive step + multi-step backtrack --------------------

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
    const string& masterPolar,
    const string& casePrefix,
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
    const double minStepAbs            = max(0.01, baseStepAbs / 64.0);
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
        if (!file_exists_nonempty(masterPolar)) return false;
        return get_best_alpha_in_range(masterPolar, progressLo, progressHi, wantMax, outA);
    };

    auto clamp_to_start_side = [&](double x) {
        if (stepSign > 0) return max(x, start);
        else              return min(x, start);
    };

    double a = start;

    for (int attempt = 0; attempt < 200000 && !finished(a); ++attempt) {
        double bestBefore = 0.0;
        bool haveBefore = get_master_best(bestBefore);

        const double sweepStep = stepSign * curStepAbs;

        string partPolar = "polar_" + casePrefix + "_" + tag + "_" + to_string(attempt) + ".dat";
        remove_file_retry(partPolar);

        const double seedAlpha = haveBefore ? bestBefore : 0.0;

        if (!write_xfoil_script_with_seeded_retry(
                scriptPath, airfoilCmd, Re, Ncrit, iter,
                seedAlpha,
                a, end, sweepStep, partPolar, useXtr,
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

        if (file_exists_nonempty(partPolar)) {
            if (!fs::exists(masterPolar)) {
                copy_file_retry(partPolar, masterPolar);
            } else {
                append_polar_rows_retry(partPolar, masterPolar);
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

            if (verbose) {
                cout << "    step growth -> |step|=" << curStepAbs << "\n";
            }
        }
    }

    return 124;
}

// -------------------- main --------------------

int main() {
    int Re = 92000;
    int Ncrit = 5;
    int iter = 250;

    string airfoilCmd = "NACA 0001";

    double aMin = -35.0, aMax = 35.0, aStep = 0.25;

    const bool ENABLE_XFOIL_PLOTS = true;

    const string casePrefix = make_case_prefix(Re, Ncrit, airfoilCmd);

    string polarOut   = "polar_"  + casePrefix + ".dat";
    string scriptPath = "script_" + casePrefix + ".in";

    const bool VERBOSE_DIAGNOSTICS = true;
    const bool SILENCE_XFOIL       = true;

    const string XFOIL_LOG_REL = "..\\xfoil_" + casePrefix + ".log";

    g_enable_xfoil_plots = ENABLE_XFOIL_PLOTS;

    cout << "Running case: Re=" << Re << " Ncrit=" << Ncrit << " iter=" << iter << "\n";
    cout << "Airfoil: " << airfoilCmd << "\n";
    cout << "Case prefix: " << casePrefix << "\n";
    cout << "XFOIL plots: " << (ENABLE_XFOIL_PLOTS ? "ON" : "OFF") << "\n";

    if (fs::exists(polarOut)) remove_file_retry(polarOut);

    string cmd;
    if (SILENCE_XFOIL) {
        // CHANGE #1: use >> so we append attempt outputs into one log file
        cmd = "cmd /C \"cd /d xfoil && xfoil.exe < ..\\" + scriptPath + " >> " + XFOIL_LOG_REL + " 2>&1\"";
    } else {
        cmd = "cmd /C \"cd /d xfoil && xfoil.exe < ..\\" + scriptPath + "\"";
    }

    DWORD poll_ms  = 250;
    DWORD stall_ms = 20000;

    const double step = fabs(aStep);

    double bestPos = 0.0;
    int retP = run_sweep_try_once_then_stop(
        cmd, scriptPath, airfoilCmd,
        Re, Ncrit, iter,
        true,
        0.0, aMax, +step,
        stall_ms, poll_ms,
        polarOut,
        casePrefix,
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
        cmd, scriptPath, airfoilCmd,
        Re, Ncrit, iter,
        true,
        0.0, aMin, -step,
        stall_ms, poll_ms,
        polarOut,
        casePrefix,
        "neg",
        aMin, 0.0,
        false,
        bestNeg,
        VERBOSE_DIAGNOSTICS
    );

    cout << "Negative sweep ret=" << retN << " bestNeg=" << bestNeg << "\n";
    if (retN == 124) cout << "WARNING: negative sweep stalled near next alpha ~" << (bestNeg - step) << "\n";

    if (retN == 124) {
        double target = bestNeg - step;
        if (target < aMin) target = aMin;

        if (bestNeg - aMin > 1e-6) {
            double bestBU = aMin;
            int retBU = run_sweep_try_once_then_stop(
                cmd, scriptPath, airfoilCmd,
                Re, Ncrit, iter,
                true,
                aMin, target, +step,
                stall_ms, poll_ms,
                polarOut,
                casePrefix,
                "bottomup",
                aMin, target,
                true,
                bestBU,
                VERBOSE_DIAGNOSTICS
            );
            cout << "Bottom-up sweep ret=" << retBU << "\n";
            if (retBU == 124) cout << "WARNING: bottom-up also stalled\n";
        }
    }

    if (fs::exists(polarOut)) cout << "Wrote " << polarOut << "\n";
    else cout << "WARNING: " << polarOut << " not found\n";

    if (SILENCE_XFOIL) {
        cout << "(XFOIL output redirected to xfoil\\xfoil_" << casePrefix << ".log)\n";
    }

    if (fs::exists(polarOut)) {
        const string rawBackup = "polar_" + casePrefix + "_raw.dat";
        const string cleanOut  = "polar_" + casePrefix + "_clean.dat";

        copy_file_retry(polarOut, rawBackup);

        bool ok = cleanup_polar_file_to_grid(polarOut, cleanOut, aStep);
        if (ok) {
            copy_file_retry(cleanOut, polarOut);
            cout << "Cleaned polar -> " << cleanOut << " (and overwrote " << polarOut << ")\n";
            cout << "Raw backup -> " << rawBackup << "\n";
        } else {
            cout << "WARNING: cleanup failed; leaving original " << polarOut << " untouched\n";
        }
    }

    if (retP == 124 || retN == 124) return 124;
    if (retP != 0) return retP;
    return retN;
}
