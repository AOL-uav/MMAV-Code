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

    // Map keeps keys sorted; each key -> best row at that alpha
    // "Best" = lowest CD (usually the least-broken solution)
    std::map<long long, PolarRow> bestByKey;
    std::map<long long, double>   bestCdByKey;

    string line;
    while (getline(in, line)) {
        PolarRow r;
        if (!parse_polar_row_7(line, r.a, r.cl, r.cd, r.cdp, r.cm, r.tx, r.bx, r.hasXtr)) continue;

        long long k = grid_key(r.a, step);
        double snapped = k * step;

        // Keep only base-grid rows (e.g., multiples of 0.25). This will drop adaptive micro-steps.
        // Tolerance: because XFOIL prints ~3 decimals, this is plenty.
        if (fabs(r.a - snapped) > 1e-3) continue;

        auto it = bestCdByKey.find(k);
        if (it == bestCdByKey.end() || r.cd < it->second) {
            bestCdByKey[k] = r.cd;
            bestByKey[k]   = r;
        }
    }

    ofstream out(outPath, ios::trunc);
    if (!out) return false;

    // Optional header (compact)
    out << "  alpha        CL          CD         CDp         CM      Top_Xtr  Bot_Xtr\n";

    out << std::fixed << std::setprecision(3);
    for (const auto& kv : bestByKey) {
        const PolarRow& r = kv.second;

        // Snap alpha to exact grid value for cleanliness
        const double aSnap = kv.first * step;

        // Match a typical XFOIL-ish alignment
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

        // restore precision for next alpha formatting
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

    // IMPORTANT: CreateProcess wants a mutable buffer.
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

    // initialize
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
        file << "G\n";   // toggle graphics
        file << "\n";    // exit PLOP menu
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

    // Attempt #1
    file << "ALFA 0\n";
    file << "INIT\n";
    file << "ASEQ " << sweepStart << " " << sweepEnd << " " << sweepStep << "\n";

    // Attempt #2 (seed retry, no INIT)
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

    // Tuneables:
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

    // clamp so we never backtrack past the sweep start
    auto clamp_to_start_side = [&](double x) {
        if (stepSign > 0) return max(x, start);
        else              return min(x, start);
    };

    double a = start;

    for (int attempt = 0; attempt < 200000 && !finished(a); ++attempt) {
        double bestBefore = 0.0;
        bool haveBefore = get_master_best(bestBefore);

        const double sweepStep = stepSign * curStepAbs;

        string partPolar = "polar_" + tag + "_" + to_string(attempt) + ".dat";
        if (fs::exists(partPolar)) fs::remove(partPolar);

        if (!write_xfoil_script_with_seeded_retry(
                scriptPath, airfoilCmd, Re, Ncrit, iter,
                a, end, sweepStep, partPolar, useXtr,
                g_enable_xfoil_plots)) {
            return -1;
        }

        if (verbose) {
            cout << "[" << tag << " #" << attempt << "] a=" << a
                 << " step=" << sweepStep
                 << " (|step|=" << curStepAbs << ")\n";
        }

        int ret = run_xfoil_with_progress_watchdog(cmd, partPolar, stall_ms, poll_ms);

        if (file_exists_nonempty(partPolar)) {
            if (!fs::exists(masterPolar)) {
                fs::copy_file(partPolar, masterPolar, fs::copy_options::overwrite_existing);
            } else {
                append_polar_rows(partPolar, masterPolar);
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

            // back up "another step or two" from last known good alpha
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

            return 124; // truly stuck
        }

        // progressed: advance
        a = nextA;
        successStreak++;

        // grow step back up after clean progress streak
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
    int Re = 140000;
    int Ncrit = 5;
    int iter = 500;

    string airfoilCmd = "NACA 0001";

    double aMin = -15.0, aMax = 35.0, aStep = 0.25;

    // --- Easy toggle next to the run params ---
    const bool ENABLE_XFOIL_PLOTS = false;

    string polarOut = "polar_test.dat";
    string scriptPath = "test_script.in";

    // --- Console/diagnostics control ---
    const bool VERBOSE_DIAGNOSTICS = true;   // your useful 1-line summaries
    const bool SILENCE_XFOIL       = true;   // stop the wall of text
    const string XFOIL_LOG_REL     = "..\\xfoil_console.log";

    // set global used inside sweep runner
    g_enable_xfoil_plots = ENABLE_XFOIL_PLOTS;

    cout << "Running case: Re=" << Re << " Ncrit=" << Ncrit << " iter=" << iter << "\n";
    cout << "XFOIL plots: " << (ENABLE_XFOIL_PLOTS ? "ON" : "OFF") << "\n";

    if (fs::exists(polarOut)) fs::remove(polarOut);

    // Build command:
    // NOTE: we redirect XFOIL output to a file to avoid terminal spam.
    string cmd;
    if (SILENCE_XFOIL) {
        cmd = "cmd /C \"cd /d xfoil && xfoil.exe < ..\\test_script.in > " + XFOIL_LOG_REL + " 2>&1\"";
    } else {
        cmd = "cmd /C \"cd /d xfoil && xfoil.exe < ..\\test_script.in\"";
    }

    DWORD poll_ms  = 250;
    DWORD stall_ms = 20000;

    const double step = fabs(aStep);

    // ---- Positive sweep: 0 -> aMax
    double bestPos = 0.0;
    int retP = run_sweep_try_once_then_stop(
        cmd, scriptPath, airfoilCmd,
        Re, Ncrit, iter,
        true,
        0.0, aMax, +step,
        stall_ms, poll_ms,
        polarOut,
        "pos",
        0.0, aMax,
        true,
        bestPos,
        VERBOSE_DIAGNOSTICS
    );

    cout << "Positive sweep ret=" << retP << " bestPos=" << bestPos << "\n";
    if (retP == 124) cout << "WARNING: positive sweep stalled near next alpha ~" << (bestPos + step) << "\n";

    // ---- If positive stuck, ONE top-down attempt
    if (retP == 124) {
        double target = bestPos + step;
        if (target > aMax) target = aMax;

        if (aMax - bestPos > 1e-6) {
            double bestTD = aMax;
            int retTD = run_sweep_try_once_then_stop(
                cmd, scriptPath, airfoilCmd,
                Re, Ncrit, iter,
                true,
                aMax, target, -step,
                stall_ms, poll_ms,
                polarOut,
                "topdown",
                target, aMax,
                false,
                bestTD,
                VERBOSE_DIAGNOSTICS
            );
            cout << "Top-down sweep ret=" << retTD << "\n";
            if (retTD == 124) cout << "WARNING: top-down also stalled\n";
        }
    }

    // ---- Negative sweep: 0 -> aMin
    double bestNeg = 0.0;
    int retN = run_sweep_try_once_then_stop(
        cmd, scriptPath, airfoilCmd,
        Re, Ncrit, iter,
        true,
        0.0, aMin, -step,
        stall_ms, poll_ms,
        polarOut,
        "neg",
        aMin, 0.0,
        false,
        bestNeg,
        VERBOSE_DIAGNOSTICS
    );

    cout << "Negative sweep ret=" << retN << " bestNeg=" << bestNeg << "\n";
    if (retN == 124) cout << "WARNING: negative sweep stalled near next alpha ~" << (bestNeg - step) << "\n";

    // ---- If negative stuck, ONE bottom-up attempt
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
        cout << "(XFOIL output redirected to xfoil\\xfoil_console.log)\n";
    }

    // -------------------- cleanup pass --------------------
    // Make a clean version on the base grid (aStep), no duplicates, sorted by alpha.
    // Keep holes for now.
    if (fs::exists(polarOut)) {
        const string rawBackup = "polar_test_raw.dat";
        const string cleanOut  = "polar_test_clean.dat";

        // backup raw
        std::error_code ec;
        fs::copy_file(polarOut, rawBackup, fs::copy_options::overwrite_existing, ec);

        bool ok = cleanup_polar_file_to_grid(polarOut, cleanOut, aStep);
        if (ok) {
            // also overwrite polarOut with the cleaned version for convenience
            fs::copy_file(cleanOut, polarOut, fs::copy_options::overwrite_existing, ec);
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
