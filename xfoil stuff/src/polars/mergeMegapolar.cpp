// mergeMegapolar.cpp
//
// Hardcoded merge of two megapolar files.
// Just compile and run. No CLI args needed.
//
// g++ mergeMegapolar.cpp -O2 -std=c++17 -o mergeMegapolar.exe
// .\mergeMegapolar.exe

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

// ========================= EDIT THESE =========================
static const string FILE_A = "data\\polars\\megapolar_A.dat";
static const string FILE_B = "data\\polars\\megapolar_B.dat";
static const string FILE_OUT = "data\\polars\\generated\\megapolar_merged.dat";
// =============================================================

struct MegaRow {
    string airfoil;
    int Re = 0, Ncrit = 0, Iter = 0;
    double alpha = 0.0, CL = 0.0, CD = 0.0, CDp = 0.0, CM = 0.0;
    bool hasXtr = false;
    double topXtr = 0.0, botXtr = 0.0;
    string note;
};

static bool parse_line(const string& line, MegaRow& r) {
    size_t i = 0;
    while (i < line.size() && isspace((unsigned char)line[i])) i++;
    if (i >= line.size()) return false;
    if (line.compare(i, 6, "Airfoil") == 0) return false;

    istringstream iss(line);
    if (!(iss >> r.airfoil >> r.Re >> r.Ncrit >> r.Iter
          >> r.alpha >> r.CL >> r.CD >> r.CDp >> r.CM))
        return false;

    string t1, t2;
    if (iss >> t1 >> t2) {
        if (t1 != "NA" && t2 != "NA") {
            r.topXtr = stod(t1);
            r.botXtr = stod(t2);
            r.hasXtr = true;
        }
    }

    iss >> r.note;
    return true;
}

static long long alpha_key(double a) {
    return llround(a * 1000.0);  // matches 3 decimal output
}

struct Key {
    string airfoil;
    int Re, Ncrit, Iter;
    long long akey;

    bool operator==(const Key& o) const {
        return airfoil == o.airfoil &&
               Re == o.Re &&
               Ncrit == o.Ncrit &&
               Iter == o.Iter &&
               akey == o.akey;
    }
};

struct KeyHash {
    size_t operator()(const Key& k) const noexcept {
        size_t h = hash<string>()(k.airfoil);
        h ^= hash<int>()(k.Re) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= hash<int>()(k.Ncrit) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= hash<int>()(k.Iter) + 0x9e3779b9 + (h<<6) + (h>>2);
        h ^= hash<long long>()(k.akey) + 0x9e3779b9 + (h<<6) + (h>>2);
        return h;
    }
};

static const MegaRow& better(const MegaRow& a, const MegaRow& b) {
    if (a.CD < b.CD) return a;
    if (b.CD < a.CD) return b;
    if (fabs(a.CM) < fabs(b.CM)) return a;
    if (fabs(b.CM) < fabs(a.CM)) return b;
    if (a.hasXtr && !b.hasXtr) return a;
    if (b.hasXtr && !a.hasXtr) return b;
    return a;
}

int main() {
    cout << "===== mergeMegapolar =====\n";
    cout << "A: " << FILE_A << "\n";
    cout << "B: " << FILE_B << "\n";
    cout << "OUT: " << FILE_OUT << "\n\n";

    if (!fs::exists(FILE_A) || !fs::exists(FILE_B)) {
        cout << "ERROR: One of the files does not exist.\n";
        return 1;
    }

    unordered_map<Key, MegaRow, KeyHash> mapRows;
    mapRows.reserve(500000);

    auto process_file = [&](const string& path) {
        ifstream in(path);
        string line;
        size_t count = 0;

        cout << "Reading " << path << "...\n";

        while (getline(in, line)) {
            MegaRow r;
            if (!parse_line(line, r)) continue;

            Key k { r.airfoil, r.Re, r.Ncrit, r.Iter, alpha_key(r.alpha) };

            auto it = mapRows.find(k);
            if (it == mapRows.end()) {
                mapRows[k] = r;
            } else {
                it->second = better(it->second, r);
            }

            if (++count % 200000 == 0)
                cout << "  parsed " << count << " rows...\n";
        }

        cout << "Finished " << path << " (" << count << " rows scanned)\n\n";
    };

    process_file(FILE_A);
    process_file(FILE_B);

    cout << "Unique rows after merge: " << mapRows.size() << "\n";
    cout << "Sorting...\n";

    vector<MegaRow> rows;
    rows.reserve(mapRows.size());
    for (auto& kv : mapRows) rows.push_back(kv.second);

    sort(rows.begin(), rows.end(), [](const MegaRow& a, const MegaRow& b) {
        if (a.airfoil != b.airfoil) return a.airfoil < b.airfoil;
        if (a.Re != b.Re) return a.Re < b.Re;
        if (a.Ncrit != b.Ncrit) return a.Ncrit < b.Ncrit;
        if (a.Iter != b.Iter) return a.Iter < b.Iter;
        return alpha_key(a.alpha) < alpha_key(b.alpha);
    });

    ofstream out(FILE_OUT);
    out << "Airfoil  Re  Ncrit  Iter  "
        << "alpha  CL  CD  CDp  CM  Top_Xtr  Bot_Xtr  Note\n";

    for (auto& r : rows) {
        out << r.airfoil << "  " << r.Re << "  " << r.Ncrit << "  " << r.Iter << "  "
            << fixed << setprecision(3) << r.alpha << "  "
            << setprecision(4) << r.CL << "  "
            << setprecision(5) << r.CD << "  "
            << setprecision(5) << r.CDp << "  "
            << setprecision(4) << r.CM << "  ";

        if (r.hasXtr)
            out << setprecision(4) << r.topXtr << "  " << r.botXtr;
        else
            out << "NA  NA";

        out << "  " << (r.note.empty() ? "BASE" : r.note) << "\n";
    }

    cout << "Done.\n";
    cout << "Wrote: " << FILE_OUT << "\n";
    cout << "Rows written: " << rows.size() << "\n";

    return 0;
}
