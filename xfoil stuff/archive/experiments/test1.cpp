#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <iomanip>

using namespace std;
namespace fs = std::filesystem;

int main() {
    // ===== CONFIGURATION =====
    int Re = 160000;
    string airfoilCmd = "NACA 0001";     
    int Ncrit = 7;
    double xtrTop = 0.10, xtrBot = 0.10;
    int iter = 500;

    double aMin = -10.0, aMax = 30.0, aStep = 0.5;

    string polarOut = "polar_test.dat";
    string scriptName = "test_script.in";
    
    // ===== INPUT VALIDATION =====
    if (Re <= 0) {
        cerr << "ERROR: Reynolds number must be positive\n";
        return 1;
    }
    if (aStep <= 0 || aMin >= aMax) {
        cerr << "ERROR: Invalid alpha range or step\n";
        return 1;
    }

    // ===== DELETE OLD FILES =====
    if (fs::exists(polarOut)) {
        fs::remove(polarOut);
        cout << "Deleted old " << polarOut << "\n";
    }

    // ===== GENERATE XFOIL SCRIPT =====
    ofstream file(scriptName);
    if (!file) {
        cerr << "ERROR: couldn't open " << scriptName << " for writing\n";
        return 1;
    }

    // Disable graphics for stability
    file << "PLOP\n";
    file << "G\n";
    file << "\n";

    // Load airfoil
    file << airfoilCmd << "\n";
    file << "PANE\n";

    // Enter OPER mode
    file << "OPER\n";
    file << "VISC " << Re << "\n";
    
    // Set viscous parameters
    file << "VPAR\n";
    file << "N " << Ncrit << "\n";
    file << "XTR " << fixed << setprecision(2) << xtrTop << " " << xtrBot << "\n";
    file << "\n";  // exit VPAR

    file << "ITER " << iter << "\n";

    // Start polar accumulation
    file << "PACC\n";
    file << "..\\" << polarOut << "\n";
    file << "\n";

    // Alpha sweep
    int numPoints = 0;
    for (double a = aMin; a <= aMax + 1e-9; a += aStep) {
        file << "ALFA " << fixed << setprecision(4) << a << "\n";
        numPoints++;
    }

    // Close polar accumulation
    file << "PACC\n";
    file << "\n";
    file << "QUIT\n";
    file.close();

    cout << "Generated script with " << numPoints << " alpha points\n";

    // ===== RUN XFOIL =====
    cout << "Running XFOIL...\n";
    int ret = system("cmd /C \"cd /d xfoil && xfoil.exe < ..\\test_script.in > ..\\xfoil_output.log 2>&1\"");
    cout << "XFOIL finished. return code = " << ret << "\n";

    // ===== VERIFY OUTPUT =====
    if (fs::exists(polarOut)) {
        // Check file size
        ifstream check(polarOut);
        check.seekg(0, ios::end);
        size_t fileSize = check.tellg();
        check.close();

        if (fileSize < 50) {
            cout << "WARNING: " << polarOut << " exists but is suspiciously small (" 
                 << fileSize << " bytes)\n";
            cout << "Check xfoil_output.log for errors\n";
        } else {
            cout << "SUCCESS: wrote " << polarOut << " (" << fileSize << " bytes)\n";
            
            // Count data lines (optional)
            ifstream polar(polarOut);
            string line;
            int dataLines = 0;
            bool inData = false;
            while (getline(polar, line)) {
                if (line.find("---") != string::npos) {
                    inData = true;
                    continue;
                }
                if (inData && !line.empty() && line[0] != ' ') {
                    dataLines++;
                }
            }
            polar.close();
            
            cout << "Polar contains " << dataLines << " data points (requested " 
                 << numPoints << ")\n";
            
            if (dataLines < numPoints * 0.5) {
                cout << "WARNING: Less than 50% of points converged. Consider:\n";
                cout << "  - Increasing ITER (current: " << iter << ")\n";
                cout << "  - Adjusting Ncrit (current: " << Ncrit << ")\n";
                cout << "  - Reducing alpha range or step size\n";
            }
        }
    } else {
        cout << "ERROR: " << polarOut << " not found\n";
        cout << "Check xfoil_output.log for details\n";
        return 1;
    }

    return 0;
}