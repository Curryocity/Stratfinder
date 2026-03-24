#include <iostream>
#include "util.hpp"
#include "inputFinder.hpp"
#include "zSolver.hpp"

namespace {

void init() {
    util::init();
}

}

int main() {
    init();

    if(true){
        // Finding jumps for slowness 1.5bm less than 50t airtime, 0.01 offset
        zSolver s;
        s.setEffect(0, 1);
        std::string out;
        const double mm  = 1.5;
        const double mmAirTime = 12;
        const int searchAirTime = 50;
        const double threshold = 0.01;
        const bool backwalled = false;
        s.poss(mm, mmAirTime, searchAirTime, threshold, backwalled, out);

        std::cout << out << "\n";

        std::cout << "StratFinder log: \n";
        s.printLog();
        std::cout << "--------------- \n\n";
    }

    if(true){
        // Finding input for slowness I 1.5bm 6-1 to ladder (perfect double 45.01)
        inputFinder f;
        f.changeSettings(4, 40);
        f.riskyPrune(true);
        f.setEffect(0, 1);
        f.logSettings();
        inputFinder::condition cond;
        cond.endAirborne = false;
        cond.x.enabled = false;
        cond.z.enabled = true;
        cond.z.mm = -1.5;
        cond.allowStrafe = false;
        cond.sideDev = -1;
        f.setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);
        double targetVz = cond.z.vel;
        double error = cond.z.tolerance;
        double mm = cond.z.mm;
        double airtime = 12;
        bool hasStrafe = cond.allowStrafe;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", allowStrafe: " << hasStrafe << "\n";

        f.matchSpeed(cond, airtime);
        f.printLog();
    }

    return 0;
}
