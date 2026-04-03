#include <iostream>
#include <vector>
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

    if(false){
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
        f.changeSettings(2, 50, -1, false);
        f.riskyPrune(false);
        f.setEffect(0, 1);
        f.setRotation(0.015);
        f.logSettings();
        inputFinder::condition cond;
        cond.endAirborne = false;
        cond.z.enabled = true;
        cond.z.mm = -1.5;
        cond.allowKeys = {1, 1, 1, 1};
        f.setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);
        double airtime = 12;
        std::cout << "------------------------------\n";
        f.logSearch(std::cout, cond, airtime);

        f.matchSpeed(cond, airtime);
        f.printLog();
    }

    if(false){
        inputFinder f;
        f.changeSettings(4, 30);
        f.riskyPrune(false);
        f.setRotation(103.13);
        f.logSettings();

        inputFinder::polorCond cond;
        double targetNorm = 0.236876;
        double normErr = 0.0001;
        double targetAngle = -104.2;
        double angleErr = 0.1;
        cond.normLb = targetNorm - normErr;
        cond.normUb = targetNorm + normErr;
        cond.angle1 = targetAngle - angleErr;
        cond.angle2 = targetAngle + angleErr;

        cond.xmm = 1;
        cond.zmm = -1;
        cond.zWalled = true;

        const int airtime = 12;
        std::cout << "------------------------------\n";
        f.logSearch(std::cout, cond, airtime);

        std::vector<inputFinder::sequence> solutions = f.matchSpeed(cond, airtime);
        std::cout << f.showSolutions(solutions, inputFinder::Polar);
    }

    return 0;
}
