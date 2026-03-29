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

    if(false){
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
        f.setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);
        double vzLB = cond.z.lb;
        double vzUB = cond.z.ub;
        double inervalWidth = cond.z.ub - cond.z.lb;
        double mm = cond.z.mm;
        double airtime = 12;
        bool hasStrafe = cond.allowStrafe;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "TargetVz: (" << util::df(vzLB) << ", " << util::df(vzUB) << "), Interval Width: " << inervalWidth << ", MM: " << util::fmt(mm) << ", Airtime: " << airtime << ", AllowStrafe: " << hasStrafe << "\n";

        f.matchSpeed(cond, airtime);
        f.printLog();
    }

    if(true){
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
        std::cout << "Polar Input Finder:\n";
        std::cout << "Norm: (" << util::df(cond.normLb) << ", " << util::df(cond.normUb)
                  << "), Angle: (" << util::df(cond.angle1) << ", " << util::df(cond.angle2)
                  << "), Airtime: " << airtime << "\n";

        std::vector<inputFinder::sequence> solutions = f.matchSpeed(cond, airtime);
        std::cout << f.showSolutions(solutions, inputFinder::Polar);
    }

    return 0;
}
