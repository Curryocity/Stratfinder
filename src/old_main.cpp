#include <iostream>
#include <vector>
#include "util.hpp"
#include "inputCracker.hpp"
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
        inputCracker cracker;
        cracker.changeSettings(2, 50, -1, false);
        cracker.riskyPrune(false);
        cracker.setEffect(0, 1);
        cracker.setRotation(0.015);
        inputCracker::condition cond;
        cond.endAirborne = false;
        cond.z.enabled = true;
        cond.z.mm = -1.5;
        cond.allowKeys = {1, 1, 1, 1};
        cracker.setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);
        double airtime = 12;
        std::cout << "------------------------------\n";

        cracker.matchSpeed(cond, airtime);
    }

    if(false){
        inputCracker cracker;
        cracker.changeSettings(4, 30);
        cracker.riskyPrune(false);
        cracker.setRotation(103.13);

        inputCracker::polorCond cond;
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

        std::vector<inputCracker::Solution> solutions = cracker.matchSpeed(cond, airtime);
        std::cout << cracker.showSolutions(solutions, inputCracker::Polar);
    }

    return 0;
}
