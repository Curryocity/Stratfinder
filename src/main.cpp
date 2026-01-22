#include <iostream>
#include "zEngine.hpp"
#include "zSolver.hpp"
#include "util.hpp"
#include "player.hpp"

void init(){
    util::init();
    zEngine::init();
}

int main() {

    // We don't have GUI yet ;(

    init();
    zSolver s;

    /*
    std::string ctx;
    s.setEffect(4, 0);
    s.poss(2, 11, 25, 0.001, false, ctx, ZSolver::normal);
    s.printLog();
    std::cout << ctx << std::endl;
    */

    player p;
    p.setF(22.222);
    p.move(1, 1, player::AIR, player::SPRINT, 5);
    p.move(1, -1, player::GROUND, player::SPRINTJUMP, 1);
    std::cout << "X: " << util::df(p.X()) << ", Z: " << util::df(p.Z()) << "\n";
    std::cout << "Vx: " << util::df(p.Vx()) << ", Vz: " << util::df(p.Vz());
    return 0;
}

