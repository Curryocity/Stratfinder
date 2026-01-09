#include <iostream>
#include "ZSolver.hpp"

int main() {

    // We don't have GUI yet
    ZSolver::init();
    ZSolver s;
    double mm = 2.875;
    int airtime = 12;
    ZSolver::fullStrat bs = s.optimalSolve(mm, airtime);

    std::cout << "\n-------------------------------------------\n";
    std::cout << "For mm = " << mm << ", mm airtime = " << airtime << "\n";
    std::cout << "Optimal nonDelayedSpeed: " << bs.nondelaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::strat2string(bs.nondelayStrat) << "\n";
    std::cout << "Optimal delayedSpeed: " << bs.delaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::strat2string(bs.delayStrat) << "\n";

    return 0;
}
