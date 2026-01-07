#include <iostream>
#include "ZSolver.hpp"

int main() {

    ZSolver::init();
    ZSolver s;
    double mm = 3;
    int airtime = 12;
    ZSolver::mmStratBoth bs = s.OptimalBoth(mm, airtime);

    std::cout << "\n-------------------------------------------\n";
    std::cout << "For mm = " << mm << ", mm airtime = " << airtime << "\n";
    std::cout << "Optimal nonDelayedSpeed: " << bs.nondelaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::stratString(bs.nondelayStrat) << "\n";
    std::cout << "Optimal delayedSpeed: " << bs.delaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::stratString(bs.delayStrat) << "\n";

    return 0;
}
