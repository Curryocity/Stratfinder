#include <iostream>
#include "ZSolver.hpp"

int main() {

    // We don't have GUI yet
    ZSolver::init();
    ZSolver s;

    /*
    double mm = 14.1875;
    int airtime = 9;
    ZSolver::fullStrat bs = s.optimalSolve(mm, airtime);

    s.printLog();

    std::cout << "\n-------------------------------------------\n";
    std::cout << "For mm = " << mm << ", mm airtime = " << airtime << "\n";
    std::cout << "Optimal nonDelayedSpeed: " << bs.nondelaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::strat2string(bs.nondelayStrat) << "\n";
    std::cout << "Optimal delayedSpeed: " << bs.delaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::strat2string(bs.delayStrat) << "\n";
    */


    std::string content;
    for(double t = 10; t <= 12; t += 1){
        for(double x = 0.875; x <= 3; x += 0.0625){
            bool hasJump = s.poss(x, t, 25, 0.0009, content);
            if(hasJump) std::cout << content;
        }
    }
    
    return 0;
}
