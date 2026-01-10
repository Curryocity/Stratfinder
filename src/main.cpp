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


    // Need some e-4 double 45s? (some were triple 45s but who cares)
    /*
    std::string content;
    for(double t_mm = 11; t_mm <= 12; t_mm += 1){
        for(double x = 0.875; x <= 3; x += 0.0625){
            bool hasJump = s.poss(x, t_mm, 25, 0.001, content);
            if(hasJump) std::cout << content;
        }
    }
    */

    double mm = 1.3125;
    int airtime = 12;

    ZSolver::fullStrat backwallStrats = s.backwallSolve(mm, airtime);

    s.printLog();

    std::cout << "\n-------------------------------------------\n";
    std::cout << "For Backwalled mm = " << mm << ", airtime = " << airtime << "\n";
    std::cout << "Optimal nonDelayedSpeed: " << backwallStrats.nondelaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::strat2string(backwallStrats.nondelayStrat) << "\n";
    std::cout << "Optimal delayedSpeed: " << backwallStrats.delaySpeed << "\n";
    std::cout << "Strat Type: " << ZSolver::strat2string(backwallStrats.delayStrat) << "\n";

    
    return 0;
}
