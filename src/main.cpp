#include <iostream>
#include "ZSolver.hpp"

int main() {

    // We don't have GUI yet
    ZSolver::init();
    ZSolver s;


    // Need some e-4 double 45s? (idc some were triple 45s)
    /*
    std::string content;
    for(double t_mm = 11; t_mm <= 12; t_mm += 1){
        for(double x = 0.875; x <= 3; x += 0.0625){
            bool hasJump = s.poss(x, t_mm, 25, 0.001, false, content);
            if(hasJump) std::cout << content;
        }
    }
    */

    double mm = 0.375;
    int airtime = 12;

    std::string content;
    bool hasJump = s.poss(mm, airtime, 25, 0.01, true, content);
    s.printLog();
    std::cout << content;

    
    return 0;
}
