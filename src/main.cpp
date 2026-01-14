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

    std::string content;
    for(double t_mm = 2; t_mm <= 14; t_mm += 1){
        for(double x = 0.0625; x <= 40; x += 0.0625){
            bool hasJump = s.poss(x, t_mm, 116, 0.0000001, true, content, ZSolver::blockage);
            if(hasJump) std::cout << content;
        }
    }

    
    return 0;
}
