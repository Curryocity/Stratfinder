#include <iostream>
#include "ZSolver.hpp"

int main() {

    // We don't have GUI yet
    ZSolver::init();
    ZSolver s;


    std::string content;
    for(double t_mm = 11; t_mm <= 12; t_mm += 1){
        for(double x = 0.0625; x <= 20; x += 0.0625){
            bool hasJump = s.poss(x, t_mm, 116, 0.000001, false, content, ZSolver::normal);
            if(hasJump) std::cout << content;
        }
    }

    
    return 0;
}
