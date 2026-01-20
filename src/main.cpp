#include <iostream>
#include "ZSolver.hpp"

int main() {

    // We don't have GUI yet
    ZSolver::init();
    ZSolver s;

    /*
    std::string content;
    for(int speed = 0; speed <= 6; speed ++){
        for(int slow = 0; slow <= 6; slow ++){
            s.setEffect(speed, slow);
            for(double t_mm = 2; t_mm <= 12; t_mm += 1){
                for(double x = 0.0625; x <= 30; x += 0.0625){
                    bool hasJump = s.poss(x, t_mm, 40, 0.0000001, false, content, ZSolver::normal);
                    if(hasJump) std::cout << content;
                }
            }
        }
    }
    */

    std::string ctx;
    s.setEffect(4, 6);
    s.poss(4.6875, 8, 50, 0.001, false, ctx, ZSolver::normal);
    s.printLog();
    std::cout << ctx << std::endl;

    
    return 0;
}