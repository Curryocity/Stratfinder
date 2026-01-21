#include <iostream>
#include "ZPlayer.hpp"
#include "ZSolver.hpp"
#include "util.hpp"

void init(){
    util::init();
    ZPlayer::init();
}

int main() {

    // We don't have GUI yet ;(

    init();
    ZSolver s;

    /*
    bool backwallq = false;
    std::string content;
    for(int speed = 0; speed <= 6; speed ++){
        for(int slow = 0; slow <= 6; slow ++){
            s.setEffect(speed, slow);
            for(double t_mm = 11; t_mm <= 12; t_mm += 1){
                for(double x = 0.0625; x <= 2; x += 0.0625){
                    ZSolver::fullStrat strat = s.optimalSolver(x, t_mm);
                    bool hasJump = s.poss(x, t_mm, 25, 0.00001, backwallq, content, ZSolver::normal, strat);
                    if(hasJump) std::cout << content;
                }
            }
        }
    }
    */


    std::string ctx;
    s.setEffect(3, 4);
    s.poss(0.625, 11, 25, 0.00001, false, ctx, ZSolver::normal);
    s.printLog();
    std::cout << ctx << std::endl;

    
    return 0;
}

