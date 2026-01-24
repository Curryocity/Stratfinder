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

 
    bool backwallq = false;
    int max_t = 50;
    double threshold = 1e-6;
    std::string content;
    
    /*
    for(int speed = 0; speed <= 255; speed ++){
        for(int slow = 0; slow <= 6; slow ++){
            s.setEffect(speed, slow);
            for(double t_mm = 2; t_mm <= 12; t_mm += 1){
                for(double x = 0.125; x <= 50; x += 0.0625){
                    zSolver::fullStrat strat = s.optimalSolver(x, t_mm);
                    bool hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::normal, strat);
                    if(hasJump) std::cout << content;
                    hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::ladder, strat);
                    if(hasJump) std::cout << content;
                    hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::blockage, strat);
                    if(hasJump) std::cout << content;
                }
            }
        }
    }
    */
   

    
    s.setEffect(8, 6);
    bool hasJump = s.poss(1.375, 11, max_t, threshold, backwallq, content, zSolver::normal);
    s.printLog();
    std::cout << content;
    



    return 0;
}

