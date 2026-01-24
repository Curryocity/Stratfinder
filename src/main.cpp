#include <iostream>
#include <optional>
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
    double threshold = 1e-9;
    std::string content;
    
    if(true){
        s.toggleLog(false);
        for(int speed = 0; speed <= 255; speed ++){
            for(int slow = 0; slow <= 6; slow ++){
                s.setEffect(speed, slow);
                for(double t_mm = 2; t_mm <= 16; t_mm += 1){
                    zSolver::fullStrat maxi = s.maxMMSolver(t_mm);
                    for(double x = 0.125; x <= 5000; x += 0.0625){
                        zSolver::fullStrat strat = s.optimalSolver(x, t_mm);
                        bool hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::normal, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::ladder, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::blockage, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::normal - 0.001, strat);
                        if(hasJump) std::cout << content;

                        if(maxi.delaySpeed - strat.delaySpeed < 1e-3){
                            if(s.equalJumpListCheck(t_mm, max_t, strat, maxi, {zSolver::normal, zSolver::ladder, zSolver::blockage, zSolver::normal - 0.001})) break;
                        }
                    }
                    if(maxi.delayTick == -1) break;
                }
            }
        }
        s.toggleLog(true);
    }
    
    if(false){
        s.clearLog();
        s.setEffect(0, 4);
        s.poss(7, 11, max_t, 1e-3, backwallq, content, zSolver::normal);
        s.printLog();
        std::cout << content;
    }

    return 0;
}

