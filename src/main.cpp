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
    int max_t = 116;
    double threshold = 1e-10;
    std::string content;
    
    if(true){
        s.toggleLog(false);
        for(int speed = 0; speed <= 255; speed ++){
            for(int slow = 0; slow <= 6; slow ++){
                s.setEffect(speed, slow);
                for(double t_mm = 2; t_mm <= 12; t_mm += 1){
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
        player p(28,5);
        p.setF(48.465f);
        p.s(1,0,1);
        p.setF(45.01f);
        p.s(1,1,4);
        for(int i = 0; i < 8; i++){
            p.setF(0);
            p.sj(1, 0,1);
            p.setF(45.01f);
            p.sa(1,1,11);
        }
        p.s(1,1,1);
        std::cout << util::df( 39.875 - p.Z()) << "\n";

        p.setZ(zSolver::normal - 0.001);

        p.setF(0);
        p.sj(1, 0,1);
        p.setF(45.01f);
        p.sa(1,1,104);

        std::cout << util::df( p.Z() - 32.875);
    }

    if(false){
        s.setEffect(0, 0);
        s.poss(3, 12, 25, 1e-2, false, content, zSolver::normal);
        s.printLog();
        std::cout << content;
    }

    return 0;
}

