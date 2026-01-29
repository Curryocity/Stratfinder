#include <cmath>
#include <iostream>
#include <optional>
#include <vector>
#include "zEngine.hpp"
#include "zSolver.hpp"
#include "util.hpp"
#include "player.hpp"
#include "inputFinder.hpp"

void init(){
    util::init();
    zEngine::init();
}

int main() {

    // We don't have GUI yet ;(

    init();
    zSolver s;

    bool backwallq = false;
    int max_t = 30;
    double threshold = 1e-5;
    std::string content;
    
    if(false){
        s.toggleLog(false);
        for(int speed = 0; speed <= 60; speed ++){
            for(int slow = 0; slow <= 6; slow ++){
                s.setEffect(speed, slow);
                for(double t_mm = 11; t_mm <= 11; t_mm += 1){
                    zSolver::fullStrat maxi = s.maxMMSolver(t_mm);
                    for(double x = 2.5; x <= 9; x += 0.0625){
                        zSolver::fullStrat strat = s.optimalSolver(x, t_mm);
                        if(strat.delayTick == 1 && (strat.nondelayStrat == zSolver::SLINGSHOT || strat.nondelayStrat == zSolver::BOOMERANG) ) {
                            bool hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::normal, strat);
                            if(hasJump) std::cout << content;
                            hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::ladder, strat);
                            if(hasJump) std::cout << content;

                        }

                        if(maxi.delaySpeed - strat.delaySpeed < 1e-3){
                            if(s.equalJumpListCheck(t_mm, max_t, strat, maxi, {zSolver::normal, zSolver::ladder})) break;
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
        s.setEffect(0, 4);
        s.poss( 4, 11, 50, 1e-2, false, content, zSolver::normal);
        s.printLog();
        std::cout << content;
    }

    if(true){
        // Finding input for slowness I 1.5bm 6-1 to ladder (perfect double 45.01)
        inputFinder f;
        f.changeSettings(4, 40);
        f.printSettings();
        f.setEffect(0, 1);
        double targetVz = -0.127684526;
        double error = 1.02e-07;
        double mm = -1.5;
        double airtime = 12;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "targetVz: " << targetVz << ", error: " << error << ", mm: " << mm << ", airtime: " << airtime << "\n";

        f.matchZSpeed(inputFinder::zCond{targetVz, error, mm}, airtime);
    }

    return 0;
}

