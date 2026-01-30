#include <cmath>
#include <iostream>
#include <optional>
#include <string>
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
    int max_t = 116;
    double threshold = 1e-10;
    std::string content;
    
    if(false){
        s.toggleLog(false);
        for(int speed = 0; speed <= 255; speed ++){
            for(int slow = 0; slow <= 6; slow ++){
                s.setEffect(speed, slow);
                for(double t_mm = 2; t_mm <= 12; t_mm += 1){
                    zSolver::fullStrat maxi = s.maxMMSolver(t_mm);
                    for(double x = 0.125; x <= 50000; x += 0.0625){
                        zSolver::fullStrat strat = s.optimalSolver(x, t_mm);
                        bool hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::normal, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::ladder, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::blockage, strat);
                        if(hasJump) std::cout << content;
                        hasJump = s.poss(x, t_mm, max_t, threshold, backwallq, content, zSolver::normal -0.001, strat);
                        if(hasJump) std::cout << content;

                        if(maxi.delaySpeed - strat.delaySpeed < 1e-3){
                            if(s.equalJumpListCheck(t_mm, max_t, strat, maxi, {zSolver::normal, zSolver::ladder,zSolver::blockage, zSolver::normal -0.001})) break;
                        }
                    }
                    if(maxi.delayTick == -1) break;
                }
            }
        }
        s.toggleLog(true);
    }

    if(false){
        s.setEffect(34, 6);
        s.poss( 5.6875, 11, 50, 1e-6, false, content, zSolver::ladder);
        s.printLog();
        std::cout << content;
    }

    if(false){
        zEngine e(0, 4);

        auto sampler1 = [](zEngine& e){
            e.chained_sj45(11, 2);
            return e.Z();
        };

        auto sampler2 = [](zEngine& e){
            e.chained_sj45(11, 2);
            e.setZ(0);
            e.sj45(21);
            return e.Z();
        };

        double minVz = s.inv(e, 4 + zSolver::normal, sampler1, false);
        double maxVz = s.inv(e, 7 - zSolver::normal, sampler2, false);

        std::cout << "minVz = " << util::df(minVz) << ", maxVz = " << util::df(maxVz) << "\n";
    }

    if(true){
        // extremely optimal fwmm for f32.965 1bm bwmm
        inputFinder f;
        f.setRotation(32.965);
        f.changeSettings(4, 40, 1.6);
        f.printSettings();
        f.setEffect(0, 0);
        inputFinder::zCond cond = inputFinder::genZCondLBUB(0.0787500268143987 - 1e-8, 0.0787500268143987 , 1, true);
        double targetVz = cond.targetVz;
        double error = cond.error;
        double mm = cond.mm;
        double airtime = 12;
        bool hasStrafe = cond.allowStrafe;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", hasStrafe: " << hasStrafe << "\n";

        f.matchZSpeed(cond, airtime);
    }

    if(false){
        // Finding input for slowness I 1.5bm 6-1 to ladder (perfect double 45.01)
        inputFinder f;
        f.changeSettings(4, 40);
        f.printSettings();
        f.setEffect(0, 1);
        inputFinder::zCond cond = inputFinder::genZCondLBUB(-0.1276844242999637, -0.1276846279184921, -1.5, false);
        double targetVz = cond.targetVz;
        double error = cond.error;
        double mm = cond.mm;
        double airtime = 12;
        bool hasStrafe = cond.allowStrafe;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", hasStrafe: " << hasStrafe << "\n";

        f.matchZSpeed(cond, airtime);
    }

    if(false){
        // Example of fw airSpeed(boomerang), note not all inputs works since the extra landing condition of boomerang
        // True triple 45.01: Speed 34, slowness 6 3bcmm 5.6875bm 12.5b tier -23 to ladder
        inputFinder f;
        f.changeSettings(4, 40);
        f.setSpeedType(true);
        f.printSettings();
        f.setEffect(34, 6);
        inputFinder::zCond cond = inputFinder::genZCondLBUB(0.0820559651430301, 0.0820548593210486, -5.6875, false);
        double targetVz = cond.targetVz;
        double error = cond.error;
        double mm = cond.mm;
        double airtime = 11;
        bool hasStrafe = cond.allowStrafe;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", hasStrafe: " << hasStrafe << "\n";

        f.matchZSpeed(cond, airtime);
    }

    return 0;
}

