#include <iostream>
#include <optional>
#include <string>
#include <vector>
#include "inputFinder.hpp"
#include "zEngine.hpp"
#include "zSolver.hpp"
#include "util.hpp"
#include "player.hpp"
#include "zInputFinder.hpp"

void init(){
    util::init();
}

int main() {

    // We don't have GUI yet ;(

    init();
    zSolver s;

    const bool backwallq = true;
    const int max_t = 116;
    const double threshold = 1e-9;
    std::string content;
    
    if(false){
        zEngine::set45Type(zEngine::F4501);
        s.toggleLog(false);
        for(int speed = 0; speed <= 255; speed ++){
            for(int slow = 0; slow <= 6; slow ++){
                s.setEffect(speed, slow);
                for(double t_mm = 2; t_mm <= 14; t_mm += 1){
                    zSolver::fullStrat maxi = s.maxMMSolver(t_mm);
                    for(double x = 0.125; x <= 50000; x += 0.0625){
                        zSolver::fullStrat strat = backwallq? s.backwallSolver(x, t_mm) : s.optimalSolver(x, t_mm);
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
        zEngine::set45Type(zEngine::LARGE_HA);
        s.setEffect(32, 6);
        s.poss( 4.75 - zSolver::normal, 4, 116, 1e-6, false, content, -zSolver::normal);
        s.printLog();
        std::cout << content;
    }

    if(false){
        zEngine e(32, 6);
        zEngine::set45Type(zEngine::LARGE_HA);

        auto sampler1 = [](zEngine& e){
            e.chained_sj45(4, 4);
            return e.Z();
        };

        auto sampler2 = [](zEngine& e){
            e.chained_sj45(4, 4);
            e.setZ(-zSolver::normal);
            e.sj45(103);
            return e.Z();
        };

        double minVz = s.inv(e, 4.75, sampler1, false);
        double maxVz = s.inv(e, 30.3125, sampler2, false);

        std::cout << "minVz = " << util::df(minVz) << ", maxVz = " << util::df(maxVz) << "\n";
    }

    if(false){
        player p(32, 6);
        // s.w(1) w.s(3) st(1) stj(4) stj(2) wa.s(2) w.s(1) wj.s(2) sta(2) st(2) w.s(2) wj.s(4) w.s(3) st(1)
        p.s(1, 0, 1);
        p.w(-1, 0, 3);
        p.w(0,0,1);
        p.wj(0, 0, 4);
        p.wj(0, 0, 2);
        p.wa(-1,0,2);
        p.w(-1,0,1);
        p.wj(-1, 0, 2);
        p.wa(0,0,2);
        p.w(0,0,2);
        p.w(-1,0,2);
        p.wj(-1, 0, 4);
        p.w(-1,0,3);
        p.w(0,0,1);
        std::cout << util::df(p.Vz()) << "\n";
        for(int i = 0; i < 4; i++){
            p.setF(0.0f);
            p.sj(1, 0, 1);
            p.setF(5898195.0f);
            p.sa(1, -1, 3);
        }
        std::cout << util::df(p.Vz()) << "\n";
        std::cout << util::df(p.Z()) << "\n";
        std::cout << util::df(p.X()) << "\n";
    }

    // set dontCareInertia to true makes it SUPER fast
    // but inputs that requires inertia is likely missed cuz it is harder to predict 

    if(false){
        zInputFinder f;
        f.setRotation(0);
        f.dontCareInertia(true);
        f.changeSettings(5, 40, 1.6);
        f.printSettings();
        f.setEffect(32, 6);
        zInputFinder::zCond cond = zInputFinder::genZCondLBUB( -0.0903412476753692, -0.0903412476512782 , -4.15, true);
        double targetVz = cond.targetVz;
        double error = cond.error;
        double mm = cond.mm;
        double airtime = 4;
        bool hasStrafe = cond.allowStrafe;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", hasStrafe: " << hasStrafe << "\n";

        f.matchZSpeed(cond, airtime);
    }



    if(false){
        // Example of fw airSpeed(boomerang), note not all inputs works since the extra landing condition of boomerang
        // True triple 45.01: Speed 34, slowness 6 3bcmm 5.6875bm 12.5b tier -23 to ladder
        zInputFinder f;
        f.changeSettings(4, 40);
        f.dontCareInertia(true);
        f.setSpeedType(true);
        f.printSettings();
        f.setEffect(34, 6);
        zInputFinder::zCond cond = zInputFinder::genZCondLBUB(0.0820559651430301, 0.0820548593210486, -5.6875, false);
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

    if(false){
        // Finding input for slowness I 1.5bm 6-1 to ladder (perfect double 45.01)
        zInputFinder f;
        f.changeSettings(4, 40);
        f.dontCareInertia(true);
        f.printSettings();
        f.setEffect(0, 1);
        zInputFinder::zCond cond = zInputFinder::genZCondLBUB(-0.1276844242999637, -0.1276846279184921, -1.5, false);
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

    // use new inputFinder
    if(true){
        // Finding input for slowness I 1.5bm 6-1 to ladder (perfect double 45.01)
        inputFinder f;
        f.changeSettings(4, 40);
        f.riskyPrune(true);
        f.printSettings();
        f.setEffect(0, 1);
        inputFinder::condition cond;
        cond.z.enabled = true;
        cond.z.mm = -1.5;
        cond.allowStrafe = false;
        cond.sideDev = -1;
        f.setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);
        double targetVz = cond.z.vel;
        double error = cond.z.tolerance;
        double mm = cond.z.mm;
        double airtime = 12;
        bool hasStrafe = cond.allowStrafe;
        std::cout << "------------------------------\n";
        std::cout << "Input Finder: \n";
        std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", allowStrafe: " << hasStrafe << "\n";

        f.matchSpeed(cond, airtime);
    }

    return 0;
}

