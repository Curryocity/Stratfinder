#include <iostream>
#include <cmath>
#include "ZPlayer.hpp"
#include "ZSolver.hpp"

using ZS = ZSolver;

void ZS::init(){
    ZPlayer::init();
}

float F = ZPlayer::FORWARD;

// finds the optimal delayed speed: Given mm(no walls), mm-airtime
ZS::mmStrat ZS::OptimalDelayed(double mm, int t)
{
    ZPlayer p;
    mm += 0.6f;

    ZS::CoreCtx c = runCore(p, mm, t, true, 0);

    ZS::mmStrat out;
    if (earlyPrune(c, out)) return out;

    double maxBwSpeed = delayloopEquilibrium(p, mm, t, c.o1.jumps);

    if(-maxBwSpeed >= -c.o2.reqBwSpeed) 
        return mmStrat{ZS::SLINGSHOT, c.o2.bwmmSpeed};

    if(-maxBwSpeed >= -c.o4.bwSpeedForBwhh) 
        return mmStrat{ZS::BOOMERANG, c.o4.bwhhSpeed};

    return mmStrat{ZS::PENDULUM, -maxBwSpeed};
}

// Runs: heuristics, bwmm, robo, bwhh (no equilibrium / no moveVec fit)
ZS::CoreCtx ZS::runCore(ZPlayer& p, double mm, int t, bool delayQ, double knownBwCap){
    CoreCtx c;
    c.o1 = ZS::mmHeuristics(p, mm, t, delayQ, knownBwCap);
    c.o2 = ZS::simpleBwmm(p, mm, t, delayQ, c.o1);
    c.o3 = ZS::tryRobo(p, mm, t, delayQ, c.o1.jumps);
    c.o4 = ZS::tryBwhh(p, mm, t, delayQ, c.o1);
    return c;
}

// Applies the shared early-return rules (slingshot / true robo / robo vs boomerang).
// Returns true if early prunable, and fills `out`.
bool ZS::earlyPrune(const CoreCtx& c, ZS::mmStrat& out){
    if (c.o2.possBwmm) {
        out = { ZS::SLINGSHOT, c.o2.bwmmSpeed };
        return true;
    }
    if (c.o3.isTrueRobo) {
        out = { ZS::TRUE_ROBO, c.o3.roboSpeed };
        return true;
    }
    if (c.o4.possBwhh) {
        if (c.o3.roboSpeed > c.o4.bwhhSpeed)
            out = { ZS::ROBO, c.o3.roboSpeed };
        else
            out = { ZS::BOOMERANG, c.o4.bwhhSpeed };
        return true;
    }
    return false;
}

double ZS::delayloopEquilibrium(ZPlayer& p, double mm, int t, int jumps){

    auto getSample = [&](double vi, double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(vi);
        p.sj45(m, 1);
        p.sa45(F, t - 1);
        p.chained_sj45(t, jumps);
        p.s45(F,1);
        return falseZtrueVz? p.getVz() : p.getZ();
    };

    // Lerp (0, z00) and (1, z01) to get (m0, mm)
    double z00 = getSample(0, 0, false);
    double z01 = getSample(0, 1, false);
    double m0 = (mm - z00)/(z01 - z00);

    // Lerp (0, z10) and (1, z11) to get (m1, mm)
    double z10 = getSample(1, 0, false);
    double z11 = getSample(1, 1, false);
    double m1 = (mm - z10)/(z11 - z10);

    // Simulate with (vi,m), solve the equation v_end = linear_func(vi) = -vi:
    double v0 = getSample(0, m0, true);
    double v1 = getSample(1, m1, true);

    return -v0/(v1-v0+1);
}


// finds the optimal non-delayed speed: Given mm(no walls), mm-airtime
ZS::mmStrat ZS::OptimalNonDelayed(double mm, int t){

    ZPlayer p;
    mm += 0.6f;

    ZS::CoreCtx c = runCore(p, mm, t, false, 0);

    ZS::mmStrat out;
    if (earlyPrune(c, out)) return out;

    ZS::mmStrat delayedStrat = OptimalDelayed(mm - 0.6f, t);
    double maxBwSpeed = -delayedStrat.optimalSpeed;

    if(-maxBwSpeed >= -c.o2.reqBwSpeed)
        return mmStrat{ZS::SLINGSHOT, c.o2.bwmmSpeed};

    if(-maxBwSpeed >= -c.o4.bwSpeedForBwhh)
        return mmStrat{ZS::BOOMERANG, c.o4.bwhhSpeed};

    // fit the best fw air strat into maxBwSpeed
    auto getSample = [&](double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(maxBwSpeed);
        p.sj45(m, 1);
        p.sa45(F, t - 1);
        p.chained_sj45(t, c.o1.jumps);
        return falseZtrueVz? p.getVz() : p.getZ();
    };

    // Lerp (0, z0), (1, z1) to find (moveVec, mm)
    double z0 = getSample(0, false);
    double z1 = getSample(1, false);
    double moveVec = (mm - z0)/(z1 - z0);

    //simulate it and get final speed
    return mmStrat{ZS::PENDULUM, getSample(moveVec, true)};
}

ZS::mmStratBoth ZSolver::OptimalBoth(double mm, int t)
{

    ZS::mmStrat delayedStrat = OptimalDelayed(mm, t);
    double maxBwSpeed = -delayedStrat.optimalSpeed;
    int dT = delayedStrat.stratType;
    double dS = delayedStrat.optimalSpeed;

    ZPlayer p;
    mm += 0.6f;

    ZS::CoreCtx c = runCore(p, mm, t, false, maxBwSpeed);

    ZS::mmStrat out;
    if (earlyPrune(c, out)) return mmStratBoth{dT, dS, out.stratType, out.optimalSpeed};

    // fit the best fw air strat into maxBwSpeed
    auto getSample = [&](double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(maxBwSpeed);
        p.sj45(m, 1);
        p.sa45(F, t - 1);
        p.chained_sj45(t, c.o1.jumps);
        return falseZtrueVz? p.getVz() : p.getZ();
    };

    // Lerp (0, z0), (1, z1) to find (moveVec, mm)
    double z0 = getSample(0, false);
    double z1 = getSample(1, false);
    double moveVec = (mm - z0)/(z1 - z0);

    //simulate it and get final speed
    return mmStratBoth{dT, dS, ZS::PENDULUM, getSample(moveVec, true)};

}

ZS::Output1 ZS::mmHeuristics(ZPlayer& p, double mm, int t, bool delayQ, double knownBwCap){
    
    // Amount of jumps an mm could fit without bw speed
    int jumps = 0;
    double overJamDis;
    double jamDis;

    ZPlayer::State prevJump;

    while (true){
        prevJump = p.getState();
        if(delayQ && jumps != 0)
            p.loadState(); //undo run 1t
        p.sj45(F, t);
        
        if(delayQ){
            p.saveState();
            p.s45(F,1);
        } 

        if(p.getZ() > mm){
            overJamDis = p.getZ();
            p.loadState(prevJump); // undo a jump
            jamDis = p.getZ();
            break;
        }
        jumps ++;
    }

    double bestBwSpeed;
    if(knownBwCap == 0){
        // [Note] Current state: prevJump
        // A lowerbound on maxBwSpeed(bestBwSpeed) is used for early pruning in stratfind
        // Get bestBwSpeed by doing full jumps on mm (filled mm with sa45 if jump == 0), then run 1t 

        if(jumps == 0){
            int pessiTicks = 0;
            while (p.getZ() < mm){
                pessiTicks ++;
                p.sa45(F, 1);
            }
            p.resetAll();
            p.sa45(F, pessiTicks - 2); // do 2 tick less pessi to fit 1t run
        }
        p.s45(F, 1);
        bestBwSpeed = -p.getVz();
    }else{
        bestBwSpeed = knownBwCap;
    }

    p.resetAll();
    p.setVz(bestBwSpeed);
    p.chained_sj45(t, jumps + 1);
    if(delayQ) p.s45(F,1);
    double bwmmDis = p.getZ();

    p.resetAll();

    return Output1{jumps, overJamDis, jamDis, bestBwSpeed, bwmmDis};
}

ZS::Output2 ZS::simpleBwmm(ZPlayer& p, double mm, int t, bool delayQ, Output1& o1){

    double bwmmSpeed = 0;
    // Lerp (0, overJamDis), (bestBwSpeed, bwmmDis) to find (reqVz, mm)
    double reqBwSpeed = o1.bestBwSpeed * (o1.overJamDis - mm) / (o1.overJamDis - o1.bwmmDis);
    bool possBwmm = false;
    std::cout << "Required BW speed: " << reqBwSpeed << "\n";

    p.resetAll();
    p.setVz(reqBwSpeed);
    p.chained_sj45(t, o1.jumps + 1);
    if(delayQ) p.s45(F,1);
    bwmmSpeed = p.getVz();

    // Early Prune: If possible to bwmm/loop with reqBwSpeed < bestBwSpeed
    if(o1.bwmmDis < mm){
        std::cout << "Trivial slingshot.\n";
        possBwmm = true;
    }

    return Output2{reqBwSpeed, bwmmSpeed, possBwmm};
}


ZS::Output3 ZS::tryRobo(ZPlayer& p, double mm, int t, bool delayQ, int jumps){

    // robo doesn't make sense on jump = 0
    if(jumps == 0) return Output3{false, 0};

    p.resetAll();
    p.s45(F, 1);
    double hhSpeed = p.getVz(); 
    p.chained_sj45(t, jumps);
    if(delayQ) p.s45(F,1);

    double hhDis = p.getZ();

    // robo could beat bwhh only when it is bwmm into hh1t
    if(hhDis <= mm) return Output3{false, 0};

    double roboSpot1; 
    p.resetAll();

    // the border of robo and true robo
    // the formula is v_0 = -0.13*(0.6/slip)^3/(1+0.91*slip), derived from v_0 + v_1 = 0
    double borderSpeed = -0.13/1.546;
    p.setVz(borderSpeed);
    p.s45(F, 1);
    p.chained_sj45(t, jumps);
    if(delayQ) p.s45(F,1);
    double borderDis = p.getZ();

    double roboBwSpeed;

    bool isTrueRobo = (borderDis >= mm);

    hhDis -= hhSpeed * isTrueRobo; // substract hh Speed from true robo (on jump tick, player is at the backedge of the mm)
    roboBwSpeed = borderSpeed * (mm - hhDis) / (borderDis - hhDis);

    p.resetAll();
    p.setVz(roboBwSpeed);

    p.s45(F,1);
    p.sj45(F,1);
    if(isTrueRobo) p.setZ(0);
    p.sa45(F, t - 1);
    p.chained_sj45(t, jumps - 1);
    if(delayQ) p.s45(F,1);

    double roboSpeed = p.getVz();
    p.resetAll();

    return Output3{isTrueRobo, roboSpeed};
}


ZS::Output4 ZS::tryBwhh(ZPlayer& p, double mm, int t, bool delayQ, Output1& o1){

    // robo doesn't make sense on jump = 0
    if(o1.jumps == 0) return Output4{-INFINITY, 0, false};

    bool possBwhh = false;

    // Fitting an initial air speed
    p.resetAll();
    p.setVzAir(1);
    p.chained_sj45(t, o1.jumps);
    if(delayQ) p.s45(F,1);
    double z3 = p.getZ();

    // Lerp (0, jamDis), (z3, 1) to find (reqFwSpeed, mm)
    double reqFwSpeed = (mm - o1.jamDis) / (z3 - o1.jamDis);
    std::cout << "Required boomerang speed: " << reqFwSpeed << "\n";

    // do borderline true bwhh
    auto getSample = [&](double vi, double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(vi);
        p.sj45(m, 1);
        p.sa45(F, t - 1);
        return falseZtrueVz? p.getVz() : p.getZ();
    };

    // Lerp (0, z00), (1, z01) to find (m0, 0)
    double z00 = getSample(0, 0, false);
    double z01 = getSample(0, 1, false);
    double m0 = (-z00)/(z01 - z00);

    // Lerp (0, z10), (1, z11) to find (m1, 0)
    double z10 = getSample(1, 0, false);
    double z11 = getSample(1, 1, false);
    double m1 = (-z10)/(z11 - z10);

    // Lerp (0, v0), (1, v1) to find (bwSpeedForBwhh, reqFwSpeed)
    double v0 = getSample(0, m0, true);
    double v1 = getSample(1, m1, true);
    double bwSpeedForBwhh = (reqFwSpeed - v0)/(v1 - v0);

    std::cout << "Required BW speed for boomerang: " << bwSpeedForBwhh << "\n";

    //desire bwhh speed
    p.resetAll();
    p.setVzAir(reqFwSpeed);
    p.chained_sj45(t, o1.jumps);
    if(delayQ) p.s45(F,1);
    double bwhhSpeed = p.getVz();

    if(-bwSpeedForBwhh < -o1.bestBwSpeed)
        possBwhh = true;

    return Output4{bwSpeedForBwhh, bwhhSpeed, possBwhh};
}

std::string ZS::stratString(int stratType) {
    switch (stratType) {
        case ZS::SLINGSHOT:
            return "Slingshot";
        case ZS::TRUE_ROBO:
            return "True_Robo";
        case ZS::ROBO:
            return "Robo";
        case ZS::BOOMERANG:
            return "Boomerang";
        case ZS::PENDULUM:
            return "Pendulum";
        default:
            return "Unnamed";
    }
}
