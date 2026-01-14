#include <iostream>
#include <cmath>
#include <ostream>
#include <iomanip>
#include <sstream>
#include "ZPlayer.hpp"
#include "ZSolver.hpp"

using ZS = ZSolver;

void ZS::init(){
    ZPlayer::init();
}

// Finds the optimal speed for delayed and nondelayed strat: Given mm, mm-airtime
ZS::fullStrat ZSolver::optimalSolver(double mm, int t)
{
    log += "\nOptimal Solver ----------------------- \n";
    log += "Target mm: " + std::to_string(mm) + ", airtime: " + std::to_string(t) + "\n";

    log += "\n- Delayed section: \n";

    // Solve delayed version first to get maxBwSpeed
    ZS::halfStrat delayedStrat = optimalDelayed(mm, t);
    int dT = delayedStrat.stratType;
    double dS = delayedStrat.optimalSpeed;
    double maxBwSpeed = -dS;

    ZPlayer p;
    mm += 0.6f;

    log += "\n- Nondelayed section: \n";
    log += "Max BW speed: " + std::to_string(maxBwSpeed) + "\n";
    // Then solve nondelayed, given the knowledge of maxBwSpeed
    ZS::CoreCtx c = solverCore(p, mm, t, false, maxBwSpeed);

    ZS::halfStrat out;
    if (earlyPrune(c, out)) return fullStrat{dT, dS, out.stratType, out.optimalSpeed};

    double pendulumSpeed = nondelayedPendulum(p, mm, t, c.o1.jumps, maxBwSpeed);
    return fullStrat{dT, dS, ZS::PENDULUM, pendulumSpeed};

}

// Finds the optimal delayed speed: Given mm, mm-airtime
ZS::halfStrat ZS::optimalDelayed(double mm, int t)
{
    ZPlayer p;
    mm += 0.6f;

    ZS::CoreCtx c = solverCore(p, mm, t, true, 0);

    ZS::halfStrat out;
    if (earlyPrune(c, out)) return out;

    double pendulumSpeed = delayedPendulum(p, mm, t, c.o1.jumps);
    if(pendulumSpeed >= -c.o2.reqBwSpeed) return halfStrat{ZS::SLINGSHOT, c.o2.slingSpeed};
    if(pendulumSpeed >= -c.o4.bwSpeedBoom) return halfStrat{ZS::BOOMERANG, c.o4.boomSpeed};

    return halfStrat{ZS::PENDULUM, pendulumSpeed};
}

// Runs: heuristics, slingShot, robo, boomerang (no equilibrium / no moveVec fit)
ZS::CoreCtx ZS::solverCore(ZPlayer& p, double mm, int t, bool delayQ, double knownBwCap){
    CoreCtx c;
    c.o1 = ZS::mmHeuristics(p, mm, t, delayQ, knownBwCap);
    c.o2 = ZS::slingShot(p, mm, t, delayQ, c.o1);
    c.o3 = ZS::robo(p, mm, t, delayQ, c.o1.jumps);
    c.o4 = ZS::boomerang(p, mm, t, delayQ, c.o1);
    return c;
}

// Applies the shared early-return rules (slingshot / true robo / robo vs boomerang).
// Returns true if early prunable, and fills `out`.
bool ZS::earlyPrune(const CoreCtx& c, ZS::halfStrat& out){
    if (c.o2.possSling) {
        out = { ZS::SLINGSHOT, c.o2.slingSpeed };
        return true;
    }
    if (c.o3.trueRoboQ) {
        out = { ZS::TRUE_ROBO, c.o3.roboSpeed };
        return true;
    }
    if (c.o4.possBoom) {
        if (c.o3.roboSpeed > c.o4.boomSpeed)
            out = { ZS::ROBO, c.o3.roboSpeed };
        else
            out = { ZS::BOOMERANG, c.o4.boomSpeed };
        return true;
    }
    return false;
}

// Gather samples, preReq knowledges for later calculation. If there is no knownBwCap, it yolos a reasonable lowerbound.
ZS::Output1 ZS::mmHeuristics(ZPlayer& p, double mm, int t, bool delayQ, double knownBwCap){
    
    // Amount of sj45(t)'s an mm could fit, without bwSpeed
    int jumps = 0;
    double overJamDis;
    double jamDis;

    ZPlayer::State prevJump;

    while (true){
        prevJump = p.getState();
        if(delayQ && jumps != 0)
            p.loadState(); // Undo run 1t
        p.sj45(t);
        
        if(delayQ){
            p.saveState();
            p.s45(1);
        } 

        if(p.Z() > mm){
            overJamDis = p.Z();
            p.loadState(prevJump); // Undo a jump
            jamDis = p.Z();
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
            while (p.Z() < mm){
                pessiTicks ++;
                p.sa45(1);
            }
            p.resetAll();
            p.sa45(pessiTicks - 2); // Do 2 tick less pessi to fit 1t run
        }
        p.s45(1);
        bestBwSpeed = -p.Vz();
        log += "Estimates BW speed lowerBound: " + std::to_string(bestBwSpeed) + "\n";
    }else{
        bestBwSpeed = knownBwCap;
    }

    p.resetAll();
    p.setVz(bestBwSpeed);
    p.chained_sj45(t, jumps + 1);
    if(delayQ) p.s45(1);
    double bwmmDis = p.Z();

    p.resetAll();

    return Output1{jumps, overJamDis, jamDis, bestBwSpeed, bwmmDis};
}

ZS::Output2 ZS::slingShot(ZPlayer& p, double mm, int t, bool delayQ, Output1& o1){

    double slingSpeed = 0;
    // Lerp (0, overJamDis), (bestBwSpeed, bwmmDis) to find (reqVz, mm)
    double reqBwSpeed = o1.bestBwSpeed * (o1.overJamDis - mm) / (o1.overJamDis - o1.bwmmDis);
    bool poss = false;
    log += "Required BW speed: " + std::to_string(reqBwSpeed) + "\n";

    if(-reqBwSpeed >= groundInertia){
        p.resetAll();
        p.setVz(reqBwSpeed);
        p.chained_sj45(t, o1.jumps + 1);
        if(delayQ) p.s45(1);
        slingSpeed = p.Vz();
    }else{
        // If reqBwSpeed hits inertia, set bwSpeed to just hit inertia barely(has the effect to extend the mm by ~0.0091575), angled the first jump tick.
        log += "This backward speed hits inertia! Fixing... \n";
        double extendedmm = mm + groundInertia;

        auto getSample = [&](double m, bool falseZtrueVz){
            p.resetAll();
            p.sj45(m, 1);
            p.sa45(t - 1);
            p.chained_sj45(t, o1.jumps);
            if(delayQ) p.s45(1);
            return falseZtrueVz? p.Vz() : p.Z();
        };

        // Lerp (0, z0), (1, z1) to find (moveVec, mm)
        double z0 = getSample(0, false);
        double z1 = getSample(1, false);
        double moveVec = (extendedmm - z0)/(z1 - z0);

        // Extending mm by ~0.0091575 is sometimes more than enough.
        if(moveVec > 1) moveVec = 1;

        slingSpeed = getSample(moveVec, true);
    }

    // ReqBwSpeed could be reached
    if(o1.bwmmDis < mm) poss = true;

    return Output2{reqBwSpeed, slingSpeed, poss};
}


ZS::Output3 ZS::robo(ZPlayer& p, double mm, int t, bool delayQ, int jumps){

    // Robo doesn't make sense when jumps == 0
    if(jumps == 0) return Output3{false, 0};

    p.resetAll();
    p.s45(1);
    double hhSpeed = p.Vz(); 
    p.chained_sj45(t, jumps);
    if(delayQ) p.s45(1);

    double hhDis = p.Z();

    // Robo could beat boomerang only when it is bwmm into hh1t
    if(hhDis <= mm) return Output3{false, 0};

    double roboSpot1; 
    p.resetAll();

    // The border of robo and true robo
    // The formula is v_0 = -0.13*(0.6/slip)^3/(1+0.91*slip), derived from v_0 + v_1 = 0
    double borderSpeed = -0.13/1.546;
    p.setVz(borderSpeed);
    p.s45(1);
    p.chained_sj45(t, jumps);
    if(delayQ) p.s45(1);
    double borderDis = p.Z();

    double roboBwSpeed;

    bool trueRoboQ = (borderDis >= mm);

    hhDis -= hhSpeed * trueRoboQ; // Substract hh Speed from distance if true robo ( Mothball: vz(bwSpeed) s45 sj | sa45(t-1) )
    roboBwSpeed = borderSpeed * (mm - hhDis) / (borderDis - hhDis);

    p.resetAll();
    p.setVz(roboBwSpeed);

    p.s45(1);
    p.sj45(1);
    if(trueRoboQ) p.setZ(0);
    p.sa45(t - 1);
    p.chained_sj45(t, jumps - 1);
    if(delayQ) p.s45(1);

    double roboSpeed = p.Vz();
    p.resetAll();

    return Output3{trueRoboQ, roboSpeed};
}


ZS::Output4 ZS::boomerang(ZPlayer& p, double mm, int t, bool delayQ, Output1& o1){

    // Bommerang doesn't make sense when jumps == 0
    if(o1.jumps == 0) return Output4{-INFINITY, 0, false};

    bool poss = false;

    // Fitting an initial air speed
    p.resetAll();
    p.setVzAir(1);
    p.chained_sj45(t, o1.jumps);
    if(delayQ) p.s45(1);
    double z3 = p.Z();

    // Lerp (0, jamDis), (z3, 1) to find (reqFwSpeed, mm)
    double reqFwSpeed = (mm - o1.jamDis) / (z3 - o1.jamDis);
    log += "Required FW airspeed: " + std::to_string(reqFwSpeed) + "\n";

    // do borderline boomerang
    auto samp = [&](double vi, double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(vi);
        p.sj45(m, 1);
        p.sa45(t - 1);
        return falseZtrueVz? p.Vz() : p.Z();
    };

    // Lerp (0, z00), (1, z01) to find (m0, 0)
    double z00 = samp(0, 0, false);
    double z01 = samp(0, 1, false);
    double m0 = (-z00)/(z01 - z00);

    // Lerp (0, z10), (1, z11) to find (m1, 0)
    double z10 = samp(1, 0, false);
    double z11 = samp(1, 1, false);
    double m1 = (-z10)/(z11 - z10);

    // Lerp (0, v0), (1, v1) to find (reqBwSpeed, reqFwSpeed)
    double v0 = samp(0, m0, true);
    double v1 = samp(1, m1, true);
    double reqBwSpeed = (reqFwSpeed - v0)/(v1 - v0);

    log += "Required BW speed for boomerang: " + std::to_string(reqBwSpeed) + "\n";

    // Simulate boomerang speed assuming it is possible
    p.resetAll();
    p.setVzAir(reqFwSpeed);
    p.chained_sj45(t, o1.jumps);
    if(delayQ) p.s45(1);
    double boomSpeed = p.Vz();

    if(-reqBwSpeed < -o1.bestBwSpeed)
        poss = true;

    return Output4{reqBwSpeed, boomSpeed, poss};
}


// Finding the velocity convergence of chained loop by solving the equation of reqBwSpeed = finalSpeed
double ZS::delayedPendulum(ZPlayer& p, double mm, int t, int jumps){

    auto samp = [&](double vi, double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(vi);
        p.sj45(m, 1);
        p.sa45(t - 1);
        p.chained_sj45(t, jumps);
        p.s45(1);
        return falseZtrueVz? p.Vz() : p.Z();
    };

    double pendulumSpeed, moveVec;
    {
        // Lerp (0, z00) and (1, z01) to get (m0, mm)
        double z00 = samp(0, 0, false);
        double z01 = samp(0, 1, false);
        double m0 = (mm - z00)/(z01 - z00);

        // Lerp (0, z10) and (1, z11) to get (m1, mm)
        double z10 = samp(1, 0, false);
        double z11 = samp(1, 1, false);
        double m1 = (mm - z10)/(z11 - z10);

        // Simulate with (vi,m), solve the equation v_end = linear_func(vi) = -vi:
        double v0 = samp(0, m0, true);
        double v1 = samp(1, m1, true);

        pendulumSpeed = v0/(v1-v0+1);
        // Lerp (0, z0) and (1, z1) to get (moveVec, mm)
        double z0 = samp(-pendulumSpeed, 0, false);
        double z1 = samp(-pendulumSpeed, 1, false);
        moveVec = (mm - z0)/(z1 - z0);
    }

    // Simulate it to check inertia shenanigans
    samp(-pendulumSpeed, moveVec, true);
    if(p.lastInertia() == -1) return pendulumSpeed;

    // INERTIA SECTION
    int inertiaTick = p.lastInertia();
    bool hitVelNeg = p.hitVelNeg();

    log += "Inertia triggered at t = " + std::to_string(inertiaTick) + " during delayed pendulum simulation.\n" + "Vz on inertia tick: " + (hitVelNeg ? "neg" : "pos") + "\n";

    auto ISamp = [&](double vi, double m){
        p.resetAll();
        p.setVz(vi);
        p.sj45(m, 1);
        p.sa45(inertiaTick - 1);
        return p.Vz();
    };

    double inertia = (inertiaTick == 1)? groundInertia : airInertia;

    double v00 = ISamp(0,0);
    double v01 = ISamp(0,1);
    double m0LB = ( -inertia - v00 ) / ( v01 - v00 );
    double m0UB = ( inertia - v00 ) / ( v01 - v00 );

    double v10 = ISamp(1,0);
    double v11 = ISamp(1,1);
    double m1LB = ( -inertia - v10 ) / ( v11 - v10 );
    double m1UB = ( inertia - v10 ) / ( v11 - v10 );

    auto fullISamp = [&](double vi, double ma, double mj, bool hitInertia, bool falseZtrueVz){
        p.resetAll();
        p.setVz(vi);
        p.sj45(mj, 1);
        p.sa45(inertiaTick - 1);
        if(hitInertia) p.forceInertiaNext();
        p.sa45(ma, 1);
        p.sa45(t - inertiaTick - 1);
        p.chained_sj45(t, jumps);
        p.s45(1);
        return falseZtrueVz? p.Vz() : p.Z();
    };

    auto convergenceVzOnInertia = [&](double m0, double m1, bool hitInertia){

        double z00 = fullISamp(0, 0, m0, hitInertia, false);
        double z01 = fullISamp(0, 1, m0, hitInertia, false);
        double ma0 = (mm - z00)/(z01 - z00);

        double z10 = fullISamp(1, 0, m1, hitInertia, false);
        double z11 = fullISamp(1, 1, m1, hitInertia, false);
        double ma1 = (mm - z10)/(z11 - z10);

        double v0 = fullISamp(0, ma0, m0, hitInertia, true);
        double v1 = fullISamp(1, ma1, m1, hitInertia, true);

        return v0/(v1-v0+1);
    };

    if(hitVelNeg){
        // Hit inertia on lowerbound, and slow down afterward
        pendulumSpeed = convergenceVzOnInertia(m0LB, m1LB, true);

        log += "Hit inertia on lowerbound, and slow down afterward \n";
    }else{
        // hit inertia on lowerbound, full speed
        double v0 = fullISamp(0, 1, m0LB, true, true);
        double v1 = fullISamp(1, 1, m1LB, true, true);

        pendulumSpeed = v0/(v1-v0+1);

        // avoid inertia above, and slow down afterward
        double tempV = convergenceVzOnInertia(m0UB, m1UB, false);

        if(tempV > pendulumSpeed){
            pendulumSpeed = tempV;
            log += "Avoid inertia on upperbound, and slow down afterward \n";
        }else {
            log += "Hit inertia on lowerbound, full speed \n";
        }
    }

    return pendulumSpeed;
}

// Given maxBwSpeed, fit the best fw air strat (angled jt sj45)
double ZS::nondelayedPendulum(ZPlayer& p, double mm, int t, int jumps, double maxBwSpeed){
    
    auto samp = [&](double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(maxBwSpeed);
        p.sj45(m, 1);
        p.sa45(t - 1);
        p.chained_sj45(t, jumps);
        return falseZtrueVz? p.Vz() : p.Z();
    };

    // Lerp (0, z0), (1, z1) to find (moveVec, mm)
    double z0 = samp(0, false);
    double z1 = samp(1, false);
    double moveVec = (mm - z0)/(z1 - z0);

    // Simulate it
    double pendulumSpeed = samp(moveVec, true);
    if(p.lastInertia() == -1) return pendulumSpeed;

    // INERTIA SECTION
    int inertiaTick = p.lastInertia();
    bool hitVelNeg = p.hitVelNeg();

    log += "Inertia triggered at t = " + std::to_string(inertiaTick) + " during nondelayed pendulum simulation.\n" + "Vz on inertia tick: " + (hitVelNeg ? "neg" : "pos") + "\n";

    // NOTE: It is proven that avoiding inertia below is never optimal, same as the ground case.

    auto ISamp = [&](double m){
        p.resetAll();
        p.setVz(maxBwSpeed);
        p.sj45(m, 1);
        p.sa45(inertiaTick - 1);
        return p.Vz();
    };

    auto fullISamp = [&](double ma, double mj, bool hitInertia, bool falseZtrueVz){
        p.resetAll();
        p.setVz(maxBwSpeed);
        p.sj45(mj, 1);
        p.sa45(inertiaTick - 1);
        if(hitInertia) p.forceInertiaNext();
        p.sa45(ma, 1);
        p.sa45(t - inertiaTick - 1);
        p.chained_sj45(t, jumps);
        return falseZtrueVz? p.Vz() : p.Z();
    };

    double inertia = (inertiaTick == 1)? groundInertia : airInertia;

    double v0 = ISamp(0);
    double v1 = ISamp(1);
    double mInertiaLB = ( -inertia - v0 ) / ( v1 - v0 );
    double mInertiaUB = ( inertia - v0 ) / ( v1 - v0 );

    if(hitVelNeg){
        // Hit inertia on lowerbound, and slow down afterward
        double x0 = fullISamp(0, mInertiaLB, true, false);
        double x1 = fullISamp(1, mInertiaLB, true, false);
        double ma = (mm - x0)/(x1 - x0);

        pendulumSpeed = fullISamp(ma, mInertiaLB, true, true);
        log += "Hit inertia on lowerbound, and slow down afterward \n";
    }else{
        // hit inertia on lowerbound, full speed
        pendulumSpeed = fullISamp(1, mInertiaLB, true, true);

        // avoid inertia above, and slow down afterward
        double x0 = fullISamp(0, mInertiaUB, false, false);
        double x1 = fullISamp(1, mInertiaUB, false, false);
        double ma = (mm - x0)/(x1 - x0);

        double tempV = fullISamp(ma, mInertiaUB, false, true);

        if(tempV > pendulumSpeed){
            pendulumSpeed = tempV;
            log += "Avoid inertia on upperbound, and slow down afterward \n";
        }else {
            log += "Hit inertia on lowerbound, full speed \n";
        }
            
    }
    
    return pendulumSpeed;
}

// ----------------- Backwall Solver ----------------

ZS::halfStrat ZS::backwallSolve(double mm, int t, bool delayQ){
    ZPlayer p;
    double bestSpeed = 0;
    int stratType = -1;

    // Amount of sj45(t)'s the mm could fit
    int jumps = 0;
    double z0, zOverJump, z1ground, z1air;

    ZPlayer::State prevJump;

    while (true){
        prevJump = p.getState();
        if(delayQ && jumps != 0) p.loadState(); // Undo run 1t
        p.sj45(t);
        
        if(delayQ){
            p.saveState();
            p.s45(1);
        } 

        if(p.Z() > mm){
            zOverJump = p.Z();
            p.loadState(prevJump); // Undo a jump
            z0 = p.Z();
            break;
        }
        jumps ++;
    }

    p.resetAll();
    p.setVz(1);
    p.chained_sj45(t, jumps);
    if(delayQ) p.s45(1);
    z1ground = p.Z();

    auto fitMax = [&](bool isGround, double z_start, double z_end){
        int count = 0;
        double usedmm = 0;

        p.resetAll();
        while(usedmm <= mm){
            p.saveState();
            if(isGround) p.s45(1);
            else p.sa45(1);
            count ++;
            usedmm = p.Z() + (z_end - z_start) * p.Vz() + z_start;
        }
        p.loadState(); // Undo last sa45
        count --;
        
        return count;
    };

    // deltaZ =  (z1ground - z0) * ground_vi + z0
    // Find maximum x such that s45(x) chained_sj45(t, jumps) <= mm

    int x = fitMax(true, z0, z1ground);

    log += "The mm could fit at most " +  std::to_string(x) + " s45's before the sprint jumps\n";

    double zRun0 = p.Z() + (z1ground - z0) * p.Vz() + z0;
    double runBaseSpeed = p.Vz();

    const bool notInertia = false, YesInertia = true;
    const bool getZ = false, getVz = true;

    // Lambda: Evaluate strategy with linear interpolation
    auto calcSpeed = [&](auto& sampler, double baseSpeed, const std::string& name, bool checkInertia = true) {
        double z_0 = sampler(0, getZ, notInertia);
        double z_1 = sampler(1, getZ, notInertia);
        double moveVec = (mm - z_0) / (z_1 - z_0);
        double speed = sampler(moveVec, getVz, notInertia);

        if (checkInertia && p.lastInertia() != -1) {
            log += "Inertia triggered during " + name + " backwall solve.\n";
            double zi0 = sampler(0, getZ, YesInertia);
            double zi1 = sampler(1, getZ, YesInertia);
            double mInertia = (mm - zi0) / (zi1 - zi0);

            double tempV = sampler(mInertia, getVz, YesInertia);
            speed = tempV > baseSpeed ? tempV : baseSpeed;
        }
        log += name + " speed: " + std::to_string(speed) + "\n";
        return speed;
    };

    // 1. Only airstrat

    // test maxPessi sa45(t-1)
    p.resetAll();
    p.sa45(t - 1);
    p.chained_sj45(t, jumps);
    if(delayQ) p.s45(1);
    double maxPessiSpeed = p.Vz();
    bool needSj = (p.Z() < mm);

    if (needSj) {

        auto samp = [&](double m, bool falseZtrueVz, bool inertiaQ = false){
            p.resetAll();
            if(inertiaQ){
                p.setVz(groundInertia);
                p.sa45(m , 1);
            } else p.sj45(m, 1);
            p.sa45(inertiaQ ? t - 2 : t - 1);
            p.chained_sj45(t, jumps);
            if(delayQ) p.s45(1);
            return falseZtrueVz? p.Vz() : p.Z();
        };

        bestSpeed = calcSpeed(samp, maxPessiSpeed, "angled_jt");
        stratType = ZS::ANGLED_JT;

    } else {
        p.resetAll();
        p.setVzAir(1);
        p.chained_sj45(t, jumps);
        if(delayQ) p.s45(1);
        z1air = p.Z();

        // deltaZ =  (z1air - z0) * air_vi + z0
        // Find maximum y such that sa45(y) chained_sj45(t, jumps) <= mm

        int y = fitMax(false, z0, z1air);
        p.chained_sj45(t, jumps);
        if(delayQ) p.s45(1);
        double pessiSpeed = p.Vz();

        auto samp = [&](double ma, bool falseZtrueVz, bool inertiaQ = false){
            p.resetAll();
            if(inertiaQ) p.setVzAir(airInertia);
            if(y > 0 || !inertiaQ){
                p.sa45(ma, 1);
                p.sa45(inertiaQ ? y - 1 : y);
                p.chained_sj45(t, jumps);
            }else{
                p.sj45(ma, 1);
                p.sa45(t - 1);
                p.chained_sj45(t, jumps - 1);
            }
            if(delayQ) p.s45(1);
            return falseZtrueVz? p.Vz() : p.Z();
        };

        bestSpeed = calcSpeed(samp, pessiSpeed, "pessi");
        stratType = ZS::PESSI;
    }

    if(!delayQ && jumps == 0){
        // Rest of the strategies doesn't make sense for them.
        return ZS::halfStrat{stratType, bestSpeed};
    }

    // 2. airstrat + s45(x)
    double zRun1 = 0;
    
    if(x > 0){
        p.resetAll();
        p.setVzAir(1);
        p.s45(x);
        p.chained_sj45(t, jumps);
        if(delayQ) p.s45(1);
        zRun1 = p.Z();

        // deltaZ =  (zRun1 - zRun0) * air_vi + zRun0
        // Find maximum y such that sa45(y) s45(x) chained_sj45(t, jumps) <= mm

        int y = fitMax(false, zRun0, zRun1);
        p.s45(x);
        p.chained_sj45(t, jumps);
        if(delayQ) p.s45(1);
        double a7runBaseSpeed = p.Vz();

        auto samp = [&](double ma, bool falseZtrueVz, bool inertiaQ = false){
            p.resetAll();
            if(inertiaQ) p.setVzAir(airInertia);
            if(y > 0 || !inertiaQ){
                p.sa45(ma, 1);
                p.sa45(inertiaQ ? y - 1 : y);
                p.s45(x);
            }else {
                // I caught this bug on 2.25bm backwalled
                p.s45(ma, 1);
                p.s45(x - 1);
            }
            p.chained_sj45(t, jumps);
            if(delayQ) p.s45(1);
            return falseZtrueVz? p.Vz() : p.Z();
        };

        double a7runSpeed = calcSpeed(samp, a7runBaseSpeed, "a7run");

        if(a7runSpeed > bestSpeed){
            bestSpeed = a7runSpeed;
            stratType = ZS::A7RUN;
        }

    }

    

    // 3. ground speed + s45(x)

    {
        auto samp = [&](double m, bool falseZtrueVz, bool inertiaQ = false){
            p.resetAll();
            if(inertiaQ) p.setVz(groundInertia);
            p.s45(m, 1);
            p.s45(inertiaQ ? x - 1 : x);
            p.chained_sj45(t, jumps);
            if(delayQ) p.s45(1);
            return falseZtrueVz? p.Vz() : p.Z();
        };

        double runSpeed = calcSpeed(samp, runBaseSpeed, "run", p.lastInertia() != -1 && x > 0);
        if(runSpeed > bestSpeed){
            bestSpeed = runSpeed;
            stratType = ZS::RUN;
        }
    }

    return ZS::halfStrat{stratType, bestSpeed};
}

ZS::fullStrat ZS::backwallSolver(double mm, int t){
    log += "\nOptimal Backwalled Solver ----------------------- \n";
    log += "Target mm: " + std::to_string(mm) + ", airtime: " + std::to_string(t) + "\n";
    log += "\n- Delayed section: \n";
    ZS::halfStrat delayed = backwallSolve(mm, t, true);
    log += "\n- Nondelayed section: \n";
    ZS::halfStrat nondelayed = backwallSolve(mm, t, false);
    return ZS::fullStrat{delayed.stratType, delayed.optimalSpeed, nondelayed.stratType, nondelayed.optimalSpeed};
}

// ----------------- Utility Functions ----------------

std::string ZS::strat2string(int stratType) {
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
        case ZS::ANGLED_JT:
            return "Angled_JT";
        case ZS::PESSI:
            return "Pessi";
        case ZS::A7RUN:
            return "A7Run";
        case ZS::RUN:
            return "Run";
        default:
            return "Unnamed";
    }
}

void ZS::printLog(){
    std::cout << "LOG ----------------------- \n";
    std::cout << log;
}

void ZS::clearLog(){
    log = "";
}

bool ZS::poss(double mm, int t_mm, int max_t, double threshold, bool backwallQ, std::string& content, double shift){
    content = "";
    bool hasJump = false;
    ZS::fullStrat bestStrat = backwallQ ? backwallSolver(mm, t_mm) : optimalSolver(mm, t_mm);
    double dS = bestStrat.delaySpeed;
    double ndS = bestStrat.nondelaySpeed;
    ZPlayer dP;
    dP.setVz(dS);
    dP.sj45(1);
    ZPlayer ndP;
    ndP.setVzAir(ndS);
    ndP.sj45(1);
    content += "\n-------------------------------------------\n";
    content += std::string("For") + (backwallQ ? " backwalled " : " ") + "mm = " + fmt(mm) + " (airtime = " + fmt(t_mm)
    + "), t <= " + std::to_string(max_t) + ", threshold = " + fmt(threshold) + ", offset:" + fmt(shift) + "\n";
    content += "- NonDelayedSpeed: " + fmt(ndS) + ", Type: " + strat2string(bestStrat.nondelayStrat) + "\n";
    content += "- DelayedSpeed: " + fmt(dS) + ", Type: " + strat2string(bestStrat.delayStrat) + "\n";
    bool delayedBetter = true;
    for(int i = 2; i <= max_t; i++){
        dP.sa45(1);
        ndP.sa45(1);
        double zb;
        if(delayedBetter){
            zb = dP.Z();
            double temp = ndP.Z();
            if(temp > zb){
                delayedBetter = false;
                zb = temp;
                content += "(Nondelayed is better than Delayed at t = " + std::to_string(i) + ")\n";
            } 
        }else{
            zb = ndP.Z();
        }
        zb += shift;
        double offset = std::fmod(zb, 0.0625);

        if (offset < threshold) {
            hasJump = true;
            double jumpDis = zb - offset;

            content += "t = " + std::to_string(i) + ": "
            + fmt(jumpDis) + " + " + fmt(offset) + " b\n";
        }
    }

    if(!hasJump){
        content += "No possible jump found.\n";
    }

    return hasJump;

}

std::string ZSolver::fmt(double x) {
    std::ostringstream oss;
    if (std::abs(x) < 1e-8)
        oss << std::scientific << std::setprecision(16);
    else if (std::abs(x) < 1e-5)
        oss << std::fixed << std::setprecision(9);
    else
        oss << std::fixed << std::setprecision(6);
    oss << x;
    std::string s = oss.str();

    // Trim trailing zeros
    if (auto pos = s.find('e'); pos != std::string::npos) {
        // scientific notation
        auto end = s.find_last_not_of('0', pos - 1);
        if (s[end] == '.') --end;
        s.erase(end + 1, pos - end - 1);
    } else {
        // fixed notation
        auto end = s.find_last_not_of('0');
        if (end != std::string::npos && s[end] == '.')
            --end;
        s.erase(end + 1);
    }

    return s;
}

