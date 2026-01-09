#include <iostream>
#include <cmath>
#include "ZPlayer.hpp"
#include "ZSolver.hpp"

using ZS = ZSolver;

void ZS::init(){
    ZPlayer::init();
}

// Finds the optimal speed for delayed and nondelayed strat: Given mm, mm-airtime
ZS::fullStrat ZSolver::optimalSolve(double mm, int t)
{
    std::cout << "LOG ----------------------- \n";

    std::cout << "- Delayed section: \n";
    // Solve delayed version first to get maxBwSpeed
    ZS::halfStrat delayedStrat = optimalDelayed(mm, t);
    int dT = delayedStrat.stratType;
    double dS = delayedStrat.optimalSpeed;
    double maxBwSpeed = -dS;

    ZPlayer p;
    mm += 0.6f;

    std::cout << "\n- Nondelayed section: \n";
    std::cout << "Max BW speed: " << maxBwSpeed << "\n";
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
        std::cout << "Estimates BW speed lowerBound: " << bestBwSpeed << "\n";
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
    std::cout << "Required BW speed: " << reqBwSpeed << "\n";

    if(-reqBwSpeed >= groundInertia){
        p.resetAll();
        p.setVz(reqBwSpeed);
        p.chained_sj45(t, o1.jumps + 1);
        if(delayQ) p.s45(1);
        slingSpeed = p.Vz();
    }else{
        // If reqBwSpeed hits inertia, set bwSpeed to just hit inertia barely(has the effect to extend the mm by ~0.0091575), angled the first jump tick.
        std::cout << "This backward speed hits inertia! Fixing... \n";
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
    std::cout << "Required FW airspeed: " << reqFwSpeed << "\n";

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

    std::cout << "Required BW speed for boomerang: " << reqBwSpeed << "\n";

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

    double pendulumSpeed = v0/(v1-v0+1);
    // Lerp (0, z0) and (1, z1) to get (moveVec, mm)
    double z0 = samp(-pendulumSpeed, 0, false);
    double z1 = samp(-pendulumSpeed, 1, false);
    double moveVec = (mm - z0)/(z1 - z0);

    // Simulate it to check inertia shenanigans
    samp(-pendulumSpeed, moveVec, true);
    if(p.lastInertia() == -1) return pendulumSpeed;

    // INERTIA SECTION
    int inertiaTick = p.lastInertia();
    bool hitVelNeg = p.hitVelNeg();

    std::cout << "Inertia triggered at t = " << inertiaTick << " during delayed pendulum simulation.\n" << "Vz on inertia tick: " << (hitVelNeg ? "neg" : "pos") << "\n";

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

    std::cout << "Inertia triggered at t = " << inertiaTick << " during nondelayed pendulum simulation.\n" << "Vz on inertia tick: " << (hitVelNeg ? "neg" : "pos") << "\n";

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

    // Lerp (0, v0), (1, v1) to find (moveVec, - airInertia) and (moveVec, airInertia)
    double v0 = ISamp(0);
    double v1 = ISamp(1);
    double mInertiaLB = ( -airInertia - v0 ) / ( v1 - v0 );
    double mInertiaUB = ( airInertia - v0 ) / ( v1 - v0 );

    if(hitVelNeg){
        // Hit inertia on lowerbound, and slow down afterward
        double x0 = fullISamp(0, mInertiaLB, true, false);
        double x1 = fullISamp(1, mInertiaLB, true, false);
        double ma = (mm - x0)/(x1 - x0);

        pendulumSpeed = fullISamp(ma, mInertiaLB, true, true);
        std::cout << "Hit inertia on lowerbound, and slow down afterward \n";
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
            std::cout << "Avoid inertia on upperbound, and slow down afterward \n";
        }else {
            std::cout << "Hit inertia on lowerbound, full speed \n";
        }
            
    }
    
    return pendulumSpeed;
}


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
        default:
            return "Unnamed";
    }
}
