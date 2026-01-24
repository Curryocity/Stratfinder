#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>

#include "zEngine.hpp"
#include "zSolver.hpp"
#include "util.hpp"

using ZS = zSolver;

// Finds the optimal speed for delayed and nondelayed strat: Given mm, mm-airtime
ZS::fullStrat zSolver::optimalSolver(double mm, int t)
{
    writeLog("\nOptimal Solver ----------------------- \n");
    writeLog("Target mm: " + util::fmt(mm) + ", airtime: " + std::to_string(t) + "\n");

    writeLog("\n- Delayed section: \n");

    // Solve delayed version first to get maxBwSpeed
    int bestDelayTick = 1;
    int delayTick = 1;
    ZS::strat delayedStrat = optimalDelayed(mm, t);
    int dT = delayedStrat.stratType;
    double dS = delayedStrat.optimalSpeed;

    zEngine p(speed, slowness);
    p.s45(1);
    double sprint45Vz = p.Vz();
    double terminalSpeed = sprint45Vz/ 0.454;
    // only low speed could delayTick > 1 benifit
    if(dS < terminalSpeed){

        writeLog( "Best vz = " +  std::to_string(dS) + "\n\n");
        int maxDelayTick = -1;
        p.resetAll();
        for(; p.Z() < mm + 0.6f; maxDelayTick ++)
            p.s45(1);

        writeLog( "maxDelayTick = " + std::to_string(maxDelayTick) + "\n");
        
        for(delayTick = std::max(2, maxDelayTick - 1); delayTick <= maxDelayTick; delayTick ++){
            ZS::strat newDelayedStrat = optimalDelayed(mm, t, delayTick);
            writeLog( "Best vz = " +  std::to_string(newDelayedStrat.optimalSpeed) + "\n\n");
            if(newDelayedStrat.optimalSpeed > dS){
                dS = newDelayedStrat.optimalSpeed;
                dT = newDelayedStrat.stratType;

                bestDelayTick = delayTick;
            }
        }
    }
    p.resetAll();
    
    double maxBwSpeed = -dS;

    mm += 0.6f;

    writeLog( "\n- Nondelayed section: \n");
    writeLog( "Max BW speed: " + std::to_string(maxBwSpeed) + "\n");
    // Then solve nondelayed, given the knowledge of maxBwSpeed
    ZS::CoreCtx c = solverCore(p, mm, t, NONDELAYED, maxBwSpeed);

    ZS::strat out;
    if (earlyPrune(c, out)) return fullStrat{dT, dS, out.stratType, out.optimalSpeed, bestDelayTick};

    double pendulumSpeed = nondelayedPendulum(p, mm, t, c.o1.jumps, maxBwSpeed);
    return fullStrat{dT, dS, ZS::PENDULUM, pendulumSpeed, bestDelayTick};

}

// Finds the optimal delayed speed: Given mm, mm-airtime
ZS::strat ZS::optimalDelayed(double mm, int t, int delayTick)
{
    writeLog( "delayTick = " + std::to_string(delayTick) + "\n");
    zEngine p(speed, slowness);
    mm += 0.6f;

    ZS::CoreCtx c = solverCore(p, mm, t, delayTick, 0);

    ZS::strat out;
    if (earlyPrune(c, out)) return out;

    double pendulumSpeed = delayedPendulum(p, mm, t, c.o1.jumps, delayTick);    
    ZS::strat bestStrat = strat{ZS::PENDULUM, pendulumSpeed};
    
    if(pendulumSpeed >= -c.o2.reqBwSpeed)bestStrat = strat{ZS::SLINGSHOT, c.o2.slingSpeed};
    if(pendulumSpeed >= -c.o4.bwSpeedBoom) bestStrat = strat{ZS::BOOMERANG, c.o4.boomSpeed};
    if(bestStrat.optimalSpeed < c.o3.roboSpeed) bestStrat = strat{ZS::ROBO, c.o3.roboSpeed};

    return bestStrat;
}

// Runs: heuristics, slingShot, robo, boomerang (no equilibrium / no moveVec fit)
ZS::CoreCtx ZS::solverCore(zEngine& p, double mm, int t, int delayTick, double knownBwCap){
    CoreCtx c;
    c.o1 = ZS::mmHeuristics(p, mm, t, delayTick, knownBwCap);
    c.o2 = ZS::slingShot(p, mm, t, delayTick, c.o1);
    c.o3 = ZS::robo(p, mm, t, delayTick, c.o1.jumps);
    c.o4 = ZS::boomerang(p, mm, t, delayTick, c.o1);
    return c;
}

// Applies the shared early-return rules (slingshot / true robo / robo vs boomerang).
// Returns true if early prunable, and fills `out`.
bool ZS::earlyPrune(const CoreCtx& c, ZS::strat& out){
    if (c.o2.possSling && (c.o2.slingSpeed > c.o3.roboSpeed) ) {
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
ZS::Output1 ZS::mmHeuristics(zEngine& p, double mm, int t, int delayTick, double knownBwCap){
    
    // Amount of sj45(t)'s an mm could fit, without bwSpeed
    int jumps = 0;
    double overJamDis;
    double jamDis;

    zEngine::State prevJump;

    
    if(delayTick > 0){
        p.saveState();
        p.s45(delayTick);
    } 

    while (true){
        prevJump = p.getState();
        if(delayTick > 0)
            p.loadState(); // Undo run
        p.sj45(t);
        
        if(delayTick > 0){
            p.saveState();
            p.s45(delayTick);
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
        p.resetAll();
        p.chained_sj45(t, jumps);

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
        writeLog( "BW speed lowerBound: " + std::to_string(bestBwSpeed) + "\n");

        
    }else{
        bestBwSpeed = knownBwCap;
    }

    
    p.resetAll();
    p.setVz(bestBwSpeed);
    p.chained_sj45(t, jumps + 1);
    p.s45(delayTick);
    double bwmmDis = p.Z();

    p.resetAll();


    return Output1{jumps, overJamDis, jamDis, bestBwSpeed, bwmmDis};
}

ZS::Output2 ZS::slingShot(zEngine& p, double mm, int t, int delayTick, Output1& o1){

    double slingSpeed = 0;
    // Lerp (0, overJamDis), (bestBwSpeed, bwmmDis) to find (reqVz, mm)
    double reqBwSpeed = o1.bestBwSpeed * (o1.overJamDis - mm) / (o1.overJamDis - o1.bwmmDis);
    bool poss = false;
    writeLog( "Required BW speed: " + std::to_string(reqBwSpeed) + "\n");

    if(-reqBwSpeed >= groundInertia){
        p.resetAll();
        p.setVz(reqBwSpeed);
        p.chained_sj45(t, o1.jumps + 1);
        p.s45(delayTick);
        slingSpeed = p.Vz();
    }else{
        // If reqBwSpeed hits inertia, set bwSpeed to just hit inertia barely(has the effect to extend the mm by ~0.0091575), angled the first jump tick.
        writeLog( "This backward speed hits inertia! Fixing... \n");
        double extendedmm = mm + groundInertia;

        auto getSample = [&](double m, bool falseZtrueVz){
            p.resetAll();
            p.sj45(m, 1);
            p.sa45(t - 1);
            p.chained_sj45(t, o1.jumps);
            p.s45(delayTick);
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

    p.resetAll();
    p.setVz(reqBwSpeed);
    p.sj45(t);
    // it becomes a boomerang
    if(p.Z() < 0){
        poss = false;
        slingSpeed = -INFINITY;
        writeLog("Slingshot isn't poss cuz you land behind the mm, it's a boomerang.\n");
    }

    return Output2{reqBwSpeed, slingSpeed, poss};
}


ZS::Output3 ZS::robo(zEngine& p, double mm, int t, int delayTick, int jumps){

    // Robo doesn't make sense
    if(jumps == 0 && delayTick <= 1) return Output3{false, 0};

    p.resetAll();
    p.s45(1);
    double hhSpeed = p.Vz(); 
    p.chained_sj45(t, jumps);
    p.s45(delayTick);

    double hhDis = p.Z();

    // Robo could beat boomerang only when it is bwmm into hh1t
    if(hhDis <= mm) return Output3{false, 0};

    double roboSpot1; 
    p.resetAll();

    // The border of robo and true robo
    // The formula is v_0 = -hhSpeed*(0.6/slip)^3/(1+0.91*slip), derived from v_0 + v_1 = 0
    double borderSpeed = -hhSpeed/(1.0 + 0.6f * 0.91f);
    p.setVz(borderSpeed);
    p.s45(1);
    p.chained_sj45(t, jumps);
    p.s45(delayTick);
    double borderDis = p.Z();

    double roboBwSpeed;

    bool trueRoboQ = (borderDis >= mm);

    hhDis -= hhSpeed * trueRoboQ; // Substract hh Speed from distance if true robo ( Mothball: vz(bwSpeed) s45 sj | sa45(t-1) )
    roboBwSpeed = borderSpeed * (mm - hhDis) / (borderDis - hhDis);

    p.resetAll();
    p.setVz(roboBwSpeed);

    p.s45(1);
    if(jumps > 0){
        p.sj45(1);
        if(trueRoboQ) p.setZ(0);
        p.sa45(t - 1);
        p.chained_sj45(t, jumps - 1);
        p.s45(delayTick);
    }else{
        p.s45(1);
        if(trueRoboQ) p.setZ(0);
        p.s45(delayTick - 1);
    }

    double roboSpeed = p.Vz();
    p.resetAll();

    writeLog( "roboSpeed: " + util::fmt(roboSpeed) + "\n");
    writeLog( "isTrueRobo? " + std::to_string(trueRoboQ) + "\n");

    return Output3{trueRoboQ, roboSpeed};
}


ZS::Output4 ZS::boomerang(zEngine& p, double mm, int t, int delayTick, Output1& o1){

    // Bommerang doesn't make sense when that
    if(o1.jumps == 0 && delayTick <= 1) return Output4{-INFINITY, 0, false};

    bool poss = false;

    // Fitting an initial air speed
    p.resetAll();
    p.setVzAir(1);
    p.chained_sj45(t, o1.jumps);
    p.s45(delayTick);
    double z3 = p.Z();

    // Lerp (0, jamDis), (z3, 1) to find (reqFwSpeed, mm)
    double reqFwSpeed = (mm - o1.jamDis) / (z3 - o1.jamDis);
    writeLog( "FW airspeed req: " + std::to_string(reqFwSpeed) + "\n");

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

    writeLog( "BW speedreq for boomerang: " + std::to_string(reqBwSpeed) + "\n");

    // Simulate boomerang speed assuming it is possible
    p.resetAll();
    p.setVzAir(reqFwSpeed);
    p.chained_sj45(t, o1.jumps);
    p.s45(delayTick);
    double boomSpeed = p.Vz();

    if(-reqBwSpeed < -o1.bestBwSpeed)
        poss = true;

    p.resetAll();
    p.setVz(reqBwSpeed);
    p.sj45(t);
    if(p.Z() < 0){
        poss = false;
        boomSpeed = -INFINITY;
        writeLog( "Even if reqBwSpeed is achievable, a full sj45(t) isn't enough to do a borderline boomerang\n");
    }

    return Output4{reqBwSpeed, boomSpeed, poss};
}


// Finding the velocity convergence of chained loop by solving the equation of reqBwSpeed = finalSpeed
double ZS::delayedPendulum(zEngine& p, double mm, int t, int jumps, int delayTick){

    auto samp = [&](double vi, double m, bool falseZtrueVz){
        p.resetAll();
        p.setVz(vi);
        p.sj45(m, 1);
        p.sa45(t - 1);
        p.chained_sj45(t, jumps);
        p.s45(delayTick);
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

    writeLog( "Inertia triggered at t = " + std::to_string(inertiaTick) + " during delayed pendulum simulation.\n" + "Vz on inertia tick: " + (hitVelNeg ? "neg" : "pos") + "\n");

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
        p.s45(delayTick);
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

        writeLog( "Hit inertia on lowerbound, and slow down afterward \n");
    }else{
        // hit inertia on lowerbound, full speed
        double v0 = fullISamp(0, 1, m0LB, true, true);
        double v1 = fullISamp(1, 1, m1LB, true, true);

        pendulumSpeed = v0/(v1-v0+1);

        // avoid inertia above, and slow down afterward
        double tempV = convergenceVzOnInertia(m0UB, m1UB, false);

        if(tempV > pendulumSpeed){
            pendulumSpeed = tempV;
            writeLog( "Avoid inertia on upperbound, and slow down afterward \n");
        }else {
            writeLog( "Hit inertia on lowerbound, full speed \n");
        }
    }

    return pendulumSpeed;
}

// Given maxBwSpeed, fit the best fw air strat (angled jt sj45)
double ZS::nondelayedPendulum(zEngine& p, double mm, int t, int jumps, double maxBwSpeed){
    
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

    writeLog( "Inertia triggered at t = " + std::to_string(inertiaTick) + " during nondelayed pendulum simulation.\n" + "Vz on inertia tick: " + (hitVelNeg ? "neg" : "pos") + "\n");

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
        writeLog( "Hit inertia on lowerbound, and slow down afterward \n");
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
            writeLog( "Avoid inertia on upperbound, and slow down afterward \n");
        }else {
            writeLog( "Hit inertia on lowerbound, full speed \n");
        }
            
    }
    
    return pendulumSpeed;
}

// ----------------- Backwall Solver ----------------

ZS::strat ZS::backwallSolve(double mm, int t, int delayTick){
    if(delayTick > 0) writeLog( "delayTick = " + util::fmt(delayTick) + "\n");
    zEngine p(speed, slowness);
    double bestSpeed = 0;
    int stratType = -1;

    // Amount of sj45(t)'s the mm could fit
    int jumps = 0;
    double z0, zOverJump, z1ground, z1air;

    zEngine::State prevJump;

    if(delayTick > 0){
        p.saveState();
        p.s45(delayTick);
    } 

    while (true){
        prevJump = p.getState();
        if(delayTick > 0)
            p.loadState(); // Undo run
        p.sj45(t);
        
        if(delayTick > 0){
            p.saveState();
            p.s45(delayTick);
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
    p.s45(delayTick);
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

    writeLog( "Fit at most s45(" +  util::fmt(x) + ") r(sj45(" + util::fmt(t) + "), " + util::fmt(jumps) + ")\n");

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
            writeLog( "Inertia triggered during " + name + " backwall solve.\n");
            double zi0 = sampler(0, getZ, YesInertia);
            double zi1 = sampler(1, getZ, YesInertia);
            double mInertia = (mm - zi0) / (zi1 - zi0);

            double tempV = sampler(mInertia, getVz, YesInertia);
            speed = tempV > baseSpeed ? tempV : baseSpeed;
        }
        writeLog( name + " speed: " + std::to_string(speed) + "\n");
        return speed;
    };

    // 1. Only airstrat

    // test maxPessi sa45(t-1)
    p.resetAll();
    p.sa45(t - 1);
    p.chained_sj45(t, jumps);
    p.s45(delayTick);
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
            p.s45(delayTick);
            return falseZtrueVz? p.Vz() : p.Z();
        };

        bestSpeed = calcSpeed(samp, maxPessiSpeed, "angled_jt");
        stratType = ZS::ANGLED_JT;

    } else {
        p.resetAll();
        p.setVzAir(1);
        p.chained_sj45(t, jumps);
        p.s45(delayTick);
        z1air = p.Z();

        // deltaZ =  (z1air - z0) * air_vi + z0
        // Find maximum y such that sa45(y) chained_sj45(t, jumps) <= mm

        int y = fitMax(false, z0, z1air);
        p.chained_sj45(t, jumps);
        p.s45(delayTick);
        double pessiSpeed = p.Vz();

        auto samp = [&](double ma, bool falseZtrueVz, bool inertiaQ = false){
            p.resetAll();
            if(inertiaQ) p.setVzAir(airInertia);
            if(y > 0 || !inertiaQ){
                p.sa45(ma, 1);
                p.sa45(inertiaQ ? y - 1 : y);
                p.chained_sj45(t, jumps);
                p.s45(delayTick);
            }else{
                if(jumps > 0){
                    p.sj45(ma, 1);
                    p.sa45(t - 1);
                    p.chained_sj45(t, jumps - 1);
                    p.s45(delayTick);
                }else{
                    // History: speed 4 4tmm 1.5bm backwalled edge case
                    p.s45(ma, 1);
                    p.s45(delayTick - 1);
                }

            }
            
            return falseZtrueVz? p.Vz() : p.Z();
        };

        bestSpeed = calcSpeed(samp, pessiSpeed, "pessi");
        stratType = ZS::PESSI;
    }

    if(delayTick == 0 && jumps == 0){
        // Rest of the strategies doesn't make sense for them.
        return ZS::strat{stratType, bestSpeed};
    }

    // 2. airstrat + s45(x)
    double zRun1 = 0;
    
    if(x > 0){
        p.resetAll();
        p.setVzAir(1);
        p.s45(x);
        p.chained_sj45(t, jumps);
        p.s45(delayTick);
        zRun1 = p.Z();

        // deltaZ =  (zRun1 - zRun0) * air_vi + zRun0
        // Find maximum y such that sa45(y) s45(x) chained_sj45(t, jumps) <= mm

        int y = fitMax(false, zRun0, zRun1);
        p.s45(x);
        p.chained_sj45(t, jumps);
        p.s45(delayTick);
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
            p.s45(delayTick);
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
            p.s45(delayTick);
            return falseZtrueVz? p.Vz() : p.Z();
        };

        double runSpeed = calcSpeed(samp, runBaseSpeed, "run", p.lastInertia() != -1 && x > 0);
        if(runSpeed > bestSpeed){
            bestSpeed = runSpeed;
            stratType = ZS::RUN;
        }
    }

    return ZS::strat{stratType, bestSpeed};
}

ZS::fullStrat ZS::backwallSolver(double mm, int t){
    writeLog( "\nOptimal Backwalled Solver ----------------------- \n");
    writeLog( "Target mm: " + util::fmt(mm) + ", airtime: " + std::to_string(t) + "\n");
    writeLog( "\n- Delayed section: \n");

    int delayTick = 1;
    ZS::strat delayed = backwallSolve(mm, t, DELAYED);

    double dS = delayed.optimalSpeed;
    int dT = delayed.stratType;

    zEngine p(speed, slowness);
    p.s45(1);
    double sprint45Vz = p.Vz();
    double terminalSpeed = sprint45Vz/ 0.454;
    double bestDelayTick = delayTick;

    p.resetAll();
    p.sj45(t);
    p.s45(1);

    // only low speed could delayTick > 1 benifit, delay long ticks were already considered if jumps == 0, skip
    if(dS < terminalSpeed && p.Z() < mm){

        writeLog( "Best vz = " +  std::to_string(dS) + "\n\n");
        int maxDelayTick = -1;
        p.resetAll();
        for(; p.Z() < mm; maxDelayTick ++)
            p.s45(1);

        writeLog( "(max) ");
        
        ZS::strat newDelayedStrat = backwallSolve(mm, t, maxDelayTick);
        writeLog( "Best vz = " +  std::to_string(newDelayedStrat.optimalSpeed) + "\n\n");
        if(newDelayedStrat.optimalSpeed > dS){
            dS = newDelayedStrat.optimalSpeed;
            dT = newDelayedStrat.stratType;

            delayTick = maxDelayTick;
        }

    }
    p.resetAll();

    p.resetAll();
    writeLog( "\n- Nondelayed section: \n");
    ZS::strat nondelayed = backwallSolve(mm, t, NONDELAYED);
    return ZS::fullStrat{dT, dS, nondelayed.stratType, nondelayed.optimalSpeed, delayTick};
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

void ZS::writeLog(std::string str){
    if(logOn) log += str;
}

void ZS::printLog(){
    std::cout << "LOG ----------------------- \n";
    std::cout << log;
}

void ZS::clearLog(){
    log = "";
}

void ZS::toggleLog(bool on){
    logOn = on;
}

bool ZS::poss(double mm, int t_mm, int max_t, double threshold, bool backwallQ, std::string& content, double shift, std::optional<fullStrat> provideStrat){
    content = "";
    bool hasJump = false;
    ZS::fullStrat strat;
    if(!provideStrat){
        strat = backwallQ ? backwallSolver(mm, t_mm) : optimalSolver(mm, t_mm);
    }else{
        strat = provideStrat.value();
    }
    
    double dS = strat.delaySpeed;
    double ndS = strat.nondelaySpeed;
    zEngine dP(speed, slowness);
    dP.setVz(dS);
    dP.sj45(1);
    zEngine ndP(speed, slowness);
    ndP.setVzAir(ndS);
    ndP.sj45(1);
    content += "\n-------------------------------------------\n";
    content += std::string("For") + (backwallQ ? " backwalled " : " ") + "mm = " + util::fmt(mm) + " (airtime = " + util::fmt(t_mm)
    + "), t <= " + std::to_string(max_t) + ", threshold = " + util::fmt(threshold) + ", offset:" + util::fmt(shift) + "\n";
    content += "- NonDelayedSpeed: " + util::df(ndS) + ", Type: " + strat2string(strat.nondelayStrat) + "\n";
    content += "- DelayedSpeed(dt=" + std::to_string(strat.delayTick) + "): " + util::fmt(dS) + ", Type: " + strat2string(strat.delayStrat) + "\n";
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

        if (offset < threshold && offset >= 0) {
            hasJump = true;
            double jumpDis = zb - offset;

            content += "t = " + std::to_string(i) + ": "
            + util::fmt(jumpDis) + " + " + util::fmt(offset) + " b\n";
        }
    }

    if(!hasJump){
        content += "No possible jump found.\n";
    }

    return hasJump;

}

void ZS::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
    std::cout << "Set Speed: " << speed << ", Slowness: " << slowness << "\n";
}

void ZS::clearEffects(){
    speed = 0;
    slowness = 0;
    std::cout << "Cleared all effects";
}

