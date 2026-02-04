#include "inputFinder.hpp"
#include "util.hpp"
#include "zEngine.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using IF = inputFinder;

void IF::setCondWithBound(axisCond& cond, double bound1, double bound2){
    cond.vel = (bound1 + bound2)/2;
    cond.tolerance = std::abs((bound1 - bound2)/2);
}

// heuristics
void IF::initHeuristics(int airtime, double zDis, double xDis){

    // initialize errRec
    zErrRec = std::vector<double>(airtime + 1);  
    xErrRec = std::vector<double>(airtime + 1);

    zEngine e(speed, slowness);
    e.s45(1);
    double gTerm = e.Vz()/(1.0 - 0.6f * 0.91f);
    e.setVz(0);
    e.sa45(1);
    double aTerm = e.Vz()/(1.0 - 0.91f)/0.6f;

    bool groundBetter = gTerm > aTerm;
    // this holds for every keystroke and axis

    getDummy();

    auto getZ = [&](player p){ return p.Z();};
    auto getX = [&](player p){ return p.X();};
    auto getVz = [&](player p){ return p.Vz();};
    auto getVx = [&](player p){ return p.Vx();};

    auto getVelUbLb = [&](player& p, std::vector<double>& terminalRecorder, auto& getVel, auto& getPos, double dis){

        int wLB = 0, aLB = 0, wUB = 0, aUB = 0;
        double noJumpLb = 0, noJumpUb = 0;

        for (int w = -1; w <= 1; w++) {
            for (int a = -1; a <= 1; a++){
                if(w == 0 && a == 0) continue;
                p.setVel();
                p.setPrevSprint((w == 1));
                int sprint = 2 *(w == 1);
                p.move(w, a, !groundBetter, sprint, 1);
                double v = getVel(p);
                
                if(v < noJumpLb)
                    wLB = w, aLB = a, noJumpLb = v;
                if(v > noJumpUb)
                    wUB = w, aUB = a, noJumpUb = v;

                if(groundBetter)
                    terminalRecorder[3*(a+1) + (w+1)] = v/(1.0 - 0.6f * 0.91f);
                else
                    terminalRecorder[3*(a+1) + (w+1)] = getVel(p)/(1.0 - 0.91f)/0.6f;
            }
        }

        noJumpLb = terminalRecorder[3*(aLB+1) + (wLB+1)];
        noJumpUb = terminalRecorder[3*(aUB+1) + (wUB+1)];

        // Return running terminal speed, since jumpingSpeed < airSpeed when ground is better
        if(groundBetter) return std::array<double, 2>{ noJumpLb, noJumpUb};

        // jumping
        auto getJumpVel = [&](int w, int a){
            int jumps = 0;
            double vel = 0;
            int sprint = 2 *(w == 1);
            p.resetAll();
            while (std::abs(getPos(p)) < dis && jumps < 3) {
                
                p.move(w, a, false, sprint, 1);
                p.move(w, a, true, sprint, airtime - 1);
                vel = getVel(p);
                jumps ++;
            }

            double jumpVel;
            if(jumps <= 3){
                jumpVel = vel;

                // trying to bargain with a random bwmm into jump
                p.resetAll();
                p.setVel(-vel, -vel);
                while (jumps --) {
                    p.move(w, a, false, sprint, 1);
                    p.move(w, a, true, sprint, airtime - 1);
                }

                if(std::abs(getPos(p)) > zDis) jumpVel = getVel(p);

            } else{ // approximate many jumps by terminal speed
                p.setVel(0,0, true);
                p.move(w, a, false, sprint, 1);
                p.move(w, a, true, sprint, airtime - 1);
                double v0 = getVel(p);

                p.setVel(1,1, true);
                p.move(w, a, false, sprint, 1);
                p.move(w, a, true, sprint, airtime - 1);
                double v1 = getVel(p);
                jumpVel = - v0/(v1 - v0 - 1.0);
            }

            // convert the speed to groundSpeed (so we can just dummy.setVz(vz, airborne = false) every time)
            return jumpVel / 0.6f;
        };

        double jumpLb = getJumpVel(wLB, aLB);
        double jumpUb = getJumpVel(wUB, aUB);

        double velLb = (jumpUb > noJumpUb)? jumpLb : noJumpLb;
        double velUb = (jumpUb > noJumpUb)? jumpUb : noJumpUb;

        return std::array<double, 2>{ velLb, velUb};
    };

    std::array<double, 2> zVelBound = getVelUbLb(getDummy() ,wasdTerminalVz, getVz, getZ, zDis);
    std::array<double, 2> xVelBound = getVelUbLb(getDummy() ,wasdTerminalVx, getVx, getX, xDis);

    this->vxLB = xVelBound[0];
    this->vxUB = xVelBound[1];
    this->vzLB = zVelBound[0];
    this->vzUB = zVelBound[1];

}

std::vector<IF::sequence> IF::matchSpeed(const condition& cond, int airtime){

    if( (!cond.x.enabled) && (!cond.z.enabled) ){
        std::cout << "Exception: None of the conditions were enabled\n";
        std::cout << "------EXIT------\n";
        return {};
    } 

    std::vector<IF::sequence> result;
    initHeuristics(airtime, std::abs(cond.z.mm) + 0.6f, std::abs(cond.x.mm) + 0.6f);

    // find input sequence via iterative deepening dfs
    for(int limit = 1; limit <= maxDepth; limit ++){
        std::vector<IF::sequence> partialResult = dfsEntry(cond, airtime, limit);
        result.reserve(result.size() + partialResult.size());
        result.insert(result.end(), partialResult.begin(), partialResult.end());
    }

    return result;

}

std::vector<IF::sequence> IF::dfsEntry(const condition& cond, int airtime, int depthLimit){
    std::cout << "-------------------------------------------------\n";
    std::cout << "Try searching depth = " << depthLimit << " inputs\n";
    std::vector<IF::sequence> result;
    sequence node;
    node.inputs = std::vector<input>();
    node.revJumps = std::vector<int>();
    node.airtime = airtime;
    node.T = 0;
    
    dfsRecursive(0, depthLimit, node, cond, result);

    return result;
}

// return true for hardPrune, false for softPrune
bool IF::dfsRecursive(int depth, int depthLimit, sequence& node, const condition& cond, std::vector<sequence>& result) {

    if(node.T > maxTicks) return true;

    const bool careZ = cond.z.enabled;
    const bool careX = cond.x.enabled;

    alphaBetaUpdate(getDummy(), node, careX, careZ);

    if (depth == depthLimit) {

        util::vec2D speedVec = estimateSpeed(node, cond.endedAirborne);

        node.lerpX.error = careX? std::abs(speedVec.x - cond.x.vel) - cond.x.tolerance - inertiaErr : 0;
        node.lerpZ.error = careZ? std::abs(speedVec.z - cond.z.vel) - cond.z.tolerance - inertiaErr : 0;
        
        if(node.lerpX.error <= 0 && node.lerpZ.error <= 0){

            node.lerpX.error = 0, node.lerpZ.error = 0;

            bool valid = exeSeq(dummy, node, cond);
            double vx = dummy.Vx(), vz = dummy.Vz();

            bool xSat = (!careX) || (std::abs(vx - cond.x.vel) <= cond.x.tolerance);
            bool zSat = (!careZ) || (std::abs(vz - cond.z.vel) <= cond.z.tolerance);

            
            if(valid && xSat && zSat){
                node.finalVx = vx, node.finalVz = vz;
                std::string vxStr = careX? (", Vx: " + util::df(vx)) : "";
                std::string vzStr = careZ? (", Vz: " + util::df(vz)) : "";
                std::cout << "\n";
                std::cout << "Found Seqeunce: " << seq2Mothball(node) 
                << "\nt = " << node.T << "(+" << std::max(0,node.airDebt) << ")" << ", Vz: " << util::df(vz) << ", Vx: " << util::df(vx) << "\n";
                result.push_back(node);
            }
        }

    }

    if(node.T > 0){
        util::vec2D minV = estimateSpeed(node, cond.endedAirborne,this->vxLB, this->vzLB);
        double minVx = minV.x - floatErr;
        double minVz = minV.z - floatErr;
        if(careX && (minVx > (cond.x.vel + cond.x.tolerance))) return true;
        if(careZ && (minVz > (cond.z.vel + cond.z.tolerance))) return true;

        util::vec2D maxV = estimateSpeed(node, cond.endedAirborne,this->vxUB, this->vzUB);
        double maxVx = maxV.x + floatErr;
        double maxVz = maxV.z + floatErr;
        if(careX && (maxVx < (cond.x.vel - cond.x.tolerance))) return true;
        if(careZ && (maxVz < (cond.z.vel - cond.z.tolerance))) return true;
    }
    
    const int baseTick = node.T;
    const bool biSymmetric = rotation == 0.0f || rotation == 180.0f || !careX;

    int prevW = 69, prevA = 69, prevT = 0;
    input prevInput;
    if(!node.inputs.empty()){
        prevInput = node.inputs.back();
        prevW = prevInput.w;
        prevA = prevInput.a;
        prevT = prevInput.t;
    }

    for (int w = -1; w <= 1; w++) {
        for (int a = -1; a <= 1; a++) { 

            if(!cond.allowStrafe && a != 0) continue;

            if(biSymmetric && ( a < 0 || (w == 0 && a != 0) ) ) continue; 
            // When biSymmetric:
            // A/D gives the same outcome, thus wlog ignore assume a>=0
            // Pressing either A/D does nothing when W/S is not held
   
            bool inputExtension = false;
            if((w == prevW) && (a == prevA)){
                // Allow the extension of same movement key only when t is maximized on previous round
                if(prevT == node.airtime) inputExtension = true;
                else continue;
            }

            // only inputExtension is allowed at maxDepth
            if(depth == depthLimit && !inputExtension) continue;

            const bool endingDepth = (depth >= depthLimit - 1);

            if(endingDepth){
                // the initial input cannot be blank
                if(w == 0 && a == 0) continue;

                util::vec2D eV = estimateSpeed(node, cond.endedAirborne);
                double eVx = eV.x;
                double eVz = eV.z;

                util::vec2D tV = terminalToSeq(w, a, node, cond.endedAirborne);
                double tVx = tV.x;
                double tVz = tV.z;

                bool zFail = careZ &&
                            ((eVz < cond.z.vel - cond.z.tolerance - inertiaErr) && (tVz < cond.z.vel - cond.z.tolerance)) ||
                            ((eVz > cond.z.vel + cond.z.tolerance + inertiaErr) && (tVz > cond.z.vel + cond.z.tolerance));

                bool xFail = careX &&
                            ((eVx < cond.x.vel - cond.x.tolerance - inertiaErr) && (tVx < cond.x.vel - cond.x.tolerance)) ||
                            ((eVx > cond.x.vel + cond.x.tolerance + inertiaErr) && (tVx > cond.x.vel + cond.x.tolerance));

                if(zFail || xFail) continue;

                xErrRec[0] = std::max(0.0, std::abs(eVx - cond.x.vel) - cond.x.tolerance - inertiaErr);
                zErrRec[0] = std::max(0.0, std::abs(eVz - cond.z.vel) - cond.z.tolerance - inertiaErr);
            }

            int pruneR = node.airtime;
            double lastErrX = xErrRec[0];
            double lastErrZ = zErrRec[0];

            // no reverse Jump
            for (int t = 1; t <= node.airtime; t++) {

                // Final airspeed must be air
                if(baseTick == 0 && cond.endedAirborne) break;

                node.inputs.push_back(IF::input{w, a, t});

                int airDebtCache = node.airDebt;
                lerp lerpX = node.lerpX;
                lerp lerpZ = node.lerpZ;
                bool airLastCache = node.airLast;
                
                node.airDebt = std::max(0, node.airDebt - t);
                node.T = baseTick + t;

                // inputExtension does not cost depth
                bool hardPrune = dfsRecursive(depth + 1 - inputExtension, depthLimit, node, cond,  result);

                xErrRec[t] = node.lerpX.error;
                zErrRec[t] = node.lerpZ.error;

                node.T = baseTick;
                node.airDebt = airDebtCache;
                node.lerpX = lerpX;
                node.lerpZ = lerpZ;
                node.airLast = airLastCache;

                node.inputs.pop_back();

                bool zErrIncrease = careZ && (node.lerpZ.error > lastErrZ);
                bool xErrIncrease = careX && (node.lerpX.error > lastErrX);

                if(hardPrune || (endingDepth && (zErrIncrease || xErrIncrease)) ) {
                    pruneR = std::min(node.airtime, t + 1);
                    break;
                } 

                lastErrX = node.lerpX.error;
                lastErrZ = node.lerpZ.error;
            }

            for(int r = std::max(0, node.airDebt); r < pruneR; r ++){

                if(cond.endedAirborne){
                    // Last tick must be air
                    if(baseTick == 0 && r > 0) break;
                }else{
                    // Last tick must be grounded, cannot reverse jump on the ending tick.
                    if(baseTick + r == 0) continue;
                }
                
                lastErrZ = zErrRec[r];
                for (int t = r + 1; t <= node.airtime; t++) {

                    node.inputs.push_back(IF::input{w, a, t});
                    node.revJumps.push_back(baseTick + r);

                    int airDebtCache = node.airDebt;
                    lerp lerpX = node.lerpX;
                    lerp lerpZ = node.lerpZ;
                    bool airLastCache = node.airLast;

                    node.airDebt = std::max(0, node.airtime - (t - r));
                    node.T = baseTick + t;

                    bool hardPrune = dfsRecursive( depth + 1 - inputExtension, depthLimit, node, cond, result);

                    node.T = baseTick;
                    node.airDebt = airDebtCache;

                    node.lerpX = lerpX;
                    node.lerpZ = lerpZ;
                    node.airLast = airLastCache;

                    node.inputs.pop_back();
                    node.revJumps.pop_back();

                    bool zErrIncrease = careZ && (node.lerpZ.error > lastErrZ);
                    bool xErrIncrease = careX && (node.lerpX.error > lastErrX);

                    if(hardPrune || (endingDepth && (zErrIncrease || xErrIncrease))) break;

                    lastErrX = node.lerpX.error;
                    lastErrZ = node.lerpZ.error;
                }
            }

        }
    }

    return false;
}

// Output false if condition is not satisfied
// The final velocity is stored in player& p
bool IF::exeSeq(player& p, const sequence& seq, const condition& cond, double initVx, double initVz, const bool mmCheck){

    int tick = seq.T;
    int airClock = (seq.airDebt == 0)? 0 : seq.airtime - seq.airDebt;
    int rjIdx = seq.revJumps.size() - 1 - (airClock > 0);

    double zmm = cond.z.mm;
    double xmm = cond.x.mm;
    const bool zMMCheck = ( (zmm != 0) && mmCheck);
    const bool xMMCheck = ( (xmm != 0) && mmCheck);

    double zMin = 0, zMax = 0, xMin = 0, xMax = 0;
    double preZ = 0, preX = 0;
    bool prevAirborne = true;
    bool airborne;

    // return true if there is violate of mm rule
    auto mmViolation = [&](const double mm, double& minPos, double& maxPos, double& prevPos, const double curPos){

        if(!airborne){
            if(prevAirborne){
                if(prevPos > maxPos) maxPos = prevPos;
                if(prevPos < minPos) minPos = prevPos;
            }else{
                bool preVzPos = (curPos - prevPos) > 0;
                if(preVzPos && prevPos > maxPos) maxPos = prevPos;
                if(!preVzPos && curPos > maxPos) maxPos = curPos;

                if(!preVzPos && prevPos < minPos) minPos = prevPos;
                if(preVzPos && curPos < minPos) minPos = curPos;
            }
        }

        prevPos = curPos;
        return (maxPos - minPos) <= (std::abs(mm) + 0.6f);
    };

    p.resetAll();
    p.setVel(initVx, initVz);

    int n = seq.inputs.size();
    for (int i = n - 1; i >= 0; i--) {
        const input in = seq.inputs[i];
        for (int j = 0; j < in.t; j++) {

            bool jumpQ = false;
            if(rjIdx >= 0 && !seq.revJumps.empty() && (tick - seq.airtime) == seq.revJumps[rjIdx]){
                jumpQ = true;
                rjIdx --;
            }

            airborne = airClock > 0;
            bool sprintQ = (in.w == 1);
            int movementType = 2 * sprintQ + (sprintQ && jumpQ);

            p.move(in.w, in.a, airborne, movementType, 1);

            if (jumpQ) airClock = seq.airtime;

            // Update mm used
            if(mmCheck){

                if(cond.sideDev >= 0){
                    if(zMMCheck && !xMMCheck && std::abs(p.X()) > cond.sideDev) return false;
                    if(!zMMCheck && xMMCheck && std::abs(p.Z()) > cond.sideDev) return false;
                }
                
                if(zMMCheck)
                    if(mmViolation(zmm, zMin, zMax, preZ, p.Z())) return false;
                if(xMMCheck)
                    if(mmViolation(xmm, xMin, xMax, preX, p.X())) return false;

                
                prevAirborne = airborne;
            }

            if (airClock > 0) airClock--;
            tick--;
        }
    }

    // The starting position of the input is invalid
    if(mmCheck){
        if (zMMCheck && ((zmm > 0 && zMax > p.Z()) || (zmm < 0 && zMin < p.Z())) )
            return false;
        if (xMMCheck && ((xmm > 0 && xMax > p.X()) || (xmm < 0 && xMin < p.X())) )
            return false;
    }

    return true;
}

void IF::alphaBetaUpdate(player& p, sequence& seq, const bool careX, const bool careZ){

    if(seq.inputs.empty()) return;
    p.toggleInertia(false);

    auto fastSamp = [&](double initVx = 0, double initVz = 0){
        int tick = seq.T;
        int airClock = (seq.airDebt == 0)? 0 : seq.airtime - seq.airDebt;
        int rjIdx = seq.revJumps.size() - 1 - (airClock > 0);
        bool airborne = false;

        p.resetAll();
        p.setVel(initVx, initVz);

        const input lastInput = seq.inputs.back();
        const bool sprintQ = (lastInput.w == 1);

        for (int j = 0; j < lastInput.t; j++) {

            bool jumpQ = false;
            if(rjIdx >= 0 && !seq.revJumps.empty() && (tick - seq.airtime) == seq.revJumps[rjIdx]){
                jumpQ = true;
                rjIdx --;
            }

            airborne = airClock > 0;
            
            int movementType = 2 * sprintQ + (sprintQ && jumpQ);

            p.move(lastInput.w, lastInput.a, airborne, movementType, 1);

            // ignoring the movement of first tick, cuz sprintDelay makes air as first tick unreliable
            if(j == 0 && airborne) p.setVel(initVx, initVz);

            if (jumpQ) airClock = seq.airtime;

            if (airClock > 0) airClock--;
            tick--;
        }


        int n = seq.inputs.size();
        if(seq.airLast && n >= 2){
            const input prevInput = seq.inputs[n - 2];
            p.sa(prevInput.w, prevInput.a, 1);
            airborne = true;
        }

        // force ground speed format
        double vx = p.Vx(), vz = p.Vz();
        if(airborne) vx /= 0.6f, vz /= 0.6f;

        vx = seq.lerpX.alpha * vx + seq.lerpX.beta;
        vz = seq.lerpZ.alpha * vz + seq.lerpZ.beta;

        util::vec2D speedVec = {vx, vz};

        return speedVec;
    };


    util::vec2D vec0 = fastSamp(0, 0);
    util::vec2D vec1 = fastSamp(1, 1);

    if(careX){
        seq.lerpX.alpha = vec1.x - vec0.x;
        seq.lerpX.beta = vec0.x;
    }
    
    if(careZ){
        seq.lerpZ.alpha = vec1.z - vec0.z;
        seq.lerpZ.beta = vec0.z;
    }
    

    seq.airLast = seq.airDebt > 0;

    p.toggleInertia(true);

}

util::vec2D IF::estimateSpeed(sequence& seq, bool endedAirborne, double initVx, double initVz){
    if(initVz == 0 && initVx == 0 && seq.airLast){ // airlast?
        getDummy();
        input lastInput = seq.inputs.back();
        dummy.sa(lastInput.w, lastInput.a, 1);
        initVx = dummy.Vx() / 0.6f;
        initVz = dummy.Vz() / 0.6f;
    }

    double vx = seq.lerpX.alpha * initVx + seq.lerpX.beta;
    double vz = seq.lerpZ.alpha * initVz + seq.lerpZ.beta;

    if(endedAirborne) vx *= 0.6f, vz *= 0.6f;

    return util::vec2D{vx, vz};
}

util::vec2D IF::terminalToSeq(int w, int a, sequence& seq, bool endedAirborne){
    double initVz = wasdTerminalVz[3*(a+1) + (w+1)];
    double initVx = wasdTerminalVx[3*(a+1) + (w+1)];
    getDummy();
    dummy.setVel(initVx, initVz);
    if(seq.airLast){
        dummy.setPrevSprint(w == 1);
        dummy.sa(w, a, 1);
    }
    initVz = dummy.Vz(), initVx = dummy.Vx();

    double vx = seq.lerpX.alpha * initVx + seq.lerpX.beta;
    double vz = seq.lerpZ.alpha * initVz + seq.lerpZ.beta;

    if(endedAirborne) vx *= 0.6f, vz *= 0.6f;

    return util::vec2D{vx, vz};
}

std::string IF::seq2Mothball(const sequence& seq) {
    std::string desc;

    int tick = seq.T;
    int airClock = (seq.airDebt == 0)? 0 : seq.airtime - seq.airDebt;
    int rjIdx = seq.revJumps.size() - 1 - (airClock > 0);

    bool streakFromJump = false;
    int streak = 0;

    int prevW = 0;
    int prevA = 0;
    int prevGAJ = 0; // 0 ground, 1 air, 2 jump

    auto flush = [&]() {
        if (streak <= 0) return;

        const bool sprintQ = (prevW == 1);
        const std::string modifier = sprintQ ? "s" : "w";
        const bool nothingQ = (prevW == 0 && prevA == 0);

        std::string gajStr;
        if (prevGAJ == 2 || (prevGAJ == 1 && streakFromJump)) gajStr = "j";
        else if (prevGAJ == 1) gajStr = "a";

        if (nothingQ) {
            if (!desc.empty())
                desc += "st" + gajStr + "(" + std::to_string(streak) + ") ";
        } else {
            std::string Wstr = (prevW == 1) ? "w" : "s";
            std::string Astr = (prevA == 1) ? "a" : "d";
            if (prevW == 0) Wstr.clear();
            if (prevA == 0) Astr.clear();

            desc += modifier + gajStr + "." + Wstr + Astr + "(" + std::to_string(streak) + ") ";
        }
    };

    int n = seq.inputs.size();
    for (int i = n - 1; i >= 0; i--) {
        const input in = seq.inputs[i];
        for (int j = 0; j < in.t; j++) {
            bool jumpQ = false;
            if(rjIdx >= 0 && (tick - seq.airtime) == seq.revJumps[rjIdx]){
                jumpQ = true;
                rjIdx --;
            }

            
            int gaj = (airClock > 0) ? 1 : 0;
            if (jumpQ) gaj = 2;

            const bool boundary =
                (streak > 0) &&
                (in.w != prevW || in.a != prevA ||
                 (gaj != prevGAJ && !(prevGAJ == 2 && gaj == 1)));

            if (boundary) {
                flush();
                streak = 0;
                streakFromJump = jumpQ; // first tick of new streak
            }

            if (jumpQ) airClock = seq.airtime;

            streak++;
            prevW = in.w;
            prevA = in.a;
            prevGAJ = gaj;

            if (airClock > 0) airClock--;
            tick --;
        }
    }

    flush();
    return desc;
}

void IF::setRotation(double rot){ rotation = rot;}

void IF::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
    std::cout << "(speed, slow) = (" << speed << ", " << slowness << ")\n";
}

void IF::changeSettings(int maxDepth, int maxTicks){
    this->maxDepth = maxDepth;
    this->maxTicks = maxTicks;
}

void IF::riskyPrune(bool yes){
    if(yes) inertiaErr = floatErr;
    else inertiaErr = 3e-3;
}

void IF::printSettings(){
    std::cout << "Input Finder Settings: \n";
    std::cout << "maxDepth = " << maxDepth << ", maxTicks = " << maxTicks << "\n";
}

player& IF::getDummy(){
    dummy.resetAll();
    dummy.setEffect(speed, slowness);
    dummy.setF(rotation);
    return dummy;
}