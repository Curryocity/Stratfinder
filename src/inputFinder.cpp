#include "inputFinder.hpp"
#include "util.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using IF = inputFinder;

IF::zCond IF::genZCondLBUB(double lb, double ub, double mm, bool allowStrafe){
    return zCond{(lb + ub)/2, std::abs((ub - lb)/2), mm, allowStrafe};
}

std::vector<IF::sequence> IF::matchZSpeed(zCond cond, int airtime){

    std::vector<IF::sequence> result;
    calcInitVzLBUB(airtime, std::abs(cond.mm) + 0.6);

    // find input sequence via iterative deepening dfs
    for(int limit = 1; limit <= maxDepth; limit ++){
        std::vector<IF::sequence> partialResult = inputDfs(cond, airtime, limit);
        result.reserve(result.size() + partialResult.size());
        result.insert(result.end(), partialResult.begin(), partialResult.end());
    }

    return result;

}

std::vector<IF::sequence> IF::inputDfs(zCond cond, int airtime, int depthLimit){
    std::cout << "-------------------------------------------------\n";
    std::cout << "Try searching depth = " << depthLimit << " inputs\n";
    std::vector<IF::sequence> result;
    sequence node;
    node.inputs = std::vector<input>();
    node.revJumps = std::vector<int>();
    node.airtime = airtime;
    node.T = 0;
    
    inputDfsRec(cond, 0, depthLimit, node, result);

    return result;
}

// return true for hardPrune, false for softPrune
bool IF::inputDfsRec(zCond cond, int depth, int depthLimit, sequence& node, std::vector<sequence>& result) {

    if(node.T > maxTicks) return true;

    alphaBetaUpdate(getDummy(), node);

    if (depth == depthLimit) {

        double estimateVz = estimateSpeed(node);

        if(std::abs(estimateVz - cond.targetVz) <= cond.error + inertia_Error){
            double vz = exeSeq(getDummy(), node, cond.mm);

            if(!std::isnan(vz) && std::abs(vz - cond.targetVz) <= cond.error){
                node.finalVz = vz;
                std::cout << "\n";
                std::cout << "Found Seqeunce: " << seqToString(node) << "\nt = " << node.T << "(+" << node.airDebt << "), Vz: " << util::df(vz) << "\n";
                // std::cout << "Estimate Speed: " << util::df(estimateVz) << ", Alpha: " << util::df(node.alpha) << ", Beta: " << util::df(node.beta) << "\n";
                result.push_back(node);
            }
        }

    }

    if(node.T > 0){
        double minVz = estimateSpeed(node, this->initVzLB) - float_Error;
        if(minVz > (cond.targetVz + cond.error)) return true;

        double maxVz = estimateSpeed(node, this->initVzUB) + float_Error;
        if(maxVz < (cond.targetVz - cond.error)) return true;
    }
    
    int baseTick = node.T;
    bool straight = rotation == 0.0f || rotation == 180.0f;

    int prevW = 69, prevA = 69, prevT = 0;
    input prevInput;
    if(!node.inputs.empty()){
        prevInput = node.inputs.back();
        prevW = prevInput.w;
        prevA = prevInput.a;
        prevT = prevInput.t;
    }

    for (int w = -1; w <= 1; w++) {
        for (int a = straight ? 0: -1; a <= 1; a++) { // utilize A/D symmetry when facing straight

            if(!cond.allowStrafe && a != 0) continue;
            // allow the extension of same movement key only when reaching max airtime int previous round
            if(prevT != node.airtime && w == prevW && a == prevA) continue;
            // pressing A/D without W/S gains same Vz as holding nothing when straight
            if(straight && w == 0 && a != 0) continue;
            // the initial input cannot be blank
            if((depth == depthLimit - 1) && w == 0 && a == 0) continue;

            bool inputExtension = (prevT == node.airtime) && (w == prevW) && (a == prevA);

            // only inputExtension is allowed at maxDepth
            if(depth == depthLimit && !inputExtension) continue;

            // no reverse Jump

            int pruneR = node.airtime;

            for (int t = 1; t <= node.airtime; t++) {

                // we want final airspeed
                if(baseTick == 0 && speedAirQ) break;

                node.inputs.push_back(IF::input{w, a, t});

                int airDebtCache = node.airDebt;
                double alphaCache = node.alpha;
                double betaCache = node.beta;
                bool salCache = node.airLast;
                
                node.airDebt = std::max(0, node.airDebt - t);
                node.T = baseTick + t;
                bool hardPrune = inputDfsRec(cond, depth + 1 - inputExtension, depthLimit, node, result);

                node.T = baseTick;
                node.airDebt = airDebtCache;
                node.alpha = alphaCache;
                node.beta = betaCache;
                node.airLast = salCache;

                node.inputs.pop_back();


                if(hardPrune){
                    pruneR = std::min(node.airtime, t + 1);
                    break;
                } 

            }

            for(int r = std::max(0, node.airDebt); r < pruneR; r ++){
                // Last tick must be grounded, cannot reverse jump on the ending tick.
                if(baseTick + r == 0 && !speedAirQ) continue;
                // we want final airspeed
                if(baseTick == 0 && r > 0 && speedAirQ) break;
                
                for (int t = r + 1; t <= node.airtime; t++) {

                    node.inputs.push_back(IF::input{w, a, t});
                    node.revJumps.push_back(baseTick + r);

                    int airDebtCache = node.airDebt;
                    double alphaCache = node.alpha;
                    double betaCache = node.beta;
                    bool salCache = node.airLast;

                    node.airDebt = std::max(0, node.airtime - (t - r));
                    node.T = baseTick + t;

                    bool hardPrune = inputDfsRec(cond, depth + 1 - inputExtension, depthLimit, node, result);

                    node.T = baseTick;
                    node.airDebt = airDebtCache;
                    node.alpha = alphaCache;
                    node.beta = betaCache;
                    node.airLast = salCache;

                    node.inputs.pop_back();
                    node.revJumps.pop_back();

                    if(hardPrune) break;
                }
            }

        }
    }

    return false;
}

double IF::exeSeq(player& p, const sequence& seq, double mm, double initVz){

    int tick = seq.T;
    int airClock = (seq.airDebt == 0)? 0 : seq.airtime - seq.airDebt;
    int rjIdx = seq.revJumps.size() - 1 - (airClock > 0);

    const bool doMMCheck = !std::isinf(mm);
    double zMax = 0, zMin = 0;
    bool prevAirborne = true;
    double preZ = 0;

    p.resetAll();
    p.setVz(initVz);

    int n = seq.inputs.size();
    for (int i = n - 1; i >= 0; i--) {
        const input in = seq.inputs[i];

        for (int j = 0; j < in.t; j++) {

            bool jumpQ = false;
            if(rjIdx >= 0 && !seq.revJumps.empty() && (tick - seq.airtime) == seq.revJumps[rjIdx]){
                jumpQ = true;
                rjIdx --;
            }

            bool airborne = airClock > 0;
            bool sprintQ = (in.w == 1);
            int movementType = 2 * sprintQ + (sprintQ && jumpQ);

            p.move(in.w, in.a, airborne, movementType, 1);

            if (jumpQ) airClock = seq.airtime;

            // Update mm used
            if(doMMCheck){
                if(std::abs(p.X()) > maxXdeviation) return NAN;

                if(!airborne){
                    if(prevAirborne){
                        if(preZ > zMax) zMax = preZ;
                        if(preZ < zMin) zMin = preZ;
                    }else{
                        double curZ = p.Z();
                        bool preVzPos = curZ - preZ > 0;
                        if(preVzPos && preZ > zMax) zMax = preZ;
                        if(!preVzPos && curZ > zMax) zMax = curZ;

                        if(!preVzPos && preZ < zMin) zMin = preZ;
                        if(preVzPos && curZ < zMin) zMin = curZ;
                    }
                }

                preZ = p.Z();
                prevAirborne = airborne;

                if(zMax - zMin > std::abs(mm) + 0.6f) return NAN;
            }

            if (airClock > 0) airClock--;

            tick--;
        }
    }

    
    if (doMMCheck && (mm * p.Z() < 0))
        return NAN;

    return p.Vz();
}

void IF::alphaBetaUpdate(player& p, sequence& seq){

    if(seq.inputs.empty()) return;
    p.toggleInertia(false);

    auto fastSamp = [&](double initVz){
        int tick = seq.T;
        int airClock = (seq.airDebt == 0)? 0 : seq.airtime - seq.airDebt;
        int rjIdx = seq.revJumps.size() - 1 - (airClock > 0);
        bool airborne = false;

        p.resetAll();
        p.setVz(initVz);

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

            if(j == 0 && airborne) p.setVz(initVz);

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
        double vz = p.Vz();
        if(airborne) vz /= 0.6f;

        return seq.alpha * vz + seq.beta;
    };


    double v0 = fastSamp(0);
    double v1 = fastSamp(1);

    seq.alpha = v1 - v0;
    seq.beta = v0;

    input prevInput = seq.inputs.back();
    seq.airLast = seq.airDebt > 0;

    p.toggleInertia(true);

}

double IF::estimateSpeed(sequence& seq, double initVz){
    if(initVz == 0 && seq.airLast){
        getDummy();
        input lastInput = seq.inputs.back();
        dummy.sa(lastInput.w, lastInput.a, 1);
        initVz = dummy.Vz() / 0.6f;
    }

    return seq.alpha * initVz + seq.beta;
}

std::string IF::seqToString(const sequence& seq) {
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

// heuristics
void IF::calcInitVzLBUB(int airtime, double distance){

    getDummy();

    int wLB, aLB, wUB, aUB;
    double groundLb = 0, groundUb = 0;

    for (int w = -1; w <= 1; w++) {
        for (int a = -1; a <= 1; a++){
            if(w == 0 && a == 0) continue;
            dummy.resetAll();
            int sprint = 2 *(w == 1);
            dummy.move(w, a, false, sprint, 1);
            double vz = dummy.Vz();
            if(vz < groundLb)
                wLB = w, aLB = a, groundLb = vz;
            if(vz > groundUb)
                wUB = w, aUB = a, groundUb = vz;
        }
    }

    // running
    groundLb /= (1.0 - 0.6f * 0.91f);
    groundUb /= (1.0 - 0.6f * 0.91f);

    // jumping
    auto getJumpVel = [&](int w, int a){
        int jumps = 0;
        double vz = 0;
        dummy.resetAll();
        while (std::abs(dummy.Z()) < distance && jumps < 3) {
            dummy.move(w, a, false, 2*(w == 1), 1);
            dummy.move(w, a, true, 2*(w == 1), airtime - 1);
            vz = dummy.Vz();
            jumps ++;
        }

        double jumpVel;
        if(jumps <= 3){
            jumpVel = vz;

            // trying to bargain with a random bwmm into jump
            dummy.resetAll();
            dummy.setVz(-vz);
            while (jumps --) {
                dummy.move(w, a, false, 2*(w == 1), 1);
                dummy.move(w, a, true, 2*(w == 1), airtime - 1);
            }

            if(std::abs(dummy.Z()) > distance) jumpVel = dummy.Vz();

        } else{ // jumps = inf
            dummy.setVz(0, true);
            dummy.move(w, a, false, 2*(w == 1), 1);
            dummy.move(w, a, true, 2*(w == 1), airtime - 1);
            double v0 = dummy.Vz();

            dummy.setVz(1, true);
            dummy.move(w, a, false, 2*(w == 1), 1);
            dummy.move(w, a, true, 2*(w == 1), airtime - 1);
            double v1 = dummy.Vz();
            jumpVel = - v0/(v1 - v0 - 1.0);
        }

        // convert the speed to groundSpeed (so we can just dummy.setVz(vz, airborne = false) every time)
        return jumpVel / 0.6f;
    };

    double jumpLb = getJumpVel(wLB, aLB);
    double jumpUb = getJumpVel(wUB, aUB);

    if(jumpUb > groundUb){
        this->initVzUB = jumpUb;
        this->initVzLB = jumpLb;
    }else{
        this->initVzUB = groundUb;
        this->initVzLB = groundLb;
    }

}

void IF::setRotation(double rot){ rotation = rot;}

void IF::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
    std::cout << "(speed, slow) = (" << speed << ", " << slowness << ")\n";
}

void IF::setSpeedType(bool airborne){
    this->speedAirQ = airborne;
    std::cout << "speedAirQ set to " << airborne << "\n";
}

void IF::changeSettings(int maxDepth, int maxTicks, double maxXdeviation){
    this->maxDepth = maxDepth;
    this->maxTicks = maxTicks;
    this->maxXdeviation = maxXdeviation;
}

void IF::printSettings(){
    std::cout << "Input Finder Settings: \n";
    std::cout << "maxDepth = " << maxDepth << ", maxTicks = " << maxTicks << ", maxXdeviation = " << maxXdeviation << "\n";
}

player& IF::getDummy(){
    dummy.resetAll();
    dummy.setEffect(speed, slowness);
    dummy.setF(rotation);
    return dummy;
}