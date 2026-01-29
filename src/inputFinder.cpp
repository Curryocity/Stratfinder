#include "inputFinder.hpp"
#include "util.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using IF = inputFinder;

IF::zCond IF::genZCondLBUB(double lb, double ub, double mm, bool allowStrafe, double maxXdev){
    return zCond{(lb + ub)/2, std::abs((ub - lb)/2), mm, allowStrafe, maxXdev};
}

std::vector<IF::ForwardSeq> IF::matchZSpeed(zCond cond, int airtime){

    std::vector<IF::ForwardSeq> result;

    // find input sequence via iterative deepening dfs
    for(int limit = 1; limit <= maxDepth; limit ++){
        std::vector<IF::ForwardSeq> partialResult = inputDfs(cond, airtime, limit);
        result.reserve(result.size() + partialResult.size());
        result.insert(result.end(), partialResult.begin(), partialResult.end());
    }

    return result;

}

std::vector<IF::ForwardSeq> IF::inputDfs(zCond cond, int airtime, int depthLimit){
    std::cout << "-------------------------------------------------\n";
    std::cout << "Try searching depth = " << depthLimit << " inputs\n";
    std::vector<IF::ForwardSeq> result;
    sequence node;
    node.inputs = std::vector<input>();
    node.revJumps = std::vector<int>();
    node.airtime = airtime;
    

    inputDfsRec(cond, 0, 0, depthLimit, node, result);

    return result;
}

// return true for hardPrune, false for softPrune
bool IF::inputDfsRec(zCond cond, int tick, int depth, int depthLimit, sequence& node, std::vector<ForwardSeq>& result) {

    if(tick > maxTicks) return true;

    if (depth == depthLimit) {
        ForwardSeq fw = buildForward(node);

        double vz = exeFwSeq(getDummy(), fw, cond.mm, cond.maxXdeviation);

        if(!std::isnan(vz) && std::abs(vz - cond.targetVz) <= cond.error){
            fw.finalVz = vz;
            std::cout << "------------------------------------------------------------\n";
            std::cout << "Found Seqeunce: " << fwSeqToString(fw) << "\nt = " << tick << "(+" << node.airDebt << "), Vz: " << util::df(vz) << "\n";
            result.emplace_back(std::move(fw));
        }

        return false;
    }

    // prune
    if(tick > 0){
        ForwardSeq fw = buildForward(node);

        double maxInitVz = getTerminalSpeed(1, 0, 0);
        double minInitVz = getTerminalSpeed(-1, 0, 0);
        double maxVz = exeFwSeq(getDummy(), fw, INFINITY, INFINITY, maxInitVz, true);
        double minVz = exeFwSeq(getDummy(), fw, INFINITY, INFINITY, minInitVz, true);

        if(maxVz < (cond.targetVz - cond.error) || minVz > (cond.targetVz + cond.error)){
            return true;
        } 
    }
    
    int baseTick = tick;
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

            // no reverse Jump

            int pruneR = node.airtime;

            for (int t = 1; t <= node.airtime; t++) {

                node.inputs.push_back(IF::input{w, a, t});

                int airDebtCache = node.airDebt;
                
                node.airDebt = std::max(0, node.airDebt - t);
                bool hardPrune = inputDfsRec(cond, baseTick + t, depth + 1 - inputExtension, depthLimit, node, result);
                node.airDebt = airDebtCache;

                node.inputs.pop_back();


                if(hardPrune){
                    pruneR = std::min(node.airtime, t + 1);
                    break;
                } 

            }

            for(int r = std::max(0, node.airDebt); r < pruneR; r ++){
                // Last tick must be grounded, cannot reverse jump on the ending tick.
                if(baseTick + r == 0) continue;
                for (int t = r + 1; t <= node.airtime; t++) {

                    node.revJumps.push_back(baseTick + r);
                    int airDebtCache = node.airDebt;

                    node.inputs.push_back(IF::input{w, a, t});

                    node.airDebt = std::max(0, node.airtime - (t - r));
                    bool hardPrune = inputDfsRec(cond, baseTick + t, depth + 1 - inputExtension, depthLimit, node, result);
                    node.airDebt = airDebtCache;

                    node.inputs.pop_back();
                    node.revJumps.pop_back();

                    if(hardPrune) break;
                }
            }

        }
    }

    return false;
}

IF::ForwardSeq IF::buildForward(const sequence& seq){
    // Build forward inputs
    std::vector<input> inputs = seq.inputs;

    int T = 0;
    for(const input& in : inputs){
        T += in.t;
    }
    int padding = seq.airtime;

    if(padding > 0)
        inputs.push_back({0,0,padding});
    T += padding;

    std::reverse(inputs.begin(), inputs.end());

    std::vector<uint8_t> isJump(T, 0);
    for (int j : seq.revJumps){
        int idx = T - j - seq.airtime;
        if(idx >= 0 && idx < T)
            isJump[idx] = 1;
        else
            std::cout << "error!!";
    }
        

    return ForwardSeq{inputs, isJump,padding, seq.airtime};
}

double IF::exeFwSeq(player p, const ForwardSeq& seq, double mm, double maxXdev, double initVz, bool initAir){

    int tick = 0;
    int airClock = 0;

    double zMax = 0, zMin = 0;

    const bool doMMCheck = !std::isinf(mm);
    bool prevAirborne = true;
    double preZ = 0;

    p.resetAll();

    for (const input& in : seq.inputs) {
        for (int i = 0; i < in.t; i++) {

            bool jumpQ = seq.isJump[tick];

            if (airClock > 0) airClock--;

            bool airborne = airClock > 0;
            bool sprintQ = (in.w == 1);
            int movementType = 2 * sprintQ + (sprintQ && jumpQ);

            if(tick == seq.startTick){
                p.setVz(initVz, initAir);
            }

            p.move(in.w, in.a, airborne, movementType, 1);

            if (jumpQ)
                airClock = seq.airtime;

            // Update mm used
            if(doMMCheck){

                if(std::abs(p.X()) > maxXdev) return NAN;

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

            tick++;
        }
    }

    
    if (doMMCheck && (mm * p.Z() < 0))
        return NAN;

    return p.Vz();
}

std::string IF::fwSeqToString(const ForwardSeq& seq) {
    std::string desc;

    int tick = 0;
    int airClock = 0;

    bool streakFromJump = (seq.isJump.empty() ? false : (seq.isJump[0] != 0));
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

    for (const IF::input& in : seq.inputs) {
        for (int i = 0; i < in.t; i++) {
            const bool jumpQ = (tick < (int)seq.isJump.size() && seq.isJump[tick] != 0);
            tick++;

            if (airClock > 0) airClock--;
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
        }
    }

    flush();
    return desc;
}

double IF::getTerminalSpeed(int w, int a, int runTick){

    getDummy();

    if(runTick == -1){
        dummy.resetAll();
        int sprintQ = (w == 1);
        dummy.move(w, a, false, 2*sprintQ, 1);
        double groundTerminal = dummy.Vz()/(1.0 - 0.91f * 0.6f);
        return groundTerminal;
    }

    dummy.resetAll();
    int sprintQ = (w == 1);
    dummy.move(w, a, true, 2*sprintQ, 1);
    dummy.setVz(0, true);
    dummy.move(w, a, true, 2*sprintQ, 1);
    double airTerminal = dummy.Vz()/(1.0 - 0.91f);

    if(runTick == 0)
        return airTerminal;

    dummy.move(w, a, false, 2*(w == 1), runTick);

    return dummy.Vz();

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

void IF::printSettings(){
    std::cout << "Input Finder Settings: \n";
    std::cout << "maxDepth = " << maxDepth << ", maxTicks = " << maxTicks << "\n";
}

player IF::getDummy(){
    dummy.resetAll();
    dummy.setEffect(speed, slowness);
    dummy.setF(rotation);
    return dummy;
}