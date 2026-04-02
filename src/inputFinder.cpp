#include "inputFinder.hpp"
#include "segLerp.hpp"
#include "util.hpp"
#include "zEngine.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using IF = inputFinder;

namespace {

double normalizeDeg(double angle) {
    double norm = std::fmod(angle, 360.0);
    if (norm < 0) norm += 360.0;
    return norm;
}

double shortArcSpan(double angle1, double angle2) {
    const double a1 = normalizeDeg(angle1);
    const double a2 = normalizeDeg(angle2);
    double span = a2 - a1;
    if (span < 0) span += 360.0;
    return span;
}

bool inArcSpan(double angle, double angle1, double angle2) {
    const double a = normalizeDeg(angle);
    const double start = normalizeDeg(angle1);
    double offset = a - start;
    if (offset < 0) offset += 360.0;
    return offset <= shortArcSpan(angle1, angle2);
}

void updatePolarBox(double angleDeg, double radius, double& xLb, double& xUb, double& zLb, double& zUb) {
    const double x = -radius * util::sin(static_cast<float>(angleDeg));
    const double z = radius * util::cos(static_cast<float>(angleDeg));
    xLb = std::min(xLb, x);
    xUb = std::max(xUb, x);
    zLb = std::min(zLb, z);
    zUb = std::max(zUb, z);
}

}

void IF::setCondWithBound(axisCond& cond, double bound1, double bound2){
    cond.lb = std::min(bound1, bound2);
    cond.ub = std::max(bound1, bound2);
    cond.enabled = true;
}

// TODO: improve/revisit this
// heuristics
void IF::initHeuristics(int airtime, double zDis, double xDis){

    zEngine e(speed, slowness);
    e.s45(1);
    double gTerm = e.Vz()/(1.0 - 0.6f * 0.91f);
    e.setVz(0);
    e.sa45(1);
    double aTerm = e.Vz()/(1.0 - 0.91f)/0.6f;
    const bool groundBetter = gTerm > aTerm;
    // this holds for every keystroke and axis

    getDummy();

    auto getZ = [&](player& p){ return p.Z();};
    auto getX = [&](player& p){ return p.X();};
    auto getVz = [&](player& p){ return p.Vz();};
    auto getVx = [&](player& p){ return p.Vx();};

    auto getVelUbLb = [&](player& p, std::vector<double>& terminalRecorder, auto& getVel, auto& getPos, double dis){

        int wLB = 0, aLB = 0, wUB = 0, aUB = 0;
        double groundLb = 0, groundUb = 0;

        for (int w = -1; w <= 1; w++) {
            for (int a = -1; a <= 1; a++){
                if(w == 0 && a == 0) continue;
                p.resetAll();
                int sprint = 2 *(w == 1);
                p.move(w, a, false, sprint, 1);
                double v = getVel(p);
                
                if(v < groundLb)
                    wLB = w, aLB = a, groundLb = v;
                if(v > groundUb)
                    wUB = w, aUB = a, groundUb = v;

                if(groundBetter){
                    terminalRecorder[3*(a+1) + (w+1)] = v/(1.0 - 0.6f * 0.91f);
                }else{
                    p.setVel(0, 0);
                    p.setPrevSprint(w==1);
                    p.move(w, a, true, sprint, 1);
                    terminalRecorder[3*(a+1) + (w+1)] = getVel(p)/(1.0 - 0.91f)/0.6f;
                }
            }
        }

        // running
        groundLb /= (1.0 - 0.6f * 0.91f);
        groundUb /= (1.0 - 0.6f * 0.91f);

        // Return running terminal speed, since jumpingSpeed < airSpeed when ground is better
        if(groundBetter) return std::array<double, 2>{ groundLb, groundUb};

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

                if(std::abs(getPos(p)) > dis) jumpVel = getVel(p);

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

        double velLb = (jumpUb > groundUb)? jumpLb : groundLb;
        double velUb = (jumpUb > groundUb)? jumpUb : groundUb;

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
    searchStats = {};

    if( (!cond.x.enabled) && (!cond.z.enabled) ){
        writeLog("Exception: None of the conditions were enabled\n");
        writeLog("------EXIT------\n");
        return {};
    } 

    std::vector<IF::sequence> result;
    initHeuristics(airtime, std::abs(cond.z.mm) + 0.6f, std::abs(cond.x.mm) + 0.6f);
    lerpUpdater.setParameters(airtime, speed, slowness, rotation);
    lerpUpdater.enableAxis(cond.x.enabled, cond.z.enabled);
    lerpUpdater.buildTransform();

    // find input sequence via iterative deepening dfs
    for(int limit = 1; limit <= maxDepth; limit ++){
        std::vector<IF::sequence> partialResult = dfsEntry(cond, airtime, limit);
        result.reserve(result.size() + partialResult.size());
        for (auto& seq : partialResult) {
            result.push_back(std::move(seq));
        }
    }

    return result;
}

std::vector<IF::sequence> IF::matchSpeed(const polorCond& cond, int airtime){
    const double angleSpan = shortArcSpan(cond.angle1, cond.angle2);
    if (angleSpan > 90.0) {
        writeLog("Exception: polorCond angle span must be <= 90 degrees\n");
        return {};
    }

    condition rectCond;
    rectCond.endAirborne = cond.endAirborne;
    rectCond.allowStrafe = true;
    rectCond.x.enabled = true;
    rectCond.z.enabled = true;
    rectCond.x.mm = cond.xmm;
    rectCond.z.mm = cond.zmm;
    rectCond.x.walled = cond.xWalled;
    rectCond.z.walled = cond.zWalled;

    const double normLb = std::max(0.0, cond.normLb);
    const double normUb = std::max(normLb, cond.normUb);

    double xLb = std::numeric_limits<double>::infinity();
    double xUb = -std::numeric_limits<double>::infinity();
    double zLb = std::numeric_limits<double>::infinity();
    double zUb = -std::numeric_limits<double>::infinity();

    std::vector<double> candidateAngles = {
        cond.angle1,
        cond.angle2,
        0.0,
        90.0,
        180.0,
        270.0
    };

    for (double angle : candidateAngles) {
        if (!inArcSpan(angle, cond.angle1, cond.angle2)) continue;
        updatePolarBox(angle, normLb, xLb, xUb, zLb, zUb);
        updatePolarBox(angle, normUb, xLb, xUb, zLb, zUb);
    }

    setCondWithBound(rectCond.x, xLb, xUb);
    setCondWithBound(rectCond.z, zLb, zUb);

    std::vector<sequence> rectResult = matchSpeed(rectCond, airtime);
    std::vector<sequence> result;
    result.reserve(rectResult.size());

    for (auto& seq : rectResult) {
        const double vx = seq.finalVx;
        const double vz = seq.finalVz;
        const double norm = std::sqrt(vx * vx + vz * vz);
        const double angle = std::atan2(-vx, vz) * 180.0 / util::PId;

        if (norm < normLb || norm > normUb) continue;
        if (!inArcSpan(angle, cond.angle1, cond.angle2)) continue;

        result.push_back(std::move(seq));
    }

    return result;
}

std::vector<IF::sequence> IF::dfsEntry(const condition& cond, int airtime, int depthLimit){
    writeLog("-------------------------------------------------\n");
    writeLog("Try searching depth = " + std::to_string(depthLimit) + " inputs\n");
    std::vector<IF::sequence> result;
    sequence node(airtime);
    node.inputs.reserve(depthLimit);
    node.T = 0;

    dfsRecursive(0, depthLimit, node, cond, result);

    return result;
}

// return true for hardPrune, false for softPrune
bool IF::dfsRecursive(int depth, int depthLimit, sequence& node, const condition& cond, std::vector<sequence>& result) {
    searchStats.inputDfsRecCalls++;

    if(node.T > maxTicks) {
        searchStats.maxTickPrunes++;
        return true;
    }

    alphaBetaUpdate(getDummy(), node);

    const bool careZ = cond.z.enabled;
    const bool careX = cond.x.enabled;

    // Hardprune section on top to maximize effectiveness
    if(node.T > 0){
        if(careX) {
            double minVx = estimateVx(node, cond.endAirborne, this->vxLB, true) - floatErr;
            if (minVx > cond.x.ub) {
                searchStats.minBoundPrunes++;
                return true;
            }
        }
        if(careZ) {
            double minVz = estimateVz(node, cond.endAirborne, this->vzLB, true) - floatErr;
            if (minVz > cond.z.ub) {
                searchStats.minBoundPrunes++;
                return true;
            }
        }

        if(careX) {
            double maxVx = estimateVx(node, cond.endAirborne, this->vxUB, true) + floatErr;
            if (maxVx < cond.x.lb) {
                searchStats.maxBoundPrunes++;
                return true;
            }
        }
        if(careZ) {
            double maxVz = estimateVz(node, cond.endAirborne, this->vzUB, true) + floatErr;
            if (maxVz < cond.z.lb) {
                searchStats.maxBoundPrunes++;
                return true;
            }
        }
    }

    double eVx = 0;
    double eVz = 0;
    const bool lastDepth = (depth == depthLimit);
    if (lastDepth) {
        if(careX) eVx = estimateVx(node, cond.endAirborne);
        if(careZ) eVz = estimateVz(node, cond.endAirborne);

        node.errX = careX? std::abs(eVx - cond.x.mid()) - cond.x.tol() - approxErr : 0;
        node.errZ = careZ? std::abs(eVz - cond.z.mid()) - cond.z.tol() - approxErr : 0;
        
        if(node.errX <= 0) node.errX = 0;
        if(node.errZ <= 0) node.errZ = 0;

        if(node.errX == 0 && node.errZ == 0){

            bool valid = exeSeq(dummy, node, cond, true);
            double vx = dummy.Vx(), vz = dummy.Vz();

            bool xSat = (!careX) || (vx >= cond.x.lb && vx <= cond.x.ub);
            bool zSat = (!careZ) || (vz >= cond.z.lb && vz <= cond.z.ub);
            
            if(valid && xSat && zSat){ 
                node.finalVx = vx, node.finalVz = vz;
                std::string vxStr = careX? (", Vx: " + util::df(vx)) : "";
                std::string vzStr = careZ? (", Vz: " + util::df(vz)) : "";
                writeLog("\n");
                writeLog("Found Seqeunce: " + seq2Mothball(node) 
                + "\nt = " + std::to_string(node.T) + "(+" + std::to_string(std::max(0, node.airDebt)) + ")" + vxStr + vzStr + "\n");
                result.push_back(node);
            }
        }

    }

    int prevW = 69, prevA = 69;
    input prevInput;
    if(!node.inputs.empty()){
        prevInput = node.inputs.back();
        prevW = prevInput.w;
        prevA = prevInput.a;
    }

    // Must be after the hardPrune section
    // Early return: maxDepth must be an inputExtension, cannot input extend airLast input
    if(lastDepth && prevInput.type == segLerp::Air) return false;

    // Main search happens after here, this is for backtracking purposes
    const nodeShapshot baseNode{node.T, node.airDebt, node.lerp0, node.lerp1, node.errX, node.errZ};

    const bool symmetric = (rotation == 0.0f || rotation == 180.0f) && (!careX);
    const bool endingDepth = (depth >= depthLimit - 1);
    const bool penultimateDepth = (depth == depthLimit - 1);

    double baseErrorX, baseErrorZ;
    if(endingDepth){
        // We compute eV on lastDepth already
        if(penultimateDepth) {
            if(careX) eVx = estimateVx(node, cond.endAirborne);
            if(careZ) eVz = estimateVz(node, cond.endAirborne);
        }
        baseErrorX = careX? std::max(0.0, std::abs(eVx - cond.x.mid()) - cond.x.tol() - approxErr) : 0;
        baseErrorZ = careZ? std::max(0.0, std::abs(eVz - cond.z.mid()) - cond.z.tol() - approxErr) : 0;
    }

    node.inputs.push_back(IF::input{0, 0, 0});

    for (int w = -1; w <= 1; w++) {
        for (int a = -1; a <= 1; a++) { 

            if(!cond.allowStrafe && a != 0) continue;

            if(symmetric && (a < 0 || (w == 0 && a != 0))) continue; 
            // When symmetric:
            // A/D gives the same outcome, thus wlog ignore assume a>=0
            // Pressing either A/D does nothing when W/S is not held
   
            bool inputExtension = (w == prevW) && (a == prevA);
            
            // only inputExtension is allowed at maxDepth
            if(lastDepth && !inputExtension) continue;

            const bool emptyInput = (w == 0) && (a == 0);
            const bool zeroInfCheck = lastDepth || (penultimateDepth && !inputExtension);

            // The initial(for regular timeline) input cannot be blank
            // It will always fail the zeroInfCheck(that is the less obvious reason tho)
            if(emptyInput && zeroInfCheck) continue;

            // 0-inf interval check
            if(zeroInfCheck){
                if(careZ){
                    double tVz = terminalVzToSeq(w, a, node, cond.endAirborne);
                    if(((eVz < cond.z.lb - approxErr) && (tVz < cond.z.lb)) 
                    || ((eVz > cond.z.ub + approxErr) && (tVz > cond.z.ub))) {
                        searchStats.zeroInfPrune++;
                        continue;
                    }
                }

                if(careX){
                    double tVx = terminalVxToSeq(w, a, node, cond.endAirborne);
                    if( ((eVx < cond.x.lb - approxErr) && (tVx < cond.x.lb))
                    || ((eVx > cond.x.ub + approxErr) && (tVx > cond.x.ub))) {
                        searchStats.zeroInfPrune++;
                        continue;
                    }

                }
            }

            // monoCheck doesn't make sense when input is empty
            const bool monoCheck = (lastDepth || (penultimateDepth && !inputExtension)) && !emptyInput;

            double lastErrX;
            double lastErrZ;

            node.inputs.back().w = w;
            node.inputs.back().a = a;

            // On ground:
            if(prevInput.type != segLerp::Air){

                // 1. Walk/run n ticks
                const bool endedGroundedMask = !(baseNode.T == 0 && cond.endAirborne); // zero if invalid
                const bool extensionMask = !(inputExtension && (prevInput.type == segLerp::Ground)); // zero if invalid

                // 31 ticks max
                int maxRunTick = std::min(extensionMask * endedGroundedMask * (maxTicks - baseNode.T), 31);
                
                if(monoCheck){
                    lastErrX = baseErrorX;
                    lastErrZ = baseErrorZ;
                }

                for (int t = 1; t <= maxRunTick; t++) {
                    node.inputs.back().t = t;
                    node.airDebt = 0;
                    node.T = baseNode.T + t;
                    node.lerp0 = baseNode.lerp0;
                    node.lerp1 = baseNode.lerp1;
                    node.inputs.back().type = segLerp::Ground;

                    // inputExtension does not cost depth
                    bool hardPrune = dfsRecursive(depth + 1 - inputExtension, depthLimit, node, cond, result);

                    const double childErrX = node.errX;
                    const double childErrZ = node.errZ;
                    backTrack(node, baseNode);

                    bool monotonicPrune = false;
                    if(monoCheck){
                        const bool zErrIncrease = careZ && (childErrZ > lastErrZ);
                        const bool xErrIncrease = careX && (childErrX > lastErrX);
                        monotonicPrune = zErrIncrease || xErrIncrease;
                    }

                    if(hardPrune || monotonicPrune) {
                        if(hardPrune) searchStats.childHardPrunesGroundRun++;
                        else searchStats.monotonicPrunesGroundRun++;
                        break;
                    } 

                    lastErrX = childErrX;
                    lastErrZ = childErrZ;
                }
                
                // 2. Do revJump (at most until reverse landed)


                const bool endedAirMask = !(baseNode.T == 0 && !cond.endAirborne); // zero if invalid
                const int revLandTick = node.airtime * endedAirMask;

                if(monoCheck){
                    lastErrX = baseErrorX;
                    lastErrZ = baseErrorZ;
                }

                for (int t = 1; t <= revLandTick; t++) {

                    node.inputs.back().t = t;
                    node.airDebt = std::max(0, node.airtime - t);
                    node.T = baseNode.T + t;
                    node.lerp0 = baseNode.lerp0;
                    node.lerp1 = baseNode.lerp1;
                    node.inputs.back().type = (t == revLandTick)? segLerp::Jump : segLerp::Air;

                    bool hardPrune = dfsRecursive(depth + 1 - inputExtension, depthLimit, node, cond, result);

                    bool monotonicPrune = false;
                    const double childErrX = node.errX;
                    const double childErrZ = node.errZ;
                    
                    backTrack(node, baseNode);

                    if(monoCheck){
                        const bool zErrIncrease = careZ && (childErrZ > lastErrZ);
                        const bool xErrIncrease = careX && (childErrX > lastErrX);
                        monotonicPrune = zErrIncrease || xErrIncrease;
                    }

                    if(hardPrune || monotonicPrune){
                        if(hardPrune) searchStats.childHardPrunesGroundAir++;
                        else searchStats.monotonicPrunesGroundAir++;
                        break;
                    } 

                    lastErrX = childErrX;
                    lastErrZ = childErrZ;

                }


            }else if(!inputExtension){  // Airborne, cannot be inputExtension

                // Extend the air ticks(at most until reverse landed)

                const int revLandTick = baseNode.airDebt;

                if(monoCheck){
                    lastErrX = baseErrorX;
                    lastErrZ = baseErrorZ;
                }

                for (int t = 1; t <= revLandTick; t++) {

                    node.inputs.back().t = t;
                    node.airDebt = std::max(0, baseNode.airDebt - t);
                    node.T = baseNode.T + t;
                    node.lerp0 = baseNode.lerp0;
                    node.lerp1 = baseNode.lerp1;
                    node.inputs.back().type = (t == revLandTick)? segLerp::Jump : segLerp::Air;

                    bool hardPrune = dfsRecursive(depth + 1, depthLimit, node, cond, result);

                    bool monotonicPrune = false;
                    const double childErrX = node.errX;
                    const double childErrZ = node.errZ;
                    
                    backTrack(node, baseNode);

                    if(monoCheck){
                        const bool zErrIncrease = careZ && (childErrZ > lastErrZ);
                        const bool xErrIncrease = careX && (childErrX > lastErrX);
                        monotonicPrune = zErrIncrease || xErrIncrease;
                    }

                    if(hardPrune || monotonicPrune){
                        if(hardPrune) searchStats.childHardPrunesAirExtend++;
                        else searchStats.monotonicPrunesAirExtend++;
                        break;
                    } 

                    lastErrX = childErrX;
                    lastErrZ = childErrZ;

                }

            }
        }
    }

    node.inputs.pop_back();

    backTrack(node, baseNode);

    return false;
}

void IF::backTrack(sequence& node, const nodeShapshot& snapShot){
    node.T = snapShot.T;
    node.airDebt = snapShot.airDebt;
    node.lerp0 = snapShot.lerp0;
    node.lerp1 = snapShot.lerp1;
    node.errX = snapShot.errX;
    node.errZ = snapShot.errZ;
}

// Output false if condition is not satisfied
// The final velocity is stored in player& p
bool IF::exeSeq(player& p, const sequence& seq, const condition& cond, const bool mmCheck){
    searchStats.exeSeqCalls++;

    int tick = seq.T;
    int airClock = (seq.airDebt == 0)? 0 : seq.airtime - seq.airDebt;

    const double zmm = cond.z.mm;
    const double xmm = cond.x.mm;
    const bool zMMCheck = ( (zmm != 0) && mmCheck);
    const bool xMMCheck = ( (xmm != 0) && mmCheck);

    double zMin = 0, zMax = 0, xMin = 0, xMax = 0;
    double zWallMin = 0, zWallMax = 0, xWallMin = 0, xWallMax = 0;
    double preZ = 0, preX = 0;
    bool prevAirborne = true;
    bool airborne = airClock > 0;

    // return true if there is violate of mm rule
    auto mmViolation = [&](const double mm, double& minPos, double& maxPos, double& prevPos, const double curPos){

        if(prevAirborne){
            if(prevPos > maxPos) maxPos = prevPos;
            if(prevPos < minPos) minPos = prevPos;
        }else{
            bool preVelPositive = (curPos - prevPos) > 0;
            if(preVelPositive && prevPos > maxPos) maxPos = prevPos;
            if(!preVelPositive && curPos > maxPos) maxPos = curPos;

            if(!preVelPositive && prevPos < minPos) minPos = prevPos;
            if(preVelPositive && curPos < minPos) minPos = curPos;
        }

        return (maxPos - minPos) > (std::abs(mm) + 0.6f);
    };

    p.resetAll();

    int n = seq.inputs.size();
    for (int i = n - 1; i >= 0; i--) {
        const input in = seq.inputs[i];
        const bool sprintQ = (in.w == 1);
        for (int t = 0; t < in.t; t++) {
            bool jumpQ = (in.type == segLerp::Jump) && (t == 0);

            airborne = airClock > 0;
            int movementType = 2 * sprintQ + (sprintQ && jumpQ);

            p.move(in.w, in.a, airborne, movementType, 1);

            if (jumpQ) airClock = seq.airtime;

            if(xMMCheck){
                xWallMin = std::min(xWallMin, p.X());
                xWallMax = std::max(xWallMax, p.X());
            }
            if(zMMCheck){
                zWallMin = std::min(zWallMin, p.Z());
                zWallMax = std::max(zWallMax, p.Z());
            }
            // Update mm used when grounded
            if(mmCheck && !airborne){
                if(zMMCheck)
                    if(mmViolation(zmm, zMin, zMax, preZ, p.Z())) return false;
                if(xMMCheck)
                    if(mmViolation(xmm, xMin, xMax, preX, p.X())) return false;
            }

            preX = p.X();
            preZ = p.Z();
            prevAirborne = airborne;

            if (airClock > 0) airClock--;
            tick--;
        }
    }
    
    if(mmCheck){
        // Final mm violation check(the above check skips the last tick update)
        if(xMMCheck){
            xMax = std::max(xMax, p.X());
            xMin = std::min(xMin, p.X());
            if((xMax - xMin) > (std::abs(xmm) + 0.6f)) return false;
        }
        if(zMMCheck){
            zMax = std::max(zMax, p.Z());
            zMin = std::min(zMin, p.Z());
            if((zMax - zMin) > (std::abs(zmm) + 0.6f)) return false;
        }

        // Walled mm check
        if (xMMCheck && cond.x.walled) {
            if (xmm < 0 && xWallMax - xMin > abs(xmm)) return false;
            if (xmm > 0 && xMax - xWallMin > abs(xmm)) return false;
        }
        if (zMMCheck && cond.z.walled) {
            if (zmm < 0 && zWallMax - zMin > abs(zmm)) return false;
            if (zmm > 0 && zMax - zWallMin > abs(zmm)) return false;
        }

        // Check if the starting position is invalid
        if (zMMCheck && ((zmm > 0 && zMax > p.Z()) || (zmm < 0 && zMin < p.Z())) )
            return false;
        if (xMMCheck && ((xmm > 0 && xMax > p.X()) || (xmm < 0 && xMin < p.X())) )
            return false;
    }

    return true;
}

// v_end = alpha * v_init + beta (assuming v were ground speed)
void IF::alphaBetaUpdate(player& p, sequence& seq){
    searchStats.alphaBetaUpdateCalls++;

    if(seq.inputs.empty()) return;
    const input lastInput = seq.inputs.back();

    lerpUpdater.updateLerp(seq.lerp0, seq.lerp1,lastInput.w, lastInput.a, lastInput.t, lastInput.type);
}

double IF::estimateVx(sequence& seq, bool endedAirborne, double initVx, bool prevSprint){
    searchStats.estimateSpeedCalls++;

    double vx;
    if(!prevSprint){ 
        vx = seq.lerp0.lerpX.alpha * initVx + seq.lerp0.lerpX.beta;
    }else{
        vx = seq.lerp1.lerpX.alpha * initVx + seq.lerp1.lerpX.beta;
    }

    // get actual speed (ground format to air)
    if(endedAirborne) vx *= 0.6f;

    return vx;
}

double IF::estimateVz(sequence& seq, bool endedAirborne, double initVz, bool prevSprint){
    searchStats.estimateSpeedCalls++;

    double vz;
    if(!prevSprint){ 
        vz = seq.lerp0.lerpZ.alpha * initVz + seq.lerp0.lerpZ.beta;
    }else{
        vz = seq.lerp1.lerpZ.alpha * initVz + seq.lerp1.lerpZ.beta;
    }

    if(endedAirborne) vz *= 0.6f;

    return vz;
}

double IF::terminalVxToSeq(int w, int a, sequence& seq, bool endedAirborne){
    double initVx = wasdTerminalVx[3*(a+1) + (w+1)];

    return estimateVx(seq, endedAirborne, initVx, true);
}

double IF::terminalVzToSeq(int w, int a, sequence& seq, bool endedAirborne){
    double initVz = wasdTerminalVz[3*(a+1) + (w+1)];

    return estimateVz(seq, endedAirborne, initVz, true);
}

std::string IF::seq2Mothball(const sequence& seq) const {
    std::string desc;

    int tick = seq.T;
    int airClock = (seq.airDebt == 0)? 0 : seq.airtime - seq.airDebt;

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
        for (int t = 0; t < in.t; t++) {
            bool jumpQ = false;
            if(t == 0 && in.type == segLerp::Jump){
                jumpQ = true;
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

std::string IF::showSolutions(const std::vector<sequence>& solutions, SolutionFormat format) const {
    std::string out;

    for (const auto& seq : solutions) {
        out += seq2Mothball(seq);
        out += "\n";

        if (format == Cartesian) {
            out += "Vx: " + util::df(seq.finalVx) + ", Vz: " + util::df(seq.finalVz);
        } else {
            const double norm = std::sqrt(seq.finalVx * seq.finalVx + seq.finalVz * seq.finalVz);
            const double angle = std::atan2(-seq.finalVx, seq.finalVz) * 180.0 / util::PId;
            out += "Norm: " + util::df(norm) + ", Angle: " + util::df(angle);
        }

        out += ", t = " + std::to_string(seq.T);
        out += "(+" + std::to_string(std::max(0, seq.airDebt)) + ")";
        out += "\n\n";
    }

    return out;
}

void IF::setRotation(double rot){ rotation = rot;}

void IF::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
}

void IF::changeSettings(int maxDepth, int maxTicks){
    this->maxDepth = maxDepth;
    this->maxTicks = maxTicks;
}

void IF::riskyPrune(bool riskQ){
    if(riskQ) approxErr = floatErr;
    else approxErr = inertiaErr;
}

void IF::logSettings(){
    writeLog("Input Finder Settings: \n");
    writeLog("maxDepth = " + std::to_string(maxDepth) + ", maxTicks = " + std::to_string(maxTicks) + "\n");
    writeLog("(speed, slow) = (" + std::to_string(speed) + ", " + std::to_string(slowness) + ")\n");
}

player& IF::getDummy(){
    dummy.resetAll();
    dummy.setEffect(speed, slowness);
    dummy.setF(rotation);
    return dummy;
}

void IF::writeLog(std::string str){
    logger.write(str);
}

void IF::printLog(){
    logger.print();
}

void IF::clearLog(){
    logger.clear();
}

void IF::toggleLog(bool on){
    logger.toggle(on);
}

IF::SearchStats IF::getSearchStats() const{
    return searchStats;
}
