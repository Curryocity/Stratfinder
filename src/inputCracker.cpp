#include "inputCracker.hpp"
#include "segLerp.hpp"
#include "util.hpp"
#include "zEngine.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using IC = inputCracker;

namespace {

std::string trimWhitespace(std::string text) {
    const std::size_t first = text.find_first_not_of(" \t\n\r\f\v");
    if (first == std::string::npos) return "";
    const std::size_t last = text.find_last_not_of(" \t\n\r\f\v");
    return text.substr(first, last - first + 1);
}

bool cancelRequested(const std::atomic_bool* flag) {
    return flag != nullptr && flag->load(std::memory_order_relaxed);
}

bool isKeyAllowed(int w, int a, const IC::WASD& rule){
    return !((!rule.w && w == 1) || (!rule.a && a == 1) || (!rule.s && w == -1) || (!rule.d && a == -1));
}

bool inputEmpty(const IC::input& in){
    return in.w == 0 && in.a == 0;
}

bool startWithEmpty(const IC::sequence& seq){
    if(seq.inputs.empty()) throw std::runtime_error{"Expected non empty inputs vector"};
    return inputEmpty(seq.inputs.back());
}

bool equalWASD(const IC::WASD& lhs, const IC::WASD& rhs){
    return (lhs.w == rhs.w) && (lhs.a == rhs.a) && (lhs.s == rhs.s) && (lhs.d == rhs.d);
}

IC::WASD toWASD(const IC::input& in){
    return {in.w == 1, in.a == 1, in.w == -1, in.a == -1};
}

IC::WASD overlap(const IC::WASD& lhs, const IC::WASD& rhs) {
    return {(lhs.w && rhs.w ), (lhs.a && rhs.a ), (lhs.s && rhs.s), (lhs.d && rhs.d)};
}

// Refund when the middle bridge state equals the overlap of the surrounding inputs,
// or when the sequence starts with an empty bridge before the first real input.
bool transitionRefund(const IC::sequence& seq, int maxTransTick, bool generalBridgeQ){
    const int n = seq.inputs.size();
    if(maxTransTick == 0 || n <= 1 || startWithEmpty(seq)) return false;
    IC::WASD cur = toWASD(seq.inputs[n-1]);
    IC::WASD bridge = toWASD(seq.inputs[n-2]);

    // non general transition: only accept "stop" as valid bridge
    if(!generalBridgeQ && !inputEmpty(seq.inputs[n-2])) return false;
    int transitionTime = seq.inputs[n-2].t;

    for(int i = n - 3; i >= 0; i--){
        IC::WASD prev = toWASD(seq.inputs[i]);
        if(!equalWASD(bridge, prev))
            return equalWASD(overlap(prev, cur), bridge) && (maxTransTick < 0 || transitionTime <= maxTransTick);
        transitionTime += seq.inputs[i].t;
    }

    bool bridgeEmpty = inputEmpty(seq.inputs[n-2]);
    return bridgeEmpty;
}

double normalizeDeg(double angle) {
    double norm = std::fmod(angle, 360.0);
    if (norm < 0) norm += 360.0;
    return norm;
}

double shortArcSpan(double angle1, double angle2) {
    const double a1 = normalizeDeg(angle1);
    const double a2 = normalizeDeg(angle2);
    const double cw = (a2 >= a1) ? (a2 - a1) : (a2 - a1 + 360.0);
    const double ccw = 360.0 - cw;
    return std::min(cw, ccw);
}

bool inArcSpan(double angle, double angle1, double angle2) {
    const double a = normalizeDeg(angle);
    const double start = normalizeDeg(angle1);
    const double end = normalizeDeg(angle2);
    const double cw = (end >= start) ? (end - start) : (end - start + 360.0);
    const double ccw = 360.0 - cw;

    if (cw <= ccw) {
        const double offset = (a >= start) ? (a - start) : (a - start + 360.0);
        return offset <= cw;
    }

    const double offset = (start >= a) ? (start - a) : (start - a + 360.0);
    return offset <= ccw;
}

void updatePolarBox(double angleDeg, double radius, double& xLb, double& xUb, double& zLb, double& zUb) {
    const double x = -radius * util::sin(static_cast<float>(angleDeg));
    const double z = radius * util::cos(static_cast<float>(angleDeg));
    xLb = std::min(xLb, x);
    xUb = std::max(xUb, x);
    zLb = std::min(zLb, z);
    zUb = std::max(zUb, z);
}

bool reachedResultLimit(std::size_t size, int limit) {
    return limit > 0 && size >= static_cast<std::size_t>(limit);
}

}

void IC::setCondWithBound(axisCond& cond, double bound1, double bound2){
    cond.lb = std::min(bound1, bound2);
    cond.ub = std::max(bound1, bound2);
    cond.enabled = true;
}

void IC::makeBannedList(const WASD& allowKeys, const bool careX, const bool careZ){
    std::fill(bannedCombs.begin(), bannedCombs.end(), false);

    for (int a = -1; a <= 1; ++a) {
        for (int w = -1; w <= 1; ++w) {
            const int idx = 3 * (a + 1) + (w + 1);
            bannedCombs[idx] = !isKeyAllowed(w, a, allowKeys);
        }
    }

    if(careX && careZ) return;

    dummy.toggleInertia(false);

    auto sameEffect = [&](int i, int j)-> bool{
        int w_i = i % 3 - 1, a_i = i/3 - 1;
        int w_j = j % 3 - 1, a_j = j/3 - 1;
        bool sprintI = (w_i == 1), sprintJ = (w_j == 1);
        dummy.resetAll();
        dummy.move(w_i, a_i, false, 2*sprintI, 1);
        double vx_i = dummy.Vx(), vz_i = dummy.Vz();
        dummy.resetAll();
        dummy.move(w_j, a_j, false, 2*sprintJ, 1);
        double vx_j = dummy.Vx(), vz_j = dummy.Vz();
        if(careX) return std::abs(vx_i - vx_j) < 1e-16;
        if(careZ) return std::abs(vz_i - vz_j) < 1e-16;

        return false;
    };

    for (int i = 0; i < 9; ++i) {
        if (bannedCombs[i]) continue;
        for (int j = i + 1; j < 9; ++j) {
            if (bannedCombs[j]) continue;
            if (sameEffect(i, j)) {
                const bool jIsStop = (j == 4);
                if (jIsStop) {
                    bannedCombs[i] = true;
                    break;
                } else {
                    bannedCombs[j] = true;
                }
            }
        }
    }

    dummy.toggleInertia(true);
}

// TODO: improve/revisit this
// heuristics
void IC::initHeuristics(int airtime, double zDis, double xDis){

    zEngine e(speed, slowness, ver);
    e.s45(1);
    double gTerm = e.Vz()/(1.0 - 0.6f * 0.91f);
    e.setVz(0);
    e.sa45(1);
    double aTerm = e.Vz()/(1.0 - 0.91f)/0.6f;
    const bool groundBetter = gTerm > aTerm;
    // this holds for every keystroke and axis

    syncDummy();

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

    std::array<double, 2> zVelBound = getVelUbLb(dummy, wasdTerminalVz, getVz, getZ, zDis);
    std::array<double, 2> xVelBound = getVelUbLb(dummy, wasdTerminalVx, getVx, getX, xDis);

    this->vxLB = xVelBound[0];
    this->vxUB = xVelBound[1];
    this->vzLB = zVelBound[0];
    this->vzUB = zVelBound[1];

}

std::vector<IC::Solution> IC::matchSpeed(const condition& cond, int airtime){
    searchStats = {};
    syncDummy();

    if( (!cond.x.enabled) && (!cond.z.enabled) ){
        return {};
    } 

    if (!(cond.allowKeys.w || cond.allowKeys.a || cond.allowKeys.s || cond.allowKeys.d)) {
        return {};
    }

    condition searchCond = cond;

    occupiedHashes.clear();
    minSolDepth = std::numeric_limits<int>::max();
    makeBannedList(searchCond.allowKeys, searchCond.x.enabled, searchCond.z.enabled);

    std::vector<IC::Solution> result;
    initHeuristics(airtime, std::abs(searchCond.z.mm) + 0.6f, std::abs(searchCond.x.mm) + 0.6f);
    lerpUpdater.setParameters(airtime, speed, slowness, rotation, ver);
    lerpUpdater.enableAxis(searchCond.x.enabled, searchCond.z.enabled);
    lerpUpdater.buildTransform();

    // find input sequence via iterative deepening dfs
    for(int limit = 1; limit <= maxDepth; limit ++){
        if (cancelRequested(cancelFlag)) break;
        resultBudget = (resultCap > 0)
            ? std::max(0, resultCap - static_cast<int>(result.size()))
            : resultCap;
        if (resultBudget == 0) break;

        std::vector<IC::Solution> partialResult = dfsEntry(searchCond, airtime, limit);
        result.reserve(result.size() + partialResult.size());
        for (auto& sol : partialResult) {
            result.push_back(std::move(sol));
        }
        if (!partialResult.empty()) {
            minSolDepth = std::min(minSolDepth, limit);
        }
        if (reachedResultLimit(result.size(), resultCap)) break;
    }

    return result;
}

std::vector<IC::Solution> IC::matchSpeed(const polorCond& cond, int airtime){
    const double angleSpan = shortArcSpan(cond.angle1, cond.angle2);
    if (angleSpan > 10.0) {
        return {};
    }

    condition rectCond;
    rectCond.endAirborne = cond.endAirborne;
    rectCond.allowKeys = cond.allowKeys;
    rectCond.x.enabled = true;
    rectCond.z.enabled = true;
    rectCond.x.mm = cond.xmm;
    rectCond.z.mm = cond.zmm;
    rectCond.x.walled = cond.xWalled;
    rectCond.z.walled = cond.zWalled;

    const double normLb = std::max(0.0, std::min(cond.normBound1, cond.normBound2));
    const double normUb = std::max(0.0, std::max(cond.normBound1, cond.normBound2));

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

    std::vector<Solution> rectResult = matchSpeed(rectCond, airtime);
    std::vector<Solution> result;
    result.reserve(rectResult.size());

    for (auto& seq : rectResult) {
        const double vx = seq.vx;
        const double vz = seq.vz;
        const double norm = std::sqrt(vx * vx + vz * vz);
        const double angle = std::atan2(-vx, vz) * 180.0 / util::PId;

        if (norm < normLb || norm > normUb) continue;
        if (!inArcSpan(angle, cond.angle1, cond.angle2)) continue;

        result.push_back(std::move(seq));
    }

    return result;
}

std::vector<IC::Solution> IC::dfsEntry(const condition& cond, int airtime, int depthLimit){
    std::vector<IC::Solution> result;
    sequence node(airtime);
    node.inputs.reserve(depthLimit);
    node.T = 0;

    dfsRecursive(0, depthLimit, node, cond, result);

    return result;
}

// return true for hardPrune, false for softPrune
bool IC::dfsRecursive(int depth, int depthLimit, sequence& node, const condition& cond, std::vector<Solution>& result) {
    searchStats.inputDfsRecCalls++;

    if (cancelRequested(cancelFlag)) return true;
    if (reachedResultLimit(result.size(), resultBudget)) return true;

    if(node.T > maxTicks) {
        searchStats.maxTickPrunes++;
        return true;
    }

    // Do not allow adding useless prefix on top of known solutions
    if(depth > 0){
        updateHash(node.hash, node.inputs.back());
        if(depth >= minSolDepth && hashMatchedKnownSolution(node.hash)) return true;
    }

    alphaBetaUpdate(node);

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
    
    if(lastDepth){
        if(careX) eVx = estimateVx(node, cond.endAirborne);
        if(careZ) eVz = estimateVz(node, cond.endAirborne);
    }

    if (lastDepth && !startWithEmpty(node) ) {
        node.errX = careX? std::abs(eVx - cond.x.mid()) - cond.x.tol() - approxErr : 0;
        node.errZ = careZ? std::abs(eVz - cond.z.mid()) - cond.z.tol() - approxErr : 0;
        
        if(node.errX <= 0) node.errX = 0;
        if(node.errZ <= 0) node.errZ = 0;

        if(node.errX == 0 && node.errZ == 0){

            bool valid = exeSeq(node, cond, true);
            double vx = dummy.Vx(), vz = dummy.Vz();

            bool xSat = (!careX) || (vx >= cond.x.lb && vx <= cond.x.ub);
            bool zSat = (!careZ) || (vz >= cond.z.lb && vz <= cond.z.ub);
            
            if(valid && xSat && zSat){ 
                node.finalVx = vx, node.finalVz = vz;
                result.push_back(Solution{
                    .mothball = seq2Mothball(node),
                    .depth = depth,
                    .T = node.T,
                    .airDebt = node.airDebt,
                    .vx = vx,
                    .vz = vz,
                });
                auto it = std::lower_bound(occupiedHashes.begin(), occupiedHashes.end(), node.hash);
                if (it == occupiedHashes.end() || *it != node.hash) {
                    occupiedHashes.insert(it, node.hash);
                }
                if (reachedResultLimit(result.size(), resultBudget)) return true;
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

    // Main search happens after here, this is for backtracking purposes
    const nodeShapshot baseNode{node.T, node.airDebt, node.lerp0, node.lerp1, node.errX, node.errZ, node.hash};

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

    node.inputs.push_back(IC::input{0, 0, 0});

    for (int w = -1; w <= 1; w++) {
        for (int a = -1; a <= 1; a++) { 
            if(bannedCombs[3 * (a + 1) + (w + 1)]) continue;
   
            bool inputExtension = (w == prevW) && (a == prevA);
            node.inputs.back().w = w;
            node.inputs.back().a = a;
            bool refund = inputExtension || transitionRefund(node, maxTransTick, allowNonEmptyBridge);
            
            // only refundable input is allowed at maxDepth
            if(lastDepth && !refund) continue;

            const bool emptyInput = (w == 0) && (a == 0);
            const bool zeroInfCheck = !emptyInput && (lastDepth || (penultimateDepth && !refund));

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
            const bool monoCheck = (lastDepth || (penultimateDepth && !refund)) && !emptyInput;

            double lastErrX;
            double lastErrZ;

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
                    bool hardPrune = dfsRecursive(depth + 1 - refund, depthLimit, node, cond, result);

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

                    bool hardPrune = dfsRecursive(depth + 1 - refund, depthLimit, node, cond, result);

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

                    bool hardPrune = dfsRecursive(depth + 1 - refund, depthLimit, node, cond, result);

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

void IC::updateHash(std::uint64_t& hash, const input& in){
    std::uint64_t x =
    (static_cast<std::uint64_t>(in.t) << 8) +
    (static_cast<std::uint64_t>(in.type) << 4) +
    (static_cast<std::uint64_t>(in.a + 1) << 2) +
     static_cast<std::uint64_t>(in.w + 1);

    x ^= x >> 33;
    x *= 0x9e3779b97f4a7c15ULL;
    x ^= x >> 29;

    hash = hash * HASH_BASE + x;
}

bool IC::hashMatchedKnownSolution(const std::uint64_t hash){
    return std::binary_search(occupiedHashes.begin(), occupiedHashes.end(), hash);
}

void IC::backTrack(sequence& node, const nodeShapshot& snapShot){
    node.T = snapShot.T;
    node.airDebt = snapShot.airDebt;
    node.lerp0 = snapShot.lerp0;
    node.lerp1 = snapShot.lerp1;
    node.errX = snapShot.errX;
    node.errZ = snapShot.errZ;
    node.hash = snapShot.hash;
}

// Output false if condition is not satisfied
// The final velocity is stored in dummy
bool IC::exeSeq(const sequence& seq, const condition& cond, const bool mmCheck){
    player& p = dummy;
    searchStats.exeSeqCalls++;

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
            if (cancelRequested(cancelFlag)) return false;
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
void IC::alphaBetaUpdate(sequence& seq){
    searchStats.alphaBetaUpdateCalls++;

    if(seq.inputs.empty()) return;
    const input lastInput = seq.inputs.back();

    lerpUpdater.updateLerp(seq.lerp0, seq.lerp1,lastInput.w, lastInput.a, lastInput.t, lastInput.type);
}

double IC::estimateVx(sequence& seq, bool endedAirborne, double initVx, bool prevSprint){
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

double IC::estimateVz(sequence& seq, bool endedAirborne, double initVz, bool prevSprint){
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

double IC::terminalVxToSeq(int w, int a, sequence& seq, bool endedAirborne){
    double initVx = wasdTerminalVx[3*(a+1) + (w+1)];

    return estimateVx(seq, endedAirborne, initVx, true);
}

double IC::terminalVzToSeq(int w, int a, sequence& seq, bool endedAirborne){
    double initVz = wasdTerminalVz[3*(a+1) + (w+1)];

    return estimateVz(seq, endedAirborne, initVz, true);
}

std::string IC::seq2Mothball(const sequence& seq) const {
    std::string desc;

    int n = seq.inputs.size();

    for (int i = n - 1; i >= 0; i--) {
        const input in = seq.inputs[i];

        const bool sprintQ = (in.w == 1);
        const std::string modifier = sprintQ ? "s" : "w";
        const bool nothingQ = (in.w == 0 && in.a == 0);

        std::string gajStr;
        switch (in.type){
            case segLerp::Jump:
                gajStr = "j";
                break;
            case segLerp::Air:
                gajStr = "a";
                break;
            default:
                gajStr = "";
                break;
        }

        if (nothingQ) {
            if (!desc.empty())
                desc += "st" + gajStr + "(" + std::to_string(in.t) + ") ";
        } else {
            std::string Wstr = (in.w == 1) ? "w" : "s";
            std::string Astr = (in.a == 1) ? "a" : "d";
            if (in.w == 0) Wstr.clear();
            if (in.a == 0) Astr.clear();

            desc += modifier + gajStr + "." + Wstr + Astr + "(" + std::to_string(in.t) + ") ";
        }

    }

    return trimWhitespace(std::move(desc));
}

std::string IC::showSolutions(const std::vector<Solution>& solutions, ConditionForm format) const {
    std::string out;

    for (const auto& seq : solutions) {
        out += seq.mothball;
        out += "\n";

        if (format == Cartesian) {
            out += "Vx: " + util::df(seq.vx) + ", Vz: " + util::df(seq.vz);
        } else {
            const double norm = std::sqrt(seq.vx * seq.vx + seq.vz * seq.vz);
            const double angle = std::atan2(-seq.vx, seq.vz) * 180.0 / util::PId;
            out += "Norm: " + util::df(norm) + ", Angle: " + util::df(angle);
        }

        out += ", depth = " + std::to_string(seq.depth);
        out += ", t = " + std::to_string(seq.T);
        out += "(+" + std::to_string(std::max(0, seq.airDebt)) + ")";
        out += "\n\n";
    }

    return out;
}

void IC::setRotation(double rot){ rotation = rot;}

void IC::setCancelFlag(std::atomic_bool* cancelFlag) {
    this->cancelFlag = cancelFlag;
}

void IC::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
}

void IC::setVersion(version ver){
    this->ver = ver;
    inertiaErr = verInertia(ver);
    approxErr = riskyPruneEnabled ? floatErr : inertiaErr;
    dummy.setVersion(ver);
}

void IC::changeSettings(int maxDepth, int maxTicks, int maxTransitionTime, bool generalBridgeQ, int resultCap){
    this->maxDepth = maxDepth;
    this->maxTicks = maxTicks;
    this->maxTransTick = maxTransitionTime;
    this->allowNonEmptyBridge = generalBridgeQ;
    this->resultCap = resultCap;
}

void IC::riskyPrune(bool riskQ){
    riskyPruneEnabled = riskQ;
    if(riskQ) approxErr = floatErr;
    else approxErr = inertiaErr;
}

void IC::syncDummy(){
    dummy.resetAll();
    dummy.setVersion(ver);
    dummy.setEffect(speed, slowness);
    dummy.setF(rotation);
}

IC::SearchStats IC::getSearchStats() const{
    return searchStats;
}
