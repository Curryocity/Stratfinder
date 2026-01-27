#include "inputFinder.hpp"
#include <cmath>
#include <iostream>
#include <semaphore>
#include <string>
#include <vector>

using IF = inputFinder;

void IF::listAllInputs(double targetVz, int airtime, double error, double maxFw, double maxBw){
    std::cout
        << "TargetVz = " << targetVz
        << ", airtime = " << airtime
        << ", error = " << error
        << ", maxFw = " << maxFw
        << ", maxBw = " << maxBw
        << "\n";

    std::vector<IF::sequence> results =
        matchZSpeed(targetVz, airtime, error, maxFw, maxBw);

    std::cout << "Found " << results.size() << " sequences\n\n";

    int idx = 0;
    for (const IF::sequence& seq : results) {
        std::string inputStr = seqToString(const_cast<IF::sequence&>(seq));
        std::cout << "[" << idx++ << "] " << inputStr << "\n";
    }
}

std::vector<IF::sequence> IF::matchZSpeed(double targetVz, int airtime, double error, double maxFw, double maxBw){

    std::vector<IF::sequence> result;

    // find input sequence via iterative deepening dfs
    for(int limit = 1; limit <= maxDepth; limit ++){
        std::vector<IF::sequence> partialResult = inputDfs(targetVz, airtime, error, maxFw, maxBw, limit);
        result.reserve(result.size() + partialResult.size());
        result.insert(result.end(), partialResult.begin(), partialResult.end());
    }

    return result;

}

std::vector<IF::sequence> IF::inputDfs(double targetVz, int airtime, double error, double maxFw, double maxBw, int depthLimit){
    std::cout << "Try searching depth = " << depthLimit << " inputs\n";
    std::vector<IF::sequence> result;
    std::vector<IF::input> inputChain;

    inputDfsRec(inputChain, 0, depthLimit, targetVz, airtime, error, maxFw, maxBw,result);

    return result;
}

void IF::inputDfsRec(std::vector<IF::input>& inputChain, int depth, int depthLimit, double targetVz, int airtime, double error, double maxFw, double maxBw, std::vector<IF::sequence>& result) {

    if (depth == depthLimit) {
        // inputChain is fixed → now try jump timings

        int T = 0;
        for(IF::input input : inputChain){
            T += input.t;
        }

        // Including jumps that happens before the inputs
        // airtime - leftover (mod airtime)
        int padding = (airtime - (T - 1)%airtime) % airtime;
        IF::input blank = IF::input{0, 0, padding};
        inputChain.push_back(blank);
        T += padding;

        std::reverse(inputChain.begin(), inputChain.end());

        std::vector<int> jumps;
        dfsJump(0, T, airtime, jumps, inputChain, targetVz, error, maxFw, maxBw,result);

        std::reverse(inputChain.begin(), inputChain.end());
        inputChain.pop_back();
        
        return;
    }

    bool straight = rotation == 0.0f || rotation == 180.0f;
    int prevW = 69, prevA = 69;
    if(!inputChain.empty()){
        prevW = inputChain.back().w;
        prevA = inputChain.back().a;
    }

    for (int w = -1; w <= 1; w++) {
        for (int a = straight ? 0: -1; a <= 1; a++) { // utilize A/D symmetry when facing straight
            for (int t = 1; t <= maxInputLength; t++) {

                // should've change t from prev input instead of repeating the same movement key
                if(w == prevW && a == prevA) break;
                // the last input cannot be blank
                if((depth == depthLimit - 1) && w == 0 && a == 0) break;
                // pressing A/D without W/S is the same as stopping
                if(straight && w == 0 && a != 0) break;

                inputChain.push_back(IF::input{w, a, t});
                inputDfsRec(inputChain, depth + 1, depthLimit, targetVz, airtime, error, maxFw, maxBw,result);

                inputChain.pop_back(); 
            }
        }
    }
}

// the last tick has to be grounded, and each consectutive jumps has to be at least airtime ticks apart
// search for all combinations
void IF::dfsJump(int nextTick, int T, int airtime, std::vector<int>& jumps, const std::vector<IF::input>& inputChain, double targetVz, double error, double maxFw, double maxBw, std::vector<IF::sequence>& result) {
    // evaluate current jump configuration
    {
        IF::sequence seq;
        seq.inputs = inputChain;
        seq.jumpMap.assign(T, false);
        seq.airtime = airtime;

        for (int j : jumps)
            seq.jumpMap[j] = true;

        player p;
        p.setF(rotation);
        double vz = exeSeq(p, seq, maxFw, maxBw);

        if (!std::isnan(vz) && std::abs(vz - targetVz) <= error) {
            std::cout << "found Seqeunce: " << seqToString(seq) << "\nVz: " << vz << "\n";
            result.push_back(seq);
        }
    }

    // try adding another jump, must be at least grounded 1t at the end
    for (int j = nextTick; j < T - airtime; j++) {
        jumps.push_back(j);
        dfsJump(j + airtime, T, airtime,jumps, inputChain, targetVz, error, maxFw, maxBw,result);
        jumps.pop_back(); // backtrack
    }
}

double IF::exeSeq(player p, IF::sequence& seq, double maxFw, double maxBw){

    int tick = 0;
    int airClock = 0;
    for(IF::input input: seq.inputs){
        for(int i = 0; i < input.t; i++){
            // ToggleSprint: assume sprinting when holding W
            bool sprintQ = (input.w == 1);
            bool jumpQ = seq.jumpMap[tick];
            int movementType = 2 * sprintQ + (sprintQ && jumpQ);
            if(airClock > 0) airClock --;
            p.move(input.w, input.a, airClock > 0, movementType, 1);
            if(jumpQ) airClock = seq.airtime;
            if(p.Z() > maxFw || p.Z() < maxBw) return NAN;
            tick ++;
        }
    }

    return p.Vz();
}

std::string IF::seqToString(IF::sequence& seq){

    std::string desc;

    int tick = 0;
    int airClock = 0;

    seq.inputs.push_back({6,7,1});
    seq.jumpMap.push_back(false);

    bool streakFromJump = seq.jumpMap[0];
    int streak = 0;
    int prevW = 0;
    int prevA = 0;
    int prevGAJ = 0; // Grounded = 0, Airborne = 1, Jump = 2

    for(IF::input input: seq.inputs){
        for(int i = 0; i < input.t; i++){
            // ToggleSprint: assume sprinting when holding W
           
            bool jumpQ = seq.jumpMap[tick++];
            
            if(airClock > 0) airClock --;
            int gaj = airClock > 0;
            if(jumpQ) gaj = 2;

            // allow the streak transition from jump to airborne
            if( (streak > 0) && (input.w != prevW || input.a != prevA || (gaj != prevGAJ && !(prevGAJ == 2 || gaj == 1))) ){ 
                bool sprintQ = (prevW == 1);
                std::string modifier = sprintQ ? "s" : "w";
                bool nothingQ = prevW == 0 && prevA == 0;
                std::string gajStr = "";

                if(prevGAJ == 2 || (prevGAJ == 1 && streakFromJump)) gajStr = "j";
                else if(prevGAJ == 1) gajStr = "a";

                if(nothingQ){
                    if(!desc.empty())
                        desc += "st" + gajStr + "(" + std::to_string(streak) + ") ";
                }else{
                    std::string Wstr = (prevW == 1)? "w" : "s";
                    std::string Astr = (prevA == 1)? "a" : "d";
                    if(prevW == 0) Wstr = "";
                    if(prevA == 0) Astr = "";

                    desc += modifier + gajStr + "." + Wstr + Astr + "(" + std::to_string(streak) + ") ";
                }

                streak = 0;
                streakFromJump = jumpQ;
            }

            if(jumpQ) airClock = seq.airtime;

            streak ++;
            prevW = input.w;
            prevA = input.a;
            prevGAJ = gaj;
        }
    }

    seq.inputs.pop_back();
    seq.jumpMap.pop_back();

    return desc;

}

void IF::setRotation(double rot){ rotation = rot;}