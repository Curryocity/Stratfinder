#include "inputFinder.hpp"
#include <cmath>

using IF = inputFinder;

std::vector<IF::sequence> IF::matchZSpeed(double targetVz, int airtime, double error, double maxFw, double maxBw){

    // find an input sequence -> try all possible jump/sprint timings
    int depth = 0;

    while(depth <= maxDepth){



    }

}

double IF::exeSeq(player p, IF::sequence seq, int airtime, double maxFw, double maxBw){

    int tick = 0;
    int airClock = 0;
    for(IF::input input: seq.inputs){
        for(int i = 0; i < input.t; i++){
            bool sprintQ = seq.sprintMap[tick];
            bool jumpQ = seq.jumpMap[tick];
            int movementType = 2 * sprintQ + (sprintQ && jumpQ);
            if(airClock > 0) airClock --;
            p.move(input.w, input.a, airClock > 0, movementType, 1);
            if(jumpQ) airClock = airtime;
            if(p.Z() > maxFw || p.Z() < maxBw) return NAN;
            tick ++;
        }
    }

    return p.Vz();
}