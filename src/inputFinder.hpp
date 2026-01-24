#pragma once
#include <cmath>
#include <vector>
#include "player.hpp"

class inputFinder {

    public:

    struct input {
        int w = 0; // -1, 0, 1
        int a = 0; // -1, 0, 1
        bool sprint;
        int t = 1;
    };

    struct sequence{
        std::vector<input> inputs;
        std::vector<bool> jumpMap;
        std::vector<bool> sprintMap;
    };

    // Depth means how many inputs to try before
    std::vector<sequence> matchZSpeed(double targetVz, int airtime = 12, double error = 1e-4, double maxFw = 0, double maxBw = -INFINITY);

    // Output the velocity after executing the sequence, if the used mm exceed maxFw. maxBw then output NaN.
    double exeSeq(player p, sequence seq, int airtime, double maxFw = 0, double maxBw = -INFINITY);

    private:

    int maxInputLength = 24;
    int maxDepth = 3;

};