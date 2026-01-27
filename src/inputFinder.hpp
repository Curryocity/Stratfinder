#pragma once
#include <cmath>
#include <vector>
#include <string>
#include "player.hpp"

class inputFinder {

    // IMPORTANT: I ASSUMED TOGGLESPRINT FOR EFFICIENCY
    public:

    struct input {
        int w = 0; // -1, 0, 1
        int a = 0; // -1, 0, 1
        int t = 1;
    };

    struct sequence{
        std::vector<input> inputs;
        std::vector<bool> jumpMap;
        int airtime;
    };

    void listAllInputs(double targetVz, int airtime = 12, double error = 1e-4, double maxFw = 0, double maxBw = -INFINITY);

    // Depth means how many inputs to try before
    std::vector<sequence> matchZSpeed(double targetVz, int airtime = 12, double error = 1e-4, double maxFw = 0, double maxBw = -INFINITY);

    std::vector<sequence> inputDfs(double targetVz, int airtime, double error, double maxFw, double maxBw, int depthLimit);
    void inputDfsRec(std::vector<input>& inputChain, int depth, int depthLimit, double targetVz, int airtime, double error, double maxFw, double maxBw, std::vector<sequence>& result);
    void dfsJump( int nextTick, int T, int airtime, std::vector<int>& jumps, const std::vector<input>& inputChain, double targetVz, double error, double maxFw, double maxBw, std::vector<sequence>& result);

    // Output the velocity after executing the sequence, if the used mm exceed maxFw. maxBw then output NaN.
    double exeSeq(player p, sequence& seq, double maxFw = 0, double maxBw = -INFINITY);

    std::string seqToString(sequence& seq);

    void setRotation(double rot);

    private:

    int maxInputLength = 20;
    int maxDepth = 3;
    float rotation = 0.0f;

};