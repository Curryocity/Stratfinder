#pragma once
#include <cmath>
#include <cstdint>
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

    struct ForwardSeq {
        std::vector<input> inputs;
        std::vector<uint8_t> isJump;
        int startTick = 0;
        int airtime = 12;
        double finalVz = 0;
    };

    struct sequence{
        std::vector<input> inputs;
        // Reverse jump: time travel airtime ticks in the past to jump so you are landing this tick
        std::vector<int> revJumps;
        int airDebt = 0;
        int airtime = 12;
    };

    struct zCond{
        double targetVz = 0;
        double error = 1e-4;
        double mm = -INFINITY;
    };

    // Depth means how many inputs to try before
    std::vector<ForwardSeq> matchZSpeed(zCond cond, int airtime);
    std::vector<ForwardSeq> inputDfs(zCond cond, int airtime, int depthLimit);

    bool inputDfsRec(zCond cond, int tick, int depth, int depthLimit, sequence& node, std::vector<ForwardSeq>& result);

    // Output the velocity after executing the sequence, if the used mm exceed maxFw. maxBw then output NaN.
    double exeFwSeq(player p, const ForwardSeq& seq, double mm, double initVz = 0, bool initAir = false);

    ForwardSeq buildForward(const sequence& seq);
    std::string fwSeqToString(const ForwardSeq& seq);

    double getTerminalSpeed(int w, int a, int runTick);

    void setEffect(int speed, int slowness);
    void setRotation(double rot);
    void changeSettings(int maxDepth, int maxTicks = 40);
    void printSettings();
    player getDummy();

    private:

    std::string log;

    // Settings
    float rotation = 0.0f;
    int speed = 0;
    int slowness = 0;
    player dummy;

    bool sprintOn = true;
    int maxDepth = 3;
    int maxTicks = 40;
    
};