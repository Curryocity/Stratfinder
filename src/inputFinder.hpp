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

    struct sequence{
        std::vector<input> inputs;
        // Reverse jump: time travel airtime ticks in the past to jump so you are landing this tick
        std::vector<int> revJumps;
        int airDebt = 0;
        int airtime;
        int T = 0;
        double finalVz = 0;
    };

    struct zCond{
        double targetVz = 0;
        double error = 1e-4;
        double mm = -65536;
        bool allowStrafe = true;
    };

    static zCond genZCondLBUB(double lb, double ub, double mm, bool allowStrafe = true);

    // Depth means how many inputs to try before
    std::vector<sequence> matchZSpeed(zCond cond, int airtime);
    std::vector<sequence> inputDfs(zCond cond, int airtime, int depthLimit);

    bool inputDfsRec(zCond cond, int depth, int depthLimit, sequence& node, std::vector<sequence>& result);

    // Output the velocity after executing the sequence, if the used mm exceed maxFw. maxBw then output NaN.
    double exeSeq(player& p, const sequence& seq, double mm, double initVz = 0);

    std::string seqToString(const sequence& seq);

    void calcInitVzLBUB();

    void setEffect(int speed, int slowness);
    void setRotation(double rot);
    void setSpeedType(bool airborne);
    void changeSettings(int maxDepth, int maxTicks = 40, double maxXdeviation = INFINITY);
    void printSettings();
    player& getDummy();

    private:

    std::string log;

    float rotation = 0.0f;
    int speed = 0;
    int slowness = 0;

    player dummy;
    double initVzUB = 0;
    double initVzLB = 0;

    int maxDepth = 3;
    int maxTicks = 40;
    bool speedAirQ = false;
    double maxXdeviation = INFINITY;
    
};