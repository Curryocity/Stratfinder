#pragma once
#include <cmath>
#include <vector>
#include <string>
#include "player.hpp"

class zInputFinder {

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
        double alpha = 1;
        double beta = 0;
        bool airLast = false;
        double error = 65536;
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

    void alphaBetaUpdate(player& p, sequence& seq);
    double estimateSpeed(sequence& seq, double initVz = 0);
    double terminalToSeq(int w, int a, sequence& seq);

    std::string seqToString(const sequence& seq);

    void initHeuristics(int airtime, double distance);

    void setEffect(int speed, int slowness);
    void setRotation(double rot);
    void setSpeedType(bool airborne);
    void changeSettings(int maxDepth, int maxTicks = 40, double maxXdeviation = INFINITY);
    void dontCareInertia(bool yes); // It is a lot faster if you don't care inertia
    void printSettings();
    player& getDummy();

    private:

    // player setting
    player dummy;
    float rotation = 0.0f;
    int speed = 0;
    int slowness = 0;

    // heuristics/pruning stuffs
    double initVzUB = 0;
    double initVzLB = 0;
    std::vector<double> errorRecorder;
    std::vector<double> wasdTerminalVel =  std::vector<double>(9); // index: 3*(a+1) + (w+1)

    // engine settings
    int maxDepth = 3;
    int maxTicks = 40;
    bool speedAirQ = false;
    double maxXdeviation = INFINITY;

    // constants
    const double float_Error = 1e-6;
    double inertia_Error = 3e-3;
    
};