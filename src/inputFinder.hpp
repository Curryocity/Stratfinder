#pragma once
#include <vector>
#include <string>
#include "util.hpp"
#include "player.hpp"

class inputFinder {

    // IMPORTANT: I ASSUMED TOGGLESPRINT FOR EFFICIENCY
    public:

    struct input {
        int w = 0; // -1, 0, 1
        int a = 0; // -1, 0, 1
        int t = 1;
    };

    struct lerp{
        double alpha = 1;
        double beta = 0;
        double error = 65536;
    };

    struct axisCond{
        bool enabled = false;
        double vel = 0;
        double tolerance = 1e-4;
        double mm = 0;
        bool walled = false;
    };

    struct condition{
        axisCond z;
        axisCond x;
        bool endedAirborne = false;
        bool allowStrafe = true;
        double sideDev = -1; // only work on non-negative number
    };

    struct sequence{
        int airtime;
        int T = 0;
        std::vector<input> inputs;
        std::vector<int> revJumps; 
        // Reverse jump: time travel airtime ticks in the past to jump so you are landing this tick
        int airDebt = 0;
        bool airLast = false;
        lerp lerpX;
        lerp lerpZ;
        double finalVx = 0;
        double finalVz = 0;
    };

    static void setCondWithBound(axisCond& cond, double bound1, double bound2);

    void initHeuristics(int airtime, double zDis, double xDis);

    // Depth means how many inputs to try before
    std::vector<sequence> matchSpeed(const condition& cond, int airtime = 12);
    std::vector<sequence> dfsEntry(const condition& cond, int airtime, int depthLimit);

    bool dfsRecursive(int depth, int depthLimit, sequence& node, const condition& cond, std::vector<sequence>& result);

    bool exeSeq(player& p, const sequence& seq, const condition& cond, double initVx = 0, double initVz = 0, bool mmCheck = true);
    void alphaBetaUpdate(player& p, sequence& seq, const bool careX, const bool careZ);

    enum Axis{X,Z};
    util::vec2D estimateSpeed(sequence& seq, bool endedAirborne, double initVx = 0, double initVz = 0);
    util::vec2D terminalToSeq(int w, int a, sequence& seq, bool endedAirborne);

    std::string seq2Mothball(const sequence& seq);

    void setEffect(int speed = 0, int slowness = 0);
    void setRotation(double rot = 0);

    void changeSettings(int maxDepth, int maxTicks);
    void riskyPrune(bool riskIt); 
    // Faster when on, it may skip inputs that requires inertia

    void printSettings();
    player& getDummy();

    private:

    // dummy's statistics
    player dummy;
    float rotation = 0.0f;
    int speed = 0;
    int slowness = 0;

    // heuristics/pruning helper 
    double vzLB = 0, vzUB = 0;
    double vxLB = 0, vxUB = 0;
    std::vector<double> zErrRec, xErrRec;
    std::vector<double> wasdTerminalVz =  std::vector<double>(9, 0); // index: 3*(a+1) + (w+1)
    std::vector<double> wasdTerminalVx =  std::vector<double>(9, 0); 

    // engine settings
    int maxDepth = 3;
    int maxTicks = 40;

    // constants to account movement approximation error using lerp
    const double floatErr = 1e-6;
    double inertiaErr = 3e-3;
    
};