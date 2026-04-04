#pragma once
#include <atomic>
#include <cstdint>
#include <iosfwd>
#include <vector>
#include <string>
#include "segLerp.hpp"
#include "util.hpp"
#include "player.hpp"

class inputCracker {

    // IMPORTANT: I ASSUMED TOGGLESPRINT FOR EFFICIENCY
    public:

    struct WASD{
        bool w = false;
        bool a = false;
        bool s = false;
        bool d = false;
    };

    struct input {
        int w = 0; // -1, 0, 1
        int a = 0; // -1, 0, 1
        int t = 1;
        segLerp::segmentType type = segLerp::None;
    };

    struct axisCond{
        bool enabled = false;
        double lb = -1e-4;
        double ub = 1e-4;
        double mm = 0;
        bool walled = false;

        double mid() const { return (lb + ub) / 2.0; }
        double tol() const { return std::abs(ub - lb) / 2.0; }
    };

    struct polorCond{
        WASD allowKeys = {1, 1, 1, 1};
        double normLb;
        double normUb;
        double angle1;
        double angle2;
        bool endAirborne = false;
        double xmm = 0;
        double zmm = 0;
        bool xWalled = false;
        bool zWalled = false;
    };

    struct condition{
        axisCond z;
        axisCond x;
        WASD allowKeys = {1, 1, 1, 1};
        bool endAirborne = false;
    };

    struct sequence{
        sequence(int airtime_): airtime(airtime_){}
        const int airtime;
        int T = 0;
        std::vector<input> inputs;
        int airDebt = 0;
        segLerp::lerp lerp0;
        segLerp::lerp lerp1;
        double errX = 65536;
        double errZ = 65536;
        double finalVx = 0;
        double finalVz = 0;
    };

    struct nodeShapshot{
        const int T;
        const int airDebt = 0;
        const segLerp::lerp lerp0;
        const segLerp::lerp lerp1;
        const double errX;
        const double errZ;
    };

    struct SearchStats {
        std::uint64_t inputDfsRecCalls = 0;
        std::uint64_t alphaBetaUpdateCalls = 0;
        std::uint64_t estimateSpeedCalls = 0;
        std::uint64_t exeSeqCalls = 0;
        std::uint64_t maxTickPrunes = 0;
        std::uint64_t minBoundPrunes = 0;
        std::uint64_t maxBoundPrunes = 0;
        std::uint64_t zeroInfPrune = 0;
        std::uint64_t childHardPrunesGroundRun = 0;
        std::uint64_t childHardPrunesGroundAir = 0;
        std::uint64_t childHardPrunesAirExtend = 0;
        std::uint64_t monotonicPrunesGroundRun = 0;
        std::uint64_t monotonicPrunesGroundAir = 0;
        std::uint64_t monotonicPrunesAirExtend = 0;
    };

    struct Solution{
        std::string mothball;
        int depth = 0;
        int T = 0;
        int airDebt = 0;
        double vx = 0;
        double vz = 0;
    };

    static void setCondWithBound(axisCond& cond, double bound1, double bound2);

    void initHeuristics(int airtime, double zDis, double xDis);

    std::vector<Solution> matchSpeed(const condition& cond, int airtime);
    std::vector<Solution> matchSpeed(const polorCond& cond, int airtime);
    std::vector<Solution> dfsEntry(const condition& cond, int airtime, int depthLimit);

    bool dfsRecursive(int depth, int depthLimit, sequence& node, const condition& cond, std::vector<Solution>& result);

    void backTrack(sequence& node, const nodeShapshot& snapShot);

    bool exeSeq(player& p, const sequence& seq, const condition& cond, bool mmCheck = true);
    void alphaBetaUpdate(player& p, sequence& seq);

    enum Axis{X,Z};
    enum ConditionForm{Cartesian = 0, Polar = 1};
    double estimateVx(sequence& seq, bool endedAirborne, double initVx = 0, bool prevSprint = false);
    double estimateVz(sequence& seq, bool endedAirborne, double initVz = 0, bool prevSprint = false);
    double terminalVxToSeq(int w, int a, sequence& seq, bool endedAirborne);
    double terminalVzToSeq(int w, int a, sequence& seq, bool endedAirborne);

    std::string seq2Mothball(const sequence& seq) const;
    std::string showSolutions(const std::vector<Solution>& solutions, ConditionForm format = Cartesian) const;

    void setEffect(int speed = 0, int slowness = 0);
    void setRotation(double rot = 0);
    void setCancelFlag(std::atomic_bool* cancelFlag);

    void changeSettings(int maxDepth, int maxTicks, int maxTransitionTime = -1, bool generalBridgeQ = false, int maxResult = 1024);
    void riskyPrune(bool riskIt); 
    // Faster when on, it may skip inputs that requires inertia

    void logSearch(std::ostream& out, const condition& cond, int airtime) const;
    void logSearch(std::ostream& out, const polorCond& cond, int airtime) const;
    player& getDummy();

    void writeLog(std::string str);
    void printLog();
    void clearLog();
    void toggleLog(bool on);
    SearchStats getSearchStats() const;

    private:

    // dummy's statistics
    player dummy;
    float rotation = 0.0f;
    int speed = 0;
    int slowness = 0;

    // heuristics/pruning helper 
    segLerp lerpUpdater = segLerp();
    double vzLB = 0, vzUB = 0;
    double vxLB = 0, vxUB = 0;
    std::vector<double> wasdTerminalVz =  std::vector<double>(9, 0); // index: 3*(a+1) + (w+1)
    std::vector<double> wasdTerminalVx =  std::vector<double>(9, 0); 

    // engine settings
    int maxDepth = 3;
    int maxTicks = 40;
    int maxTransTick = -1;
    bool allowNonEmptyBridge = false;
    int maxResult = 1024;
    int resultBudget = maxResult;
    std::atomic_bool* cancelFlag = nullptr;

    // constants to account movement approximation error using lerp
    const double floatErr = 1e-5;
    const double inertiaErr = 0.005; // Tested value, may not be the best
    double approxErr = inertiaErr;
    SearchStats searchStats;
    Logger logger;
    
};
