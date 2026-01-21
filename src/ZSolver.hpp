#pragma once
#include <optional>
#include <string>
#include "ZPlayer.hpp"

class ZSolver{
    public:

    static constexpr int NONDELAYED = 0;
    static constexpr int DELAYED = 1;

    static constexpr int SLINGSHOT = 0; // Bw speed into sj45
    static constexpr int TRUE_ROBO = 1; 
    static constexpr int ROBO = 2;
    static constexpr int BOOMERANG = 3; // Fw airspeed into sj45
    static constexpr int PENDULUM = 4; // Chained loops
    // backwall strat types
    static constexpr int ANGLED_JT = 5;
    static constexpr int PESSI = 6;
    static constexpr int A7RUN = 7;
    static constexpr int RUN = 8;


    static constexpr double groundInertia = 0.005/0.6/0.91;
    static constexpr double airInertia = 0.005/0.91;

    struct strat{
        int stratType;
        double optimalSpeed;
    };

    struct fullStrat{
        int delayStrat;
        double delaySpeed;
        int nondelayStrat;
        double nondelaySpeed;
        int delayTick;
    };

    struct Output1{
        int jumps;
        double overJamDis;
        double jamDis;
        double bestBwSpeed;
        double bwmmDis;

    };

    struct Output2{
        double reqBwSpeed;
        double slingSpeed;
        bool possSling;
    };

    struct Output3{
        bool trueRoboQ;
        double roboSpeed;
    };

    struct Output4{
        double bwSpeedBoom;
        double boomSpeed;
        bool possBoom;
    };

    struct CoreCtx {
        Output1 o1;
        Output2 o2;
        Output3 o3;
        Output4 o4;
    };


    static void init();

    fullStrat optimalSolver(double mm, int t);
    strat optimalDelayed(double mm, int t, int delayTick = DELAYED);

    CoreCtx solverCore(ZPlayer& p, double mm, int t, int delayTick, double knownBwCap);
    bool earlyPrune(const CoreCtx& c, strat& out);
    double delayedPendulum(ZPlayer& p, double mm, int t, int jumps, int delayTick);
    double nondelayedPendulum(ZPlayer& p, double mm, int t, int jumps, double maxBwSpeed);

    Output1 mmHeuristics(ZPlayer& p, double mm, int t, int delayTick, double knownBestBwSpeed);
    Output2 slingShot   (ZPlayer& p, double mm, int t, int delayTick, Output1& o1);
    Output3 robo        (ZPlayer& p, double mm, int t, int delayTick, int jumps);
    Output4 boomerang   (ZPlayer& p, double mm, int t, int delayTick, Output1& o1);

    fullStrat backwallSolver(double mm, int t);
    strat backwallSolve(double mm, int t, int delayTick);
    static std::string strat2string(int stratType);
    void printLog();
    void clearLog();
    bool poss(double mm, int t_mm, int maxt, double threshold, bool backwallQ, std::string& content, double shift = normal, std::optional<fullStrat> provideStrat = std::nullopt);
    static constexpr double blockage = 0;
    static constexpr double ladder = 0.30000001192092896;
    static constexpr double normal = 0.6000000238418579;
    std::string fmt(double x);
    std::string df(double x, int precision = 16);

    void setEffect(int speed, int slowness);
    void clearEffects();


    private:
    int speed = 0;
    int slowness = 0;
    std::string log;
};