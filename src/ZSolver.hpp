#pragma once
#include <string>
#include "ZPlayer.hpp"

class ZSolver{
    public:

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

    struct halfStrat{
        int stratType;
        double optimalSpeed;
    };

    struct fullStrat{
        int delayStrat;
        double delaySpeed;
        int nondelayStrat;
        double nondelaySpeed;
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

    fullStrat optimalSolve(double mm, int t);
    halfStrat optimalDelayed(double mm, int t);

    CoreCtx solverCore(ZPlayer& p, double mm, int t, bool delayQ, double knownBwCap);
    bool earlyPrune(const CoreCtx& c, halfStrat& out);
    double delayedPendulum(ZPlayer& p, double mm, int t, int jumps);
    double nondelayedPendulum(ZPlayer& p, double mm, int t, int jumps, double maxBwSpeed);

    Output1 mmHeuristics(ZPlayer& p, double mm, int t, bool delayQ, double knownBestBwSpeed);
    Output2 slingShot   (ZPlayer& p, double mm, int t, bool delayQ, Output1& o1);
    Output3 robo        (ZPlayer& p, double mm, int t, bool delayQ, int jumps);
    Output4 boomerang   (ZPlayer& p, double mm, int t, bool delayQ, Output1& o1);

    fullStrat backwallSolve(double mm, int t);
    halfStrat backwallSolve(double mm, int t, bool delayQ);

    static std::string strat2string(int stratType);
    void printLog();
    void clearLog();
    bool poss(double mm, int t_mm, int maxt, double threshold, std::string& content);

    private:
    std::string log;
};