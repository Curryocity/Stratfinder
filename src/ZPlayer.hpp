#pragma once

class ZPlayer {
public:

    struct State
    {
        double z = 0.0;
        double vz = 0.0;

        float prev_slip = -1;
        bool prev_sprint = false;
    };
    
    static constexpr float FORWARD  =  1.0f;
    static constexpr float BACKWARD = -1.0f;
    static constexpr float STOP     =  0.0f;

    static constexpr int GROUND = 0;
    static constexpr int AIR = 1;

    static constexpr float GROUND_SLIP = 0.6f;
    static constexpr double DEFAULT_INERTIA = 0.005;

    /* trig */
    static float sin45;
    static float cos45;

    /* MUST be called once before simulation */
    static void init();

    ZPlayer(int speed = 0, int slowness = 0);

    void simpleMove(float moveVec, bool airborne, bool sprintJumpQ,  int repeat);

    void sj45(float moveVec, int duration);
    void sa45(float moveVec, int duration);
    void s45(float moveVec, int duration);

    void sj45(int duration);
    void sa45(int duration);
    void s45(int duration);

    void chained_sj45(int airtime, int repeat);

    double Z();
    double Vz();

    void setZ(double z);
    void setVz(double vz);
    void setVzAir(double vz);

    void saveState();
    State getState();
    void loadState();
    void loadState(const State& s);

    void toggleInertia(bool on);
    void forceInertiaNext();
    void sprintDelay(bool delayQ);
    void setEffect(int speed, int slowness);
    void clearEffects();

    void resetAll();
    void resetClock();
    int lastInertia();
    bool hitVelNeg();

private:

    double z = 0.0;
    double vz = 0.0;

    float prev_slip = -1;
    bool prev_sprint = false;

    bool inertia_on = false; 
    bool forceInertia = false; // If true, next vz update will apply inertia, then toggle off
    bool sprint_delay = true; 

    int clock = 0;  // increases every tick, used for inertia timing
    int last_inertia = -1; // the clock value of last inertia trigger
    bool hit_vel_neg = false; // in the last inertia hit, was vz < 0?

    int speed = 0;
    int slowness = 0;

    State savestate;
};
