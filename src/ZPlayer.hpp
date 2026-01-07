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
    

    /* movement orientations */
    static constexpr float FORWARD  =  1.0f;
    static constexpr float BACKWARD = -1.0f;
    static constexpr float STOP     =  0.0f;

    /* movement types */
    static constexpr int NONE = -1;
    static constexpr int WALK = 0;
    static constexpr int SNEAK = 1;
    static constexpr int SPRINT = 2;
    static constexpr int SPRINTJUMP = 3;

    static constexpr int GROUND = 0;
    static constexpr int AIR = 1;

    static constexpr float PI = 3.14159265358979323846f;

    /* trig */
    static float SIN_TABLE[65536];
    static float sin45;
    static float cos45;

    /* MUST be called once before simulation */
    static void init();

    static inline float sin(float deg);
    static inline float cos(float deg);

    void simpleMove(float moveVec, bool airborne, bool sprintJumpQ,  int repeat);

    void sj45(float moveVec, int duration);
    void sa45(float moveVec, int duration);
    void s45(float moveVec, int duration);

    void sj45(int duration);
    void sa45(int duration);
    void s45(int duration);

    void chained_sj45(int airtime, int repeat);

    double getZ();
    double getVz();

    void setZ(double z);
    void setVz(double vz);
    void setVzAir(double vz);

    void saveState();
    State getState();
    void loadState();
    void loadState(const State& s);

    void inertia(double inertia);
    void sprintDelay(bool delayQ);

    void resetAll();

private:
    static constexpr float GROUND_SLIP = 0.6f;
    static constexpr double DEFAULT_INERTIA = 0.005;

    double z = 0.0;
    double vz = 0.0;

    float prev_slip = -1;
    bool prev_sprint = false;

    double inertia_threshold = DEFAULT_INERTIA;
    bool sprint_delay = true;

    State savestate;
};

/* macro cleanup */
#undef GEN_GAJ
#undef GEN_J
#undef GEN_GA
#undef STRAFES
#undef TYPES
