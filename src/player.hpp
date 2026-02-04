#pragma once

/*
 * ============================================================
 *  Movement grammar (compile-time API generation)
 *
 *  Naming rule:
 *    [s | w | sn] [a | j]? 
 *
 *    s   = sprint
 *    w   = walk
 *    sn  = sneak
 *    a   = air
 *    j   = jump
 *
 *
 *  These macros generate the full movement API automatically.
 * ============================================================
 */

#define TYPES(X) \
    X(s,  SPRINT, SPRINTJUMP) \
    X(w,  NORMAL,   NORMAL) \
    X(sn, SNEAK,  SNEAK)

#define GEN_GAJ(name, mode, jumpMode) \
    inline void name(float w, float a, int t) { \
        move(w, a, GROUND, mode, t); \
    } \
    inline void name##a(float w, float a, int t) { \
        move(w, a, AIR, mode, t); \
    }\
    inline void name##j(float w, float a, int t) { \
        move(w, a, GROUND, jumpMode, 1); \
        if (t > 1) \
            move(w, a, AIR, mode, t - 1); \
    }

class player {
public:

    struct State
    {
        double x = 0.0;
        double z = 0.0;
        double vx = 0.0;
        double vz = 0.0;

        float rotation = 0;

        float prev_slip = -1;
        bool prev_sprint = false;
    };


    static constexpr int GROUND = 0, AIR = 1;

    static constexpr float GROUND_SLIP = 0.6f;
    static constexpr double DEFAULT_INERTIA = 0.005;

    player(int speed = 0, int slowness = 0);

    //movement types
    static constexpr int NORMAL = 0;
    static constexpr int SNEAK = 1;
    static constexpr int SPRINT = 2;
    static constexpr int SPRINTJUMP = 3;

    void move(float forward, float strafe, bool airborne, int movementType, int repeat);

    /* ===== auto-generated movement function ===== */
    TYPES(GEN_GAJ)

    double F();
    double X();
    double Z();
    double Vx();
    double Vz();

    void setF(float rot = 0);
    void setX(double x = 0);
    void setZ(double z = 0);
    void setVx(double vx = 0, bool airborne = false);
    void setVz(double vz = 0, bool airborne = false);
    void setVel(double vx = 0, double vz = 0, bool airborne = false);

    void saveState();
    State getState();
    void loadState();
    void loadState(const State& s);

    void toggleInertia(bool on);
    void forceInertiaNext();
    void setPrevSprint(bool value);
    void sprintDelay(bool delayQ);
    void setEffect(int speed = 0, int slowness = 0);

    void resetAll();

private:

    double x = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vz = 0.0;

    float rotation = 0.0;

    float prev_slip = -1;
    bool prev_sprint = false;

    bool inertia_on = true; 
    bool sprint_delay = true; 

    int speed = 0;
    int slowness = 0;

    State savestate;
};
