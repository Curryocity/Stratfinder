#pragma once

/*
 * ============================================================
 *  Movement grammar (compile-time API generation)
 *
 *  Naming rule:
 *    [s | w | sn] [a | j]? [45]?
 *
 *    s   = sprint
 *    w   = walk
 *    sn  = sneak
 *    a   = air
 *    j   = jump
 *    45  = 45° strafe
 *
 *  Special rule:
 *    - sprintjump does NOT allow 45° on the jump tick
 *    - walkjump / sneakjump DO allow 45° on the jump tick
 *
 *  These macros generate the full movement API automatically.
 * ============================================================
 */

/* [s | w | sn] */
#define TYPES(X) \
    X(s,  SPRINT, SPRINTJUMP, NO45) \
    X(w,  WALK,   WALK,       YES45) \
    X(sn, SNEAK,  SNEAK,      YES45)

/* [45?] */
#define STRAFES(X, name, mode, jumpMode, jumptick45) \
    X(name, mode, jumpMode,      ,  NO45, jumptick45) \
    X(name, mode, jumpMode, 45, YES45, jumptick45)

/* ground / air */
#define GEN_GA(name, mode, jumpMode, suf, has45, jumptick45) \
    inline void name##suf(float o, int t) { \
        simpleMove(o, GROUND, mode, has45, t); \
    } \
    inline void name##a##suf(float o, int t) { \
        simpleMove(o, AIR, mode, has45, t); \
    }

/* jump */
#define GEN_J(name, mode, jumpMode, suf, has45, jumptick45) \
    inline void name##j##suf(float o, int t) { \
        simpleMove(o, GROUND, jumpMode, jumptick45, 1); \
        if (t > 1) \
            simpleMove(o, AIR, mode, has45, t - 1); \
    }

/* full grammar */
#define GEN_GAJ(name, mode, jumpMode, jumptick45) \
    STRAFES(GEN_GA, name, mode, jumpMode, jumptick45) \
    STRAFES(GEN_J,  name, mode, jumpMode, jumptick45)

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

    /* flags */
    static constexpr int NO45 = 0;
    static constexpr int YES45 = 1;
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

    /*
     * Core movement primitive.
     * Everything funnels through this.
     *
     * moveVec : forward/backward (+ modifiers)
     * airborne    : ground or air tick
     * movementType: WALK / SPRINT / SNEAK / SPRINTJUMP
     * has45       : 45° strafe flag
     * repeat      : tick count
     */
    void simpleMove(float moveVec,
                    bool airborne,
                    int movementType,
                    bool has45,
                    int repeat);

    /* ===== auto-generated movement function ===== */
    TYPES(GEN_GAJ)

    void st(int duration);
    void stj(int duration);
    void sta(int duration);

    void chained_sj(int airtime, int repeat);
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
