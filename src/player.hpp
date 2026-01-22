#pragma once

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

    static constexpr int GROUND = 0;
    static constexpr int AIR = 1;

    static constexpr float GROUND_SLIP = 0.6f;
    static constexpr double DEFAULT_INERTIA = 0.005;


    player(int speed = 0, int slowness = 0);

    //movement types
    static constexpr int NORMAL = 0;
    static constexpr int SNEAK = 1;
    static constexpr int SPRINT = 2;
    static constexpr int SPRINTJUMP = 3;

    void move(float forward, float strafe, bool airborne, int movementType, int repeat);

    double F();
    double X();
    double Z();
    double Vx();
    double Vz();

    void setF(float rot);
    void setX(double x);
    void setZ(double z);
    void setVx(double vx, bool airborne);
    void setVz(double vz, bool airborne);

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
