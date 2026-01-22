#include <cmath>
#include "zEngine.hpp"
#include "util.hpp"

float zEngine::sin45 = 0.0f;
float zEngine::cos45 = 0.0f;

/* Precompute sine table */
void zEngine::init() {
    sin45 = util::sin(45.01f);
    cos45 = util::cos(45.01f);
}

zEngine::zEngine(int speed, int slowness) : speed(speed), slowness(slowness){}

/*
 * Used for zSolver, sprinted 45 zAxis movement only
 */
void zEngine::simMove(float moveVec, bool airborne, bool sprintJumpQ, int repeat) {
    while (repeat--) {

        float slip = airborne ? 1.0f : GROUND_SLIP;
        z += vz;

        if (prev_slip == -1)
            prev_slip = slip;

        vz *= 0.91f * prev_slip;

        if (std::abs(vz) < DEFAULT_INERTIA && vz != 0) {
            last_inertia = clock;
            hit_vel_neg = (vz < 0);
            if(inertia_on) vz = 0;
        }

        // artificial inertia application
        if(forceInertia){
            vz = 0;
            forceInertia = false;
        }

        /* base movement(force sprinted) */
        float accel;
        if(airborne){
            accel = 0.02f;
        } else{
            accel = 0.1f;
            if (speed > 0)    accel *= 1 + 0.2f * speed;
            if (slowness > 0) accel *= 1 + (-0.15f) * slowness;
            if (accel < 0) accel = 0;
        }
        
        accel *= 1.0 + 0.3f;  // sprinting multiplier

        /* ground drag */
        if (!airborne) {
            float drag = 0.91f * slip;
            accel *= 0.16277136f / (drag * drag * drag);
        }

        /* sprint-jump boost */
        if (sprintJumpQ)
            vz += moveVec * 0.2f;

        /* replicate "forward" & "strafe" in the code */
        float mu = 0.98f;

        /* normalize, no 45 on sprintjump tick */
        float dist = sprintJumpQ ? mu : std::sqrtf(mu * mu + mu * mu);
        if (dist > 1.0f)
            accel /= dist;

        mu *= accel;

        /* apply motion */
        vz += moveVec * (sprintJumpQ ? mu : (mu * cos45 + mu * sin45));

        prev_slip = slip;
        clock ++;
    }
}

double zEngine::Z(){
    return z;
}

double zEngine::Vz(){
    return vz;
}

void zEngine::setZ(double value){
    z = value;
}

void zEngine::setVz(double value){
    vz = value;
}

void zEngine::setVzAir(double value){
    vz = value;
    prev_slip = 1.0f;
}

void zEngine::resetAll()
{
    z = 0;
    vz = 0;
    prev_slip = -1;
    resetClock();
}

void zEngine::resetClock(){
    clock = 0;
    last_inertia = -1;
    forceInertia = false;
}

int zEngine::lastInertia(){
    return last_inertia;
}

bool zEngine::hitVelNeg(){
    return hit_vel_neg;
}

void zEngine::saveState(){
    savestate.z = z;
    savestate.vz = vz;
    savestate.prev_slip = prev_slip;
}

zEngine::State zEngine::getState(){
    return { z, vz, prev_slip };
}

void zEngine::loadState(const zEngine::State& s) {
    z = s.z;
    vz = s.vz;
    prev_slip = s.prev_slip;
}

void zEngine::toggleInertia(bool on){
    inertia_on = on;
}

void zEngine::forceInertiaNext(){
    forceInertia = true;
}


void zEngine::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
}

void zEngine::clearEffects(){
    speed = 0;
    slowness = 0;
}

void zEngine::loadState(){
    z = savestate.z;
    vz = savestate.vz;
    prev_slip = savestate.prev_slip;
}

void zEngine::sj45(float moveVec, int duration){
    simMove(moveVec, GROUND, true, 1);
    simMove(moveVec, AIR, false, duration - 1);
}

void zEngine::sa45(float moveVec, int duration){
    simMove(moveVec, AIR, false, duration);
}

void zEngine::s45(float moveVec, int duration){
    simMove(moveVec, GROUND, false, duration);
}

void zEngine::sj45(int duration){ sj45(FORWARD, duration);}
void zEngine::sa45(int duration){ sa45(FORWARD, duration);}
void zEngine::s45 (int duration){  s45(FORWARD, duration);}

void zEngine::chained_sj45(int airtime, int repeat){
    while (repeat--) 
        sj45(airtime);
}

