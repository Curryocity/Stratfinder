#include <cmath>
#include "ZPlayer.hpp"
#include "util.hpp"

float ZPlayer::sin45 = 0.0f;
float ZPlayer::cos45 = 0.0f;

/* Precompute sine table */
void ZPlayer::init() {
    sin45 = util::sin(45.01f);
    cos45 = util::cos(45.01f);
}

ZPlayer::ZPlayer(int speed, int slowness) : speed(speed), slowness(slowness){}

/*
 * Used for zSolver, sprinted 45 zAxis movement only
 */
void ZPlayer::simpleMove(float moveVec, bool airborne, bool sprintJumpQ, int repeat) {
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

double ZPlayer::Z(){
    return z;
}

double ZPlayer::Vz(){
    return vz;
}

void ZPlayer::setZ(double value){
    z = value;
}

void ZPlayer::setVz(double value){
    vz = value;
}

void ZPlayer::setVzAir(double value){
    vz = value;
    prev_slip = 1.0f;
}

void ZPlayer::resetAll()
{
    z = 0;
    vz = 0;
    prev_slip = -1;
    prev_sprint = false;
    resetClock();
}

void ZPlayer::resetClock(){
    clock = 0;
    last_inertia = -1;
    forceInertia = false;
}

int ZPlayer::lastInertia(){
    return last_inertia;
}

bool ZPlayer::hitVelNeg(){
    return hit_vel_neg;
}

void ZPlayer::saveState(){
    savestate.z = z;
    savestate.vz = vz;
    savestate.prev_slip = prev_slip;
    savestate.prev_sprint = prev_sprint;
}

ZPlayer::State ZPlayer::getState(){
    return { z, vz, prev_slip, prev_sprint };
}

void ZPlayer::loadState(const ZPlayer::State& s) {
    z = s.z;
    vz = s.vz;
    prev_slip = s.prev_slip;
    prev_sprint = s.prev_sprint;
}

void ZPlayer::toggleInertia(bool on){
    inertia_on = on;
}

void ZPlayer::forceInertiaNext(){
    forceInertia = true;
}

void ZPlayer::sprintDelay(bool delayQ){
    sprint_delay = delayQ;
}

void ZPlayer::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
}

void ZPlayer::clearEffects(){
    speed = 0;
    slowness = 0;
}

void ZPlayer::loadState(){
    z = savestate.z;
    vz = savestate.vz;
    prev_slip = savestate.prev_slip;
    prev_sprint = savestate.prev_sprint;
}

void ZPlayer::sj45(float moveVec, int duration){
    simpleMove(moveVec, GROUND, true, 1);
    simpleMove(moveVec, AIR, false, duration - 1);
}

void ZPlayer::sa45(float moveVec, int duration){
    simpleMove(moveVec, AIR, false, duration);
}

void ZPlayer::s45(float moveVec, int duration){
    simpleMove(moveVec, GROUND, false, duration);
}

void ZPlayer::sj45(int duration){ sj45(FORWARD, duration);}
void ZPlayer::sa45(int duration){ sa45(FORWARD, duration);}
void ZPlayer::s45 (int duration){  s45(FORWARD, duration);}

void ZPlayer::chained_sj45(int airtime, int repeat){
    while (repeat--) 
        sj45(airtime);
}

