#include <cmath>
#include "ZPlayer.hpp"

/* static storage */
float ZPlayer::SIN_TABLE[65536];
float ZPlayer::sin45 = 0.0f;
float ZPlayer::cos45 = 0.0f;

/*
 * Precompute sine table.
 */
void ZPlayer::init() {
    for (int i = 0; i < 65536; ++i) {
        SIN_TABLE[i] = std::sin(i * M_PI * 2.0 / 65536.0);
    }

    sin45 = sin(45.01f);
    cos45 = cos(45.01f);
}

inline float ZPlayer::sin(float deg) {
    float rad = deg * PI / 180.0f;
    return SIN_TABLE[(int)(rad * 10430.378f) & 65535];
}

inline float ZPlayer::cos(float deg) {
    float rad = deg * PI / 180.0f;
    return SIN_TABLE[(int)(rad * 10430.378f + 16384.0f) & 65535];
}

/*
 * Only covers simple forward and backward movement
 */
void ZPlayer::simpleMove(float moveVec,
                          bool airborne,
                          int movementType,
                          bool has45,
                          int repeat) {
    while (repeat--) {

        float slip = airborne ? 1.0f : GROUND_SLIP;
        z += vz;

        if (prev_slip == -1)
            prev_slip = slip;

        vz *= 0.91f * prev_slip;

        // if (std::abs(vz) < inertia_threshold) vz = 0;
        
        // no acceleration
        if(moveVec == 0){
            prev_slip = slip;
            prev_sprint = movementType >= SPRINT;
            continue;
        } 

        /* base movement */
        float accel = airborne ? 0.02f : 0.1f;
        bool sprint = movementType >= SPRINT;

        /* sprint multiplier */
        if ((sprint && !airborne) || ( ((prev_sprint && sprint_delay) || (sprint && !sprint_delay)) && airborne))
            accel *= 1.300000011920929f;

        /* ground drag */
        if (!airborne) {
            float drag = 0.91f * slip;
            accel *= 0.16277136f / (drag * drag * drag);
        }

        /* sprint-jump boost */
        if (movementType == SPRINTJUMP)
            vz += moveVec * 0.2f;

        /* replicate "forward" & "strafe" in the code */
        float mu = 1.0f;

        /* sneak slowdown */
        if (movementType == SNEAK) 
            mu *= 0.3f;

        mu *= 0.98f;

        /* normalize */
        float dist = has45 ? std::sqrtf(mu * mu + mu * mu) : mu;
        if (dist > 1.0f)
            accel /= dist;

        mu *= accel;

        /* apply motion */
        vz += moveVec * (has45 ? (mu * cos45 + mu * sin45) : mu);

        prev_slip = slip;
        prev_sprint = sprint;
    }
}

double ZPlayer::getZ(){
    return z;
}

double ZPlayer::getVz(){
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

void ZPlayer::inertia(double inertia){
    inertia_threshold = inertia;
}

void ZPlayer::sprintDelay(bool delayQ){
    sprint_delay = delayQ;
}

void ZPlayer::loadState(){
    z = savestate.z;
    vz = savestate.vz;
    prev_slip = savestate.prev_slip;
    prev_sprint = savestate.prev_sprint;
}

void ZPlayer::st(int duration){
    simpleMove(0, GROUND, NONE, NO45, duration);
}

void ZPlayer::sta(int duration){
    simpleMove(0, AIR, NONE, NO45, duration);
}

void ZPlayer::chained_sj(int airtime, int repeat){
    while (repeat--) 
        sj(FORWARD, airtime);
}

void ZPlayer::chained_sj45(int airtime, int repeat){
    while (repeat--) 
        sj45(FORWARD, airtime);
}

void ZPlayer::stj(int duration)
{
    simpleMove(0, GROUND, NONE, NO45, duration);
}


