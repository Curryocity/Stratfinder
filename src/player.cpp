#include <cmath>
#include "player.hpp"
#include "util.hpp"


player::player(int speed, int slowness) : speed(speed), slowness(slowness){}

void player::move(float w, float a, bool airborne, int movementType, int repeat) {
    while (repeat--) {

        float foward = w;
        float strafe = a;

        float slip = airborne ? 1.0f : GROUND_SLIP;
        x += vx;
        z += vz;

        if (prev_slip == -1)
            prev_slip = slip;

        vx *=  0.91f * prev_slip;
        vz *= 0.91f * prev_slip;

        if(inertia_on){
            if (std::abs(vx) < DEFAULT_INERTIA) vx = 0;
            if (std::abs(vz) < DEFAULT_INERTIA) vz = 0;
        }

        /* base movement */
        double accel;
        if(airborne){
            accel = 0.02f;
        } else{
            accel = 0.1f;
            if (speed > 0)    accel *= 1 + (double) 0.2f *  speed;
            if (slowness > 0) accel *= 1 + (double) -0.15f *  slowness;
            if (accel < 0) accel = 0;
        }

        bool sprinting = movementType >= player::SPRINT;
        if( (sprinting && (!airborne || !sprint_delay) ) || (prev_sprint && sprint_delay && airborne) )
            accel *= 1.300000011920929;  // sprinting multiplier

        float accelf = (float) accel;

        /* ground drag */
        if (!airborne) {
            float drag = 0.91f * slip;
            accelf *= 0.16277136f / (drag * drag * drag);
        }

        /* sprint-jump boost */
        if (movementType == player::SPRINTJUMP){
            float rad = rotation * 0.017453292f;
            vx -= util::sinr(rad) * 0.2f;
            vz += util::cosr(rad) * 0.2f;
        }

        if(movementType == player::SNEAK){
            foward *= 0.3f;
            strafe *= 0.3f;
        }
            
        foward *= 0.98f;
        strafe *= 0.98f;

        float dist = std::sqrtf(foward * foward + strafe * strafe);
        if (dist > 1.0f) accelf /= dist;

        foward *= accelf;
        strafe *= accelf;

        /* apply motion */
        vx += strafe * util::cos(rotation) - foward * util::sin(rotation);
        vz += foward * util::cos(rotation) + strafe * util::sin(rotation);

        prev_slip = slip;
        prev_sprint = sprinting;
    }
}

double player::F(){ return rotation;}
double player::X(){ return x;}
double player::Z(){ return z;}
double player::Vx(){ return vx;}
double player::Vz(){ return vz;}

void player::setF(float rot){ rotation = rot;}
void player::setX(double value){ x = value;}
void player::setZ(double value){ z = value;}
void player::setVx(double value, bool airborne){
    vx = value;
    prev_slip = airborne? 1.0f: GROUND_SLIP;
}
void player::setVz(double value, bool airborne){
    vz = value;
    prev_slip = airborne? 1.0f: GROUND_SLIP;
}

void player::resetAll(){
    x = 0;
    z = 0;
    vx = 0;
    vz = 0;
    prev_slip = -1;
    prev_sprint = false;
}

void player::saveState(){
    savestate = { x, z, vx, vz, rotation, prev_slip, prev_sprint };
}

player::State player::getState(){
    return { x,z,vx, vz, rotation, prev_slip, prev_sprint };
}

void player::loadState(const player::State& s) {
    z = s.z;
    vz = s.vz;
    prev_slip = s.prev_slip;
    prev_sprint = s.prev_sprint;
}

void player::toggleInertia(bool on){
    inertia_on = on;
}

void player::setPrevSprint(bool value){
    prev_sprint = value;
}

void player::sprintDelay(bool delayQ){
    sprint_delay = delayQ;
}

void player::setEffect(int speed, int slowness){
    this->speed = speed;
    this->slowness = slowness;
}

void player::clearEffects(){
    speed = 0;
    slowness = 0;
}

void player::loadState(){
    z = savestate.z;
    vz = savestate.vz;
    prev_slip = savestate.prev_slip;
    prev_sprint = savestate.prev_sprint;
}
