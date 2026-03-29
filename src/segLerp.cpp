#include "segLerp.hpp"
#include "player.hpp"

namespace {

segLerp::axisLerp composeAxis(const segLerp::axisLerp& outer, const segLerp::axisLerp& inner) {
    return {
        outer.alpha * inner.alpha,
        outer.alpha * inner.beta + outer.beta,
    };
}

segLerp::lerp composeLerp(const segLerp::lerp& outer, const segLerp::lerp& inner) {
    return {
        composeAxis(outer.lerpX, inner.lerpX),
        composeAxis(outer.lerpZ, inner.lerpZ),
    };
}

segLerp::lerp identityLerp() {
    return {};
}

segLerp::sprDel identitySprDel() {
    return {identityLerp(), identityLerp()};
}

}  // Namespace End

void segLerp::setParameters(int airtime, int speed, int slowness, double rotation) {
    this->airtime = airtime;
    this->speed = speed;
    this->slowness = slowness;
    this->rotation = rotation;
}

void segLerp::enableAxis(bool x, bool z) {
    enableX = x;
    enableZ = z;
}

int segLerp::prefixIdx(int w, int a, segmentType type) const {
    if (w < -1 || w > 1 || a < -1 || a > 1) return -1;
    if (type != Air && type != Jump) return -1;

    const int wIdx = w + 1;
    const int aIdx = a + 1;
    const int typeIdx = (type == Air) ? 0 : 1;

    return (typeIdx * 3 + aIdx) * 3 + wIdx;
}

int segLerp::idx(int w, int a, int t, segmentType type) const {
    if (w < -1 || w > 1 || a < -1 || a > 1) return -1;
    if (type == None) return -1;

    const int wIdx = w + 1;
    const int aIdx = a + 1;
    const int perInput = 3 * 3;

    if (type == Ground) {
        if (t < 0 || t > groundCap) return -1;
        return (t * 3 + aIdx) * 3 + wIdx;
    }

    if (type == Jump) type = Air; // The tail were all airborne
    if (t < 0 || t >= airtime) return -1;

    const int airBase = (groundCap + 1) * perInput;
    return airBase + (t * 3 + aIdx) * 3 + wIdx;
}

void segLerp::buildTransform() {
    const int groundEntryCount = (groundCap + 1) * 3 * 3;
    const int airEntryCount = airtime * 3 * 3;
    const int entryCount = groundEntryCount + airEntryCount;
    const int prefixEntryCount = 2 * 3 * 3;
    lerpTable.assign(entryCount, identityLerp());
    prefixTransform.assign(prefixEntryCount, identitySprDel());

    player p(speed, slowness);
    p.setF(rotation);
    p.toggleInertia(false);

    // Prefix lerp to handle sprintDelay
    // You don't need to "simplify" boilerplates sometimes
    for(int w = -1; w <= 1; w ++){
        for(int a = -1; a <= 1; a++){

            const bool sprintQ = (w == 1);
            double vx0, vx1, vz0, vz1;
            lerp l;
            sprDel sdel;

            // Air walk
            segmentType type = Air;
            p.setVel(0, 0);
            p.setPrevSprint(false);
            p.move(w, a, true, 0, 1);
            vx0 = p.Vx()/0.6; vz0 = p.Vz()/0.6;

            p.setVel(1, 1);
            p.setPrevSprint(false);
            p.move(w, a, true, 0, 1);
            vx1 = p.Vx()/0.6; vz1 = p.Vz()/0.6;

            l.lerpX.alpha = vx1 - vx0;
            l.lerpX.beta = vx0;
            l.lerpZ.alpha = vz1 - vz0;
            l.lerpZ.beta = vz0;

            sdel.prevSpr0 = l;

            // Air sprint
            p.setVel(0, 0);
            p.setPrevSprint(true);
            p.move(w, a, true, 2 * sprintQ, 1);
            vx0 = p.Vx()/0.6; vz0 = p.Vz()/0.6;

            p.setVel(1, 1);
            p.setPrevSprint(true);
            p.move(w, a, true, 2 * sprintQ, 1);
            vx1 = p.Vx()/0.6; vz1 = p.Vz()/0.6;

            l.lerpX.alpha = vx1 - vx0;
            l.lerpX.beta = vx0;
            l.lerpZ.alpha = vz1 - vz0;
            l.lerpZ.beta = vz0;

            sdel.prevSpr1 = l;
            prefixTransform[prefixIdx(w, a, type)] = sdel;

            // Jump
            type = Jump;
            p.setVel(0, 0);
            p.move(w, a, false, 3 * sprintQ, 1);
            vx0 = p.Vx(); vz0 = p.Vz();

            p.setVel(1, 1);
            p.move(w, a, false, 3 * sprintQ, 1);
            vx1 = p.Vx(); vz1 = p.Vz();

            l.lerpX.alpha = vx1 - vx0;
            l.lerpX.beta = vx0;
            l.lerpZ.alpha = vz1 - vz0;
            l.lerpZ.beta = vz0;

            sdel.prevSpr0 = l;
            sdel.prevSpr1 = l;
            prefixTransform[prefixIdx(w, a, type)] = sdel;
        }
    }


    // Tail lerp t = 0... airtime
    for(int w = -1; w <= 1; w ++){
        for(int a = -1; a <= 1; a++){
            const bool sprintQ = (w == 1);

            for(int t = 1; t <= groundCap; t++){
                double vx0, vx1, vz0, vz1;
                lerp l;

                // Ground
                segmentType type = Ground;
                p.setVel(0, 0);
                p.move(w, a, false, 2 * sprintQ, t);
                vx0 = p.Vx(); vz0 = p.Vz();

                p.setVel(1, 1);
                p.move(w, a, false, 2 * sprintQ, t);
                vx1 = p.Vx(); vz1 = p.Vz();

                l.lerpX.alpha = vx1 - vx0;
                l.lerpX.beta = vx0;
                l.lerpZ.alpha = vz1 - vz0;
                l.lerpZ.beta = vz0;

                lerpTable[idx(w, a, t, type)] = l;
            }

            for(int t = 1; t < airtime; t++){
                double vx0, vx1, vz0, vz1;
                lerp l;

                // Air
                segmentType type = Air;
                type = Air;
                p.setVel(0, 0);
                p.setPrevSprint(sprintQ);
                p.move(w, a, true, 2 * sprintQ, t);
                vx0 = p.Vx()/0.6; vz0 = p.Vz()/0.6;

                p.setVel(1, 1);
                p.setPrevSprint(sprintQ);
                p.move(w, a, true, 2 * sprintQ, t);
                vx1 = p.Vx()/0.6; vz1 = p.Vz()/0.6;

                l.lerpX.alpha = vx1 - vx0;
                l.lerpX.beta = vx0;
                l.lerpZ.alpha = vz1 - vz0;
                l.lerpZ.beta = vz0;

                lerpTable[idx(w, a, t, type)] = l;
            }
        }
    }

}

void segLerp::updateLerp(lerp& lerp0, lerp& lerp1, int w, int a, int t, segmentType type) {
    if(type == None) return;
    const lerp theLerp = (w == 1) ? lerp1 : lerp0;

    if (enableX) {
        if(type == Ground){
            if (t < 1 || t > groundCap) return;
            const axisLerp groundX = lerpTable[idx(w, a, t, type)].lerpX;
            lerp0.lerpX = composeAxis(theLerp.lerpX, groundX);
            lerp1.lerpX = composeAxis(theLerp.lerpX, groundX);
        }else{
            const int preIdx = prefixIdx(w, a, type);
            const int tableIdx = idx(w, a, t - 1, type);
            if (tableIdx < 0 || preIdx < 0) return;
            const sprDel& prefix = prefixTransform[preIdx];
            const axisLerp pre0X = prefix.prevSpr0.lerpX;
            const axisLerp pre1X = prefix.prevSpr1.lerpX;
            const axisLerp tailX = lerpTable[tableIdx].lerpX;
            lerp0.lerpX = composeAxis(theLerp.lerpX, composeAxis(tailX, pre0X));
            lerp1.lerpX = composeAxis(theLerp.lerpX, composeAxis(tailX, pre1X));
        }
    }

    if (enableZ) {
        if(type == Ground){
            if (t < 1 || t > groundCap) return;
            const axisLerp groundZ = lerpTable[idx(w, a, t, type)].lerpZ;
            lerp0.lerpZ = composeAxis(theLerp.lerpZ, groundZ);
            lerp1.lerpZ = composeAxis(theLerp.lerpZ, groundZ);
        }else{
            const int preIdx = prefixIdx(w, a, type);
            const int tableIdx = idx(w, a, t - 1, type);
            if (tableIdx < 0 || preIdx < 0) return;
            const sprDel& prefix = prefixTransform[preIdx];
            const axisLerp pre0Z = prefix.prevSpr0.lerpZ;
            const axisLerp pre1Z = prefix.prevSpr1.lerpZ;
            const axisLerp tailZ = lerpTable[tableIdx].lerpZ;
            lerp0.lerpZ = composeAxis(theLerp.lerpZ, composeAxis(tailZ, pre0Z));
            lerp1.lerpZ = composeAxis(theLerp.lerpZ, composeAxis(tailZ, pre1Z));
        }
    }
}
