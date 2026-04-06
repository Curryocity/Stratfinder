#pragma once

#include "version.hpp"
#include <vector>
struct segLerp{

    enum segmentType{None, Ground, Air, Jump};

    struct axisLerp{
        double alpha = 1;
        double beta = 0;
    };

     struct lerp{
        axisLerp lerpX;
        axisLerp lerpZ;
    };

    struct sprDel {
        lerp prevSpr0;
        lerp prevSpr1;
    };


    void setParameters(int airtime, int speed, int slowness, double rotation, version ver);
    void enableAxis(bool x, bool z);
    
    int prefixIdx(int w, int a, segmentType type) const;
    int idx(int w, int a, int t, segmentType type) const;
    void buildTransform();
    void updateLerp(lerp& lerp0, lerp& lerp1, int w, int a, int t, segmentType type);

    private:

    static constexpr int groundCap = 31;

    int airtime = 12;
    int speed = 0, slowness = 0;
    double rotation = 0;
    version ver = version::v1_8_9;

    bool enableX = true;
    bool enableZ = true;

    std::vector<lerp> lerpTable;
    std::vector<sprDel> prefixTransform;

};
