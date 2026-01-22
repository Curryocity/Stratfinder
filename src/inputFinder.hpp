#pragma once
#include <cmath>
#include <vector>
#include "player.hpp"

class inputFinder {

    public:

    std::vector<std::vector<player::action>> matchZSpeed(double targetVz, double error, int depth, double maxFw = 0, double maxBw = -INFINITY);

};
