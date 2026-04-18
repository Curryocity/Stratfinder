#include <iostream>
#include "player.hpp"
#include "util.hpp"


namespace {

void init() {
    util::init();
}

}

int main() {
    init();

    player p;

    p.s(1,1,1);

    std::cout << "Vz: " << util::df(p.Vz(), 17) << "\n";


    return 0;
}
