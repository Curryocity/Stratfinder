#include <chrono>
#include <iostream>
#include "inputFinder.hpp"

#include "util.hpp"
#include "zInputFinder.hpp"

void init(){
    util::init();
}

struct BenchmarkStats {
    double avgUs = 0;
    double minUs = 0;
    double maxUs = 0;
    std::size_t resultCount = 0;
};

template<typename Func>
BenchmarkStats benchmarkUs(Func func, int runs = 50) {
    BenchmarkStats stats;
    stats.resultCount = func().size();

    double totalUs = 0;
    stats.minUs = INFINITY;
    stats.maxUs = 0;

    for (int i = 0; i < runs; i++) {
        auto start = std::chrono::steady_clock::now();
        auto result = func();
        auto end = std::chrono::steady_clock::now();

        double elapsedUs = std::chrono::duration<double, std::micro>(end - start).count();
        totalUs += elapsedUs;
        stats.minUs = std::min(stats.minUs, elapsedUs);
        stats.maxUs = std::max(stats.maxUs, elapsedUs);
        stats.resultCount = result.size();
    }

    stats.avgUs = totalUs / runs;
    return stats;
}

void printBenchMark(const BenchmarkStats& stats){
    std::cout << "Amounts of inputs found: " << stats.resultCount << "\n";
    std::cout << "Avg time: " << stats.avgUs << " us\n";
    std::cout << "Min time: " << stats.minUs << " us\n";
    std::cout << "Max time: " << stats.maxUs << " us\n";
}

int main() {

    // We don't have GUI yet ;(

    init();

    if(true){
        // Finding input for slowness I 1.5bm 6-1 to ladder (perfect double 45.01)
        // use zInputFinder
        auto testZinputFinder = [](bool riskIt, int runs){
            zInputFinder f;
            f.toggleLog(false);
            f.changeSettings(4, 40);
            f.dontCareInertia(riskIt);
            f.setEffect(0, 1);
            zInputFinder::zCond cond = zInputFinder::genZCondLBUB(-0.1276844242999637, -0.1276846279184921, -1.5, false);
            cond.endAirborn = false;
            cond.sideDev = -1;
            double targetVz = cond.targetVz;
            double error = cond.error;
            double mm = cond.mm;
            double airtime = 12;
            bool hasStrafe = cond.allowStrafe;
            std::cout << "------------------------------\n";
            std::cout << "zInputFinder: \n";
            std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", hasStrafe: " << hasStrafe << "\n";
            std::cout << "riskyPrune? " << riskIt << "\n";

            BenchmarkStats stats = benchmarkUs([&]() {
                return f.matchZSpeed(cond, airtime);
            }, runs);

            printBenchMark(stats);
        };

        int runs = 50;
        testZinputFinder(false, runs);
        testZinputFinder(true, runs);

        // use new inputFinder
        auto testInputFinder = [](bool riskIt, int runs){
            inputFinder f;
            f.toggleLog(false);
            f.changeSettings(4, 40);
            f.riskyPrune(riskIt);
            f.logSettings();
            f.setEffect(0, 1);
            inputFinder::condition cond;
            cond.endAirborne = false;
            cond.x.enabled = false;
            cond.z.enabled = true;
            cond.z.mm = -1.5;
            cond.allowStrafe = false;
            cond.sideDev = -1;
            f.setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);
            double targetVz = cond.z.vel;
            double error = cond.z.tolerance;
            double mm = cond.z.mm;
            double airtime = 12;
            bool hasStrafe = cond.allowStrafe;
            std::cout << "------------------------------\n";
            std::cout << "InputFinder: \n";
            std::cout << "targetVz: " << util::df(targetVz) << ", error: " << util::df(error) << ", mm: " << util::fmt(mm) << ", airtime: " << airtime << ", allowStrafe: " << hasStrafe << "\n";
            std::cout << "riskyPrune? " << riskIt << "\n";

            BenchmarkStats stats = benchmarkUs([&]() {
                return f.matchSpeed(cond, airtime);
            }, runs);

            printBenchMark(stats);
        };

        testInputFinder(false, runs);
        testInputFinder(true, runs);
    }

    return 0;
}
