#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <iostream>
#include "inputCracker.hpp"
#include "util.hpp"

namespace {

void init() {
    util::init();
}

struct BenchmarkStats {
    double avgUs = 0;
    double minUs = 0;
    double maxUs = 0;
    std::size_t resultCount = 0;
    int runs = 0;
    inputCracker::SearchStats searchStats;
};

template<typename Func, typename StatsFunc>
BenchmarkStats benchmarkUs(Func func, StatsFunc statsFunc, int runs = 100) {
    BenchmarkStats stats;
    stats.runs = runs;

    func();

    auto measuredResult = func();
    stats.resultCount = measuredResult.size();
    stats.searchStats = statsFunc();

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

void printBenchmark(const BenchmarkStats& stats) {
    std::cout << "Amounts of inputs found: " << stats.resultCount << "\n";
    std::cout << "Avg time: " << stats.avgUs << " us\n";
    std::cout << "Min time: " << stats.minUs << " us\n";
    std::cout << "Max time: " << stats.maxUs << " us\n";
    std::cout << "Per-run inputDfsRec calls: " << stats.searchStats.inputDfsRecCalls << "\n";
    std::cout << "Per-run alphaBetaUpdate calls: " << stats.searchStats.alphaBetaUpdateCalls << "\n";
    std::cout << "Per-run estimateSpeed calls: " << stats.searchStats.estimateSpeedCalls << "\n";
    std::cout << "Per-run exeSeq calls: " << stats.searchStats.exeSeqCalls << "\n";
    std::cout << "Per-run maxTick prunes: " << stats.searchStats.maxTickPrunes << "\n";
    std::cout << "Per-run minBound prunes: " << stats.searchStats.minBoundPrunes << "\n";
    std::cout << "Per-run maxBound prunes: " << stats.searchStats.maxBoundPrunes << "\n";
    std::cout << "Per-run zeroInf prunes: " << stats.searchStats.zeroInfPrune << "\n";
    std::cout << "Per-run childHard prunes (ground run): " << stats.searchStats.childHardPrunesGroundRun << "\n";
    std::cout << "Per-run childHard prunes (ground air): " << stats.searchStats.childHardPrunesGroundAir << "\n";
    std::cout << "Per-run childHard prunes (air extend): " << stats.searchStats.childHardPrunesAirExtend << "\n";
    std::cout << "Per-run monotonic prunes (ground run): " << stats.searchStats.monotonicPrunesGroundRun << "\n";
    std::cout << "Per-run monotonic prunes (ground air): " << stats.searchStats.monotonicPrunesGroundAir << "\n";
    std::cout << "Per-run monotonic prunes (air extend): " << stats.searchStats.monotonicPrunesAirExtend << "\n";
}

}

int main() {
    init();

    const int depth = 2;
    const int maxTick = 50;
    const int airtime = 12;
    const int runs = 100;

    inputCracker::condition cond;
    cond.endAirborne = false;
    cond.x.enabled = false;
    cond.z.enabled = true;
    cond.z.mm = -1.5;
    inputCracker::setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);

    auto printHeader = [&](bool riskIt) {
        std::cout << "------------------------------\n";
        std::cout << "Depth: " << depth << ", MaxTicks: " << maxTick << ", Runs: " << runs << "\n";
        std::cout << "RiskyPrune: " << riskIt << "\n";
    };

    for (bool riskIt : {false, true}) {
        inputCracker cracker;
        cracker.changeSettings(depth, maxTick, -1);
        cracker.riskyPrune(riskIt);
        cracker.setEffect(0, 1);
        cracker.setRotation(0.015);

        BenchmarkStats inputStats = benchmarkUs(
            [&]() {
                return cracker.matchSpeed(cond, airtime);
            },
            [&]() {
                return cracker.getSearchStats();
            },
            runs
        );

        printHeader(riskIt);
        std::cout << "InputCracker:\n";
        printBenchmark(inputStats);
    }

    return 0;
}
