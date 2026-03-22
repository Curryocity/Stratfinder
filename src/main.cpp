#include <chrono>
#include <iostream>
#include "inputFinder.hpp"
#include "util.hpp"

void init(){
    util::init();
}

struct BenchmarkStats {
    double avgUs = 0;
    double minUs = 0;
    double maxUs = 0;
    std::size_t resultCount = 0;
    int runs = 0;
    inputFinder::SearchStats searchStats;
};

void accumulateSearchStats(inputFinder::SearchStats& dst, const inputFinder::SearchStats& src) {
    dst.inputDfsRecCalls += src.inputDfsRecCalls;
    dst.alphaBetaUpdateCalls += src.alphaBetaUpdateCalls;
    dst.estimateSpeedCalls += src.estimateSpeedCalls;
    dst.exeSeqCalls += src.exeSeqCalls;
    dst.maxTickPrunes += src.maxTickPrunes;
    dst.minBoundPrunes += src.minBoundPrunes;
    dst.maxBoundPrunes += src.maxBoundPrunes;
    dst.endDepthRejects += src.endDepthRejects;
    dst.childHardPrunesNoRJ += src.childHardPrunesNoRJ;
    dst.childHardPrunesRJ += src.childHardPrunesRJ;
    dst.monotonicPrunesNoRJ += src.monotonicPrunesNoRJ;
    dst.monotonicPrunesRJ += src.monotonicPrunesRJ;
}

template<typename Func, typename StatsFunc>
BenchmarkStats benchmarkUs(Func func, StatsFunc statsFunc, int runs = 50) {
    BenchmarkStats stats;
    stats.runs = runs;
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
        accumulateSearchStats(stats.searchStats, statsFunc());
    }

    stats.avgUs = totalUs / runs;
    return stats;
}

void printBenchMark(const BenchmarkStats& stats){
    std::cout << "Amounts of inputs found: " << stats.resultCount << "\n";
    std::cout << "Avg time: " << stats.avgUs << " us\n";
    std::cout << "Min time: " << stats.minUs << " us\n";
    std::cout << "Max time: " << stats.maxUs << " us\n";
    const double runs = static_cast<double>(stats.runs);
    std::cout << "Avg inputDfsRec calls: " << (stats.searchStats.inputDfsRecCalls / runs) << "\n";
    std::cout << "Avg alphaBetaUpdate calls: " << (stats.searchStats.alphaBetaUpdateCalls / runs) << "\n";
    std::cout << "Avg estimateSpeed calls: " << (stats.searchStats.estimateSpeedCalls / runs) << "\n";
    std::cout << "Avg exeSeq calls: " << (stats.searchStats.exeSeqCalls / runs) << "\n";
    std::cout << "Avg maxTick prunes: " << (stats.searchStats.maxTickPrunes / runs) << "\n";
    std::cout << "Avg minBound prunes: " << (stats.searchStats.minBoundPrunes / runs) << "\n";
    std::cout << "Avg maxBound prunes: " << (stats.searchStats.maxBoundPrunes / runs) << "\n";
    std::cout << "Avg endDepth rejects: " << (stats.searchStats.endDepthRejects / runs) << "\n";
    std::cout << "Avg childHard prunes (no RJ): " << (stats.searchStats.childHardPrunesNoRJ / runs) << "\n";
    std::cout << "Avg childHard prunes (RJ): " << (stats.searchStats.childHardPrunesRJ / runs) << "\n";
    std::cout << "Avg monotonic prunes (no RJ): " << (stats.searchStats.monotonicPrunesNoRJ / runs) << "\n";
    std::cout << "Avg monotonic prunes (RJ): " << (stats.searchStats.monotonicPrunesRJ / runs) << "\n";
}

int main() {
    init();

    const int depth = 4;
    const int airtime = 12;
    const int runs = 100;
    const bool allowStrafe = false;

    inputFinder::condition cond;
    cond.endAirborne = false;
    cond.x.enabled = false;
    cond.z.enabled = true;
    cond.z.mm = -1.5;
    cond.allowStrafe = allowStrafe;
    cond.sideDev = 0.5;
    inputFinder::setCondWithBound(cond.z, -0.1276844242999637, -0.1276846279184921);

    auto printHeader = [&](bool riskIt) {
        std::cout << "------------------------------\n";
        std::cout << "depth: " << depth << ", airtime: " << airtime << ", runs: " << runs << "\n";
        std::cout << "allowStrafe: " << allowStrafe << ", riskyPrune: " << riskIt << "\n";
        std::cout << "targetVz: " << util::df(cond.z.vel)
                  << ", error: " << util::df(cond.z.tolerance)
                  << ", mm: " << util::fmt(cond.z.mm) << "\n";
    };

    for (bool riskIt : {false, true}) {
        inputFinder f;
        f.toggleLog(false);
        f.changeSettings(depth, 40);
        f.riskyPrune(riskIt);
        f.setEffect(0, 1);

        BenchmarkStats inputStats = benchmarkUs(
            [&]() {
                return f.matchSpeed(cond, airtime);
            },
            [&]() {
                return f.getSearchStats();
            },
            runs
        );

        printHeader(riskIt);
        std::cout << "InputFinder:\n";
        printBenchMark(inputStats);
    }

    return 0;
}
