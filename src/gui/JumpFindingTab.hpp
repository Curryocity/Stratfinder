#pragma once

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "GuiCommon.hpp"
#include "../zSolver.hpp"

namespace gui {

class JumpFindingTab {
public:
    void pump();
    void renderInputPanel(const AppResources& resources);
    void renderOutputPanel(const AppResources& resources);

private:
    using ZS = zSolver;

    struct SearchMatch {
        int speed = 0;
        int slowness = 0;
        int mmAirtime = 0;
        double mm = 0.0;
        std::string shiftLabel;
        double shiftValue = 0.0;
        ZS::fullStrat strat{};
        ZS::JumpList jumpList;
    };

    struct SearchResult {
        std::vector<SearchMatch> matches;
        std::string errorMsg;
        bool returnErrorQ = false;
        bool cancelled = false;
        double elapsedMs = 0.0;
        long long processedCandidates = 0;
        long long totalCandidates = 0;
        int matchesFound = 0;
    };

    struct ProgressState {
        std::atomic<long long> processedCandidates{0};
        std::atomic<long long> totalCandidates{0};
        std::atomic<long long> currentMmPixel{0};
        std::atomic<int> currentSpeed{0};
        std::atomic<int> currentSlowness{0};
        std::atomic<int> currentMmAirtime{0};
        std::atomic<int> matchesFound{0};
        std::atomic<int> phase{0};
        std::atomic<long long> frozenElapsedMs{0};
    };

    std::string minMmText_ = "0.125";
    std::string maxMmText_ = "5000";

    int minMmAirtime_ = 2;
    int maxMmAirtime_ = 12;
    int maxJumpAirtime_ = 116;

    int minSpeed_ = 0;
    int maxSpeed_ = 255;
    int minSlowness_ = 0;
    int maxSlowness_ = 6;

    std::string thresholdText_ = "1e-9";
    bool useBlockageShift_ = true;
    bool useLadderShift_ = true;
    bool useNormalShift_ = true;
    bool useWaterShift_ = true;
    bool backwalled_ = false;

    std::vector<SearchMatch> matches_;
    std::string errorMsg_;
    bool returnErrorQ_ = false;
    bool searchRunning_ = false;
    bool hasSearched_ = false;
    std::string statusMsg_ = "Idle";
    double elapsedMs_ = 0.0;
    long long processedCandidates_ = 0;
    long long totalCandidates_ = 0;
    int matchesFound_ = 0;
    std::chrono::steady_clock::time_point searchStartTime_{};
    std::shared_ptr<std::atomic_bool> cancelToken_;
    std::shared_ptr<ProgressState> progress_;
    std::future<SearchResult> searchFuture_;

    void startSearch();
};

} // namespace gui
