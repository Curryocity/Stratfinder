#pragma once

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "GuiCommon.hpp"
#include "../inputCracker.hpp"

namespace gui {

class InputCrackerTab {
public:
    void pump();
    void renderInputPanel(const AppResources& resources);
    void renderOutputPanel(const AppResources& resources);

private:
    using IC = inputCracker;

    struct CrackResult {
        std::vector<IC::Solution> sols;
        version ver = version::v1_8_9;
        std::string errorMsg;
        bool returnErrorQ = false;
        bool cancelled = false;
        double elapsedMs = 0.0;
    };

    struct CartesianFormState {
        bool enableX = false;
        bool enableZ = true;
        bool xWalled = false;
        bool zWalled = false;
        std::string xLbText = "-0.1";
        std::string xUbText = "0.1";
        std::string xMmText = "0";
        std::string zLbText = "-0.1276846279184921";
        std::string zUbText = "-0.1276844242999637";
        std::string zMmText = "-1.5";
    };

    struct PolarFormState {
        std::string normLbText = "0.236";
        std::string normUbText = "0.238";
        std::string angle1Text = "-103.5";
        std::string angle2Text = "-104.5";
        std::string xMmText = "1";
        std::string zMmText = "-1";
        bool xWalled = false;
        bool zWalled = true;
    };

    enum ResultSort {
        SortByDepth = 0,
        SortByTicks = 1,
        SortByTicksAirDebt = 2,
    };

    IC::ConditionForm coordType_ = IC::Cartesian;
    CartesianFormState cart_;
    PolarFormState polar_;

    int mmAirtime_ = 12;
    bool endAirborne_ = false;
    IC::WASD allowKeys_ = {true, true, true, true};

    std::string rotationText_ = "0.0";
    int speed_ = 0;
    int slowness_ = 1;

    int maxDepth_ = 3;
    int maxTicks_ = 40;
    int maxTransTick_ = 16;
    int resultCap_ = 1024;
    bool allowNonEmptyBridge_ = false;
    bool riskyLerp_ = true;

    std::vector<IC::Solution> sols_;
    std::vector<std::string> markedMothballs_;
    int resultSort_ = SortByDepth;
    bool showMarkedOnly_ = false;
    std::string suffixFilter_;
    std::string errorMsg_;
    bool returnErrorQ_ = false;
    bool crackRunning_ = false;
    bool hasSearched_ = false;
    std::string statusMsg_ = "Idle";
    double elapsedMs_ = 0.0;
    version selectedVer_ = version::v1_8_9;
    version crackVer_ = version::v1_8_9;
    std::chrono::steady_clock::time_point crackStartTime_{};
    std::shared_ptr<std::atomic_bool> cancelToken_;
    std::future<CrackResult> crackFuture_;

    void crack(version ver);
    void renderCartesianForm();
    void renderPolarForm();
    bool mothballMarked(const std::string& mothball) const;
    void setMothballMarked(const std::string& mothball, bool marked);
};

} // namespace gui
