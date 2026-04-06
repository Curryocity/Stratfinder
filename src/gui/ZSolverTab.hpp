#pragma once

#include <chrono>
#include <future>
#include <string>

#include "GuiCommon.hpp"
#include "../zSolver.hpp"

namespace gui {

class ZSolverTab {
public:
    void pump();
    void renderInputPanel(const AppResources& resources);
    void renderOutputPanel(const AppResources& resources);

private:
    using ZS = zSolver;
    enum Mode {
        Search = 0,
        Standard = 1,
    };

    struct StandardJumpResult {
        int airtime = -1;
        double jumpDistance = 0.0;
        double landingOffset = 0.0;
        bool nondelayedBetter = false;
    };

    struct SolverResult {
        ZS::fullStrat strat{};
        ZS::JumpList jumpList;
        StandardJumpResult standardJump;
        std::string logText;
        std::string errorMsg;
        bool returnErrorQ = false;
        bool hasJump = false;
        double elapsedMs = 0.0;
    };

    std::string mmText_ = "1.0";
    int mmAirtime_ = 12;
    int speed_ = 0;
    int slowness_ = 0;
    int mode_ = Standard;
    int maxAirtime_ = 25;
    int jumpAirtime_ = 12;
    std::string thresholdText_ = "0.01";
    std::string shiftText_ = "0.6";
    bool backwalled_ = false;

    ZS::fullStrat solverStrat_;
    ZS::JumpList jumpList_;
    StandardJumpResult standardJump_;
    std::string solverLogText_;
    std::string solverErrorMsg_;
    bool solverReturnErrorQ_ = false;
    bool solverRunning_ = false;
    bool solverHasRun_ = false;
    bool solverHasJump_ = false;
    std::string solverStatusMsg_ = "Idle";
    double solverElapsedMs_ = 0.0;
    std::chrono::steady_clock::time_point solverStartTime_{};
    std::future<SolverResult> solverFuture_;

    void solve();
    void drawStratSummary() const;
    static StandardJumpResult evalStandardJump(const ZS::fullStrat& strat, int airtime, float shift, int speed, int slowness);
};

} // namespace gui
