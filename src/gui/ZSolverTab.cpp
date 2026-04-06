#include "ZSolverTab.hpp"

#include <chrono>
#include <exception>


namespace gui {

namespace {

std::string solverStratName(int stratType) {
    return stratType < 0 ? "N/A" : zSolver::strat2string(stratType);
}

} // namespace

void ZSolverTab::pump() {
    if (!solverRunning_ || !solverFuture_.valid()) return;
    if (solverFuture_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return;

    try {
        SolverResult result = solverFuture_.get();
        solverStrat_ = std::move(result.strat);
        jumpList_ = std::move(result.jumpList);
        solverLogText_ = std::move(result.logText);
        solverErrorMsg_ = std::move(result.errorMsg);
        solverReturnErrorQ_ = result.returnErrorQ;
        solverHasJump_ = result.hasJump;
        solverElapsedMs_ = result.elapsedMs;
        solverStatusMsg_ = solverReturnErrorQ_ ? "Solve Failed" : "Finished";
    } catch (const std::exception& e) {
        jumpList_ = {};
        solverLogText_.clear();
        solverErrorMsg_ = e.what();
        solverReturnErrorQ_ = true;
        solverElapsedMs_ = 0.0;
        solverStatusMsg_ = "Solve Failed";
    } catch (...) {
        jumpList_ = {};
        solverLogText_.clear();
        solverErrorMsg_ = "Unknown error";
        solverReturnErrorQ_ = true;
        solverElapsedMs_ = 0.0;
        solverStatusMsg_ = "Solve Failed";
    }

    solverRunning_ = false;
}

void ZSolverTab::solve() {
    if (solverRunning_) return;

    solverStrat_ = {};
    jumpList_ = {};
    solverLogText_.clear();
    solverErrorMsg_.clear();
    solverReturnErrorQ_ = false;
    solverHasJump_ = false;

    const auto mmText = mmText_;
    const int mmAirtime = mmAirtime_;
    const int speed = speed_;
    const int slowness = slowness_;
    const int maxTicks = maxAirtime_;
    const auto thresholdText = thresholdText_;
    const auto shiftText = shiftText_;
    const bool backwalled = backwalled_;

    auto failParse = [&](const std::string& msg) {
        solverErrorMsg_ = msg;
        solverReturnErrorQ_ = true;
        solverStatusMsg_ = "Solve Failed";
        solverHasRun_ = true;
    };

    if (mmAirtime <= 0) {
        failParse("MM Airtime must be positive");
        return;
    }

    double mm = 0.0;
    double threshold = 0.0;
    float shift = 0.0f;

    if (!parseDoubleStrict(mmText, mm)) {
        failParse("Invalid number for MM");
        return;
    }
    if (backwalled) {
        if (mm < 0.0 || mm >= 60000000.0) {
            failParse("Backwalled MM must satisfy 0 <= MM < 60000000");
            return;
        }
    } else if (mm < -0.6 || mm >= 60000000.0) {
        failParse("MM must satisfy -0.6 <= MM < 60000000");
        return;
    }
    if (maxTicks < 2) {
        failParse("Max Search T must be at least 2");
        return;
    }
    if (!parseDoubleStrict(thresholdText, threshold)) {
        failParse("Invalid number for Threshold");
        return;
    }
    if (threshold <= 0.0 || threshold > 0.0625) {
        failParse("Threshold must satisfy 0 < threshold <= 0.0625");
        return;
    }
    if (!parseFloatStrict(shiftText, shift)) {
        failParse("Invalid number for Shift");
        return;
    }

    solverRunning_ = true;
    solverHasRun_ = true;
    solverStatusMsg_ = "Solving...";
    solverElapsedMs_ = 0.0;
    solverStartTime_ = std::chrono::steady_clock::now();

    solverFuture_ = std::async(std::launch::async, [=]() -> SolverResult {
        SolverResult result;
        try {
            const auto start = std::chrono::steady_clock::now();
            ZS solver;
            solver.clearLog();
            solver.toggleLog(true);
            solver.setEffect(speed, slowness);

            const ZS::fullStrat strat = backwalled
                ? solver.backwallSolver(mm, mmAirtime)
                : solver.optimalSolver(mm, mmAirtime);
            result.strat = strat;
            result.hasJump = solver.poss(result.jumpList, strat, maxTicks, threshold, backwalled, shift);

            result.logText = solver.getLog();
            const auto end = std::chrono::steady_clock::now();
            result.elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
            return result;
        } catch (const std::exception& e) {
            result.errorMsg = e.what();
            result.returnErrorQ = true;
        } catch (...) {
            result.errorMsg = "Unknown error";
            result.returnErrorQ = true;
        }

        return result;
    });
}

void ZSolverTab::renderInputPanel(const AppResources& resources) {
    const bool pushedUiFont = resources.uiFont != nullptr;
    if (pushedUiFont) ImGui::PushFont(resources.uiFont);

    ImGui::SeparatorText("zSolver");

    ImGui::Spacing();
    drawLabeledTextInput("MM:", "##solverMm", 120.0f, mmText_);
    ImGui::SameLine();
    if (ImGui::Button(backwalled_ ? "Backwalled##solverWallMode" : "Normal##solverWallMode")) {
        backwalled_ = !backwalled_;
    }

    ImGui::AlignTextToFramePadding();
    ImGui::Text("MM Airtime:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60.0f);
    ImGui::InputInt("##solverAirtime", &mmAirtime_, 0, 0);

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Speed/Slowness:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##solverSpeed", &speed_, 0, 0);
    ImGui::SameLine(0, 5);
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##solverSlowness", &slowness_, 0, 0);

    ImGui::SeparatorText("Search");

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Max Airtime:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70.0f);
    ImGui::InputInt("##solverMaxAirtime", &maxAirtime_, 0, 0);

    drawLabeledTextInput("Threshold:", "##solverThreshold", 100.0f, thresholdText_);
    drawLabeledTextInput("Shift:", "##solverShift", 100.0f, shiftText_);

    ImGui::TextDisabled("Tip: 0.0 = blockage, 0.3 = ladder, 0.6 = normal.");

    ImGui::Spacing();
    const ImVec4 solveButton = brighten(ImGui::GetStyle().Colors[ImGuiCol_Button], 1.08f);
    const ImVec4 solveHovered = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered], 1.12f);
    const ImVec4 solveActive = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonActive], 1.16f);
    ImGui::PushStyleColor(ImGuiCol_Button, solveButton);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, solveHovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, solveActive);
    if (ImGui::Button(solverRunning_ ? "Solving..." : "Run zSolver", ImVec2(-1.0f, 0.0f)) && !solverRunning_) {
        solve();
    }
    ImGui::PopStyleColor(3);

    if (pushedUiFont) ImGui::PopFont();
}

void ZSolverTab::drawStratSummary() const {
    const ZS::fullStrat& value = solverStrat_;
    ImGui::Text("Nondelayed");
    ImGui::Text("- Strat Type: %s", solverStratName(value.nondelayStrat).c_str());
    ImGui::Text("- Vz: %.15g", value.nondelaySpeed);
    ImGui::Text("Delayed (%dt)", value.delayTick);
    ImGui::Text("- Strat Type: %s", solverStratName(value.delayStrat).c_str());
    ImGui::Text("- Vz: %.15g", value.delaySpeed);
}

void ZSolverTab::renderOutputPanel(const AppResources& resources) {
    const bool pushedUiFont = resources.uiFont != nullptr;
    const bool pushedCodeFont = resources.codeFont != nullptr;
    if (pushedUiFont) ImGui::PushFont(resources.uiFont);

    ImGui::SeparatorText("Result");
    if (pushedCodeFont) ImGui::PushFont(resources.codeFont);

    ImGui::BeginChild("ZSolverResultList", ImVec2(0.0f, 0.0f), false);
    if (solverReturnErrorQ_) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", solverErrorMsg_.c_str());
    }

    if (!solverRunning_ && !solverReturnErrorQ_ && !solverHasRun_) {
        ImGui::TextDisabled("No solver result yet.");
    } else if (!solverReturnErrorQ_) {
        drawStratSummary();

        ImGui::Spacing();
        if (!solverRunning_) {
            ImGui::TextDisabled(
                "Solved in %s",
                formatElapsedHmsMs(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double, std::milli>(solverElapsedMs_)
                )).c_str()
            );
        } else {
            ImGui::TextDisabled(
                "Solving... %s",
                formatElapsedHms(std::chrono::steady_clock::now() - solverStartTime_).c_str()
            );
        }

        if (!jumpList_.jumps.empty()) {
            ImGui::Spacing();
            ImGui::SeparatorText("Jump List");

            for (std::size_t i = 0; i < jumpList_.jumps.size(); ++i) {
                const auto& jump = jumpList_.jumps[i];
                if (jumpList_.firstNondelayedIdx == static_cast<int>(i)) {
                    ImGui::TextDisabled("Nondelayed becomes better at t = %d", jumpList_.firstNondelayedTick);
                }
                ImGui::Text(
                    "t = %d: %.15g + %.15g b",
                    jump.airtime,
                    jump.jumpDistance,
                    jump.landingOffset
                );
            }
        }

        if (!solverLogText_.empty()) {
            ImGui::Spacing();
            if (ImGui::CollapsingHeader("Solver Log")) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
                ImGui::TextUnformatted(solverLogText_.c_str());
                ImGui::PopStyleColor();
            }
        }
    }

    ImGui::EndChild();
    if (pushedCodeFont) ImGui::PopFont();
    if (pushedUiFont) ImGui::PopFont();
}

} // namespace gui
