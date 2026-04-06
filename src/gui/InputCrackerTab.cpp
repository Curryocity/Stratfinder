#include "InputCrackerTab.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <numeric>

namespace gui {

namespace {

const char* kCoordTypes[2] = {"Cartesian", "Polar"};

}

void InputCrackerTab::pump() {
    if (!crackRunning_ || !crackFuture_.valid()) return;
    if (crackFuture_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return;

    try {
        CrackResult result = crackFuture_.get();
        sols_ = std::move(result.sols);
        errorMsg_ = std::move(result.errorMsg);
        returnErrorQ_ = result.returnErrorQ;
        elapsedMs_ = result.elapsedMs;
        if (result.cancelled) {
            statusMsg_ = "Halted";
        } else {
            statusMsg_ = returnErrorQ_ ? "Search Failed" : "Finished";
        }
    } catch (const std::exception& e) {
        sols_.clear();
        errorMsg_ = e.what();
        returnErrorQ_ = true;
        elapsedMs_ = 0.0;
        statusMsg_ = "Search Failed";
    } catch (...) {
        sols_.clear();
        errorMsg_ = "Unknown error";
        returnErrorQ_ = true;
        elapsedMs_ = 0.0;
        statusMsg_ = "Search Failed";
    }

    crackRunning_ = false;
}

void InputCrackerTab::crack() {
    if (crackRunning_) return;

    sols_.clear();
    errorMsg_.clear();
    returnErrorQ_ = false;

    const auto coordType = coordType_;
    const auto allowKeys = allowKeys_;
    const auto cart = cart_;
    const auto polar = polar_;
    const int airtime = mmAirtime_;
    const bool endAirborne = endAirborne_;
    const auto rotationText = rotationText_;
    const int speed = speed_;
    const int slowness = slowness_;
    const int maxDepth = maxDepth_;
    const int maxTicks = maxTicks_;
    const int maxTransTick = maxTransTick_;
    const int resultCap = resultCap_;
    const bool allowNonEmptyBridge = allowNonEmptyBridge_;
    const bool riskyLerp = riskyLerp_;

    auto failParse = [&](const std::string& msg) {
        errorMsg_ = msg;
        returnErrorQ_ = true;
        statusMsg_ = "Search Failed";
        hasSearched_ = true;
    };

    double rotation = 0.0;
    if (!parseDoubleStrict(rotationText, rotation)) {
        failParse("Invalid number for Facing");
        return;
    }

    double normLb = 0.0, normUb = 0.0, angle1 = 0.0, angle2 = 0.0;
    double xLb = 0.0, xUb = 0.0, xMm = 0.0;
    double zLb = 0.0, zUb = 0.0, zMm = 0.0;

    if (coordType == IC::Cartesian) {
        if (!cart.enableX && !cart.enableZ) {
            failParse("At least one of Vx or Vz must be enabled");
            return;
        }

        if (cart.enableX) {
            if (!parseDoubleStrict(cart.xLbText, xLb)) { failParse("Invalid number for Vx Lowerbound"); return; }
            if (!parseDoubleStrict(cart.xUbText, xUb)) { failParse("Invalid number for Vx Upperbound"); return; }
            if (!parseDoubleStrict(cart.xMmText, xMm)) { failParse("Invalid number for X MM"); return; }
        }

        if (cart.enableZ) {
            if (!parseDoubleStrict(cart.zLbText, zLb)) { failParse("Invalid number for Vz Lowerbound"); return; }
            if (!parseDoubleStrict(cart.zUbText, zUb)) { failParse("Invalid number for Vz Upperbound"); return; }
            if (!parseDoubleStrict(cart.zMmText, zMm)) { failParse("Invalid number for Z MM"); return; }
        }
    } else {
        if (!parseDoubleStrict(polar.normLbText, normLb)) { failParse("Invalid number for Norm Lowerbound"); return; }
        if (!parseDoubleStrict(polar.normUbText, normUb)) { failParse("Invalid number for Norm Upperbound"); return; }
        if (!parseDoubleStrict(polar.angle1Text, angle1)) { failParse("Invalid number for Angle 1"); return; }
        if (!parseDoubleStrict(polar.angle2Text, angle2)) { failParse("Invalid number for Angle 2"); return; }
        if (!parseDoubleStrict(polar.xMmText, xMm)) { failParse("Invalid number for X MM"); return; }
        if (!parseDoubleStrict(polar.zMmText, zMm)) { failParse("Invalid number for Z MM"); return; }
        if (shortArcSpan(angle1, angle2) > 10.0) {
            failParse("Polar angle span must be <= 10 degrees");
            return;
        }
    }

    crackRunning_ = true;
    hasSearched_ = true;
    statusMsg_ = "Cracking...";
    elapsedMs_ = 0.0;
    crackStartTime_ = std::chrono::steady_clock::now();
    cancelToken_ = std::make_shared<std::atomic_bool>(false);
    const auto cancelToken = cancelToken_;

    crackFuture_ = std::async(std::launch::async, [=]() -> CrackResult {
        CrackResult result;
        try {
            const auto start = std::chrono::steady_clock::now();
            IC cracker;
            cracker.setCancelFlag(cancelToken.get());
            cracker.changeSettings(maxDepth, maxTicks, maxTransTick, allowNonEmptyBridge, resultCap);
            cracker.riskyPrune(riskyLerp);
            cracker.setEffect(speed, slowness);
            cracker.setRotation(rotation);

            if (coordType == IC::Cartesian) {
                IC::condition cond;
                cond.endAirborne = endAirborne;
                cond.allowKeys = allowKeys;

                cond.x.enabled = cart.enableX;
                cond.x.walled = cart.xWalled;
                cond.z.enabled = cart.enableZ;
                cond.z.walled = cart.zWalled;

                if (cart.enableX) {
                    cond.x.lb = xLb;
                    cond.x.ub = xUb;
                    cond.x.mm = xMm;
                }

                if (cart.enableZ) {
                    cond.z.lb = zLb;
                    cond.z.ub = zUb;
                    cond.z.mm = zMm;
                }

                result.sols = cracker.matchSpeed(cond, airtime);
            } else {
                IC::polorCond cond;
                cond.allowKeys = allowKeys;
                cond.normLb = normLb;
                cond.normUb = normUb;
                cond.angle1 = angle1;
                cond.angle2 = angle2;
                cond.endAirborne = endAirborne;
                cond.xmm = xMm;
                cond.zmm = zMm;
                cond.xWalled = polar.xWalled;
                cond.zWalled = polar.zWalled;

                result.sols = cracker.matchSpeed(cond, airtime);
            }

            const auto end = std::chrono::steady_clock::now();
            result.elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
            result.cancelled = cancelToken->load(std::memory_order_relaxed);
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

bool InputCrackerTab::mothballMarked(const std::string& mothball) const {
    return std::find(markedMothballs_.begin(), markedMothballs_.end(), mothball) != markedMothballs_.end();
}

void InputCrackerTab::setMothballMarked(const std::string& mothball, bool marked) {
    auto it = std::find(markedMothballs_.begin(), markedMothballs_.end(), mothball);
    if (marked) {
        if (it == markedMothballs_.end()) {
            markedMothballs_.push_back(mothball);
        }
        return;
    }

    if (it != markedMothballs_.end()) {
        markedMothballs_.erase(it);
    }
}

void InputCrackerTab::renderCartesianForm() {
    ImGui::Spacing();
    ImGui::Checkbox("Enable X", &cart_.enableX);
    ImGui::SameLine();
    ImGui::Checkbox("Enable Z", &cart_.enableZ);

    if (cart_.enableX) {
        ImGui::SeparatorText("X Conditions");
        drawLabeledTextInput("Vel Lowerbound:", "##xLower", 180.0f, cart_.xLbText);
        drawLabeledTextInput("Vel Upperbound:", "##xUpper", 180.0f, cart_.xUbText);
        drawLabeledTextInput("MM:", "##xMm", 80.0f, cart_.xMmText);
        ImGui::SameLine();
        if (ImGui::Button(cart_.xWalled ? "Walled##x" : "Normal##x")) {
            cart_.xWalled = !cart_.xWalled;
        }
    }

    if (cart_.enableZ) {
        ImGui::SeparatorText("Z Conditions");
        drawLabeledTextInput("Vel Lowerbound:", "##zLower", 180.0f, cart_.zLbText);
        drawLabeledTextInput("Vel Upperbound:", "##zUpper", 180.0f, cart_.zUbText);
        drawLabeledTextInput("MM:", "##zMm", 80.0f, cart_.zMmText);
        ImGui::SameLine();
        if (ImGui::Button(cart_.zWalled ? "Walled##z" : "Normal##z")) {
            cart_.zWalled = !cart_.zWalled;
        }
    }
}

void InputCrackerTab::renderPolarForm() {
    ImGui::Spacing();
    ImGui::SeparatorText("Polar Conditions");
    drawLabeledTextInput("Norm Lowerbound:", "##normLower", 180.0f, polar_.normLbText);
    drawLabeledTextInput("Norm Upperbound:", "##normUpper", 180.0f, polar_.normUbText);
    drawLabeledTextInput("Angle 1:", "##angle1", 180.0f, polar_.angle1Text);
    drawLabeledTextInput("Angle 2:", "##angle2", 180.0f, polar_.angle2Text);

    ImGui::SeparatorText("Momentum");
    drawLabeledTextInput("X MM:", "##polarXmm", 80.0f, polar_.xMmText);
    ImGui::SameLine();
    if (ImGui::Button(polar_.xWalled ? "Walled##polarX" : "Normal##polarX")) {
        polar_.xWalled = !polar_.xWalled;
    }

    drawLabeledTextInput("Z MM:", "##polarZmm", 80.0f, polar_.zMmText);
    ImGui::SameLine();
    if (ImGui::Button(polar_.zWalled ? "Walled##polarZ" : "Normal##polarZ")) {
        polar_.zWalled = !polar_.zWalled;
    }
}

void InputCrackerTab::renderInputPanel(const AppResources& resources) {
    const bool pushedUiFont = resources.uiFont != nullptr;
    if (pushedUiFont) ImGui::PushFont(resources.uiFont);

    ImGui::SeparatorText("Input Cracker");

    ImGui::Spacing();
    ImGui::Text("Speed Type:");
    ImGui::SameLine();
    if (ImGui::Button(endAirborne_ ? "Air" : "Ground")) {
        endAirborne_ = !endAirborne_;
    }

    ImGui::AlignTextToFramePadding();
    ImGui::Text("MM Airtime:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##Airtime", &mmAirtime_, 0, 0);
    ImGui::SameLine(0, 15);
    drawLabeledTextInput("Facing:", "##facing", 100.0f, rotationText_);

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Allowed Keys:");
    ImGui::SameLine();
    drawKeyToggle("W", allowKeys_.w);
    ImGui::SameLine();
    drawKeyToggle("A", allowKeys_.a);
    ImGui::SameLine();
    drawKeyToggle("S", allowKeys_.s);
    ImGui::SameLine();
    drawKeyToggle("D", allowKeys_.d);

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Speed/Slowness:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##Speed", &speed_, 0, 0);
    ImGui::SameLine(0, 5);
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##Slowness", &slowness_, 0, 0);

    ImGui::SeparatorText("Geometry");
    ImGui::SetNextItemWidth(150.0f);

    int coordIdx = static_cast<int>(coordType_);
    if (ImGui::Combo("##coordType", &coordIdx, kCoordTypes, IM_ARRAYSIZE(kCoordTypes))) {
        coordType_ = static_cast<IC::ConditionForm>(coordIdx);
    }

    if (coordType_ == IC::Cartesian) {
        renderCartesianForm();
    } else {
        renderPolarForm();
    }

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Engine Setting")) {
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Max Depth:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60.0f);
        ImGui::InputInt("##MaxDepth", &maxDepth_, 0, 0);

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Max Ticks:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60.0f);
        ImGui::InputInt("##MaxTicks", &maxTicks_, 0, 0);

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Max Transition:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60.0f);
        ImGui::InputInt("##MaxTransition", &maxTransTick_, 0, 0);

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Result Cap:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputInt("##ResultCap", &resultCap_, 0, 0);

        ImGui::Checkbox("Allow Non-Empty Bridge", &allowNonEmptyBridge_);
        ImGui::Checkbox("Risky Lerp (Recommended)", &riskyLerp_);
    }

    ImGui::Spacing();
    const ImVec4 crackButton = brighten(ImGui::GetStyle().Colors[ImGuiCol_Button], 1.08f);
    const ImVec4 crackHovered = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered], 1.12f);
    const ImVec4 crackActive = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonActive], 1.16f);
    ImGui::PushStyleColor(ImGuiCol_Button, crackButton);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, crackHovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, crackActive);
    if (ImGui::Button(crackRunning_ ? "Halt" : "Crack Inputs!", ImVec2(-1.0f, 0.0f))) {
        if (crackRunning_) {
            if (cancelToken_) cancelToken_->store(true, std::memory_order_relaxed);
        } else {
            crack();
        }
    }
    ImGui::PopStyleColor(3);

    if (pushedUiFont) ImGui::PopFont();
}

void InputCrackerTab::renderOutputPanel(const AppResources& resources) {
    const bool pushedUiFont = resources.uiFont != nullptr;
    const bool pushedCodeFont = resources.codeFont != nullptr;
    if (pushedUiFont) ImGui::PushFont(resources.uiFont);

    ImGui::SeparatorText("Result");
    if (pushedCodeFont) ImGui::PushFont(resources.codeFont);

    ImGui::Text("Status: %s", crackRunning_ ? "Cracking..." : statusMsg_.c_str());

    if (returnErrorQ_) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", errorMsg_.c_str());
    } else if (!crackRunning_) {
        ImGui::Text(
            "Elapse Time: %s",
            formatElapsedHmsMs(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double, std::milli>(elapsedMs_)
            )).c_str()
        );
    } else {
        ImGui::Text("Elapse Time: %s", formatElapsedHms(std::chrono::steady_clock::now() - crackStartTime_).c_str());
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::BeginChild("SolutionList", ImVec2(0.0f, 0.0f), false);
    if (!crackRunning_ && !returnErrorQ_ && sols_.empty() && !hasSearched_) {
        ImGui::TextDisabled("No solutions yet.");
    } else if (!crackRunning_ && !returnErrorQ_ && sols_.empty()) {
        ImGui::TextDisabled("Search finished with no solutions.");
    }

    std::vector<std::size_t> order(sols_.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](std::size_t lhsIdx, std::size_t rhsIdx) {
        const auto& lhs = sols_[lhsIdx];
        const auto& rhs = sols_[rhsIdx];

        if (resultSort_ == SortByTicks) {
            if (lhs.T != rhs.T) return lhs.T < rhs.T;
        } else if (resultSort_ == SortByTicksAirDebt) {
            const int lhsCombo = lhs.T + std::max(0, lhs.airDebt);
            const int rhsCombo = rhs.T + std::max(0, rhs.airDebt);
            if (lhsCombo != rhsCombo) return lhsCombo < rhsCombo;
        } else {
            if (lhs.depth != rhs.depth) return lhs.depth < rhs.depth;
        }

        if (lhs.depth != rhs.depth) return lhs.depth < rhs.depth;
        if (lhs.T != rhs.T) return lhs.T < rhs.T;
        if (lhs.airDebt != rhs.airDebt) return lhs.airDebt < rhs.airDebt;
        return lhsIdx < rhsIdx;
    });

    if (showMarkedOnly_) {
        order.erase(
            std::remove_if(order.begin(), order.end(), [&](std::size_t idx) {
                return !mothballMarked(sols_[idx].mothball);
            }),
            order.end()
        );
    }

    const std::string trimmedSuffixFilter = trimWhitespace(suffixFilter_);
    if (!trimmedSuffixFilter.empty()) {
        order.erase(
            std::remove_if(order.begin(), order.end(), [&](std::size_t idx) {
                return !sols_[idx].mothball.ends_with(trimmedSuffixFilter);
            }),
            order.end()
        );
    }

    if (!crackRunning_ && !returnErrorQ_ && order.empty()) {
        if (showMarkedOnly_ && !trimmedSuffixFilter.empty()) {
            ImGui::TextDisabled("No marked results match the current suffix filter.");
        } else if (showMarkedOnly_) {
            ImGui::TextDisabled("No marked results in the current list.");
        } else if (!trimmedSuffixFilter.empty()) {
            ImGui::TextDisabled("No results match the current suffix filter.");
        }
    }

    if (!returnErrorQ_ && !crackRunning_) {
        ImGui::Text("Solution(s): %zu | Shown: %zu", sols_.size(), order.size());
        static const char* kSortItems[] = {"Depth", "Length (without prejump)", "Length (with prejump)"};
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Sort By:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(240.0f);
        ImGui::Combo("##resultSort", &resultSort_, kSortItems, IM_ARRAYSIZE(kSortItems));
        ImGui::SameLine();
        if (ImGui::Button(showMarkedOnly_ ? "Show Marked" : "Show All")) {
            showMarkedOnly_ = !showMarkedOnly_;
        }
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Suffix Filter:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(260.0f);
        ImGui::InputText("##suffixFilter", &suffixFilter_);
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
    }

    for (std::size_t row = 0; row < order.size(); ++row) {
        const std::size_t i = order[row];
        const auto& sol = sols_[i];
        ImGui::PushID(static_cast<int>(i));
        const float copyButtonSize = 16.0f;
        const float copyColumnWidth = resources.copyIconTexture != 0
            ? copyButtonSize + ImGui::GetStyle().FramePadding.x * 2.0f
            : ImGui::CalcTextSize("Cp").x + ImGui::GetStyle().FramePadding.x * 2.0f;
        const bool showVx = (coordType_ == IC::Polar) || cart_.enableX;
        const bool showVz = (coordType_ == IC::Polar) || cart_.enableZ;

        bool copyClicked = false;
        bool marked = mothballMarked(sol.mothball);

        if (ImGui::BeginTable("##mothballRow", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX | ImGuiTableFlags_NoBordersInBody)) {
            ImGui::TableSetupColumn("Main", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Copy", ImGuiTableColumnFlags_WidthFixed, copyColumnWidth);
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            if (ImGui::Checkbox("##marked", &marked)) {
                setMothballMarked(sol.mothball, marked);
            }
            ImGui::SameLine();
            ImGui::AlignTextToFramePadding();
            ImGui::TextWrapped("%s", sol.mothball.c_str());

            ImGui::TableSetColumnIndex(1);
            if (resources.copyIconTexture != 0) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(1, 1, 1, 0.08f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1, 1, 1, 0.14f));
                copyClicked = ImGui::ImageButton(
                    "##copyIcon",
                    resources.copyIconTexture,
                    ImVec2(copyButtonSize, copyButtonSize),
                    ImVec2(0, 0),
                    ImVec2(1, 1),
                    ImVec4(0, 0, 0, 0),
                    ImVec4(1, 1, 1, 1)
                );
                ImGui::PopStyleColor(3);
            } else {
                copyClicked = ImGui::Button("Cp");
            }

            ImGui::EndTable();
        }

        if (copyClicked) {
            ImGui::SetClipboardText(sol.mothball.c_str());
        }

        if (showVx && showVz) {
            ImGui::Text(
                "depth = %d | t = %d (+%d) | vx = %.15g | vz = %.15g",
                sol.depth,
                sol.T,
                std::max(0, sol.airDebt),
                sol.vx,
                sol.vz
            );
        } else if (showVx) {
            ImGui::Text(
                "depth = %d | t = %d (+%d) | vx = %.15g",
                sol.depth,
                sol.T,
                std::max(0, sol.airDebt),
                sol.vx
            );
        } else if (showVz) {
            ImGui::Text(
                "depth = %d | t = %d (+%d) | vz = %.15g",
                sol.depth,
                sol.T,
                std::max(0, sol.airDebt),
                sol.vz
            );
        }

        if (row + 1 < order.size()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
        }

        ImGui::PopID();
    }

    ImGui::EndChild();
    if (pushedCodeFont) ImGui::PopFont();
    if (pushedUiFont) ImGui::PopFont();
}

} // namespace gui
