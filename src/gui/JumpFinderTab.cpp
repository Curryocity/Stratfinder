#include "JumpFinderTab.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <sstream>
#include <utility>

namespace gui {

namespace {

constexpr double kPixel = 0.0625;
constexpr double kWaterShift = zSolver::normal - 0.001;

std::string solverStratName(int stratType) {
    return stratType < 0 ? "N/A" : zSolver::strat2string(stratType);
}

long long ceilPixelIndex(double mm) {
    return static_cast<long long>(std::ceil(mm / kPixel - 1e-12));
}

long long floorPixelIndex(double mm) {
    return static_cast<long long>(std::floor(mm / kPixel + 1e-12));
}

std::string formatValue(double value) {
    std::ostringstream out;
    out << std::setprecision(15) << value;
    return out.str();
}

} // namespace

void JumpFinderTab::pump() {
    if (!searchRunning_ || !searchFuture_.valid()) return;
    if (searchFuture_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return;

    try {
        SearchResult result = searchFuture_.get();
        matches_ = std::move(result.matches);
        errorMsg_ = std::move(result.errorMsg);
        returnErrorQ_ = result.returnErrorQ;
        elapsedMs_ = result.elapsedMs;
        processedCandidates_ = result.processedCandidates;
        totalCandidates_ = result.totalCandidates;
        matchesFound_ = result.matchesFound;
        searchVer_ = result.ver;
        searchF45Type_ = result.f45Type;
        if (result.cancelled) {
            statusMsg_ = "Halted";
        } else if (result.capReached) {
            statusMsg_ = "Match Cap Reached";
        } else {
            statusMsg_ = returnErrorQ_ ? "Search Failed" : "Finished";
        }
    } catch (const std::exception& e) {
        matches_.clear();
        errorMsg_ = e.what();
        returnErrorQ_ = true;
        elapsedMs_ = 0.0;
        processedCandidates_ = 0;
        totalCandidates_ = 0;
        matchesFound_ = 0;
        searchF45Type_ = f45Type_;
        statusMsg_ = "Search Failed";
    } catch (...) {
        matches_.clear();
        errorMsg_ = "Unknown error";
        returnErrorQ_ = true;
        elapsedMs_ = 0.0;
        processedCandidates_ = 0;
        totalCandidates_ = 0;
        matchesFound_ = 0;
        searchF45Type_ = f45Type_;
        statusMsg_ = "Search Failed";
    }

    searchRunning_ = false;
    progress_.reset();
    cancelToken_.reset();
}

void JumpFinderTab::startSearch(version ver) {
    if (searchRunning_) return;

    matches_.clear();
    errorMsg_.clear();
    returnErrorQ_ = false;
    processedCandidates_ = 0;
    totalCandidates_ = 0;
    matchesFound_ = 0;

    const auto minMmText = minMmText_;
    const auto maxMmText = maxMmText_;
    const int minMmAirtime = minMmAirtime_;
    const int maxMmAirtime = maxMmAirtime_;
    const int maxJumpAirtime = maxJumpAirtime_;
    const int minSpeed = minSpeed_;
    const int maxSpeed = maxSpeed_;
    const int minSlowness = minSlowness_;
    const int maxSlowness = maxSlowness_;
    const int f45Type = f45Type_;
    const int matchCap = matchCap_;
    const auto thresholdText = thresholdText_;
    const bool useBlockageShift = useBlockageShift_;
    const bool useLadderShift = useLadderShift_;
    const bool useNormalShift = useNormalShift_;
    const bool useWaterShift = useWaterShift_;
    const bool backwalled = backwalled_;

    auto failParse = [&](const std::string& msg) {
        errorMsg_ = msg;
        returnErrorQ_ = true;
        statusMsg_ = "Search Failed";
        hasSearched_ = true;
    };

    double minMm = 0.0;
    double maxMm = 0.0;
    double threshold = 0.0;
    if (!parseDoubleStrict(minMmText, minMm)) {
        failParse("Invalid number for Min MM");
        return;
    }
    if (!parseDoubleStrict(maxMmText, maxMm)) {
        failParse("Invalid number for Max MM");
        return;
    }
    if (!parseDoubleStrict(thresholdText, threshold)) {
        failParse("Invalid number for Threshold");
        return;
    }

    if (backwalled) {
        if (minMm < 0.0 || maxMm >= 60000000.0) {
            failParse("Backwalled MM range must satisfy 0 <= MM < 60000000");
            return;
        }
    } else if (minMm < -0.6 || maxMm >= 60000000.0) {
        failParse("MM range must satisfy -0.6 <= MM < 60000000");
        return;
    }

    if (minMm > maxMm) {
        failParse("Min MM must be <= Max MM");
        return;
    }

    if (minMmAirtime < 2 || maxMmAirtime < 2) {
        failParse("MM Airtime range must be at least 2");
        return;
    }
    if (minMmAirtime > maxMmAirtime) {
        failParse("Min MM Airtime must be <= Max MM Airtime");
        return;
    }

    if (maxJumpAirtime < 2) {
        failParse("Max Jump Airtime must be at least 2");
        return;
    }

    if (minSpeed < 0 || maxSpeed > 255 || minSpeed > maxSpeed) {
        failParse("Speed range must satisfy 0 <= min <= max <= 255");
        return;
    }
    if (minSlowness < 0 || maxSlowness > 6 || minSlowness > maxSlowness) {
        failParse("Slowness range must satisfy 0 <= min <= max <= 6");
        return;
    }
    if (matchCap <= 0) {
        failParse("Match cap must be positive");
        return;
    }
    if (threshold <= 0.0 || threshold > kPixel) {
        failParse("Threshold must satisfy 0 < threshold <= 0.0625");
        return;
    }

    std::vector<std::pair<std::string, double>> selectedShifts;
    if (useBlockageShift) selectedShifts.push_back({"Blockage", ZS::blockage});
    if (useLadderShift) selectedShifts.push_back({"Ladder", ZS::ladder});
    if (useNormalShift) selectedShifts.push_back({"Normal", ZS::normal});
    if (useWaterShift) selectedShifts.push_back({"Water", kWaterShift});

    if (selectedShifts.empty()) {
        failParse("Select at least one shift");
        return;
    }

    const long long minPixel = ceilPixelIndex(minMm);
    const long long maxPixel = floorPixelIndex(maxMm);
    if (minPixel > maxPixel) {
        failParse("MM range must contain at least one 0.0625-aligned value");
        return;
    }

    const long long mmCount = maxPixel - minPixel + 1;
    const long long speedCount = static_cast<long long>(maxSpeed - minSpeed + 1);
    const long long slownessCount = static_cast<long long>(maxSlowness - minSlowness + 1);
    const long long mmAirtimeCount = static_cast<long long>(maxMmAirtime - minMmAirtime + 1);
    const long long totalCandidates = speedCount * slownessCount * mmAirtimeCount * mmCount;

    searchRunning_ = true;
    hasSearched_ = true;
    statusMsg_ = "Searching...";
    elapsedMs_ = 0.0;
    searchVer_ = ver;
    searchF45Type_ = f45Type;
    searchStartTime_ = std::chrono::steady_clock::now();
    cancelToken_ = std::make_shared<std::atomic_bool>(false);
    progress_ = std::make_shared<ProgressState>();
    progress_->totalCandidates.store(totalCandidates, std::memory_order_relaxed);
    progress_->currentSpeed.store(minSpeed, std::memory_order_relaxed);
    progress_->currentSlowness.store(minSlowness, std::memory_order_relaxed);
    progress_->currentMmAirtime.store(minMmAirtime, std::memory_order_relaxed);
    progress_->currentMmPixel.store(minPixel, std::memory_order_relaxed);
    progress_->phase.store(0, std::memory_order_relaxed);
    progress_->frozenElapsedMs.store(0, std::memory_order_relaxed);
    const auto cancelToken = cancelToken_;
    const auto progress = progress_;

    searchFuture_ = std::async(std::launch::async, [=]() -> SearchResult {
        SearchResult result;
        try {
            const auto start = std::chrono::steady_clock::now();
            zEngine::set45Type(f45TypeFromIdx(f45Type));
            ZS solver;
            solver.clearLog();
            solver.toggleLog(false);
            solver.setVersion(ver);
            result.ver = ver;
            result.f45Type = f45Type;

            std::vector<double> comparisonShifts;
            comparisonShifts.reserve(selectedShifts.size());
            for (const auto& shift : selectedShifts) {
                comparisonShifts.push_back(shift.second);
            }

            bool cancelled = false;
            bool capReached = false;
            for (int speed = minSpeed; speed <= maxSpeed && !cancelled && !capReached; ++speed) {
                progress->currentSpeed.store(speed, std::memory_order_relaxed);

                for (int slowness = minSlowness; slowness <= maxSlowness && !cancelled && !capReached; ++slowness) {
                    progress->currentSlowness.store(slowness, std::memory_order_relaxed);
                    solver.setEffect(speed, slowness);

                    for (int mmAirtime = minMmAirtime; mmAirtime <= maxMmAirtime; ++mmAirtime) {
                        if (cancelToken->load(std::memory_order_relaxed)) {
                            cancelled = true;
                            break;
                        }
                        if (capReached) break;

                        progress->currentMmAirtime.store(mmAirtime, std::memory_order_relaxed);
                        const bool useFastBreak = true;
                        const ZS::fullStrat maxi = solver.maxMMSolver(mmAirtime);

                        for (long long mmPixel = minPixel; mmPixel <= maxPixel; ++mmPixel) {
                            if (cancelToken->load(std::memory_order_relaxed)) {
                                cancelled = true;
                                break;
                            }
                            if (capReached) break;

                            progress->currentMmPixel.store(mmPixel, std::memory_order_relaxed);
                            const double mm = static_cast<double>(mmPixel) * kPixel;
                            const ZS::fullStrat strat = backwalled
                                ? solver.backwallSolver(mm, mmAirtime)
                                : solver.mmSolver(mm, mmAirtime);

                            for (const auto& shift : selectedShifts) {
                                ZS::JumpList jumpList;
                                if (!solver.poss(jumpList, strat, maxJumpAirtime, threshold, shift.second)) {
                                    continue;
                                }

                                result.matches.push_back({
                                    speed,
                                    slowness,
                                    mmAirtime,
                                    mm,
                                    shift.first,
                                    shift.second,
                                    ver,
                                    f45Type,
                                    strat,
                                    std::move(jumpList)
                                });
                                progress->matchesFound.store(
                                    static_cast<int>(result.matches.size()),
                                    std::memory_order_relaxed
                                );
                                if (static_cast<int>(result.matches.size()) >= matchCap) {
                                    capReached = true;
                                    break;
                                }
                            }

                            progress->processedCandidates.fetch_add(1, std::memory_order_relaxed);

                            if (
                                useFastBreak &&
                                std::abs(maxi.delaySpeed - strat.delaySpeed) < 1e-3 &&
                                solver.equalJumpListCheck(maxJumpAirtime, strat, maxi, comparisonShifts)
                            ) {
                                const long long skippedMm = maxPixel - mmPixel;
                                if (skippedMm > 0) {
                                    progress->processedCandidates.fetch_add(skippedMm, std::memory_order_relaxed);
                                }
                                break;
                            }
                        }

                        if (cancelled) break;
                        if (capReached) break;

                        if (useFastBreak && maxi.delayTick == -1) {
                            const long long skippedMmAirtimes =
                                static_cast<long long>(maxMmAirtime - mmAirtime) * mmCount;
                            if (skippedMmAirtimes > 0) {
                                progress->processedCandidates.fetch_add(
                                    skippedMmAirtimes,
                                    std::memory_order_relaxed
                                );
                            }
                            break;
                        }
                    }
                }
            }

            const auto end = std::chrono::steady_clock::now();
            result.cancelled = cancelToken->load(std::memory_order_relaxed);
            const long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            progress->frozenElapsedMs.store(elapsedMs, std::memory_order_relaxed);
            result.capReached = capReached;
            if (!result.cancelled) {
                progress->phase.store(1, std::memory_order_relaxed);
            }
            result.elapsedMs = static_cast<double>(elapsedMs);
            result.totalCandidates = progress->totalCandidates.load(std::memory_order_relaxed);
            result.processedCandidates = progress->processedCandidates.load(std::memory_order_relaxed);
            result.matchesFound = progress->matchesFound.load(std::memory_order_relaxed);
            return result;
        } catch (const std::exception& e) {
            result.errorMsg = e.what();
            result.returnErrorQ = true;
        } catch (...) {
            result.errorMsg = "Unknown error";
            result.returnErrorQ = true;
        }

        result.processedCandidates = progress->processedCandidates.load(std::memory_order_relaxed);
        result.totalCandidates = progress->totalCandidates.load(std::memory_order_relaxed);
        result.matchesFound = progress->matchesFound.load(std::memory_order_relaxed);
        return result;
    });
}

void JumpFinderTab::renderInputPanel(const AppResources& resources) {
    const bool pushedUiFont = resources.uiFont != nullptr;
    if (pushedUiFont) ImGui::PushFont(resources.uiFont);

    ImGui::SeparatorText("Jump Finding");

    ImGui::Spacing();
    drawVersionInput("Version:", "##jumpFindingVersion", selectedVer_);

    ImGui::TextWrapped(
        "This tab searches MM+jump combinations across ranges of MM, airtime, speed, slowness, and shift presets."
    );

    ImGui::SeparatorText("Ranges");
    ImGui::AlignTextToFramePadding();
    ImGui::Text("MM:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100.0f);
    ImGui::InputText("##jumpFindingMinMm", &minMmText_);
    ImGui::SameLine();
    ImGui::TextDisabled("to");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100.0f);
    ImGui::InputText("##jumpFindingMaxMm", &maxMmText_);
    ImGui::SameLine();
    if (ImGui::Button(backwalled_ ? "Backwalled##jumpFindingWallMode" : "Normal##jumpFindingWallMode")) {
        backwalled_ = !backwalled_;
    }
    ImGui::TextDisabled("MM step is fixed at 0.0625 (one pixel).");

    ImGui::AlignTextToFramePadding();
    ImGui::Text("MM Airtime:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(55.0f);
    ImGui::InputInt("##jumpFindingMinMmAirtime", &minMmAirtime_, 0, 0);
    ImGui::SameLine();
    ImGui::TextDisabled("to");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(55.0f);
    ImGui::InputInt("##jumpFindingMaxMmAirtime", &maxMmAirtime_, 0, 0);

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Speed:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(55.0f);
    ImGui::InputInt("##jumpFindingMinSpeed", &minSpeed_, 0, 0);
    ImGui::SameLine();
    ImGui::TextDisabled("to");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(55.0f);
    ImGui::InputInt("##jumpFindingMaxSpeed", &maxSpeed_, 0, 0);

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Slowness:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(55.0f);
    ImGui::InputInt("##jumpFindingMinSlowness", &minSlowness_, 0, 0);
    ImGui::SameLine();
    ImGui::TextDisabled("to");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(55.0f);
    ImGui::InputInt("##jumpFindingMaxSlowness", &maxSlowness_, 0, 0);

    if (draw45TypeInput("45 Type:", "##jumpFinding45Type", f45Type_)) {
        zEngine::set45Type(f45TypeFromIdx(f45Type_));
    }

    ImGui::SeparatorText("Jump");
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Max Jump Airtime:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(70.0f);
    ImGui::InputInt("##jumpFindingMaxJumpAirtime", &maxJumpAirtime_, 0, 0);

    drawLabeledTextInput("Threshold:", "##jumpFindingThreshold", 100.0f, thresholdText_);

    ImGui::TextDisabled("Uses the max-MM jump-list check to skip equivalent tails.");

    ImGui::SeparatorText("Shifts");
    ImGui::Checkbox("Blockage (0.0)", &useBlockageShift_);
    ImGui::Checkbox("Ladder (0.3)", &useLadderShift_);
    ImGui::Checkbox("Normal (0.6)", &useNormalShift_);
    ImGui::Checkbox("Water (normal - 0.001)", &useWaterShift_);
    ImGui::TextDisabled("Match cap: %d", matchCap_);

    ImGui::Spacing();
    const ImVec4 searchButton = brighten(ImGui::GetStyle().Colors[ImGuiCol_Button], 1.08f);
    const ImVec4 searchHovered = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered], 1.12f);
    const ImVec4 searchActive = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonActive], 1.16f);
    ImGui::PushStyleColor(ImGuiCol_Button, searchButton);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, searchHovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, searchActive);
    if (ImGui::Button(searchRunning_ ? "Halt" : "Start Jump Finding", ImVec2(-1.0f, 0.0f))) {
        if (searchRunning_) {
            if (cancelToken_) cancelToken_->store(true, std::memory_order_relaxed);
        } else {
            startSearch(selectedVer_);
        }
    }
    ImGui::PopStyleColor(3);

    if (pushedUiFont) ImGui::PopFont();
}

void JumpFinderTab::renderOutputPanel(const AppResources& resources) {
    const bool pushedUiFont = resources.uiFont != nullptr;
    const bool pushedCodeFont = resources.codeFont != nullptr;
    if (pushedUiFont) ImGui::PushFont(resources.uiFont);

    ImGui::SeparatorText("Result");
    if (pushedCodeFont) ImGui::PushFont(resources.codeFont);

    ImGui::BeginChild("JumpFindingResultList", ImVec2(0.0f, 0.0f), false);

    const long long processed = searchRunning_ && progress_
        ? progress_->processedCandidates.load(std::memory_order_relaxed)
        : processedCandidates_;
    const long long total = searchRunning_ && progress_
        ? progress_->totalCandidates.load(std::memory_order_relaxed)
        : totalCandidates_;
    const int liveMatches = searchRunning_ && progress_
        ? progress_->matchesFound.load(std::memory_order_relaxed)
        : matchesFound_;
    const bool finalizing = searchRunning_ && progress_ && progress_->phase.load(std::memory_order_relaxed) == 1;
    const long long frozenElapsedMs = searchRunning_ && progress_
        ? progress_->frozenElapsedMs.load(std::memory_order_relaxed)
        : 0;

    const double percent = total > 0 ? (100.0 * static_cast<double>(processed) / static_cast<double>(total)) : 0.0;

    if (returnErrorQ_) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", errorMsg_.c_str());
    } else if (!hasSearched_) {
        ImGui::TextDisabled("No jump-finding run yet.");
    } else {
        ImGui::Text("Status: %s", searchRunning_ ? (finalizing ? "Finalizing..." : "Searching...") : statusMsg_.c_str());
        ImGui::Text("Version: %s", verName(searchVer_));
        ImGui::Text("45 Type: %s", f45TypeName(searchF45Type_));
        if (searchRunning_) {
            if (finalizing) {
                ImGui::TextDisabled(
                    "Elapsed: %s",
                    formatElapsedHmsMs(std::chrono::milliseconds(frozenElapsedMs)).c_str()
                );
            } else {
                ImGui::TextDisabled(
                    "Elapsed: %s",
                    formatElapsedHms(std::chrono::steady_clock::now() - searchStartTime_).c_str()
                );
            }
        } else {
            ImGui::TextDisabled(
                "Elapsed: %s",
                formatElapsedHmsMs(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double, std::milli>(elapsedMs_)
                )).c_str()
            );
        }

        ImGui::Text("Progress: %.2f%% (%lld / %lld MM checks)", percent, processed, total);
        ImGui::Text("Matches Found: %d", liveMatches);

        if (searchRunning_ && progress_ && !finalizing) {
            const int currentSpeed = progress_->currentSpeed.load(std::memory_order_relaxed);
            const int currentSlowness = progress_->currentSlowness.load(std::memory_order_relaxed);
            const int currentMmAirtime = progress_->currentMmAirtime.load(std::memory_order_relaxed);
            const long long currentMmPixel = progress_->currentMmPixel.load(std::memory_order_relaxed);
            ImGui::Text(
                "Current: speed %d, slowness %d, MM airtime %d, MM %s",
                currentSpeed,
                currentSlowness,
                currentMmAirtime,
                formatValue(static_cast<double>(currentMmPixel) * kPixel).c_str()
            );
        } else if (finalizing) {
            ImGui::TextDisabled("Collecting final results...");
        }
    }

    if (!searchRunning_ && !returnErrorQ_) {
        ImGui::Spacing();
        if (matches_.empty()) {
            if (statusMsg_ == "Halted") {
                ImGui::TextDisabled("Run halted before any matches were collected.");
            } else if (statusMsg_ == "Match Cap Reached") {
                ImGui::TextDisabled("Run stopped after reaching the match cap.");
            } else {
                ImGui::TextDisabled("Search finished with no matches.");
            }
        } else {
            ImGui::Text("Match(es): %zu", matches_.size());
            if (statusMsg_ == "Match Cap Reached") {
                ImGui::TextDisabled("Stopped after reaching the match cap of %d.", matchCap_);
            }
            for (std::size_t i = 0; i < matches_.size(); ++i) {
                const SearchMatch& match = matches_[i];
                std::ostringstream title;
                title << "Sp-" << match.speed
                      << " Sl-" << match.slowness
                      << " | MM t=" << match.mmAirtime
                      << " | MM=" << formatValue(match.mm)
                      << " | " << f45TypeName(match.f45Type)
                      << " | " << match.shiftLabel
                      << " | " << match.jumpList.jumps.size() << " jump";
                if (match.jumpList.jumps.size() != 1) title << "s";
                ImGui::Spacing();
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::CollapsingHeader(title.str().c_str())) {
                    ImGui::Text("MM: %s", formatValue(match.mm).c_str());
                    ImGui::Text("MM Airtime: %d", match.mmAirtime);
                    ImGui::Text("Speed/Slowness: %d / %d", match.speed, match.slowness);
                    ImGui::Text("Version: %s", verName(match.ver));
                    ImGui::Text("Shift: %s (%s)", match.shiftLabel.c_str(), formatValue(match.shiftValue).c_str());

                    ImGui::Spacing();
                    ImGui::Text("Nondelayed");
                    ImGui::Text("- Strat Type: %s", solverStratName(match.strat.nondelayStrat).c_str());
                    ImGui::Text("- Vz: %s", formatValue(match.strat.nondelaySpeed).c_str());
                    ImGui::Text("Delayed (%dt)", match.strat.delayTick);
                    ImGui::Text("- Strat Type: %s", solverStratName(match.strat.delayStrat).c_str());
                    ImGui::Text("- Vz: %s", formatValue(match.strat.delaySpeed).c_str());

                    ImGui::Spacing();
                    ImGui::Text("Jump List");
                    for (std::size_t jumpIdx = 0; jumpIdx < match.jumpList.jumps.size(); ++jumpIdx) {
                        if (match.jumpList.firstNondelayedIdx == static_cast<int>(jumpIdx)) {
                            ImGui::TextDisabled(
                                "Nondelayed becomes better at t = %d",
                                match.jumpList.firstNondelayedTick
                            );
                        }

                        const auto& jump = match.jumpList.jumps[jumpIdx];
                        ImGui::Text(
                            "t = %d: %s + %s b",
                            jump.airtime,
                            formatValue(jump.jumpDistance).c_str(),
                            formatValue(jump.landingOffset).c_str()
                        );
                    }
                }
                ImGui::PopID();
            }
        }
    } else if (searchRunning_) {
        ImGui::Spacing();
        ImGui::TextDisabled("Matches will appear when the current run finishes or is halted.");
    }

    ImGui::EndChild();
    if (pushedCodeFont) ImGui::PopFont();
    if (pushedUiFont) ImGui::PopFont();
}

} // namespace gui
