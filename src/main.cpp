#include "inputCracker.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <vector>

#if !defined(GL_SILENCE_DEPRECATION)
#define GL_SILENCE_DEPRECATION
#endif

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif
#include <GLFW/glfw3.h>

#include "../third_party/imgui/imgui.h"
#include "../third_party/imgui/misc/cpp/imgui_stdlib.h"
#include "../third_party/imgui/backends/imgui_impl_glfw.h"
#include "../third_party/imgui/backends/imgui_impl_opengl3.h"

using IC = inputCracker;

namespace {

const char* kWindowTitle = "Input Cracker";
constexpr float kDefaultSplitRatio = 0.5f;

struct CrackResult {
    std::vector<IC::Solution> sols;
    std::string errorMsg;
    bool returnErrorQ = false;
    bool cancelled = false;
    double elapsedMs = 0.0;
};

struct GuiState {
    float leftWidth = 0.0f;
    bool leftWidthInitialized = false;
    IC::ConditionForm coordType = IC::Cartesian;
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

    int airtime = 12;
    bool endAirborne = false;
    IC::WASD allowKeys = {true, true, true, true};

    std::string rotationText = "0.0";
    int speed = 0;
    int slowness = 1;

    // engine settings
    int maxDepth = 3;
    int maxTicks = 40;
    int maxTransTick = 16;
    bool allowNonEmptyBridge = false;

    bool riskyLerp = true;

    // Output
    std::vector<IC::Solution> sols;
    std::string errorMsg;
    bool returnErrorQ = false;
    bool crackRunning = false;
    bool hasSearched = false;
    std::string statusMsg = "Idle";
    double elapsedMs = 0.0;
    std::chrono::steady_clock::time_point crackStartTime{};
    std::shared_ptr<std::atomic_bool> cancelToken;
    std::future<CrackResult> crackFuture;
};

struct RGB {
    float r;
    float g;
    float b;
};

ImFont* codeFont = nullptr;
ImFont* bigCodeFont = nullptr;
ImFont* uiFont = nullptr;

void glfwErrorCallback(int error, const char* description) {
    std::fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

RGB mix(const RGB& lhs, const RGB& rhs, float t) {
    return {
        lhs.r + (rhs.r - lhs.r) * t,
        lhs.g + (rhs.g - lhs.g) * t,
        lhs.b + (rhs.b - lhs.b) * t,
    };
}

ImVec4 rgba(const RGB& rgb, float a = 1.0f) {
    return {rgb.r, rgb.g, rgb.b, a};
}

ImVec4 scale(const RGB& rgb, float mult, float a = 1.0f) {
    return {
        std::fmin(rgb.r * mult, 1.0f),
        std::fmin(rgb.g * mult, 1.0f),
        std::fmin(rgb.b * mult, 1.0f),
        a,
    };
}

ImVec4 brighten(const ImVec4& color, float mult) {
    return {
        std::min(color.x * mult, 1.0f),
        std::min(color.y * mult, 1.0f),
        std::min(color.z * mult, 1.0f),
        color.w,
    };
}

std::string formatElapsedHms(std::chrono::steady_clock::duration elapsed) {
    const auto totalSeconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    const long long hours = totalSeconds / 3600;
    const long long minutes = (totalSeconds % 3600) / 60;
    const long long seconds = totalSeconds % 60;

    if (hours > 0) {
        return std::to_string(hours) + "h " + std::to_string(minutes) + "m " + std::to_string(seconds) + "s";
    }
    if (minutes > 0) {
        return std::to_string(minutes) + "m " + std::to_string(seconds) + "s";
    }
    return std::to_string(seconds) + "s";
}

std::string formatElapsedHmsMs(std::chrono::steady_clock::duration elapsed) {
    const auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
    const long long hours = totalMs / 3600000;
    const long long minutes = (totalMs % 3600000) / 60000;
    const long long seconds = (totalMs % 60000) / 1000;
    const long long millis = totalMs % 1000;

    std::string out;
    if (hours > 0) out += std::to_string(hours) + "h ";
    if (hours > 0 || minutes > 0) out += std::to_string(minutes) + "m ";
    out += std::to_string(seconds) + "s ";
    out += std::to_string(millis) + "ms";
    return out;
}

void applyAccent(const RGB& accent) {
    auto& c = ImGui::GetStyle().Colors;

    const RGB dark = {0.10f, 0.10f, 0.10f};
    const RGB mid = mix(dark, accent, 0.35f);
    const RGB soft = mix(dark, accent, 0.20f);
    const RGB tabBg = mix(dark, accent, 0.28f);
    const RGB tabActive = mix(dark, accent, 0.52f);

    c[ImGuiCol_Button] = scale(accent, 0.85f, 0.85f);
    c[ImGuiCol_ButtonHovered] = scale(accent, 1.00f, 0.95f);
    c[ImGuiCol_ButtonActive] = scale(accent, 1.15f, 1.00f);

    c[ImGuiCol_Header] = scale(accent, 0.70f, 0.85f);
    c[ImGuiCol_HeaderHovered] = scale(accent, 0.85f, 0.90f);
    c[ImGuiCol_HeaderActive] = scale(accent, 1.00f, 0.95f);

    c[ImGuiCol_SliderGrab] = scale(accent, 1.10f);
    c[ImGuiCol_SliderGrabActive] = scale(accent, 1.25f);

    c[ImGuiCol_CheckMark] = scale(accent, 1.30f);
    c[ImGuiCol_NavHighlight] = scale(accent, 1.30f);

    c[ImGuiCol_Border] = scale(accent, 0.75f, 0.80f);
    c[ImGuiCol_Separator] = scale(accent, 0.75f, 0.90f);

    c[ImGuiCol_TableBorderLight] = rgba(soft, 0.65f);
    c[ImGuiCol_TableBorderStrong] = rgba(mid, 0.85f);
    c[ImGuiCol_TableRowBg] = rgba(soft, 0.60f);
    c[ImGuiCol_TableRowBgAlt] = rgba(mid, 0.60f);

    c[ImGuiCol_Tab] = rgba(tabBg, 0.90f);
    c[ImGuiCol_TabHovered] = rgba(tabActive, 0.95f);
    c[ImGuiCol_TabActive] = rgba(tabActive, 1.00f);
    c[ImGuiCol_TabUnfocused] = rgba(tabBg, 0.70f);
    c[ImGuiCol_TabUnfocusedActive] = rgba(tabActive, 0.80f);
}

void applyTheme() {
    ImGuiStyle& style = ImGui::GetStyle();

    style.WindowRounding = 7.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 5.0f;
    style.GrabRounding = 4.0f;
    style.ScrollbarRounding = 6.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.WindowPadding = ImVec2(12.0f, 10.0f);
    style.FramePadding = ImVec2(9.0f, 6.0f);
    style.ItemSpacing = ImVec2(9.0f, 8.0f);

    auto& c = style.Colors;
    c[ImGuiCol_Text] = {0.95f, 0.95f, 0.95f, 1.0f};
    c[ImGuiCol_TextDisabled] = {0.6f, 0.6f, 0.6f, 1.0f};
    c[ImGuiCol_TextSelectedBg] = {0.8f, 0.8f, 0.8f, 0.30f};

    c[ImGuiCol_WindowBg] = {0.04f, 0.04f, 0.04f, 1.0f};
    c[ImGuiCol_ChildBg] = {0.06f, 0.06f, 0.06f, 1.0f};
    c[ImGuiCol_PopupBg] = {0.10f, 0.10f, 0.10f, 1.0f};

    c[ImGuiCol_FrameBg] = {0.25f, 0.25f, 0.25f, 1.0f};
    c[ImGuiCol_FrameBgHovered] = {0.25f, 0.25f, 0.25f, 1.0f};
    c[ImGuiCol_FrameBgActive] = {0.30f, 0.30f, 0.30f, 1.0f};

    c[ImGuiCol_TitleBg] = {0.10f, 0.10f, 0.10f, 1.0f};
    c[ImGuiCol_TitleBgActive] = {0.15f, 0.15f, 0.15f, 1.0f};

    applyAccent({0.45f, 0.39f, 0.60f});
}

void initFonts() {
    ImGuiIO& io = ImGui::GetIO();
    const std::filesystem::path codeFontPath = "asset/fonts/JetBrainsMono-Regular.ttf";
    const std::filesystem::path uiFontPath = "asset/fonts/MinecraftRegular.otf";

    if (std::filesystem::exists(codeFontPath)) {
        codeFont = io.Fonts->AddFontFromFileTTF(codeFontPath.string().c_str(), 16.0f);
        bigCodeFont = io.Fonts->AddFontFromFileTTF(codeFontPath.string().c_str(), 22.0f);
    }

    if (std::filesystem::exists(uiFontPath)) {
        uiFont = io.Fonts->AddFontFromFileTTF(uiFontPath.string().c_str(), 16.0f);
    }

    if (uiFont != nullptr) io.FontDefault = uiFont;
}

void drawSplitter(float& leftWidth, float minLeft, float minRight) {
    ImGuiStyle& style = ImGui::GetStyle();
    const float splitterWidth = 8.0f;
    const float availWidth = ImGui::GetContentRegionAvail().x;
    const float maxLeft = leftWidth + availWidth - minRight - splitterWidth;
    leftWidth = std::clamp(leftWidth, minLeft, maxLeft);

    ImGui::SameLine(0.0f, style.ItemSpacing.x);
    ImGui::InvisibleButton("PanelSplitter", ImVec2(splitterWidth, -1.0f));
    if (ImGui::IsItemActive()) {
        leftWidth += ImGui::GetIO().MouseDelta.x;
        leftWidth = std::clamp(leftWidth, minLeft, maxLeft);
    }

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max = ImGui::GetItemRectMax();
    const ImU32 color = ImGui::GetColorU32(ImGui::IsItemActive() || ImGui::IsItemHovered()
        ? ImGuiCol_SeparatorActive
        : ImGuiCol_Separator);
    drawList->AddLine(
        ImVec2((min.x + max.x) * 0.5f, min.y + 4.0f),
        ImVec2((min.x + max.x) * 0.5f, max.y - 4.0f),
        color,
        2.0f
    );
}

void drawLabeledTextInput(const char* label, const char* id, float width, std::string& text) {
    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s", label);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(width);
    ImGui::InputText(id, &text);
}

void drawKeyToggle(const char* label, bool& enabled) {
    ImGui::PushStyleColor(
        ImGuiCol_Button,
        enabled ? ImGui::GetStyle().Colors[ImGuiCol_Button]
                : ImVec4(0.18f, 0.18f, 0.18f, 0.85f)
    );
    ImGui::PushStyleColor(
        ImGuiCol_ButtonHovered,
        enabled ? ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered]
                : ImVec4(0.24f, 0.24f, 0.24f, 0.95f)
    );
    ImGui::PushStyleColor(
        ImGuiCol_ButtonActive,
        enabled ? ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]
                : ImVec4(0.30f, 0.30f, 0.30f, 1.00f)
    );

    if (ImGui::Button(label, ImVec2(32.0f, 0.0f))) {
        enabled = !enabled;
    }

    ImGui::PopStyleColor(3);
}

void pumpCrack(GuiState& state) {
    if (!state.crackRunning || !state.crackFuture.valid()) return;
    if (state.crackFuture.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return;

    try {
        CrackResult result = state.crackFuture.get();
        state.sols = std::move(result.sols);
        state.errorMsg = std::move(result.errorMsg);
        state.returnErrorQ = result.returnErrorQ;
        state.elapsedMs = result.elapsedMs;
        if (result.cancelled) {
            state.statusMsg = "Halted";
        } else {
            state.statusMsg = state.returnErrorQ
                ? "Search Failed"
                : "Finished";
        }
    } catch (const std::exception& e) {
        state.sols.clear();
        state.errorMsg = e.what();
        state.returnErrorQ = true;
        state.elapsedMs = 0.0;
        state.statusMsg = "Search failed";
    } catch (...) {
        state.sols.clear();
        state.errorMsg = "Unknown error";
        state.returnErrorQ = true;
        state.elapsedMs = 0.0;
        state.statusMsg = "Search failed";
    }

    state.crackRunning = false;
}

void crack(GuiState& state) {
    if (state.crackRunning) return;

    state.sols.clear();
    state.errorMsg.clear();
    state.returnErrorQ = false;
    state.crackRunning = true;
    state.hasSearched = true;
    state.statusMsg = "Cracking...";
    state.elapsedMs = 0.0;
    state.crackStartTime = std::chrono::steady_clock::now();
    state.cancelToken = std::make_shared<std::atomic_bool>(false);

    const auto coordType = state.coordType;
    const auto allowKeys = state.allowKeys;
    const bool enableX = state.enableX;
    const bool enableZ = state.enableZ;
    const bool xWalled = state.xWalled;
    const bool zWalled = state.zWalled;
    const auto xLbText = state.xLbText;
    const auto xUbText = state.xUbText;
    const auto xMmText = state.xMmText;
    const auto zLbText = state.zLbText;
    const auto zUbText = state.zUbText;
    const auto zMmText = state.zMmText;
    const int airtime = state.airtime;
    const bool endAirborne = state.endAirborne;
    const auto rotationText = state.rotationText;
    const int speed = state.speed;
    const int slowness = state.slowness;
    const int maxDepth = state.maxDepth;
    const int maxTicks = state.maxTicks;
    const int maxTransTick = state.maxTransTick;
    const bool allowNonEmptyBridge = state.allowNonEmptyBridge;
    const bool riskyLerp = state.riskyLerp;
    const auto cancelToken = state.cancelToken;

    state.crackFuture = std::async(std::launch::async, [=]() -> CrackResult {
        CrackResult result;
        try {
            const auto start = std::chrono::steady_clock::now();
            inputCracker cracker;
            cracker.setCancelFlag(cancelToken.get());
            cracker.changeSettings(maxDepth, maxTicks, maxTransTick, allowNonEmptyBridge);
            cracker.riskyPrune(riskyLerp);
            cracker.setEffect(speed, slowness);
            cracker.setRotation(std::stod(rotationText));

            if (coordType == IC::Cartesian) {
                IC::condition cond;
                cond.endAirborne = endAirborne;
                cond.allowKeys = allowKeys;

                cond.x.enabled = enableX;
                cond.x.walled = xWalled;
                cond.z.enabled = enableZ;
                cond.z.walled = zWalled;

                if (enableX) {
                    cond.x.lb = std::stod(xLbText);
                    cond.x.ub = std::stod(xUbText);
                    cond.x.mm = std::stod(xMmText);
                }

                if (enableZ) {
                    cond.z.lb = std::stod(zLbText);
                    cond.z.ub = std::stod(zUbText);
                    cond.z.mm = std::stod(zMmText);
                }

                result.sols = cracker.matchSpeed(cond, airtime);
                const auto end = std::chrono::steady_clock::now();
                result.elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
                result.cancelled = cancelToken->load(std::memory_order_relaxed);
                return result;
            }

            result.errorMsg = "Polar crack is not wired yet.";
            result.returnErrorQ = true;
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

void cartesianGUI(GuiState& state);
void polarGUI(GuiState& state);

static const char* coordTypes[2] = {"Cartesian", "Polar"};
void inputPanel(GuiState& state) {
    ImGui::PushFont(uiFont);

    ImGui::SeparatorText("Input Cracker");

    ImGui::Spacing();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Geometry:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150.0f);

    int coordIdx = (int)state.coordType;
    if(ImGui::Combo("##coordType", &coordIdx, coordTypes, 2)){
        state.coordType = (IC::ConditionForm) coordIdx;
    }

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Speed Type:");
    ImGui::SameLine();
    if(ImGui::Button(state.endAirborne ? "Air" : "Ground")){
        state.endAirborne = !state.endAirborne;
    }

    ImGui::AlignTextToFramePadding();
    ImGui::Text("MM Airtime:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##Airtime", &state.airtime, 0, 0);
    ImGui::SameLine(0, 15);
    drawLabeledTextInput("Facing:", "##facing", 100.0f, state.rotationText);

    ImGui::AlignTextToFramePadding();
    ImGui::Text("Allowed Keys:");
    ImGui::SameLine();
    drawKeyToggle("W", state.allowKeys.w);
    ImGui::SameLine();
    drawKeyToggle("A", state.allowKeys.a);
    ImGui::SameLine();
    drawKeyToggle("S", state.allowKeys.s);
    ImGui::SameLine();
    drawKeyToggle("D", state.allowKeys.d);


    ImGui::AlignTextToFramePadding();
    ImGui::Text("Speed/Slowness:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##Speed", &state.speed, 0, 0);
    ImGui::SameLine(0, 5);
    ImGui::SetNextItemWidth(50.0f);
    ImGui::InputInt("##Slowness", &state.slowness, 0, 0);

    if(state.coordType == IC::Cartesian)
        cartesianGUI(state);
    else
        polarGUI(state);

    ImGui::Spacing();
    if (ImGui::CollapsingHeader("Engine Setting")) {
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Max Depth:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60.0f);
        ImGui::InputInt("##MaxDepth", &state.maxDepth, 0, 0);

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Max Ticks:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60.0f);
        ImGui::InputInt("##MaxTicks", &state.maxTicks, 0, 0);

        ImGui::AlignTextToFramePadding();
        ImGui::Text("Max Transition:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(60.0f);
        ImGui::InputInt("##MaxTransition", &state.maxTransTick, 0, 0);

        ImGui::Checkbox("Allow Non-Empty Bridge", &state.allowNonEmptyBridge);
        ImGui::Checkbox("Risky Lerp (Recommended)", &state.riskyLerp);
    }

    ImGui::Spacing();
    const ImVec4 crackButton = brighten(ImGui::GetStyle().Colors[ImGuiCol_Button], 1.08f);
    const ImVec4 crackHovered = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonHovered], 1.12f);
    const ImVec4 crackActive = brighten(ImGui::GetStyle().Colors[ImGuiCol_ButtonActive], 1.16f);
    ImGui::PushStyleColor(ImGuiCol_Button, crackButton);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, crackHovered);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, crackActive);
    if (ImGui::Button(state.crackRunning ? "Halt" : "Crack Inputs!", ImVec2(-1.0f, 0.0f))) {
        if (state.crackRunning) {
            if (state.cancelToken) state.cancelToken->store(true, std::memory_order_relaxed);
        } else {
        crack(state);
        }
    }
    ImGui::PopStyleColor(3);


    ImGui::PopFont();
}

void cartesianGUI(GuiState& state){
    ImGui::Spacing();
    ImGui::Checkbox("Enable X", &state.enableX);
    ImGui::SameLine();
    ImGui::Checkbox("Enable Z", &state.enableZ);

    if(state.enableX){
        ImGui::SeparatorText("X Conditions");
        drawLabeledTextInput("Vel Lowerbound:", "##xLower", 180.0f, state.xLbText);
        drawLabeledTextInput("Vel Upperbound:", "##xUpper", 180.0f, state.xUbText);
        drawLabeledTextInput("MM:", "##xMm", 80.0f, state.xMmText);
        ImGui::SameLine();
        if(ImGui::Button(state.xWalled ? "Walled##x" : "Normal##x")){
            state.xWalled = !state.xWalled;
        }
    }

    if(state.enableZ){
        ImGui::SeparatorText("Z Conditions");
        drawLabeledTextInput("Vel Lowerbound:", "##zLower", 180.0f, state.zLbText);
        drawLabeledTextInput("Vel Upperbound:", "##zUpper", 180.0f, state.zUbText);
        drawLabeledTextInput("MM:", "##zMm", 80.0f, state.zMmText);
        ImGui::SameLine();
        ImGui::SameLine();
        if(ImGui::Button(state.zWalled ? "Walled##z" : "Normal##z")){
            state.zWalled = !state.zWalled;
        }
    }
}

void polarGUI(GuiState& state){
    
}

void outputPanel(GuiState& state) {
    ImGui::PushFont(uiFont);

    const char* runningText = "Cracking...";

    ImGui::SeparatorText("Result");
    if (codeFont != nullptr) ImGui::PushFont(codeFont);

    ImGui::Text("Status: %s", state.crackRunning ? runningText : state.statusMsg.c_str());

    if (state.returnErrorQ) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", state.errorMsg.c_str());
    } else if (!state.crackRunning) {
        ImGui::Text("Solution(s): %zu", state.sols.size());
        if (state.hasSearched) {
            ImGui::Text(
                "Elapse Time: %s",
                formatElapsedHmsMs(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double, std::milli>(state.elapsedMs)
                )).c_str()
            );
        }
    } else {
        ImGui::Text(
            "Elapse Time: %s",
            formatElapsedHms(std::chrono::steady_clock::now() - state.crackStartTime).c_str()
        );
    }
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::BeginChild("SolutionList", ImVec2(0.0f, 0.0f), false);
    if (!state.crackRunning && !state.returnErrorQ && state.sols.empty() && !state.hasSearched) {
        ImGui::TextDisabled("No solutions yet.");
    } else if (!state.crackRunning && !state.returnErrorQ && state.sols.empty()) {
        ImGui::TextDisabled("Search finished with no solutions.");
    }

    for (std::size_t i = 0; i < state.sols.size(); ++i) {
        const auto& sol = state.sols[i];
        ImGui::PushID(static_cast<int>(i));


        ImGui::TextWrapped("%s", sol.mothball.c_str());

        ImGui::Text(
            "depth = %d | t = %d (+%d) | vx = %.15g | vz = %.15g",
            sol.depth,
            sol.T,
            std::max(0, sol.airDebt),
            sol.vx,
            sol.vz
        );

        if (i + 1 < state.sols.size()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
        }

        ImGui::PopID();
    }

    if (codeFont != nullptr) ImGui::PopFont();
    ImGui::EndChild();
    ImGui::PopFont();
}

} // namespace

int main() {
    util::init();
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) return 1;

    const char* glslVersion = "#version 330 core";

#if defined(__APPLE__)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 800, kWindowTitle, nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    applyTheme();
    initFonts();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    GuiState state;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        pumpCrack(state);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
        ImGui::Begin(
            "Input Cracker",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoSavedSettings
        );

        const float minLeft = 260.0f;
        const float minRight = 320.0f;
        const float splitterWidth = 8.0f;
        const float availableWidth = ImGui::GetContentRegionAvail().x;
        const float maxLeft = std::max(minLeft, availableWidth - minRight - splitterWidth);
        if (!state.leftWidthInitialized) {
            state.leftWidth = std::clamp(availableWidth * kDefaultSplitRatio, minLeft, maxLeft);
            state.leftWidthInitialized = true;
        }

        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 10.0f));
        ImGui::BeginChild("InputPanel", ImVec2(state.leftWidth, 0.0f), true);
        inputPanel(state);
        ImGui::EndChild();
        ImGui::PopStyleVar();

        drawSplitter(state.leftWidth, minLeft, minRight);

        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 10.0f));
        ImGui::BeginChild("OutputPanel", ImVec2(0.0f, 0.0f), true);
        outputPanel(state);
        ImGui::EndChild();
        ImGui::PopStyleVar();

        ImGui::End();

        ImGui::Render();
        int displayW = 0;
        int displayH = 0;
        glfwGetFramebufferSize(window, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
