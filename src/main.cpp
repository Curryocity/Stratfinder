#include "inputFinder.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>

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
#include "../third_party/imgui/backends/imgui_impl_glfw.h"
#include "../third_party/imgui/backends/imgui_impl_opengl3.h"

using IF = inputFinder;

namespace {

const char* kWindowTitle = "Input Cracker";

struct GuiState {
    float leftWidth = 420.0f;
    IF::ConditionForm coordType = IF::Cartesian;
    IF::condition cond;
    IF::polorCond pCond;
    float rotation = 0.0f;
    int speed = 0;
    int slowness = 0;

    // engine settings
    int maxDepth = 3;
    int maxTicks = 40;
    int maxTransTick = -1;
    bool allowNonEmptyBridge = false;

    bool riskyLerp = true;
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

static const char* coordTypes[2] = {"Cartesian", "Polar"};
void inputPanel(GuiState& state) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 10.0f));
    ImGui::BeginChild("InputPanel", ImVec2(0, 0), true);
    ImGui::PopStyleVar();

    ImGui::PushFont(uiFont);

    ImGui::SeparatorText("Input Cracker");

    ImGui::Spacing();
    ImGui::AlignTextToFramePadding();
    ImGui::Text("Geometry:");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150.0f);

    int coordIdx = (int)state.coordType;
    if(ImGui::Combo("##coordType", &coordIdx, coordTypes, 2)){
        state.coordType = (IF::ConditionForm) coordIdx;
    }

    ImGui::PopFont();
    ImGui::EndChild();
}

void outputPanel(GuiState& state) {

}

} // namespace

int main() {
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

        ImGui::BeginChild("InputPanel", ImVec2(state.leftWidth, 0.0f), true);
        inputPanel(state);
        ImGui::EndChild();

        drawSplitter(state.leftWidth, minLeft, minRight);

        ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x);
        ImGui::BeginChild("OutputPanel", ImVec2(0.0f, 0.0f), true);
        outputPanel(state);
        ImGui::EndChild();

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
