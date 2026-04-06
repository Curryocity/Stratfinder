#include "gui/GuiCommon.hpp"
#include "gui/InputCrackerTab.hpp"
#include "gui/JumpFindingTab.hpp"
#include "gui/ZSolverTab.hpp"
#include "util.hpp"

#include <cstdio>
#include <filesystem>
#include <vector>

#if !defined(GL_SILENCE_DEPRECATION)
#define GL_SILENCE_DEPRECATION
#endif

#if defined(__APPLE__)
#include <ApplicationServices/ApplicationServices.h>
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif
#include <GLFW/glfw3.h>

#include "../third_party/imgui/imgui.h"
#include "../third_party/imgui/backends/imgui_impl_glfw.h"
#include "../third_party/imgui/backends/imgui_impl_opengl3.h"

namespace {

const char* kWindowTitle = "Stratfinder";
constexpr float kDefaultSplitRatio = 0.5f;

struct RGB {
    float r;
    float g;
    float b;
};

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

void initFonts(gui::AppResources& resources) {
    ImGuiIO& io = ImGui::GetIO();
    const std::filesystem::path codeFontPath{std::string("asset/fonts/JetBrainsMono-Regular.ttf")};
    const std::filesystem::path uiFontPath{std::string("asset/fonts/MinecraftRegular.otf")};

    if (std::filesystem::exists(codeFontPath)) {
        resources.codeFont = io.Fonts->AddFontFromFileTTF(codeFontPath.string().c_str(), 16.0f);
    }

    if (std::filesystem::exists(uiFontPath)) {
        resources.uiFont = io.Fonts->AddFontFromFileTTF(uiFontPath.string().c_str(), 16.0f);
    }

    if (resources.uiFont != nullptr) io.FontDefault = resources.uiFont;
}

#if defined(__APPLE__)
bool loadTextureFromPng(const std::filesystem::path& path, GLuint& outTexture) {
    if (!std::filesystem::exists(path)) return false;

    const std::string pathString = path.string();
    CFURLRef url = CFURLCreateFromFileSystemRepresentation(
        nullptr,
        reinterpret_cast<const UInt8*>(pathString.c_str()),
        static_cast<CFIndex>(pathString.size()),
        false
    );
    if (url == nullptr) return false;

    CGImageSourceRef source = CGImageSourceCreateWithURL(url, nullptr);
    CFRelease(url);
    if (source == nullptr) return false;

    CGImageRef image = CGImageSourceCreateImageAtIndex(source, 0, nullptr);
    CFRelease(source);
    if (image == nullptr) return false;

    const size_t width = CGImageGetWidth(image);
    const size_t height = CGImageGetHeight(image);
    std::vector<std::uint8_t> pixels(width * height * 4);

    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(
        pixels.data(),
        width,
        height,
        8,
        width * 4,
        colorSpace,
        static_cast<CGBitmapInfo>(kCGImageAlphaPremultipliedLast) | kCGBitmapByteOrder32Big
    );
    CGColorSpaceRelease(colorSpace);

    if (context == nullptr) {
        CGImageRelease(image);
        return false;
    }

    CGContextTranslateCTM(context, 0.0, static_cast<CGFloat>(height));
    CGContextScaleCTM(context, 1.0, -1.0);
    CGContextDrawImage(context, CGRectMake(0, 0, width, height), image);
    CGContextRelease(context);
    CGImageRelease(image);

    GLuint texture = 0;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_RGBA,
        static_cast<GLsizei>(width),
        static_cast<GLsizei>(height),
        0,
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        pixels.data()
    );
    glBindTexture(GL_TEXTURE_2D, 0);

    outTexture = texture;
    return true;
}
#endif

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

    gui::AppResources resources;
    initFonts(resources);

    GLuint copyIconTexture = 0;
#if defined(__APPLE__)
    if (loadTextureFromPng(std::filesystem::path{std::string("asset/icons/copyIcon.png")}, copyIconTexture)) {
        resources.copyIconTexture = static_cast<ImTextureID>(copyIconTexture);
    }
#endif

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    gui::InputCrackerTab inputCrackerTab;
    gui::ZSolverTab zSolverTab;
    gui::JumpFindingTab jumpFindingTab;

    float leftWidth = 0.0f;
    bool leftWidthInitialized = false;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        inputCrackerTab.pump();
        zSolverTab.pump();
        jumpFindingTab.pump();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
        ImGui::Begin(
            "Stratfinder",
            nullptr,
            ImGuiWindowFlags_NoDecoration |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoSavedSettings
        );

        auto renderPanels = [&](auto& tab) {
            const float minLeft = 260.0f;
            const float minRight = 320.0f;
            const float splitterWidth = 8.0f;
            const float availableWidth = ImGui::GetContentRegionAvail().x;
            const float maxLeft = std::max(minLeft, availableWidth - minRight - splitterWidth);
            if (!leftWidthInitialized) {
                leftWidth = std::clamp(availableWidth * kDefaultSplitRatio, minLeft, maxLeft);
                leftWidthInitialized = true;
            }

            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 10.0f));
            ImGui::BeginChild("InputPanel", ImVec2(leftWidth, 0.0f), true);
            tab.renderInputPanel(resources);
            ImGui::EndChild();
            ImGui::PopStyleVar();

            gui::drawSplitter(leftWidth, minLeft, minRight);

            ImGui::SameLine(0.0f, ImGui::GetStyle().ItemSpacing.x);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24.0f, 10.0f));
            ImGui::BeginChild("OutputPanel", ImVec2(0.0f, 0.0f), true);
            tab.renderOutputPanel(resources);
            ImGui::EndChild();
            ImGui::PopStyleVar();
        };

        if (ImGui::BeginTabBar("ToolTabs")) {
            if (ImGui::BeginTabItem("Input Cracker")) {
                renderPanels(inputCrackerTab);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("zSolver")) {
                renderPanels(zSolverTab);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Jump Finding")) {
                renderPanels(jumpFindingTab);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

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
    if (copyIconTexture != 0) {
        glDeleteTextures(1, &copyIconTexture);
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
