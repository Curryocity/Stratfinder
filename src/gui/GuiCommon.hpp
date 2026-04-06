#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

#include "../version.hpp"
#include "../../third_party/imgui/imgui.h"
#include "../../third_party/imgui/misc/cpp/imgui_stdlib.h"

namespace gui {

struct AppResources {
    ImFont* codeFont = nullptr;
    ImFont* uiFont = nullptr;
    ImTextureID copyIconTexture = 0;
};

inline std::string trimWhitespace(std::string text) {
    const std::size_t first = text.find_first_not_of(" \t\n\r\f\v");
    if (first == std::string::npos) return "";
    const std::size_t last = text.find_last_not_of(" \t\n\r\f\v");
    return text.substr(first, last - first + 1);
}

inline std::string formatElapsedHms(std::chrono::steady_clock::duration elapsed) {
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

inline std::string formatElapsedHmsMs(std::chrono::steady_clock::duration elapsed) {
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

inline bool parseDoubleStrict(const std::string& text, double& out) {
    if (text.empty()) return false;

    errno = 0;
    char* end = nullptr;
    const double value = std::strtod(text.c_str(), &end);

    if (end == text.c_str()) return false;
    if (errno == ERANGE) return false;
    if (end == nullptr || *end != '\0') return false;

    out = value;
    return true;
}

inline bool parseFloatStrict(const std::string& text, float& out) {
    double temp = 0.0;
    if (!parseDoubleStrict(text, temp)) return false;
    out = static_cast<float>(temp);
    return std::isfinite(out);
}

inline double normalizeDeg(double angle) {
    double out = std::fmod(angle, 360.0);
    if (out < 0.0) out += 360.0;
    return out;
}

inline double shortArcSpan(double angle1, double angle2) {
    const double a1 = normalizeDeg(angle1);
    const double a2 = normalizeDeg(angle2);
    const double cw = (a2 >= a1) ? (a2 - a1) : (a2 - a1 + 360.0);
    const double ccw = 360.0 - cw;
    return std::min(cw, ccw);
}

inline ImVec4 brighten(const ImVec4& color, float mult) {
    return {
        std::min(color.x * mult, 1.0f),
        std::min(color.y * mult, 1.0f),
        std::min(color.z * mult, 1.0f),
        color.w,
    };
}

inline void drawLabeledTextInput(const char* label, const char* id, float width, std::string& text) {
    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s", label);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(width);
    ImGui::InputText(id, &text);
}

inline bool drawVersionInput(const char* label, const char* id, version& ver) {
    static const char* kVersionItems[] = {"1.8.9", "Modern"};
    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s", label);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(170.0f);
    int selected = verIdx(ver);
    const bool changed = ImGui::Combo(id, &selected, kVersionItems, IM_ARRAYSIZE(kVersionItems));
    if (changed) {
        ver = verFromIdx(selected);
    }
    return changed;
}

inline void drawKeyToggle(const char* label, bool& enabled) {
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

inline void drawSplitter(float& leftWidth, float minLeft, float minRight) {
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

} // namespace gui
