#pragma once

#include <cstdint>

enum class version : std::uint8_t {
    v1_8_9 = 0,
    modern = 1,
};

constexpr const char* verName(version ver) {
    switch (ver) {
        case version::v1_8_9:
            return "1.8.9";
        case version::modern:
            return "Modern";
    }
    return "1.8.9";
}

constexpr bool verSprintDelay(version ver) {
    return ver == version::v1_8_9;
}

constexpr double verInertia(version ver) {
    return ver == version::v1_8_9 ? 0.005 : 0.003;
}

constexpr int verIdx(version ver) {
    return ver == version::modern ? 1 : 0;
}

constexpr version verFromIdx(int index) {
    return index == 1 ? version::modern : version::v1_8_9;
}
