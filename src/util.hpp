#pragma once

#include <cmath>
#include <iomanip>
#include <string>
#include <sstream>

class util{
    public:

    struct vec2D{double x = 0; double z = 0;};

    static constexpr float PIf = 3.14159265358979323846f;
    static constexpr double PId = 3.14159265358979323846264338327950288;

    inline static float SIN_TABLE[65536];

    /* MUST be called once before simulation */
    static void init(){
        for (int i = 0; i < 65536; ++i) 
            SIN_TABLE[i] = std::sin(i * PId * 2.0 / 65536.0);
    }

    static inline float sinr(float rad){
        return SIN_TABLE[(int)(rad * 10430.378f) & 65535];
    }

    static inline float sin(float deg){
        float rad = deg * PIf / 180.0f;
        return SIN_TABLE[(int)(rad * 10430.378f) & 65535];
    }

    static inline float cosr(float rad){
        return SIN_TABLE[(int)(rad * 10430.378f + 16384.0f) & 65535];
    }

    static inline float cos(float deg){
        float rad = deg * PIf / 180.0f;
        return SIN_TABLE[(int)(rad * 10430.378f + 16384.0f) & 65535];
    }

    static std::string fmt(double x) {
        std::ostringstream oss;
        if (std::abs(x) < 1e-8 && x != 0)
            oss << std::scientific << std::setprecision(8);
        else if (std::abs(x) < 1e-5)
            oss << std::fixed << std::setprecision(9);
        else
            oss << std::fixed << std::setprecision(6);
        oss << x;
        std::string s = oss.str();

        // Trim trailing zeros
        if (auto pos = s.find('e'); pos != std::string::npos) {
            // scientific notation
            auto end = s.find_last_not_of('0', pos - 1);
            if (s[end] == '.') --end;
            s.erase(end + 1, pos - end - 1);
        } else {
            // fixed notation
            auto end = s.find_last_not_of('0');
            if (end != std::string::npos && s[end] == '.')
                --end;
            s.erase(end + 1);
        }

        return s;
    }

    static std::string df(double x, int precision = 16) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision);
        oss << x;
        std::string s = oss.str();
        auto end = s.find_last_not_of('0');
        if (end != std::string::npos && s[end] == '.')
            --end;
        s.erase(end + 1);

        return s;
    }

};