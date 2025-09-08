#ifndef ROBOTCONTROL_COMMON_UTILS_H
#define ROBOTCONTROL_COMMON_UTILS_H

#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <cctype>
#include <cmath>
#include "unitree/idl/go2/WirelessController_.hpp"


// Trim leading/trailing whitespace
inline std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

// Parse comma-separated list into fixed-size array
template <typename T, std::size_t N>
inline bool parse_list(const std::string& v, std::array<T, N>& out) {
    std::array<T, N> tmp{};
    std::istringstream iss(v);
    std::string tok;
    std::size_t i = 0;
    while (std::getline(iss, tok, ',')) {
        if (i >= N) return false;
        std::istringstream vs(trim(tok));
        T val{};
        if (!(vs >> val)) return false;
        tmp[i++] = val;
        std::cout << val << ", ";
    }
    std::cout << std::endl;
    if (i != N) return false;
    out = tmp;
    return true;
}

// Compute inverse mapping (robot index -> policy index) from policy->robot array
template <std::size_t N>
inline void compute_pol2rob_from_sdk2policy(const std::array<int, N>& sdk2policy,
                                         std::array<int, N>& pol2rob) {
    for (std::size_t i = 0; i < N; ++i) pol2rob[i] = static_cast<int>(i);
    for (std::size_t pi = 0; pi < N; ++pi) {
        int ri = sdk2policy[pi];
        if (ri >= 0 && ri < static_cast<int>(N)) {
            pol2rob[ri] = static_cast<int>(pi);
        } else {
            std::cerr << "[SDK-UTILS] joint index out of range in joint_idx_sdk2policy at pi="
                      << pi << " val=" << ri << " (keeping identity)\n";
        }
    }
}

// Overload for std::vector
inline void compute_pol2rob_from_sdk2policy(const std::vector<int>& sdk2policy,
                                         std::vector<int>& pol2rob) {
    const size_t n = sdk2policy.size();
    pol2rob.resize(n);
    for (size_t i = 0; i < n; ++i) pol2rob[i] = static_cast<int>(i);
    for (size_t pi = 0; pi < n; ++pi) {
        int ri = sdk2policy[pi];
        if (ri >= 0 && ri < static_cast<int>(n)) {
            pol2rob[ri] = static_cast<int>(pi);
        } else {
            std::cerr << "[SDK-UTILS] joint index out of range in joint_idx_sdk2policy at pi="
                      << pi << " val=" << ri << " (keeping identity)\n";
        }
    }
}

namespace unitree::common
{
    // union for keys
    typedef union
    {
        struct
        {
            uint8_t R1 : 1;
            uint8_t L1 : 1;
            uint8_t start : 1;
            uint8_t select : 1;
            uint8_t R2 : 1;
            uint8_t L2 : 1;
            uint8_t F1 : 1;
            uint8_t F2 : 1;
            uint8_t A : 1;
            uint8_t B : 1;
            uint8_t X : 1;
            uint8_t Y : 1;
            uint8_t up : 1;
            uint8_t right : 1;
            uint8_t down : 1;
            uint8_t left : 1;
        } components;
        uint16_t value;
    } xKeySwitchUnion;

    // single button class
    class Button
    {
    public:
        Button() {}

        void update(bool state)
        {
            on_press = state ? state != pressed : false;
            on_release = state ? false : state != pressed;
            pressed = state;
        }

        bool pressed = false;
        bool on_press = false;
        bool on_release = false;
    };

    // full gamepad
    class Gamepad
    {
    public:
        Gamepad() {}

        void Update(unitree_go::msg::dds_::WirelessController_ &key_msg)
        {
            // update stick values with smooth and deadzone
            lx = lx * (1 - smooth) + (std::fabs(key_msg.lx()) < dead_zone ? 0.0 : key_msg.lx()) * smooth;
            rx = rx * (1 - smooth) + (std::fabs(key_msg.rx()) < dead_zone ? 0.0 : key_msg.rx()) * smooth;
            ry = ry * (1 - smooth) + (std::fabs(key_msg.ry()) < dead_zone ? 0.0 : key_msg.ry()) * smooth;
            ly = ly * (1 - smooth) + (std::fabs(key_msg.ly()) < dead_zone ? 0.0 : key_msg.ly()) * smooth;

            // update button states
            key.value = key_msg.keys();

            R1.update(key.components.R1);
            L1.update(key.components.L1);
            start.update(key.components.start);
            select.update(key.components.select);
            R2.update(key.components.R2);
            L2.update(key.components.L2);
            F1.update(key.components.F1);
            F2.update(key.components.F2);
            A.update(key.components.A);
            B.update(key.components.B);
            X.update(key.components.X);
            Y.update(key.components.Y);
            up.update(key.components.up);
            right.update(key.components.right);
            down.update(key.components.down);
            left.update(key.components.left);
        }

        float smooth = 0.03;
        float dead_zone = 0.01;

        float lx = 0.;
        float rx = 0.;
        float ry = 0.;
        float ly = 0.;

        Button R1;
        Button L1;
        Button start;
        Button select;
        Button R2;
        Button L2;
        Button F1;
        Button F2;
        Button A;
        Button B;
        Button X;
        Button Y;
        Button up;
        Button right;
        Button down;
        Button left;

    private:
        xKeySwitchUnion key;
    };
} // namespace unitree::common

#endif // ROBOTCONTROL_COMMON_UTILS_H