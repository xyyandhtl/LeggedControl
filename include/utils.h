#ifndef ROBOTCONTROL_COMMON_UTILS_H
#define ROBOTCONTROL_COMMON_UTILS_H

#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <cctype>

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


#endif // ROBOTCONTROL_COMMON_UTILS_H