#pragma once

#include <cstddef>
#include <array>

namespace type_tool {
template <size_t N>
std::array<double, N> float_array_to_double(const std::array<float, N>& float_arr) {
    std::array<double, N> double_arr{};
    for (size_t i = 0; i < N; ++i) {
        double_arr[i] = static_cast<double>(float_arr[i]);
    }
    return double_arr;
}
}
