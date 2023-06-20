#pragma once

#include <ptm/portem.hpp>
#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>

namespace kin {
    void print_test();

    constexpr float float_infinity = std::numeric_limits<float>::infinity();
    constexpr float float_max = std::numeric_limits<float>::max();
    constexpr float float_min = std::numeric_limits<float>::min();

    inline float cross(glm::vec2 a, glm::vec2 b) {
        return a.x * b.y - a.y * b.x;
    }

    inline bool nearly_equal(float a, float b, float max = 0.0001f) {
        return glm::abs(a - b) < max;
    }

    inline bool nearly_equal(glm::vec2 a, glm::vec2 b, float max = 0.0001f) {
        return glm::abs(a.x - b.x) < max && glm::abs(a.y - b.y) < max;
    }

    inline float sqaure(float value) {
        return value * value;
    }
}