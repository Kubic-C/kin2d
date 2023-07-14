#pragma once

#include "base.hpp"

namespace kin {
    inline float min(float x, float y) {
        if(x < y) {
            return x;
        } else {
            return y;
        }
    }

    inline float fast_sin(float x){
        return std::sin(x);
    }

    inline float fast_cos(float x){
        return std::cos(x);
    }

    inline glm::vec2 fast_rotate(glm::vec2 v, float a) {
        glm::vec2 result;
		const float cos = fast_cos(a);
		const float sin = fast_sin(a);

		result.x = v.x * cos - v.y * sin;
		result.y = v.x * sin + v.y * cos;
		return result;
    }

    // rotate using precalcuted sin and cos
    inline glm::vec2 fast_rotate_w_precalc(glm::vec2 v, float sin, float cos) {
        glm::vec2 result;

		result.x = v.x * cos - v.y * sin;
		result.y = v.x * sin + v.y * cos;
		return result;
    }
}