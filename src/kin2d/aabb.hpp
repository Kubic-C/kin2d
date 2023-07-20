#pragma once

#include "base.hpp"

namespace kin {
    struct aabb_t {
        float min[2];
        float max[2];
    };

    // doess aabb1 collide with aabb2
    inline bool aabb_collide(const aabb_t& aabb1, const aabb_t& aabb2) {
        // tooken from box2d: https://github.com/erincatto/box2d, which is licensed under Erin Cato using the MIT license

        glm::vec2 d1, d2;
        d1 = glm::vec2(aabb2.min[0], aabb2.min[1]) - glm::vec2(aabb1.max[0], aabb1.max[1]);
        d2 = glm::vec2(aabb1.min[0], aabb1.min[1]) - glm::vec2(aabb2.max[0], aabb2.max[1]);

        if (d1.x > 0.0f || d1.y > 0.0f)
            return false;

        if (d2.x > 0.0f || d2.y > 0.0f)
            return false;

        return true;
    }

    typedef std::array<glm::vec2, 4> box_vertices_t;
    typedef std::array<glm::vec2, 2> box_normals_t;
}