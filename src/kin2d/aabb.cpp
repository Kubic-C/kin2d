#include "aabb.hpp"

namespace kin {
    bool aabb_collide(const aabb_t& aabb1, const aabb_t& aabb2) {
        // tooken from box2d: https://github.com/erincatto/box2d, which is licensed under MIT

        glm::vec2 d1, d2;
        d1 = aabb2.bl - aabb1.tr;
        d2 = aabb1.bl - aabb2.tr;

        if (d1.x > 0.0f || d1.y > 0.0f)
            return false;

        if (d2.x > 0.0f || d2.y > 0.0f)
            return false;

        return true;
    }

    bool aabb_contains(const aabb_t& aabb1, const aabb_t& aabb2) {
        return aabb1.bl.x <= aabb2.bl.x && 
               aabb1.bl.y <= aabb2.bl.y &&
               aabb1.tr.x >= aabb2.tr.x &&
               aabb1.tr.y >= aabb2.tr.y;
    }
}