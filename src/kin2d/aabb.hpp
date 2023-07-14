#pragma once

#include "base.hpp"

namespace kin {
    struct aabb_t {
        glm::vec2 pos = {ptm::blatent_f, ptm::blatent_f};
        float     hw  = ptm::blatent_f;
        float     hh  = ptm::blatent_f;
    };

    // doess aabb1 collide with aabb2
    // this function is called up to 39 million times swith 300 shapes if ran for 15 seconds
    // so it should most definitley be inlined
    inline bool aabb_collide(const aabb_t& aabb1, const aabb_t& aabb2) {
        assert(aabb1.pos.x != ptm::blatent_f || aabb1.pos.y != ptm::blatent_f ||
                  aabb1.hw != ptm::blatent_f || aabb1.hw != ptm::blatent_f);
        assert(aabb2.pos.x != ptm::blatent_f || aabb2.pos.y != ptm::blatent_f ||
                  aabb2.hw != ptm::blatent_f || aabb2.hw != ptm::blatent_f);
        // tooken from box2d: https://github.com/erincatto/box2d, which is licensed under Erin Cato using the MIT license

        glm::vec2 d1, d2;
        d1 = (aabb2.pos - glm::vec2(aabb2.hw, aabb2.hh)) - (aabb1.pos + glm::vec2(aabb1.hw, aabb1.hh));
        d2 = (aabb1.pos - glm::vec2(aabb1.hw, aabb1.hh)) - (aabb2.pos + glm::vec2(aabb2.hw, aabb2.hh));

        if (d1.x > 0.0f || d1.y > 0.0f)
            return false;

        if (d2.x > 0.0f || d2.y > 0.0f)
            return false;

        return true;
    }

    typedef std::array<glm::vec2, 4> box_vertices_t;
    typedef std::array<glm::vec2, 2> box_normals_t;
}