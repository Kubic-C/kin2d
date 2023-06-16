#pragma once

#include "base.hpp"

namespace kin {
    struct aabb_t {
        aabb_t()
            : bl(0.0f), tr(0.0f) {}
        
        aabb_t(glm::vec2 bl, glm::vec2 tr) 
            : bl(bl), tr(tr) {}

        glm::vec2 bl, tr;
    };

    // doess aabb1 collide with aabb2
    bool aabb_collide(const aabb_t& aabb1, const aabb_t& aabb2);

    // does aabb1 fully contain aabb2
    bool aabb_contains(const aabb_t& aabb1, const aabb_t& aabb2);

    typedef std::array<glm::vec2, 4> box_vertices_t;
    typedef std::array<glm::vec2, 2> box_normals_t;
}