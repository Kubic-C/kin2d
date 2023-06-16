#pragma once

#include "aabb.hpp"
#include "quad_tree.hpp"

namespace kin {
    struct transform_t {
        transform_t();
        transform_t(glm::vec2 pos, float rot);

        glm::vec2 pos;
        float     rot;
    };

    struct obb_t : public transform_t, public quad_tree_element_t {
        obb_t(glm::vec2 pos, float rot, float hw, float hh);

        void update_vertices();
        void set_dimensions(float hw, float hh);

        const aabb_t& get_aabb() const;;

        float hw;
        float hh;

        box_vertices_t world_vertices;
        box_normals_t normals;
        aabb_t aabb;
    };
}