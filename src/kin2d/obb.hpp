#pragma once

#include "aabb.hpp"

namespace kin {
    struct transform_t {
        transform_t();
        transform_t(glm::vec2 pos, float rot);

        virtual ~transform_t() = default;

        virtual glm::vec2 get_world_point(glm::vec2 point) const;
        virtual glm::vec2 get_world_pos() const { return pos; }
        virtual glm::vec2 get_local_pos() const { return pos; }
        virtual float     get_world_rot() const { return rot; }

        glm::vec2 pos;
        float     rot;
    };

    struct obb_t : transform_t {
    public:
        obb_t(glm::vec2 pos, float rot, float hw, float hh);

        void set_dimensions(float hw, float hh);
        float compute_mass(float density);

    public: 
        float hw;
        float hh; 

        box_vertices_t world_vertices;
        box_normals_t normals;
    };

    struct rtree_element_t : aabb_t {
        obb_t* obb;

        bool operator==(const rtree_element_t& other) {
            return obb = other.obb;
        }
    };
}