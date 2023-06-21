#pragma once

#include "aabb.hpp"
#include "quad_tree.hpp"

namespace kin {
    struct transform_t {
        transform_t();
        transform_t(glm::vec2 pos, float rot);

        virtual ~transform_t() = default;

        glm::vec2 get_world_point(glm::vec2 point);
        virtual glm::vec2 get_world_pos() const { return pos; }
        virtual glm::vec2 get_local_pos() const { return pos; }
        virtual float     get_world_rot() const { return rot; }

        glm::vec2 pos;
        float     rot;
    };

    struct obb_t : public transform_t, public quad_tree_element_t {
    public:
        obb_t(glm::vec2 pos, float rot, float hw, float hh);

        virtual void update_vertices();
        void set_dimensions(float hw, float hh);
        const aabb_t& get_aabb() const override;
        float compute_mass(float density);

    public: 
        float hw;
        float hh; 

        box_vertices_t world_vertices;
        box_normals_t normals;
        aabb_t aabb;
    };
}