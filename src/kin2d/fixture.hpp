#pragma once

#include "obb.hpp"

#define FIXTURE_MAGIC 0xFA1F32F

namespace kin {
    class rigid_body_t;

    struct fixture_def_t {
        float density = 1.0f;
        float restitution = 0.0f;
        float static_friction = 1.0f;
        float dynamic_friction = 1.0f;
        float hw = 1.0f, hh = 1.0f;
        glm::vec2 rel_pos = {0.0f, 0.0f};
    };

    struct fixture_t : obb_t, ptm::doubly_linked_list_element_t {
        fixture_t(rigid_body_t* body, const fixture_def_t& def);
        ~fixture_t();

        // del_mass_from_body is for internal use only, do not override
        void set_density(float density, bool del_mass_from_body = false);
        virtual glm::vec2 get_local_pos() const override { return pos; }
        virtual glm::vec2 get_world_pos() const override;
        virtual float     get_world_rot() const override;
        void update_vertices() override;

#ifndef NDEBUG
        int magic_number;
#endif

        float tensor;
        float mass;
        float restitution;
        float density;
        float static_friction;
        float dynamic_friction;

        rigid_body_t* body;
    };
}