#pragma once

#include "obb.hpp"

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
        fixture_t(rigid_body_t* body, int relement_id, const fixture_def_t& def);
        ~fixture_t();

        // del_mass_from_body is for internal use only, do not override
        void set_density(float density, bool del_mass_from_body = true);
        virtual glm::vec2 get_local_pos() const override { return pos; }
        virtual glm::vec2 get_world_pos() const override;
        virtual float     get_world_rot() const override;
        void update_vertices();

        float tensor           = ptm::blatent_f;
        float mass             = ptm::blatent_f;
        float restitution      = ptm::blatent_f;
        float density          = ptm::blatent_f;
        float static_friction  = ptm::blatent_f;
        float dynamic_friction = ptm::blatent_f;
        int   qt_id            = ptm::blatent_i32;

        rigid_body_t* body;
        int relement_id;
    };
}