#pragma once

#include "fixture.hpp"

#define BODY_MAGIC 0xFFBD01F

namespace kin {
    class world_t;

    typedef std::function<void(fixture_t* fixture)> fixture_callback_t;

    enum body_type_t: int {
        body_type_static = 0,
        body_type_dynamic = 1
    };

    inline uint32_t body_count = 0;

    struct rigid_body_t : public transform_t, public ptm::doubly_linked_list_element_t {
        friend class world_t;
        friend class fixture_t;

        rigid_body_t() { assert(false); }
        rigid_body_t(world_t* world, glm::vec2 pos, float rot, body_type_t type)
            : transform_t(pos, rot), world(world), type(type), magic_number(BODY_MAGIC), body_num(++body_count) {}
        ~rigid_body_t();

        void update(float delta_time);

        glm::vec2 get_world_point(glm::vec2 point);

        void apply_angular_velocity(float velocity);
        void apply_linear_velocity(glm::vec2 velocity);
        void apply_force(glm::vec2 force);
        void apply_force_at_point(glm::vec2 force, glm::vec2 point);
        fixture_t* create_fixture(const fixture_def_t& def);
        void       destroy_fixture(fixture_t* fixture);

        bool has_fixtures() { return !fixtures.is_empty(); }

        void iterate_fixtures(fixture_callback_t fixture);

    public:
#ifndef NDEBUG
        uint32_t body_num;
        uint32_t magic_number;
#endif

        body_type_t type;

        float     intertia      = 0.0f;
        float     invintertia   = 0.0f;
        
        glm::vec2 center_of_mass = {0.0f, 0.0f};
        float     mass        = 0.0f;
        float     invmass     = 0.0f;

        float     torque      = 0.0f;
        float     angular_vel = 0.0f;

        glm::vec2 linear_vel  = {0.0f, 0.0f};
        glm::vec2 forces      = {0.0f, 0.0f};

    protected:
        world_t* world;
        ptm::doubly_linked_list_header_t<fixture_t> fixtures;

        // the center of mass with no average calculations applied
        glm::vec2 total_center_of_mass = {0.0f, 0.0f};

        void add_mass(glm::vec2 rel_center, float mass, float tensor);
        void remove_mass(glm::vec2 rel_center, float mass, float tensor);

        void compute_center_of_mass();
        void compute_invmass();
        void compute_invintertia();
    };
}