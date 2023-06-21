#pragma once

#include "fixture.hpp"

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
        rigid_body_t(world_t* world, glm::vec2 pos, float rot, body_type_t type);
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
        bool is_static() { return type == body_type_static; }

        void iterate_fixtures(fixture_callback_t fixture);

    public:
        body_type_t type = (body_type_t)ptm::blatent_i32;

        float     inertia      = ptm::blatent_f;
        float     invinertia   = ptm::blatent_f;
        
        // the rotated center of mass
        glm::vec2 rot_center_of_mass = {ptm::blatent_f, ptm::blatent_f};
        glm::vec2 center_of_mass     = {ptm::blatent_f, ptm::blatent_f};
        float     mass        = ptm::blatent_f;
        float     invmass     = ptm::blatent_f;

        float     torque      = ptm::blatent_f;
        float     angular_vel = ptm::blatent_f;

        glm::vec2 linear_vel  = {ptm::blatent_f, ptm::blatent_f};
        glm::vec2 forces      = {ptm::blatent_f, ptm::blatent_f};

    protected:
        world_t* world = nullptr;
        ptm::doubly_linked_list_header_t<fixture_t> fixtures;

        // the center of mass with no average calculations applied
        glm::vec2 total_center_of_mass = {0.0f, 0.0f};

        void add_mass(glm::vec2 rel_center, float mass, float tensor);
        void remove_mass(glm::vec2 rel_center, float mass, float tensor);

        void set_zero();
        void compute_rot_com();
        void compute_center_of_mass();
        void compute_invmass();
        void compute_invintertia();
    };
}