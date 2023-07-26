#pragma once

#include "fixture.hpp"
#include "math.hpp"

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

        glm::vec2 get_world_pos() const override {
            return pos + center_of_mass;
        }

        glm::vec2 get_world_point(glm::vec2 point) const override {
            glm::vec2 rot_point = fast_rotate_w_precalc(point - center_of_mass, psin, pcos);

            return (rot_point + center_of_mass) + pos;
        }

        // You must call this function when setting the rotation of the rigid body
        void set_rotation(float rot);

        void apply_angular_velocity(float velocity);
        void apply_linear_velocity(glm::vec2 velocity);
        void apply_force(glm::vec2 force);
        void apply_force_at_point(glm::vec2 force, glm::vec2 point);
        fixture_t* create_fixture(const fixture_def_t& def);
        void       destroy_fixture(fixture_t* fixture);

        bool has_fixtures() { return !fixtures.is_empty(); }
        bool is_static() { return type == body_type_static; }

        void iterate_fixtures(fixture_callback_t fixture);

        void set_position(glm::vec2 pos) {
            this->pos =  center_of_mass + pos;
        }

        void add_position(glm::vec2 add) {
            this->pos += add;
        }

    public:
        body_type_t type = (body_type_t)ptm::blatent_i32;

        float     inertia      = ptm::blatent_f;
        float     invinertia   = ptm::blatent_f;
        
        // the rotated center of mass
        glm::vec2 center_of_mass     = {ptm::blatent_f, ptm::blatent_f};
        float     mass        = ptm::blatent_f;
        float     invmass     = ptm::blatent_f;

        float     torque      = ptm::blatent_f;
        float     angular_vel = ptm::blatent_f;

        glm::vec2 linear_vel  = {ptm::blatent_f, ptm::blatent_f};
        glm::vec2 forces      = {ptm::blatent_f, ptm::blatent_f};

        world_t* world = nullptr;

    protected:
        float psin = ptm::blatent_f, pcos = ptm::blatent_f; // precalculated sin and cos

        ptm::doubly_linked_list_header_t<fixture_t> fixtures;

        // the center of mass with no average calculations applied
        glm::vec2 total_center_of_mass = {0.0f, 0.0f};

        void add_mass(glm::vec2 rel_center, float mass, float tensor);
        void remove_mass(glm::vec2 rel_center, float mass, float tensor);

        void set_zero();
        void compute_sincos();
        void compute_center_of_mass();
        void compute_invmass();
        void compute_invintertia();
    };
}