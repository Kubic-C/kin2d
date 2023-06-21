#include "body.hpp"
#include "world.hpp"

namespace kin {
    rigid_body_t::rigid_body_t(world_t* world, glm::vec2 pos, float rot, body_type_t type)
        : transform_t(pos, rot), world(world), type(type) {

        // sets all forces, velocities, and mass to zero
        set_zero();
    }

    rigid_body_t::~rigid_body_t() {
        iterate_fixtures([&](fixture_t* fixture){
            destroy_fixture(fixture);
        });
    }

    void rigid_body_t::set_zero() {
        inertia      = 0.0f;
        invinertia   = 0.0f;
        
        center_of_mass = {0.0f, 0.0f};
        mass        = 0.0f;
        invmass     = 0.0f;

        torque      = 0.0f;
        angular_vel = 0.0f;

        linear_vel  = {0.0f, 0.0f};
        forces      = {0.0f, 0.0f};
        rot_center_of_mass = {0.0f, 0.0f};
    }

    void rigid_body_t::update(float delta_time) {
        if(is_static()) {
            // we check this because I want to insure if the user
            // ever switches the type of a body, it doesn't have a bunch
            // of built up forces that havent been cleared
            assert(forces.x == 0.0f && forces.y == 0.0f && torque == 0.0f);

            return;
        }   

        {
            linear_vel += delta_time * invmass * forces;

            pos += linear_vel * delta_time;
            forces   = {0.0f, 0.0f};
        }

        {
            angular_vel += delta_time * invinertia * torque;

            rot += angular_vel * delta_time;
            torque = 0.0f;
    
            compute_rot_com();
        }
    }

    void rigid_body_t::apply_angular_velocity(float velocity) {
        angular_vel += velocity * (float)type;
    }

    void rigid_body_t::apply_linear_velocity(glm::vec2 velocity) {
        linear_vel += velocity * (float)type;
    }

    void rigid_body_t::apply_force(glm::vec2 force) {
        forces += force * (float)type;
    };

    void rigid_body_t::apply_force_at_point(glm::vec2 force, glm::vec2 point) {
        forces += force * (float)type;
        torque += cross(point, force) * (float)type;
    }

    fixture_t* rigid_body_t::create_fixture(const fixture_def_t& def) {
        fixture_t* new_fixture = world->fixture_pool.create(1, this, def);

        return new_fixture;
    }

    void rigid_body_t::destroy_fixture(fixture_t* fixture) {
        world->fixture_pool.destroy(fixture, 1);
    }

    void rigid_body_t::iterate_fixtures(fixture_callback_t callback) {
        fixture_t* fixture = dynamic_cast<fixture_t*>(fixtures.first);
        while(fixture != nullptr) {
            // in case of the callback destroying the fixture
            // this prevents reading invalid memory compared to if we were 
            // to instead do this: fixture = fixture->next
            fixture_t* next = dynamic_cast<fixture_t*>(fixture->next);

            callback(fixture);

            fixture = next;
        }
    }

    void rigid_body_t::add_mass(glm::vec2 rel_center, float add_mass, float add_tensor) {
        total_center_of_mass += rel_center * add_mass;
        mass += add_mass;
        compute_invmass();
    
        inertia += add_tensor;
        compute_invintertia();
        compute_center_of_mass();
        compute_rot_com();
    }

    void rigid_body_t::remove_mass(glm::vec2 rel_center, float rem_mass, float rem_tensor) {
        total_center_of_mass -= rel_center * rem_mass;
        mass -= rem_mass;
        compute_invmass();
    
        inertia -= rem_tensor;
        compute_invintertia();
        compute_center_of_mass();
        compute_rot_com();
    }

    void rigid_body_t::compute_rot_com() {
        rot_center_of_mass = glm::rotate(center_of_mass, rot);
    }

    void rigid_body_t::compute_center_of_mass() {
        center_of_mass = total_center_of_mass * invmass;
    }

    void rigid_body_t::compute_invmass() {
        if(is_static()) {
            invmass = 0.0f;
            return;
        }

        invmass = 1.0f / mass;
    }   

    void rigid_body_t::compute_invintertia() {
        if(is_static()) {
            invinertia = 0.0f;
            return;
        }

        invinertia = 1.0f / inertia;
    }
}