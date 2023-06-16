#include "body.hpp"

namespace kin {
    void rigid_body_t::update(float delta_time) {
        {
            glm::vec2 accel = forces * invmass;

            linear_vel += accel * delta_time;

            pos += linear_vel * delta_time;
            forces   = {0.0f, 0.0f};

        }

        {
            float accel = torque * invtensor;

            angular_vel += accel * delta_time;

            rot += angular_vel * delta_time;
            torque = 0.0f;
        }

        update_vertices();
    }

    void rigid_body_t::set_density(float new_density) {
        mass = new_density * hw * hh;
        compute_invmass();
        compute_tensor();
    }

    void rigid_body_t::apply_force(glm::vec2 force) {
        forces += force;
    };

    void rigid_body_t::apply_force_at_point(glm::vec2 force, glm::vec2 point) {
        forces += force;
        torque += cross(point, force);
    }

    void rigid_body_t::compute_invmass() {
        invmass = 1.0f / mass;
    }   

    void rigid_body_t::compute_tensor() {
        tensor    = (hw * hh) * mass;
        invtensor = 1.0f / tensor;
    }
}