#pragma once

#include "obb.hpp"

namespace kin {
    struct rigid_body_t : public obb_t {
        friend class world_t;

        rigid_body_t(glm::vec2 pos, float rot, float hw, float hh)
            : obb_t(pos, rot, hw, hh) {}

        void update(float delta_time);

        glm::vec2 get_world_point(glm::vec2 point);

        void set_density(float new_density);
        void apply_force(glm::vec2 force);
        void apply_force_at_point(glm::vec2 force, glm::vec2 point);

        float     tensor      = 1.0f;
        float     invtensor   = 1.0f;
        
        float     mass        = 1.0f;
        float     invmass     = 1.0f;

        float     torque      = 0.0f;
        float     angular_vel = 0.0f;

        glm::vec2 linear_vel  = {0.0f, 0.0f};
        glm::vec2 forces      = {0.0f, 0.0f};

        float     restitution = 0.0f;

        rigid_body_t* prev = nullptr;
        rigid_body_t* next = nullptr;
 
    protected:
        void compute_invmass();
        void compute_tensor();
    };
}