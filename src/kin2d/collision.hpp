#pragma once

#include "body.hpp"

namespace kin {
    struct collision_manifold_t {
        uint8_t                  count = 0;
        std::array<glm::vec2, 2> points = {};
 
        glm::vec2 normal = {0.0f, 0.0f};
        float     depth = 0.0f;
    
        float restitution;
        float static_friction;                      
        float dynamic_friction;

        void add(glm::vec2 point) {
            if(count != points.size()) {
                points[count] = point;
                count++;
            }
        }
    };

    // test if there is a collision using Seperating Axis Theorem
    bool sat_test(obb_t& obb1, obb_t& obb2, collision_manifold_t& manifold);

    // computes the manifolds of a collision
    void compute_manifold(obb_t& obb1, obb_t& obb2, collision_manifold_t& manifold);

    // Solve a collision if it exists. Note: this will not resolve the bodies
    // linear or angular velocity if there is a collision
    bool solve_collision_if_there(fixture_t& fix1, fixture_t& fix2, collision_manifold_t& manifold);

    // resolve bodies linear and angular velocity using the impulse method
    void impulse_method(rigid_body_t& body1, rigid_body_t& body2, collision_manifold_t& manifold);

    // resolve bodies using the average of two contact points
    // note: experimental
    void impulse_method_AVG(rigid_body_t& body1, rigid_body_t& body2, collision_manifold_t& manifold);
}