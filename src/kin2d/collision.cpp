#include "collision.hpp"
#include "world.hpp"

namespace kin {
    bool solve_collision_if_there(fixture_t& fix1, fixture_t& fix2, collision_manifold_t& manifold) {
        rigid_body_t& body1 = *fix1.body;
        rigid_body_t& body2 = *fix2.body;
        world_t* world = body1.world;

        if(!aabb_collide(world->relement(fix1.relement_id), world->relement(fix2.relement_id)))
            return false;
        
        if(!sat_test(fix1, fix2, manifold))
            return false;

        const glm::vec2 amount = (manifold.normal * manifold.depth) * 0.5f;
        glm::vec2 amount1;
        glm::vec2 amount2;

        if(body1.is_static())  {
            amount1 = {0.0f, 0.0f};
            amount2 = amount;
        } else if(body2.is_static()) {
            amount2 = {0.0f, 0.0f};
            amount1 = amount;
        } else {
            amount1 = amount;
            amount2 = amount1;
        }

        body1.pos -= amount1;
        body2.pos += amount2;   

        if(manifold.depth >= 10.0f) {
            printf("long depth %f\n", manifold.depth);
        }

        body1.iterate_fixtures([=](fixture_t* fixture){
            fixture->update_vertices();
        });
        body2.iterate_fixtures([=](fixture_t* fixture){
            fixture->update_vertices();
        });

        compute_manifold(fix1, fix2, manifold);

        manifold.restitution = glm::max(fix1.restitution, fix2.restitution);
        manifold.static_friction = (fix1.static_friction + fix2.static_friction) * 0.5f;
        manifold.dynamic_friction = (fix1.dynamic_friction + fix2.dynamic_friction) * 0.5f;

        return true;
    }
}