#include "world.hpp"
#include "collision.hpp"

namespace kin {
    world_t::world_t() 
        : gravity({0.0f, -9.81f}) {
    }

    world_t::world_t(glm::vec2 gravity)
        : gravity(gravity) {
    }

    world_t::~world_t() {
        rigid_body_t* cur = dynamic_cast<rigid_body_t*>(bodies.first);
        while(cur != nullptr) {
            rigid_body_t* next = dynamic_cast<rigid_body_t*>(cur->next);

            body_pool.destroy(cur, 1);

            cur = next;
        }
    }

    rigid_body_t* world_t::create_rigid_body(glm::vec2 pos, float rot, body_type_t type) {
        rigid_body_t* new_body = body_pool.create(1, this, pos, rot, type);

        bodies.push_back(new_body);

        body_count++;

        return new_body;
    }

    void world_t::destroy_rigid_body(rigid_body_t* body) {
        body_count--;

        bodies.remove_element(body);

        body_pool.destroy(body, 1);
    }

    void world_t::solve_collisions_by_linear() {
        iterate_bodies([&](kin::rigid_body_t* body) {
            if(!body->has_fixtures())
                return;

            body->iterate_fixtures([&](fixture_t* fixture1){ 
                root.for_each([&](const PhBoxF<2>& box, fixture_t* fixture){
                    fixture_t& fixture2 = *(fixture_t*)fixture;

                    if(fixture1->body == fixture2.body) {
                        return;
                    }
                    if(fixture1->body->is_static() && 
                       fixture2.body->is_static()) {
                        return;
                    }

                    collision_manifold_t manifold;
                    if(solve_collision_if_there(*fixture1, fixture2, manifold)) {
                        impulse_method(*fixture1->body, *fixture2.body, manifold);
                    } 
                }, FilterNoOp());
            });
        });
    }

    void world_t::update(float delta_time, uint32_t iterations) {
        float step = delta_time / (float)iterations;

        for(uint32_t i = 0; i < iterations; i++) {
            // root.clear();

            iterate_bodies([&](kin::rigid_body_t* body){
                if(!body->has_fixtures())
                    return;

                body->apply_linear_velocity(gravity * step);
                body->update(step);

                body->iterate_fixtures([&](fixture_t* fixture){
                    // Reinserting into quad tree
                    // update_vertices() changes the AABB
                    // so we must call it after removing the old AABB
                    // from the tree
                    auto old = fixture->update_vertices(); 
                
                    root.relocate(old, fixture->aabb.key());
                });
            });

            solve_collisions_by_linear();
        }
    }

    size_t world_t::count() {
        return body_count;
    }

    void world_t::iterate_bodies(body_callback_t callback) {
        rigid_body_t* cur = dynamic_cast<rigid_body_t*>(bodies.first);
        while(cur != nullptr) {
            callback(cur);

            cur = dynamic_cast<rigid_body_t*>(cur->next);
        }
    }

    void world_t::set_gravity(glm::vec2 gravity) {
        this->gravity = gravity;
    }
}