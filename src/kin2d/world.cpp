#include "world.hpp"
#include "collision.hpp"

namespace kin {
    world_t::world_t() 
        : gravity({0.0f, -9.81f}) {}

    world_t::world_t(glm::vec2 gravity)
        : gravity(gravity) {}

    world_t::~world_t() {
        rigid_body_t* cur = dynamic_cast<rigid_body_t*>(bodies.first);
        while(cur != nullptr) {
            rigid_body_t* next = dynamic_cast<rigid_body_t*>(cur->next);

            body_pool.destruct(cur, 1);

            cur = next;
        }

        if(root.leaf || root.children) {
            node_clear(root, root);
        }
    }

    void world_t::verify_unique_bodies() {
        iterate_bodies([&](kin::rigid_body_t* body1){
            iterate_bodies([&](kin::rigid_body_t* body2){
                if(body1 == body2)
                    return;
                
                assert(body1->body_num != body2->body_num);
            });
        });
    }

    rigid_body_t* world_t::create_rigid_body(glm::vec2 pos, float rot, body_type_t type) {
        rigid_body_t* new_body = body_pool.create(1, this, pos, rot, type);

        bodies.push_back(new_body);

        return new_body;
    }

    void world_t::destroy_rigid_body(rigid_body_t* body) {
        bodies.remove_element(body);

        body_pool.destruct(body, 1);
    }

    void world_t::set_tree_dimensions(glm::vec2 pos, float hw) {
        root_pos = pos;
        root_hw = hw;

        tree_prepare(root, pos, hw);
    }

    void solve_collisions_in_leaf(quad_tree_element_t* element, const aabb_t& leaf_aabb, leaf_list_t& fixtures) {
        fixture_t* fixture1 = (fixture_t*)element;
        
        for(auto fixture : fixtures) {
            fixture_t* fixture2 = (fixture_t*)fixture;

            if(fixture2->traversed || fixture1->body == fixture2->body) {
                continue;
            }

            collision_manifold_t manifold;
            if(solve_collision_if_there(*fixture1, *fixture2, manifold)) {
                impulse_method(*fixture1->body, *fixture2->body, manifold);
            }
        }
    }

    void world_t::update(float delta_time, uint32_t iterations) {
        float step = delta_time / (float)iterations;

        for(uint32_t i = 0; i < iterations; i++) {
            verify_unique_bodies();

            tree_clear_children(root); // clearing children from tree every update

            iterate_bodies([&](kin::rigid_body_t* body){
                if(!body->has_fixtures())
                    return;

                body->apply_linear_velocity(gravity * step);
                body->update(step);

                body->iterate_fixtures([&](fixture_t* fixture){
                    fixture->traversed = false;
                    fixture->update_vertices();
                    tree_insert(root, fixture);
                });
            });

            // Im using a method to iterate elements found here:
            // https://stackoverflow.com/questions/41946007/efficient-and-well-explained-implementation-of-a-quadtree-for-2d-collision-det
            tree_iterate_leaves(root, [&](const leaf_list_t& list){
                for(quad_tree_element_t* element : list) {
                    if(!element->traversed) {
                        tree_search_w_callback(root, element, solve_collisions_in_leaf);

                        element->traversed = true;
                    }
                }
            });
        }
    }

    void world_t::iterate_bodies(body_callback_t callback) {
        rigid_body_t* cur = dynamic_cast<rigid_body_t*>(bodies.first);
        while(cur != nullptr) {
            callback(cur);

            cur = dynamic_cast<rigid_body_t*>(cur->next);
        }
    }
}