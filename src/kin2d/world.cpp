#include "world.hpp"
#include "collision.hpp"

namespace kin {
    world_t::world_t() 
        : gravity({0.0f, -9.81f}) {
        tree_prepare(root, root_pos, root_hw);
    }

    world_t::world_t(glm::vec2 gravity)
        : gravity(gravity) {
        tree_prepare(root, root_pos, root_hw);
    }

    world_t::~world_t() {
        rigid_body_t* cur = dynamic_cast<rigid_body_t*>(bodies.first);
        while(cur != nullptr) {
            rigid_body_t* next = dynamic_cast<rigid_body_t*>(cur->next);

            body_pool.destroy(cur, 1);

            cur = next;
        }

        if(root.leaf || root.children) {
            node_clear(root, root);
        }
    }

    rigid_body_t* world_t::create_rigid_body(glm::vec2 pos, float rot, body_type_t type) {
        rigid_body_t* new_body = body_pool.create(1, this, pos, rot, type);

        bodies.push_back(new_body);

        return new_body;
    }

    void world_t::destroy_rigid_body(rigid_body_t* body) {
        bodies.remove_element(body);

        body_pool.destroy(body, 1);
    }

    void world_t::set_tree_dimensions(glm::vec2 pos, float hw) {
        root_pos = pos;
        root_hw = hw;

        tree_prepare(root, pos, hw);
    }

    void world_t::solve_collisions() {
        iterate_bodies([&](kin::rigid_body_t* body) {
            if(!body->has_fixtures() || body->is_static())
                return;

            body->iterate_fixtures([&](fixture_t* fixture){ 
                tree_search_w_callback(root, fixture, [&](quad_tree_element_t* element) {
                    fixture_t* fixture1 = (fixture_t*)fixture;
                    fixture_t* fixture2 = (fixture_t*)element;

                    if(fixture2->traversed ||
                        fixture1->body == fixture2->body) {
                        return;
                    }

                    collision_manifold_t manifold;
                    if(solve_collision_if_there(*fixture1, *fixture2, manifold)) {
                        impulse_method_AVG(*fixture1->body, *fixture2->body, manifold);
                    }
                });

                fixture->traversed = true;
            });
        });
    }

    void world_t::update(float delta_time, uint32_t iterations) {
        float step = delta_time / (float)iterations;

        for(uint32_t i = 0; i < iterations; i++) {
            tree_clear_children(root); // clearing children from tree every update

            iterate_bodies([&](kin::rigid_body_t* body){
                if(!body->has_fixtures())
                    return;

                body->apply_linear_velocity(gravity * step);
                body->update(step);

                body->iterate_fixtures([&](fixture_t* fixture){
                    fixture->reset(); // resets quad tree data

                    if(!body->is_static()) {
                        fixture->update_vertices();
                    }

                    tree_insert(root, fixture);
                });
            });

            // Im using a method to iterate elements found here:
            // https://stackoverflow.com/questions/41946007/efficient-and-well-explained-implementation-of-a-quadtree-for-2d-collision-det
            // tree_iterate_leaves(root, [&](const leaf_list_t& list){
            //     for(quad_tree_element_t* element : list) {
            //         if(!element->traversed &&
            //            !((fixture_t*)element)->body->is_static()) {
            //             tree_search_w_callback(root, element, solve_collisions_in_leaf);

            //             element->traversed = true;
            //         }
            //     }
            // });

            solve_collisions();
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