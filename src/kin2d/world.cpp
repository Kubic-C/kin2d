#include "world.hpp"
#include "collision.hpp"

namespace kin {
    world_t::world_t() 
        : gravity({0.0f, -9.81f}) {}

    world_t::world_t(glm::vec2 gravity)
        : gravity(gravity) {}

    world_t::~world_t() {
        rigid_body_t* cur = first;
        while(cur != nullptr) {
            rigid_body_t* next = cur->next;

            body_pool.destruct(cur, 1);

            cur = next;
        }
    }

    rigid_body_t& world_t::create_rigid_body(glm::vec2 pos, glm::vec2 half_size, float density) {
        rigid_body_t* new_body = body_pool.create(1, pos, half_size.x, half_size.y, density);

        new_body->set_density(density);

        add_to_list(new_body);

        return *new_body;
    }

    void world_t::destroy_rigid_body(rigid_body_t& body) {
        remove_from_list(&body);

        body_pool.destruct(&body, 1);
    }

    void world_t::set_tree_dimensions(glm::vec2 pos, float hw) {
        root_pos = pos;
        root_hw = hw;
    }

    void solve_collisions_in_leaf(quad_tree_element_t& element, const leaf_list_t& bodies) {
        for(auto body : bodies) {
            if(body->traversed || &element == body) {
                continue;
            }

            rigid_body_t& body1 = *(rigid_body_t*)&element;
            rigid_body_t& body2 = *(rigid_body_t*)body;

            collision_manifold_t manifold;
            if(solve_collision_if_there(body1, body2, manifold)) {
                impulse_method(body1, body2, manifold);
            }
        }
    }

    void world_t::update(float delta_time) {
        tree_prepare(root, root_pos, root_hw);

        rigid_body_t* cur = first;
        while(cur != nullptr) {
            cur->linear_vel += gravity;

            cur->update(delta_time);
            tree_insert(root, *cur);

            cur = cur->next;
        }

        cur = first;
        while(cur != nullptr) {
            tree_search_w_callback(root, *cur, solve_collisions_in_leaf);

            cur->traversed = true;

            cur = cur->next;
        }

        node_clear(root, root);
    }

    void world_t::add_to_list(rigid_body_t* body) {
        if(first == nullptr) {
            first = body;
            last = body;
        } else {
            rigid_body_t* old_end = last;
            old_end->next = body;
            body->prev = old_end;
            last = body;
        }
    }

    void world_t::remove_from_list(rigid_body_t* body) {
        rigid_body_t* prev = body->prev;
        rigid_body_t* next = body->next;

        if(prev)
            prev->next = next;

        if(next)
            next->prev = prev;

        if(body == first) 
            first = next;

        if(body == last)
            last = prev;
    }
}