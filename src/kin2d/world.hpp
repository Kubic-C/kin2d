#pragma once

#include "body.hpp"

namespace kin {
    typedef std::function<void(kin::rigid_body_t* body)> body_callback_t;

    class world_t {
        friend class rigid_body_t;

    public:
        world_t();
        world_t(glm::vec2 gravity);
        ~world_t();

        // create a rigid body
        rigid_body_t* create_rigid_body(glm::vec2 pos, float rot, body_type_t type);

        // destroy a rigid body
        void destroy_rigid_body(rigid_body_t* body);

        // set the dimensions of a quad tree  
        void set_tree_dimensions(glm::vec2 pos, float hw);

        // update all objects in the quad tree
        void update(float delta_time, uint32_t iterations);

        // iterate through all bodies using a function
        void iterate_bodies(body_callback_t callback);

        // get the quad tree of world
        quad_tree_t& quad_tree() { return root; };

        // get the last body created
        // for debug purposes
        rigid_body_t* last_body() { return dynamic_cast<rigid_body_t*>(bodies.last); }

    private:
        void solve_collisions();

        ptm::doubly_linked_list_header_t<rigid_body_t> bodies;

        ptm::object_pool_t<rigid_body_t> body_pool;
        ptm::object_pool_t<fixture_t>    fixture_pool;

        glm::vec2 root_pos = {0.0f, 0.0f};
        float     root_hw  = 1000.0f;

        quad_tree_t root;
        glm::vec2 gravity = {ptm::blatent_f, ptm::blatent_f};
    };
}