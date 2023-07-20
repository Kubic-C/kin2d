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

        // update all objects in the quad tree
        void update(float delta_time, uint32_t iterations);

        // iterate through all bodies using a function
        void iterate_bodies(body_callback_t callback);

        // get the last body created
        // for debug purposes
        rigid_body_t* last_body() { return dynamic_cast<rigid_body_t*>(bodies.last); }

        // the amount of bodies in the world
        size_t count();

        // print profile
        void print_profiles(int denom = -1);

        // set gravity
        void set_gravity(glm::vec2 gravity);

        spatial::RTree<float, rtree_element_t, 2> root;
    private:
        void solve_collisions_by_linear();
        void solve_collisions_by_leaf();

        size_t body_count = 0;

        profiler_t profiler;
        float dt_total          = 0.0f;
        float clean_every       = 0.25f;

        ptm::doubly_linked_list_header_t<rigid_body_t> bodies;

        ptm::object_pool_t<rigid_body_t> body_pool = {1000};
        ptm::object_pool_t<rtree_element_t> relement_pool = {1000};
        ptm::object_pool_t<fixture_t>    fixture_pool = {1000};

        glm::vec2 gravity = {ptm::blatent_f, ptm::blatent_f};
    };
}