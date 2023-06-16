#pragma once

#include "aabb.hpp"

namespace kin {
    struct quad_tree_element_t {
        virtual const aabb_t& get_aabb() const = 0;

        bool traversed = false;
    };

    typedef std::vector<quad_tree_element_t*, ptm::indirect_allocator_t<quad_tree_element_t*>> leaf_list_t;

    struct quad_tree_leaf_t {
        quad_tree_leaf_t(ptm::memory_pool_t<quad_tree_element_t*>& pool) 
            : objects(ptm::indirect_allocator_t(&pool)) {}

        leaf_list_t objects;

        // for collision detection
        quad_tree_leaf_t* prev = nullptr;
        quad_tree_leaf_t* next = nullptr;
    };

    struct quad_tree_node_t {
        quad_tree_node_t* parent = nullptr;
        quad_tree_node_t* children = nullptr;
        quad_tree_leaf_t* leaf = nullptr;
    };

    struct quad_tree_t : quad_tree_node_t {
        aabb_t aabb;
        float hw;

        quad_tree_leaf_t* first = nullptr;
        quad_tree_leaf_t* last   = nullptr;

        ptm::memory_pool_t<quad_tree_element_t*> element_pool{1000};
        ptm::object_pool_t<quad_tree_node_t> node_pool{1000}; 
        ptm::object_pool_t<quad_tree_leaf_t> leaf_pool{200}; 
    };

    typedef std::function<void(quad_tree_element_t& element, const leaf_list_t& objects)> search_callback_t;
    typedef std::function<void(const leaf_list_t& objects)> leaf_callback;
    typedef std::function<void(const quad_tree_node_t& node, const aabb_t& aabb)> node_callback;

    void root_leaf_list_add(quad_tree_t& root, quad_tree_leaf_t* leaf);
    void root_leaf_list_remove(quad_tree_t& root, quad_tree_leaf_t* leaf);

    void node_iterate(quad_tree_t& root, quad_tree_node_t& node, const aabb_t& aabb, node_callback callback);
    void node_insert(quad_tree_t& root, quad_tree_node_t& leaf, quad_tree_element_t& element, const aabb_t& aabb, float hw, uint32_t depth);
    void node_clear(quad_tree_t& root, quad_tree_node_t& node);
    void node_search_w_callback(quad_tree_node_t& node, const aabb_t& aabb, float hw, quad_tree_element_t& element, search_callback_t callback);

    void tree_prepare(quad_tree_t& root, glm::vec2 pos, float hw);
    void tree_insert(quad_tree_t& root, quad_tree_element_t& element);
    void tree_iterate_nodes(quad_tree_t& root, node_callback callback);
    void tree_iterate_leaves(quad_tree_t& root, leaf_callback callback);
    void tree_search_w_callback(quad_tree_t& root, quad_tree_element_t& element, search_callback_t callback);
}