#pragma once

#include "aabb.hpp"
#include "settings.hpp"
#include "fixture.hpp"

namespace kin {
    class fixture_t;

    struct qt_element_t {
        qt_element_t() = default;
        qt_element_t(qt_element_t&& other) = default;
        qt_element_t(const qt_element_t& other) = default;

        qt_element_t& operator=(const qt_element_t& other) {
            fixture   = other.fixture;
            traversed = other.traversed; 
            return *this;
        }

        fixture_t* fixture;
        bool traversed;
    };

    struct qt_full_node_t {
        // The index of the node
        int index;

        // the depth of this node
        int depth;

        // The aabb and hw of the node
        aabb_t aabb;
    };

    struct qt_node_t {
        // Points to the first child if this node is a inner node (branch) or the first
        // element if this node is a leaf.
        int32_t first_child;

        // Stores the number of elements in the leaf or -1 if it this node is
        // not a leaf.
        int32_t count;
    };

    struct qt_element_node_t {
        // The next element in the leaf or -1 for end of list
        int next;

        // A index to the element that this node repersents
        int element;
    };

    typedef std::function<void(fixture_t*)> search_callback_t;
    typedef std::function<void(glm::vec2 pos, float w)> draw_callback_t;
    typedef ptm::small_list_t<qt_full_node_t, 128> full_node_list_t;
    typedef ptm::small_list_t<int, 128> node_list_t;


    struct qt_root_t {
    public:
        qt_root_t();
        qt_root_t(size_t expected_elements);

        void insert_element_at_node(const qt_full_node_t& node, int elt);
        int  insert(fixture_t* fixture);
        void search(const aabb_t& aabb, search_callback_t callback);
        void remove(fixture_t* element);
        void cleanup();

        void draw_tree(draw_callback_t draw);

    public:
        // Stores all the elements in the quadtree.
        ptm::free_list_t<qt_element_t> elts;

        // Stores all the element nodes in the quadtree.
        ptm::free_list_t<qt_element_node_t> elt_nodes;

        // Stores all the nodes in the quadtree. The first node in this
        // sequence is always the root.
        std::vector<qt_node_t> nodes;

        // Stores the quadtree extents.
        aabb_t root_aabb = {};

        // Stores the first free node in the quadtree to be reclaimed as 4
        // contiguous nodes at once. A value of -1 indicates that the free
        // list is empty, at which point we simply insert 4 nodes to the
        // back of the nodes array.
        int free_node = -1;
    };
}


namespace kin{
    inline void set_quad_aabbs(const aabb_t& parent, std::array<aabb_t, 4>& aabb) {
        const float hw  = parent.hw;
        const float hhw = parent.hw / 2.0f;
        aabb = {
            (aabb_t){glm::vec2(parent.pos - hhw)                     , hhw, hhw},
            (aabb_t){glm::vec2(parent.pos.x + hhw, parent.pos.y - hhw), hhw, hhw},
            (aabb_t){glm::vec2(parent.pos + hhw)                     , hhw, hhw},
            (aabb_t){glm::vec2(parent.pos.x - hhw, parent.pos.y + hhw), hhw, hhw}
        };
    }
}

namespace kin {
    inline qt_root_t::qt_root_t() {
        nodes.push_back(qt_node_t{-1, 0});
    }

    inline qt_root_t::qt_root_t(size_t expected_elements) {
        nodes.push_back(qt_node_t{-1, 0});

        nodes.reserve(expected_elements * settings.max_tree_depth);
        elts.reserve(expected_elements);
        elt_nodes.reserve(expected_elements * settings.max_tree_depth + expected_elements);
    }

    inline void qt_root_t::insert_element_at_node(const qt_full_node_t& node, int elt) {
        qt_element_t& element = elts[elt];
        
        full_node_list_t to_process;
        to_process.emplace_back(node);

        while(to_process.size() > 0) {
            qt_full_node_t full_node = to_process.pop_back();
            qt_node_t* node = &nodes[full_node.index];

            if(node->count != -1) {
                if(node->count + 1 > settings.max_elements_in_leaf && 
                   full_node.depth < settings.max_tree_depth) {
                    std::array<aabb_t, 4> aabbs;
                    set_quad_aabbs(full_node.aabb, aabbs);
                    int old_elements = nodes[full_node.index].first_child;

                    // The switch from using 
                    // | qt_node_t* node | to | nodes[full_node.index] |
                    // is because when inserting and breaking up the leaf
                    // its possible that the nodes array may resize
                    // to accomadate more nodes while we are still using node
                    // therefore making the pointer a wild pointer.

                    if(free_node == -1) {
                        nodes[full_node.index].first_child = nodes.size();
                        nodes.push_back({-1, 0});
                        nodes.push_back({-1, 0});
                        nodes.push_back({-1, 0});
                        nodes.push_back({-1, 0});
                    } else {
                        nodes[full_node.index].first_child = free_node;
                        free_node        = nodes[free_node].first_child;

                        for(uint32_t i = 0; i < 4; i++) {
                            nodes[nodes[full_node.index].first_child + i].first_child = -1;
                            nodes[nodes[full_node.index].first_child + i].count = 0;
                        }
                    }

                    // insert the leaf's elements into the children
                    int cur = old_elements;
                    while(cur != -1) {
                        int elt  = elt_nodes[cur].element;
                        int next = elt_nodes[cur].next;
                        elt_nodes.erase(cur); 

                        for(int i = 0; i < 4; i++) {
                            // Although I am recursively calling this function
                            // it will not be nested more the twice.
                            if(aabb_collide(aabbs[i], elts[elt].fixture->aabb)) {
                                insert_element_at_node(qt_full_node_t{nodes[full_node.index].first_child + i, full_node.depth + 1, aabbs[i]}, elt);
                            }
                        }

                        cur = next;
                    }
                    // we need this the node to be proccessed again
                    // since its no longer a leaf
                    nodes[full_node.index].count = -1;
                    to_process.emplace_back(full_node);
                } else {
                    int new_first_element = elt_nodes.insert({node->first_child, elt});
                    node->first_child = new_first_element;     
                    node->count++;               
                }
            } else {
                std::array<aabb_t, 4> aabbs;
                set_quad_aabbs(full_node.aabb, aabbs);

                for(int i = 0; i < 4; i++) {
                    if(aabb_collide(aabbs[i], element.fixture->aabb)) {
                        to_process.emplace_back(qt_full_node_t{node->first_child + i, full_node.depth + 1, aabbs[i]});
                    }
                }
            }
        }
    }

    inline int qt_root_t::insert(fixture_t* fixture) {
        qt_element_t add;
        add.fixture   = fixture;
        add.traversed = false;
        fixture->qt_id = elts.insert(add);

        insert_element_at_node({0, 0, root_aabb}, fixture->qt_id);

        return fixture->qt_id;
    }

    inline void qt_root_t::remove(fixture_t* fixture) {
        full_node_list_t to_process;
        int elt_id = fixture->qt_id;

        to_process.emplace_back({0, 0, root_aabb});

        while(to_process.size() > 0) {
            qt_full_node_t full_node = to_process.pop_back();
            qt_node_t* node = &nodes[full_node.index];

            if(node->count != -1) {
                int cur = node->first_child;
                int prev = -1;
                int next = -1;
                bool rem = false;

                while(cur != -1) {
                    next = elt_nodes[cur].next;

                    if(elt_nodes[cur].element == elt_id) {
                        elt_nodes.erase(cur);

                        rem = true;

                        if(prev > -1) {
                            elt_nodes[prev].next = next;
                        } else {
                            // This node was first_child. update first_child to next
                            node->first_child = next;
                        }

                        break;
                    }

                    prev = cur;
                    cur = next;
                }

                if(!rem) {
                    printf("major error\n");
                    throw 694204004;
                }

                node->count--;
            } else {
                std::array<aabb_t, 4> aabbs;
                set_quad_aabbs(full_node.aabb, aabbs);

                for(int i = 0; i < 4; i++) {
                    if(aabb_collide(aabbs[i], fixture->aabb)) {
                        to_process.emplace_back(qt_full_node_t{node->first_child + i, full_node.depth + 1, aabbs[i]});
                    }
                }
            }
        }

        elts.erase(elt_id);
        fixture->qt_id = -1;
    }

    inline void qt_root_t::search(const aabb_t& aabb, search_callback_t callback) {
        full_node_list_t to_process;

        to_process.emplace_back({0, 0, root_aabb});

        while(to_process.size() > 0) {
            qt_full_node_t full_node = to_process.pop_back();
            qt_node_t* node = &nodes[full_node.index];

            if(node->count != -1) {
                int cur = node->first_child;
                while(cur != -1) {
                    callback(elts[elt_nodes[cur].element].fixture);

                    cur = elt_nodes[cur].next;
                }
            } else {
                std::array<aabb_t, 4> aabbs;
                set_quad_aabbs(full_node.aabb, aabbs);

                for(int i = 0; i < 4; i++) {
                    if(aabb_collide(aabbs[i], aabb)) {
                        to_process.emplace_back(qt_full_node_t{node->first_child + i, full_node.depth + 1, aabbs[i]});
                    }
                }
            }
        }
    }

    inline void qt_root_t::cleanup() {
        // Only process the root if it's not a leaf.
        node_list_t to_process;
        if (nodes[0].count == -1)
            to_process.emplace_back(0);

        while(to_process.size() > 0) {
            const int node_index = to_process.pop_back();
            qt_node_t& node = nodes[node_index];

            // Loop through the children.
            int num_empty_leaves = 0;
            for (int j = 0; j < 4; ++j)
            {
                const int child_index = node.first_child + j;
                const qt_node_t& child = nodes[child_index];

                // Increment empty leaf count if the child is an empty 
                // leaf. Otherwise if the child is a branch, add it to
                // the stack to be processed in the next iteration.
                if (child.count == 0)
                    ++num_empty_leaves;
                else if (child.count == -1)
                    to_process.emplace_back(child_index);
            }

            // If all the children were empty leaves, remove them and 
            // make this node the new empty leaf.
            if (num_empty_leaves == 4)
            {
                // Push all 4 children to the free list.
                nodes[node.first_child].first_child = free_node;
                free_node = node.first_child;

                // Make this node the new empty leaf.
                node.first_child = -1;
                node.count = 0;
            }
        }
    }

    inline void qt_root_t::draw_tree(draw_callback_t draw) {
        full_node_list_t to_process;

        to_process.emplace_back({0, 0, root_aabb});

        while(to_process.size() > 0) {
            qt_full_node_t full_node = to_process.pop_back();
            qt_node_t* node = &nodes[full_node.index];

            std::array<aabb_t, 4> aabbs;
            set_quad_aabbs(full_node.aabb, aabbs);

            draw(full_node.aabb.pos, full_node.aabb.hw * 2);

            if(node->count == -1)
                for(int i = 0; i < 4; i++) {
                    to_process.emplace_back(qt_full_node_t{node->first_child + i, full_node.depth + 1, aabbs[i]});
                }
        }
    }
}