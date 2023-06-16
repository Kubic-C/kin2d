#include "quad_tree.hpp"
#include "settings.hpp"

namespace kin {
    std::array<aabb_t, 4> get_quad_aabbs(const aabb_t& parent, float hw) {
        return {
            aabb_t(parent.bl, (parent.bl + hw)),
            aabb_t(glm::vec2(parent.bl.x + hw, parent.bl.y), glm::vec2(parent.tr.x, parent.tr.y - hw)),
            aabb_t(parent.tr - hw, parent.tr),
            aabb_t(glm::vec2(parent.bl.x, parent.bl.y + hw), glm::vec2(parent.tr.x - hw, parent.tr.y))
        };
    }

    void root_leaf_list_add(quad_tree_t& root, quad_tree_leaf_t* leaf) {
        if(root.first == nullptr) {
            root.first = leaf;
            root.last = leaf;
        } else {
            quad_tree_leaf_t* old_end = root.last;
            old_end->next = leaf;
            leaf->prev = old_end;
            root.last = leaf;
        }
    }

    void root_leaf_list_remove(quad_tree_t& root, quad_tree_leaf_t* leaf) {
        quad_tree_leaf_t* prev = leaf->prev;
        quad_tree_leaf_t* next = leaf->next;

        if(prev)
            prev->next = next;

        if(next)
            next->prev = prev;

        if(leaf == root.first) {
            root.first = next;
        }

        if(leaf == root.last)
            root.last = prev;
    }

    void node_iterate(quad_tree_t& root, quad_tree_node_t& node, const aabb_t& aabb, float hw, node_callback callback) {
        float hhw = hw * 0.5f;
        std::array<aabb_t, 4> node_aabb = get_quad_aabbs(aabb, hw);

        callback(node, aabb);

        if(node.children != nullptr)
            for(uint32_t i = 0; i < 4; i++) {
                node_iterate(root, node.children[i], node_aabb[i], hhw, callback);
            }
    }

    void node_insert(quad_tree_t& root, quad_tree_node_t& node, quad_tree_element_t& element, const aabb_t& aabb, float hw, uint32_t depth) {
        float hhw = hw * 0.5f;
        std::array<aabb_t, 4> node_aabb = get_quad_aabbs(aabb, hw);

        if(!node.children) {
            // the node is a leaf
            if(depth + 1 < settings.max_tree_depth && 
               node.leaf->objects.size() + 1 >= settings.max_elements_in_leaf) {
                // turn the leaf into a node and add children leaves
                // remember to delete the leaf attached to this node
                quad_tree_leaf_t* leaf = node.leaf;

                root_leaf_list_remove(root, leaf);

                node.children = root.node_pool.create(4);

                for(uint32_t i = 0; i < 4; i++) {
                    node.children[i].parent = &node;
                    node.children[i].leaf   = root.leaf_pool.create(1, root.element_pool);
                    root_leaf_list_add(root, node.children[i].leaf);

                    if(aabb_collide(node_aabb[i], element.get_aabb())) {
                        node_insert(root, node.children[i], element, node_aabb[i], hhw, depth + 1);
                    }     

                    // also move our children
                    for(auto& object : leaf->objects) {
                        if(aabb_collide(node_aabb[i], object->get_aabb())) {
                            node_insert(root, node.children[i], *object, node_aabb[i], hhw, depth + 1);
                        }   
                    }   
                }
        
                // finally remove this leaf 
                node.leaf = nullptr;
                root.leaf_pool.destruct(leaf, 1);
            } else {
                // this leaf cant go deeper or does not have enough to turn into
                // a inner node

                node.leaf->objects.push_back(&element);
            }
        } else {
            // this is a node so just pass it to the children

            for(uint32_t i = 0; i < 4; i++) {
                if(aabb_collide(node_aabb[i], element.get_aabb())) {
                    node_insert(root, node.children[i], element, node_aabb[i], hhw, depth + 1);
                } 
            }
        }
    }

    void node_clear(quad_tree_t& root, quad_tree_node_t& node) {
        if(node.children == nullptr) {
            root_leaf_list_remove(root, node.leaf);
            root.leaf_pool.destruct(node.leaf, 1);
            node.leaf = nullptr;
        } else {
            for(uint32_t i = 0; i < 4; i++) {
                node_clear(root, node.children[i]);
            }
        
            root.node_pool.destruct(node.children, 4);
            node.children = nullptr;
        }
    }

    void tree_prepare(quad_tree_t& root, glm::vec2 pos, float hw) {
        if(root.children || root.leaf) {
            node_clear(root, root);
        }

        root.leaf = root.leaf_pool.create(1, root.element_pool);
        root.aabb.bl = pos - glm::vec2(hw, hw);
        root.aabb.tr = pos + glm::vec2(hw, hw);
        root.hw = hw;

        root_leaf_list_add(root, root.leaf);
    }

    void tree_insert(quad_tree_t& root, quad_tree_element_t& element) {
        node_insert(root, root, element, root.aabb, root.hw, 0);
    }

    void tree_iterate_nodes(quad_tree_t& root, node_callback callback) {
        node_iterate(root, root, root.aabb, root.hw, callback);
    }

    void tree_iterate_leaves(quad_tree_t& root, leaf_callback callback) {
        quad_tree_leaf_t* cur = root.first;
        while(cur != nullptr) {
            if(cur->objects.size() > 1)
                callback(cur->objects);

            cur = cur->next;
        }
    }

    void node_search_w_callback(quad_tree_node_t& node, const aabb_t& aabb, float hw, quad_tree_element_t& element, search_callback_t callback) {
        float hhw = hw * 0.5f;
        std::array<aabb_t, 4> node_aabb = get_quad_aabbs(aabb, hw);
        
        if(!node.children) {
            callback(element, node.leaf->objects);
        } else {
            for(uint32_t i = 0; i < 4; i++) {
                if(aabb_collide(node_aabb[i], element.get_aabb())) {
                    node_search_w_callback(node.children[i], node_aabb[i], hhw, element, callback);
                }
            }
        }
    }

    void tree_search_w_callback(quad_tree_t& root, quad_tree_element_t& element, search_callback_t callback) {
        node_search_w_callback(root, root.aabb, root.hw, element, callback);
    }
}
