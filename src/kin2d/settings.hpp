#pragma once

#include "base.hpp"

namespace kin {
    inline struct {
        uint8_t max_tree_depth = 5;
        uint8_t max_elements_in_leaf = 8;
    } settings;
}