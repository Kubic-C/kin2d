#pragma once

#include "base.hpp"

namespace kin {
    inline struct {
        uint8_t max_tree_depth = 6;
        uint8_t max_elements_in_leaf = 4;
    } settings;
}