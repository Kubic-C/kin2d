#include "obb.hpp"
#include "math.hpp"

namespace kin {
    transform_t::transform_t()
        : pos(0.0f), rot(0.0f) {}

    transform_t::transform_t(glm::vec2 pos, float rot) 
        : pos(pos), rot(rot) {}

    glm::vec2 transform_t::get_world_point(glm::vec2 point) const {
        return fast_rotate(point, rot) + pos;
    }

    obb_t::obb_t(glm::vec2 pos, float rot, float hw, float hh)
        : transform_t(pos, rot) {
        set_dimensions(hw, hh);
    }

    void obb_t::set_dimensions(float hw, float hh) {
        this->hw = hw;
        this->hh = hh;
    }

    float obb_t::compute_mass(float density) {
        return (hw * 2.0f) * (hh * 2.0f) * density;
    }
}