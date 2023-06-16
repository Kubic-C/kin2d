#include "obb.hpp"

namespace kin {
    transform_t::transform_t()
        : pos(0.0f), rot(rot) {}

    transform_t::transform_t(glm::vec2 pos, float rot) 
        : pos(pos), rot(rot) {}

    glm::vec2 transform_t::get_world_point(glm::vec2 point) {
        return glm::rotate(point, rot) + pos;
    }

    obb_t::obb_t(glm::vec2 pos, float rot, float hw, float hh)
        : transform_t(pos, rot) {
        set_dimensions(hw, hh);
        update_vertices();
    }

    void obb_t::update_vertices() {
        box_vertices_t local_vertices = {
            glm::vec2(-hw, -hh),
            glm::vec2( hw, -hh),
            glm::vec2( hw,  hh),
            glm::vec2(-hw,  hh)
        };

        box_normals_t local_normals = {
            glm::vec2(-1.0f, 0.0f ),
            glm::vec2( 0.0f, -1.0f)
        };

        for(uint8_t i = 0; i < 2; i++) {
            normals[i] = glm::rotate(local_normals[i], rot);
        }

        for(int i = 0; i < 4; i++) {
            world_vertices[i] = glm::rotate(local_vertices[i], rot);
            world_vertices[i] += pos;

            if(world_vertices[i].x < aabb.bl.x) {
                aabb.bl.x = world_vertices[i].x;
            }
            if(world_vertices[i].x >  aabb.tr.x) {
                aabb.tr.x = world_vertices[i].x;
            }
            if(world_vertices[i].y < aabb.bl.y) {
                aabb.bl.y = world_vertices[i].y;
            }
            if(world_vertices[i].y > aabb.tr.y) {
                aabb.tr.y = world_vertices[i].y;
            }
        }
    }

    void obb_t::set_dimensions(float hw, float hh) {
        this->hw = hw;
        this->hh = hh;

        update_vertices();
    }

    const aabb_t& obb_t::get_aabb() const { 
        return aabb; 
    };
}