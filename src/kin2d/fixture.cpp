#include "fixture.hpp"
#include "body.hpp"

namespace kin {
    fixture_t::fixture_t(rigid_body_t* body, const fixture_def_t& def) 
        : body(body), restitution(def.restitution), obb_t(def.rel_pos, 0.0f, def.hw, def.hh), static_friction(def.static_friction), dynamic_friction(def.dynamic_friction) { 
        body->fixtures.push_front(this);

        set_density(def.density, false);

        update_vertices();
    } 

    fixture_t::~fixture_t() {
        body->remove_mass(pos, mass, tensor);
        body->fixtures.remove_element(this);
    }

    void fixture_t::set_density(float new_density, bool del_mass_from_body) {
        density = new_density;

        if(del_mass_from_body) {
            body->remove_mass(pos, mass, tensor);
        }

        mass = compute_mass(new_density);

        const float one_twelth = 1.0f / 12.0f; 
        tensor = one_twelth * mass * (hw * hw + hh * hh);

        // we pass the position as the center of mass as for 
        // every rectangle regardless of size, the center of mass
        // is ALWAYS in the center
        body->add_mass(pos, mass, tensor);
    }

    glm::vec2 fixture_t::get_world_pos() const {
        return body->get_world_pos() + glm::rotate(pos, body->rot);
    }

    float fixture_t::get_world_rot() const {
        return body->get_world_rot();
    }
    
    void fixture_t::update_vertices() {
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
            normals[i] = glm::rotate(local_normals[i], body->rot);
        }

        aabb.bl = {float_max, float_max};
        aabb.tr = {float_min, float_min};

        for(int i = 0; i < 4; i++) {
            // the shape should be rotated by its relative position and the bodies center of mass
            world_vertices[i] = glm::rotate(local_vertices[i] + (pos - body->center_of_mass), body->rot);
            world_vertices[i] += body->pos;

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
}