#include "fixture.hpp"
#include "body.hpp"
#include "math.hpp"

namespace kin {
    fixture_t::fixture_t(rigid_body_t* body, rtree_element_t* element, const fixture_def_t& def) 
        : body(body), restitution(def.restitution), obb_t(def.rel_pos, 0.0f, def.hw, def.hh), static_friction(def.static_friction), dynamic_friction(def.dynamic_friction), relement(element) { 
        body->fixtures.push_front(this);

        set_density(def.density, false);

        relement->obb = this;
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
        const float w = 2.0 * hw;
        const float h = 2.0 * hh;
        float distance = glm::length2(pos);
        tensor = (one_twelth * mass * (w * w + h * h)) + (mass * distance);

        // we pass the position as the center of mass as for 
        // every rectangle regardless of size, the center of mass
        // is ALWAYS in the center
        body->add_mass(pos, mass, tensor);
    }

    glm::vec2 fixture_t::get_world_pos() const {
        return body->get_world_pos() + fast_rotate(pos - body->center_of_mass, body->rot);
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

        normals[0] = fast_rotate_w_precalc(glm::vec2(-1.0f, 0.0f ), body->psin, body->pcos);
        normals[1] = fast_rotate_w_precalc(glm::vec2( 0.0f, -1.0f), body->psin, body->pcos);

        relement->min[0] = float_max;
        relement->min[1] = float_max;
        relement->max[0] = float_min;
        relement->max[1] = float_min;

        for(int i = 0; i < 4; i++) {
            // the shape should be rotated by its relative position and the bodies center of mass
            world_vertices[i] = body->get_world_point(local_vertices[i] + pos);

            if(world_vertices[i].x < relement->min[0]) {
                relement->min[0] = world_vertices[i].x;
            }
            if(world_vertices[i].x >  relement->max[0]) {
                relement->max[0] = world_vertices[i].x;
            }
            if(world_vertices[i].y < relement->min[1]) {
                relement->min[1] = world_vertices[i].y;
            }
            if(world_vertices[i].y > relement->max[1]) {
                relement->max[1] = world_vertices[i].y;
            }
        }
    }
}