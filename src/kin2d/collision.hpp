#pragma once

#include "body.hpp"

namespace kin {
    struct collision_manifold_t {
        uint8_t                  count = 0;
        std::array<glm::vec2, 2> points = {};
 
        glm::vec2 normal = {0.0f, 0.0f};
        float     depth = 0.0f;
    
        float restitution;
        float static_friction;                      
        float dynamic_friction;

        void add(glm::vec2 point) {
            if(count != points.size()) {
                points[count] = point;
                count++;
            }
        }
    };

    // test if there is a collision using Seperating Axis Theorem
    struct min_max_t {
        float min;
        float max;
    };

    inline bool contains(min_max_t& min_max1, min_max_t& min_max2) {
        return min_max1.min <= min_max2.min && min_max2.max <= min_max1.max;
    }

    inline min_max_t project_vertices(const std::array<glm::vec2, 4>& vertices, glm::vec2 axis) {
        min_max_t min_max;
        min_max.min = glm::dot(vertices[0], axis);
        min_max.max = min_max.min;

        for(uint32_t i = 1; i < 4; i++) {
            float proj_point = glm::dot(vertices[i], axis);

            if(proj_point < min_max.min) {
                min_max.min = proj_point;
            } else if(proj_point > min_max.max) {
                min_max.max = proj_point;
            }
        }

        return min_max;
    }

    inline bool sat_test_vertices(const box_vertices_t& vertices1, const box_vertices_t& vertices2, const box_normals_t& normals, glm::vec2& normal, float& depth) {
        for(uint8_t i = 0; i < 2; i++) {
            // shape of the object on a 1D line
            min_max_t shape1 = project_vertices(vertices1, normals[i]);
            min_max_t shape2 = project_vertices(vertices2, normals[i]);

            if(!(shape1.max >= shape2.min && shape2.max >= shape1.min)) {
                // they are not collding
                return false;
            } else {
                float new_depth = std::max(0.0f, std::min(shape1.max, shape2.max) - std::max(shape1.min, shape2.min));

                if(new_depth <= depth) {
                    normal = normals[i];
                    depth  = new_depth;

                    // we check to see if this value is on the left or right side of shape1
                    float direction = shape1.max - shape2.max;
                    if(direction > 0) {
                        normal *= -1.0f;
                    }
                }
            }
        } 

        return true;
    }

    // uses the seperating axis theorem test to see if two shapes are collding
    inline bool sat_test(obb_t& obb1, obb_t& obb2, collision_manifold_t& manifold) {
        manifold.depth = std::numeric_limits<float>::max();
        manifold.normal = {0.0f, 0.0f};

        if(!sat_test_vertices(obb1.world_vertices, obb2.world_vertices, obb1.normals, manifold.normal, manifold.depth)) {
            return false;
        }

        if(!sat_test_vertices(obb1.world_vertices, obb2.world_vertices, obb2.normals, manifold.normal, manifold.depth)) {
            return false;
        }

        return true;
    }

    inline float point_segment_distance(glm::vec2 p, glm::vec2 v1, glm::vec2 v2, glm::vec2& cp) {
        // credit goes to https://www.youtube.com/watch?v=egmZJU-1zPU&ab_channel=Two-BitCoding
        // for this function, incredible channel and resource

        glm::vec2 p_to_v1 = p - v1;
        glm::vec2 v1_to_v2 = v2 - v1;
        float proj = glm::dot(p_to_v1, v1_to_v2);
        float length = glm::length2(v1_to_v2);

        float d = proj / length;

        if(d <= 0.0f) {
            cp = v1;
        } else if(d >= 1.0f) {
            cp = v2;
        } else {
            cp = v1 + v1_to_v2 * d;
        }

        return glm::distance(p, cp);
    }

    // finds the closest point on points1 to one of points2's edges
    inline void find_closest_points(glm::vec2& cp1, glm::vec2& cp2, uint32_t& count, float& distance1, 
        const box_vertices_t& points1, const box_vertices_t& points2) {
        for(uint32_t i = 0; i < points1.size(); i++) {
            glm::vec2 p = points1[i];

            for(uint32_t j = 0; j < points2.size(); j++) {
                glm::vec2 prev = points2[j == 0 ? points2.size() - 1 : j - 1];
                glm::vec2 cur  = points2[j];
            
                glm::vec2 cp;
                float distance = point_segment_distance(p, prev, cur, cp);

                if(nearly_equal(distance, distance1, 0.1f)) {
                    if(!nearly_equal(cp, cp1, 0.005f)) {
                        count = 2;
                        cp2 = cp;
                    }
                } else if(distance <= distance1) {
                    count = 1;
                    cp1 = cp;
                    distance1 = distance;
                }
            }
        }
    }

    // computes the collision manifold between two OBBs
    inline void compute_manifold(obb_t& obb1, obb_t& obb2, collision_manifold_t& manifold) {
        float min_distance = std::numeric_limits<float>::max();
        uint32_t count = 0;
        glm::vec2 min_cp;
        glm::vec2 min_cp2;

        find_closest_points(min_cp, min_cp2, count, min_distance, obb1.world_vertices, obb2.world_vertices);
        find_closest_points(min_cp, min_cp2, count, min_distance, obb2.world_vertices, obb1.world_vertices);

        assert(count != 0);

        manifold.add(min_cp);
        if(count == 2) {
            manifold.add(min_cp2);
        }
    }

    // solves a collision between two fixtures if they are intersecting, returns found
    // results to collision manifold. Does NOT solve impulses
    bool solve_collision_if_there(fixture_t& fix1, fixture_t& fix2, collision_manifold_t& manifold);
    
    struct impulse_t {
        glm::vec2 r1;
        glm::vec2 r2;
        float j = 0.0f;
        glm::vec2 friction_impulse = {0.0f, 0.0f};
    };

    // solve for new velocties for two bodies given a collision manifodl
    inline void impulse_method(rigid_body_t& body1, rigid_body_t& body2, collision_manifold_t& manifold) {
        impulse_t impulse;
        glm::vec2 average = manifold.points[0];

        if(manifold.count == 2) {
            average += manifold.points[1];
            average /= 2.0f;
        }

        { // normal calculations
            impulse.r1 = body1.get_world_pos() - average;
            impulse.r2 = body2.get_world_pos() - average;

            glm::vec2 r1_perp = {impulse.r1.y, -impulse.r1.x};
            glm::vec2 r2_perp = {impulse.r2.y, -impulse.r2.x};

            glm::vec2 angular_linear1 = r1_perp * body1.angular_vel;
            glm::vec2 angular_linear2 = r2_perp * body2.angular_vel;

            glm::vec2 rel_vel = 
                (body2.linear_vel + angular_linear2) - 
                (body1.linear_vel + angular_linear1);

            float rel_vel_dot_n = glm::dot(rel_vel, manifold.normal);
            if(rel_vel_dot_n > 0.0f)
                return;

            float r1_perp_dot_n = glm::dot(r1_perp, manifold.normal);
            float r2_perp_dot_n = glm::dot(r2_perp, manifold.normal);

            float denom = 
                body1.invmass + body2.invmass + 
                (sqaure(r1_perp_dot_n) * body1.invinertia) + 
                (sqaure(r2_perp_dot_n) * body2.invinertia);
            
            impulse.j = -(1.0f + manifold.restitution) * rel_vel_dot_n;
            impulse.j /= denom;
            impulse.j /= (float)manifold.count;

            // apply impulse
            glm::vec2 impulsej = impulse.j * manifold.normal;

            body1.linear_vel  -= impulsej * body1.invmass;
            body1.angular_vel -= cross(impulsej, impulse.r1) * body1.invinertia;
            body2.linear_vel  += impulsej * body2.invmass;
            body2.angular_vel += cross(impulsej, impulse.r2) * body2.invinertia;
        }

        { // tangent calculations
            glm::vec2 r1_perp = {impulse.r1.y, -impulse.r1.x};
            glm::vec2 r2_perp = {impulse.r2.y, -impulse.r2.x};

            glm::vec2 angular_linear1 = r1_perp * body1.angular_vel;
            glm::vec2 angular_linear2 = r2_perp * body2.angular_vel;

            glm::vec2 rel_vel = 
                (body2.linear_vel + angular_linear2) - (body1.linear_vel + angular_linear1);

            glm::vec2 tangent = rel_vel - glm::dot(rel_vel, manifold.normal) * manifold.normal;
            if(nearly_equal(tangent, {0.0f, 0.0f}))
                return;
            else
                tangent = glm::normalize(tangent);

            float r1_perp_dot_t = glm::dot(r1_perp, tangent);
            float r2_perp_dot_t = glm::dot(r2_perp, tangent);

            float denom = 
                body1.invmass + body2.invmass + 
               (sqaure(r1_perp_dot_t) * body1.invinertia) + 
               (sqaure(r2_perp_dot_t) * body2.invinertia);

            float jt = -glm::dot(rel_vel, tangent);
            jt /= denom;
            jt /= manifold.count;

            if(glm::abs(jt) <= impulse.j * manifold.static_friction) {
                impulse.friction_impulse = jt * tangent;
            } else {
                impulse.friction_impulse = -impulse.j * tangent * manifold.dynamic_friction;
            }

            body1.linear_vel -= impulse.friction_impulse * body1.invmass;
            body1.angular_vel -= cross(impulse.friction_impulse, impulse.r1) * body1.invinertia;

            body2.linear_vel += impulse.friction_impulse * body2.invmass;
            body2.angular_vel += cross(impulse.friction_impulse, impulse.r2) * body2.invinertia;
        }
    }
}