#include "collision.hpp"

namespace kin {
    struct min_max_t {
        float min;
        float max;
    };

    bool contains(min_max_t& min_max1, min_max_t& min_max2) {
        return min_max1.min <= min_max2.min && min_max2.max <= min_max1.max;
    }

    min_max_t project_vertices(const std::array<glm::vec2, 4>& vertices, glm::vec2 axis) {
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

    bool sat_test_vertices(const box_vertices_t& vertices1, const box_vertices_t& vertices2, const box_normals_t& normals, glm::vec2& normal, float& depth) {
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

    bool sat_test(obb_t& obb1, obb_t& obb2, collision_manifold_t& manifold) {
        manifold.depth = std::numeric_limits<float>::max();
        manifold.normal = {0.0f, 0.0f};

        if(!sat_test_vertices(obb1.world_vertices, obb2.world_vertices, obb1.normals, manifold.normal, manifold.depth)) {
            return false;
        }

        // prevents testing the same axises again
        // we can ONLY do this because we know both are boxes
        if(!nearly_equal(obb1.get_world_rot(), obb2.get_world_rot())) {
            if(!sat_test_vertices(obb1.world_vertices, obb2.world_vertices, obb2.normals, manifold.normal, manifold.depth)) {
                return false;
            }
        }

        return true;
    }

    float point_segment_distance(glm::vec2 p, glm::vec2 v1, glm::vec2 v2, glm::vec2& cp) {
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

    void find_closest_points(glm::vec2& cp1, glm::vec2& cp2, uint32_t& count, float& distance1, 
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

    void compute_manifold(obb_t& obb1, obb_t& obb2, collision_manifold_t& manifold) {
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

    bool solve_collision_if_there(fixture_t& fix1, fixture_t& fix2, collision_manifold_t& manifold) {
        rigid_body_t& body1 = *fix1.body;
        rigid_body_t& body2 = *fix2.body;

        if(!aabb_collide(fix1.aabb, fix2.aabb))
            return false;
        
        if(!sat_test(fix1, fix2, manifold))
            return false;

        const glm::vec2 amount = manifold.normal * manifold.depth;
        const float invtotal_mass = 1.0f / (body1.mass + body2.mass); 
        const glm::vec2 amount1 = amount * (body1.mass * invtotal_mass);
        const glm::vec2 amount2 = amount * (body2.mass * invtotal_mass);

        body1.pos -= amount1;
        body2.pos += amount2;   

        for(auto& point : fix1.world_vertices) {
            point -= amount1;
        }

        for(auto& point : fix2.world_vertices) {
            point += amount2;
        }

        compute_manifold(fix1, fix2, manifold);

        manifold.restitution = glm::max(fix1.restitution, fix2.restitution);
        manifold.static_friction = (fix1.static_friction + fix2.static_friction) * 0.5f;
        manifold.dynamic_friction = (fix1.dynamic_friction + fix2.dynamic_friction) * 0.5f;

        return true;
    }
    
    struct impulse_t {
        glm::vec2 r1;
        glm::vec2 r2;
        float j = 0.0f;
        glm::vec2 friction_impulse = {0.0f, 0.0f};
    };

    void impulse_method(rigid_body_t& body1, rigid_body_t& body2, collision_manifold_t& manifold) {
        std::array<impulse_t, 2> impulses;
        
        for(uint32_t i = 0; i < manifold.count; i++) {
            impulses[i].r1 = body1.pos - manifold.points[i];
            impulses[i].r2 = body2.pos - manifold.points[i];

            glm::vec2 r1_perp = {impulses[i].r1.y, -impulses[i].r1.x};
            glm::vec2 r2_perp = {impulses[i].r2.y, -impulses[i].r2.x};

            glm::vec2 angular_linear1 = r1_perp * body1.angular_vel;
            glm::vec2 angular_linear2 = r2_perp * body2.angular_vel;

            glm::vec2 rel_vel = 
                (body2.linear_vel + angular_linear2) - (body1.linear_vel + angular_linear1);

            float rel_vel_dot_n = glm::dot(rel_vel, manifold.normal);
            if(rel_vel_dot_n > 0.0f)
                continue;

            float r1_perp_dot_n = glm::dot(r1_perp, manifold.normal);
            float r2_perp_dot_n = glm::dot(r2_perp, manifold.normal);

            float denom = 
                body1.invmass + body2.invmass + 
                (sqaure(r1_perp_dot_n) * body1.invintertia) + 
                (sqaure(r2_perp_dot_n) * body2.invintertia);

            impulses[i].j = -(1 + manifold.restitution) * rel_vel_dot_n;
            impulses[i].j /= denom;
            impulses[i].j /= manifold.count;
        }

        for(uint32_t i = 0; i < manifold.count; i++) {
            glm::vec2 impulse = impulses[i].j * manifold.normal;

            // if(impulses[i].j >= 300.0f) {
            //     printf("impulse[i].j: %f\n", impulses[i].j);
            // }

            body1.linear_vel -= impulse * body1.invmass;
            body1.angular_vel -= cross(impulse, impulses[i].r1) * body1.invintertia;

            body2.linear_vel += impulse * body2.invmass;
            body2.angular_vel += cross(impulse, impulses[i].r2) * body2.invintertia;
        }

        // friction calculations

        for(uint32_t i = 0; i < manifold.count; i++) {
            glm::vec2 r1_perp = {impulses[i].r1.y, -impulses[i].r1.x};
            glm::vec2 r2_perp = {impulses[i].r2.y, -impulses[i].r2.x};

            glm::vec2 angular_linear1 = r1_perp * body1.angular_vel;
            glm::vec2 angular_linear2 = r2_perp * body2.angular_vel;

            glm::vec2 rel_vel = 
                (body2.linear_vel + angular_linear2) - (body1.linear_vel + angular_linear1);

            glm::vec2 tangent = rel_vel - glm::dot(rel_vel, manifold.normal) * manifold.normal;
            if(nearly_equal(tangent, {0.0f, 0.0f}))
                continue;
            else
                tangent = glm::normalize(tangent);

            float r1_perp_dot_t = glm::dot(r1_perp, tangent);
            float r2_perp_dot_t = glm::dot(r2_perp, tangent);

            float denom = 
                body1.invmass + body2.invmass + 
               (sqaure(r1_perp_dot_t) * body1.invintertia) + 
               (sqaure(r2_perp_dot_t) * body2.invintertia);

            float jt = -glm::dot(rel_vel, tangent);
            jt /= denom;
            jt /= manifold.count;

            if(glm::abs(jt) <= impulses[i].j * manifold.static_friction) {
                impulses[i].friction_impulse = jt * tangent;
            } else {
                impulses[i].friction_impulse = -impulses[i].j * tangent * manifold.dynamic_friction;
            }
        }

        for(uint32_t i = 0; i < manifold.count; i++) {
            body1.linear_vel -= impulses[i].friction_impulse * body1.invmass;
            body1.angular_vel -= cross(impulses[i].friction_impulse, impulses[i].r1) * body1.invintertia;

            body2.linear_vel += impulses[i].friction_impulse * body2.invmass;
            body2.angular_vel += cross(impulses[i].friction_impulse, impulses[i].r2) * body2.invintertia;
        }
    }
}