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

                // checking for containment
                if (contains(shape1, shape2) || contains(shape2, shape1)) {
                    double mins = glm::abs(shape1.min - shape2.min);
                    double maxs = glm::abs(shape1.max - shape2.max);

                    if (mins < maxs) {
                        depth += mins;
                    } else {
                        depth += maxs;
                    }
                }

                if(new_depth < depth) {
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

        if(!nearly_equal(obb1.rot, obb2.rot)) {
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

        if(d <= 0) {
            cp = v1;
        } else if(d >= 1) {
            cp = v2;
        } else {
            cp = v1 + v1_to_v2 * d;
        }

        return glm::distance(p, cp);
    }

    void find_closest_points(glm::vec2& cp1, glm::vec2& cp2, float& distance1, 
        const box_vertices_t& points1, const box_vertices_t& points2) {
        for(uint32_t i = 0; i < points1.size(); i++) {
            const glm::vec2& p = points1[i];

            glm::vec2 prev = points2.back();
            for(uint32_t j = 0; j < points2.size(); j++) {
                if(j != 0) {
                    prev = points2[j - 1];
                }

                glm::vec2 cur = points2[j];
            
                glm::vec2 cp;
                float distance = point_segment_distance(p, prev, cur, cp);

                if(nearly_equal(distance, distance1)) {
                    if(nearly_equal(cp, cp1, 0.1f))
                        continue;

                    cp2 = cp;
                } else if(distance < distance1) {
                    cp1 = cp;
                    distance1 = distance;
                }
            }
        }
    }

    void compute_manifold(obb_t& obb1, obb_t& obb2, collision_manifold_t& manifold) {
        float min_distance = std::numeric_limits<float>::max();
        glm::vec2 min_cp;
        glm::vec2 min_cp2 = {std::numeric_limits<float>::max(), 0.0f};

        find_closest_points(min_cp, min_cp2, min_distance, obb1.world_vertices, obb2.world_vertices);
        find_closest_points(min_cp, min_cp2, min_distance, obb2.world_vertices, obb1.world_vertices);

        manifold.add(min_cp);
        if(min_cp2.x != std::numeric_limits<float>::max()) {
            manifold.add(min_cp2);
        }
    }

    bool solve_collision_if_there(rigid_body_t& body1, rigid_body_t& body2, collision_manifold_t& manifold) {
        if(!aabb_collide(body1.aabb, body2.aabb))
            return false;
        
        if(!sat_test(body1, body2, manifold))
            return false;

        const glm::vec2 amount = manifold.normal * manifold.depth;
        const float inv_total_mass = 1.0f / (body1.mass + body2.mass); 
        const glm::vec2 amount1 = amount * (body2.mass * inv_total_mass);
        const glm::vec2 amount2 = amount * (body1.mass * inv_total_mass);

        body1.pos -= amount1;
        body2.pos += amount2;   

        for(auto& point : body1.world_vertices) {
            point -= amount1;
        }

        for(auto& point : body2.world_vertices) {
            point += amount2;
        }

        compute_manifold(body1, body2, manifold);

        return true;
    }

    void impulse_method(rigid_body_t& body1, rigid_body_t& body2, collision_manifold_t& manifold) {
        glm::vec2 vrel = body1.linear_vel - body2.linear_vel;
        float restitution = glm::max(body1.restitution, body2.restitution);

        float impulse_velocity = -(1.0f + restitution) * glm::dot(vrel, manifold.normal);
        float angular_vel1 = 0.0f;
        float angular_vel2 = 0.0f;
        float angular_change = 0.0f;

        { // this all has to do with angular change lol
            for(uint8_t i = 0; i < manifold.count; i++) {
                glm::vec2 relcp1 = body1.pos - manifold.points[i];
                glm::vec2 relcp2 = body2.pos - manifold.points[i];
                
                float crosscp1 = body1.invtensor * cross(relcp1, manifold.normal);
                float crosscp2 = body2.invtensor * cross(relcp2, manifold.normal);
                angular_change += glm::dot(
                                (crosscp1 * relcp1) + 
                                (crosscp2 * relcp2), manifold.normal);

                angular_vel1 += crosscp1;
                angular_vel2 += crosscp2;
            }
        }

        float lower_div = body1.invmass + body2.invmass + glm::abs(angular_change); 
        
        impulse_velocity /= lower_div; 

        body1.angular_vel += angular_vel1;
        body2.angular_vel -= angular_vel2;

        body1.linear_vel += body1.invmass * impulse_velocity * manifold.normal;
        body2.linear_vel -= body2.invmass * impulse_velocity * manifold.normal;
    }
}