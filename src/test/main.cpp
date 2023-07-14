#include <kin2d/kin2d.hpp>
#include <kin2d/math.hpp>

int main() {
    kin::print_test();

    // testing cos, sin, tan speed
    kin::profiler_t trig;
    int sinid = trig.create_profile("sin");
    int cosid = trig.create_profile("cos");
    int tanid = trig.create_profile("rot");
    size_t test_size = 100;

    for(size_t i = 0; i < test_size; i++) {
        trig.start_profile(sinid);
        volatile float imanop = kin::fast_sin(glm::radians((float)(i % 360)));
        trig.end_profile(sinid);
    }

    for(size_t i = 0; i < test_size; i++) {
        trig.start_profile(cosid);
        volatile float imanop = kin::fast_cos(glm::radians((float)(i % 360)));
        trig.end_profile(cosid);
    }

    for(size_t i = 0; i < test_size; i++) {
        trig.start_profile(tanid);
        volatile glm::vec2 imanop = kin::fast_rotate({1.0f, 0.0f}, glm::radians((float)(i % 360)));
        trig.end_profile(tanid);

        // printf("imanop %f %f\n", imanop.x, imanop.y);
    }

    trig.print_profiles(-1);

    kin::world_t world;

    auto body = world.create_rigid_body({0.0f, 0.0f}, 1.0f, kin::body_type_dynamic);
    world.create_rigid_body({0.0f, 0.0f}, 1.0f, kin::body_type_dynamic);
    world.create_rigid_body({0.0f, 0.0f}, 1.0f, kin::body_type_dynamic);
    world.create_rigid_body({0.0f, 0.0f}, 1.0f, kin::body_type_dynamic);
    world.create_rigid_body({0.0f, 0.0f}, 1.0f, kin::body_type_static);
    
    world.update(0.016f, 8);
    world.update(0.016f, 8);
    world.update(0.016f, 8);
    world.update(0.016f, 8);
    world.update(0.016f, 8);

    // if we can return, that means no seg faults. So its basically
    // stable enough
    
    return 0;
}