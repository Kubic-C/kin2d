#include <kin2d/kin2d.hpp>

int main() {
    kin::print_test();

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