#include <kin2d/kin2d.hpp>

int main() {
    kin::print_test();

    kin::world_t world;

    auto& body = world.create_rigid_body({0.0f, 0.0f}, {10.0f, 10.0f}, 1.0f);
    world.create_rigid_body({0.0f, 0.0f}, {10.0f, 10.0f}, 1.0f);
    world.create_rigid_body({0.0f, 0.0f}, {10.0f, 10.0f}, 1.0f);
    world.create_rigid_body({0.0f, 0.0f}, {10.0f, 10.0f}, 1.0f);
    world.create_rigid_body({0.0f, 0.0f}, {10.0f, 10.0f}, 1.0f);
    
    world.update(0.016f);
    world.update(0.016f);
    world.update(0.016f);
    world.update(0.016f);
    world.update(0.016f);
    
    return 0;
}