add_library(kin2d STATIC "base.hpp" "base.cpp")

target_link_libraries(kin2d PUBLIC portem glm)

target_sources(kin2d PRIVATE
    "aabb.hpp" "aabb.cpp"
    "obb.hpp" "obb.cpp"
    "quad_tree.hpp" "quad_tree.cpp"
    "body.hpp" "body.cpp"
    "world.hpp" "world.cpp"
    "collision.hpp" "collision.cpp"
    "fixture.hpp" "fixture.cpp")
 
target_sources(kin2d PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/kin2d.hpp")
