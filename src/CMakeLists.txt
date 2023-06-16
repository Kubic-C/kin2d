
add_subdirectory(kin2d)

target_include_directories(kin2d PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/")

add_subdirectory(test)
