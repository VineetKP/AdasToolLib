cmake_minimum_required(VERSION 3.16)
project(quaternion_app)

add_library(adas_min_q STATIC
    src/quaternion.cpp
)
target_include_directories(adas_min_q PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(adas_min_q PUBLIC cxx_std_20)
target_compile_options(adas_min_q PRIVATE -Wall -Wextra -Wpedantic)

add_executable(quaternion_app src/main.cpp)
target_include_directories(quaternion_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(quaternion_app PRIVATE cxx_std_20)
target_compile_options(quaternion_app PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(quaternion_app PRIVATE adas_min_q)
add_executable(test_quaternion test/test_quaternion.cpp)
target_include_directories(test_quaternion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(test_quaternion PRIVATE adas_min_q)
add_test(NAME quaternion_test COMMAND test_quaternion)
