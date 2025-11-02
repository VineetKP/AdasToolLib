cmake_minimum_required(VERSION 3.16)
project(camera_projection)

add_library(adas_min_cp STATIC
	src/transformers.cpp
)
target_include_directories(adas_min_cp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(adas_min_cp PUBLIC cxx_std_20)
target_compile_options(adas_min_cp PRIVATE -Wall -Wextra -Wpedantic)

add_executable(camera_projection src/main.cpp)
target_include_directories(camera_projection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(camera_projection PRIVATE cxx_std_20)
target_compile_options(camera_projection PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(camera_projection PRIVATE adas_min_cp)
