cmake_minimum_required(VERSION 3.16)
project(advanced_app)

add_library(adas_min STATIC
	src/transformers.cpp
)
target_include_directories(adas_min PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(adas_min PUBLIC cxx_std_20)
target_compile_options(adas_min PRIVATE -Wall -Wextra -Wpedantic)

add_executable(advanced_app src/main.cpp)
target_include_directories(advanced_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(advanced_app PRIVATE cxx_std_20)
target_compile_options(advanced_app PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(advanced_app PRIVATE adas_min)
