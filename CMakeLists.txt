cmake_minimum_required(VERSION 3.16)
project(adas_tools
    VERSION 0.1.0
    DESCRIPTION "ADAS Supporting Tools Library"
    LANGUAGES CXX
)

# =====================
# 🧩 Project settings
# =====================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_TESTS "Build unit tests" ON)

# =====================
# 📚 Library target
# =====================
add_library(adas_tools
    src/transformers.cpp
    src/quaternion.cpp
)

target_include_directories(adas_tools
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_compile_features(adas_tools PUBLIC cxx_std_20)
target_compile_options(adas_tools PRIVATE -Wall -Wextra -Wpedantic)

# Example of linking to an external dependency
# find_package(fmt REQUIRED)
# target_link_libraries(adas_tools PUBLIC fmt::fmt)

# =====================
# 💡 Example program
# =====================
if(BUILD_EXAMPLES)
    # Build a small executable from examples/main_app.cpp so the example can be run directly
    add_executable(adas_tools_app examples/main_app.cpp)
    target_include_directories(adas_tools_app PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(adas_tools_app PRIVATE cxx_std_20)
    target_compile_options(adas_tools_app PRIVATE -Wall -Wextra -Wpedantic)
    # If other library components are required later, link adas_tools
    target_link_libraries(adas_tools_app PRIVATE adas_tools)

    # Add a richer example demonstrating transformers + quaternion
    add_executable(example_usage examples/example_usage.cpp)
    target_include_directories(example_usage PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(example_usage PRIVATE cxx_std_20)
    target_compile_options(example_usage PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(example_usage PRIVATE adas_tools)

    # Sensor offsets example
    add_executable(sensor_offsets examples/sensor_offsets.cpp)
    target_include_directories(sensor_offsets PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(sensor_offsets PRIVATE cxx_std_20)
    target_compile_options(sensor_offsets PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(sensor_offsets PRIVATE adas_tools)

    # Chain example
    add_executable(sensor_chain examples/sensor_chain.cpp)
    target_include_directories(sensor_chain PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(sensor_chain PRIVATE cxx_std_20)
    target_compile_options(sensor_chain PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(sensor_chain PRIVATE adas_tools)

    # Quaternion interpolation walk
    add_executable(quaternion_walk examples/quaternion_walk.cpp)
    target_include_directories(quaternion_walk PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(quaternion_walk PRIVATE cxx_std_20)
    target_compile_options(quaternion_walk PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(quaternion_walk PRIVATE adas_tools)
endif()

if(BUILD_TESTS)
    add_executable(tests tests/test_transformers.cpp)
    target_include_directories(tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(tests PRIVATE cxx_std_20)
    target_compile_options(tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(tests PRIVATE adas_tools)

    # Slerp correctness + benchmark
    add_executable(test_slerp tests/test_slerp.cpp)
    target_include_directories(test_slerp PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(test_slerp PRIVATE cxx_std_20)
    target_compile_options(test_slerp PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(test_slerp PRIVATE adas_tools)

    add_executable(test_slerp_edge tests/test_slerp_edge.cpp)
    target_include_directories(test_slerp_edge PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(test_slerp_edge PRIVATE cxx_std_20)
    target_compile_options(test_slerp_edge PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(test_slerp_edge PRIVATE adas_tools)

    add_executable(test_gimbal tests/test_gimbal.cpp)
    target_include_directories(test_gimbal PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(test_gimbal PRIVATE cxx_std_20)
    target_compile_options(test_gimbal PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(test_gimbal PRIVATE adas_tools)
endif()

# =====================
# 🧪 Unit tests
# =====================
# if(BUILD_TESTS)
#     include(FetchContent)
#     FetchContent_Declare(
#         Catch2
#         GIT_REPOSITORY https://github.com/catchorg/Catch2.git
#         GIT_TAG v3.4.0
#     )
#     FetchContent_MakeAvailable(Catch2)

#     add_executable(tests tests/test_module1.cpp)
#     target_link_libraries(tests PRIVATE adas_tools Catch2::Catch2WithMain)

#     include(CTest)
#     include(Catch)
#     catch_discover_tests(tests)
# endif()
