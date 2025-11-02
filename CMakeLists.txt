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
    # Only build the focused advanced example to keep workspace minimal
    add_executable(advanced_transform_examples examples/advanced_transform_examples.cpp)
    target_include_directories(advanced_transform_examples PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_features(advanced_transform_examples PRIVATE cxx_std_20)
    target_compile_options(advanced_transform_examples PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(advanced_transform_examples PRIVATE adas_tools)
    
    # visualization example: projects a lidar point and writes an image
    add_executable(visualize_projection examples/visualize_projection.cpp)
    target_include_directories(visualize_projection PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/examples)
    target_compile_features(visualize_projection PRIVATE cxx_std_20)
    target_compile_options(visualize_projection PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(visualize_projection PRIVATE adas_tools)
endif()

if(BUILD_TESTS)
    message(STATUS "BUILD_TESTS enabled but no tests are configured in this minimal workspace.")
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
