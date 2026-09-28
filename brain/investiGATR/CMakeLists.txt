# investiGATR: shared types, reference resolution, path planning, tests.

# Shared value types and interfaces: poses, field, state source, references,
# motion model, paths. communiGATR and actuGATR build on these.
add_library(investigatr_types STATIC
    src/geometry.cpp
    src/field.cpp
    src/state_source.cpp
    src/reference.cpp
    src/motion_model.cpp
    src/path.cpp
)
target_include_directories(investigatr_types PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(investigatr_types PUBLIC cxx_std_17)

# Planner and footprint clearance.
add_library(investigatr STATIC
    src/collision.cpp
    src/planner.cpp
)
target_link_libraries(investigatr PUBLIC investigatr_types)

# Test files.
set(INVESTIGATR_TEST_SOURCES
    tests/geometry_gtest.cpp
    tests/field_gtest.cpp
    tests/motion_model_gtest.cpp
    tests/path_gtest.cpp
    tests/reference_gtest.cpp
    tests/clearance_gtest.cpp
    tests/planner_gtest.cpp
    tests/planner_exit_gtest.cpp
    tests/planner_edge_gtest.cpp
    tests/planner_random_gtest.cpp
)

add_executable(investigatr_tests ${INVESTIGATR_TEST_SOURCES})
target_link_libraries(investigatr_tests PRIVATE investigatr GTest::gtest_main)
gtest_discover_tests(investigatr_tests)

if(NOT MSVC)
    target_compile_options(investigatr_types PRIVATE -Wall -Wextra)
    target_compile_options(investigatr PRIVATE -Wall -Wextra)
    target_compile_options(investigatr_tests PRIVATE -Wall -Wextra)
endif()
