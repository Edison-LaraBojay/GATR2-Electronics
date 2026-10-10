# spectaGATR Brain publisher: portable core and the spectaGATR wire format,
# host tests. pros/ holds PROS-only sources and is never built here.

get_filename_component(GATR2_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
set(SPECTAGATR_PROTOCOL_DIR ${GATR2_REPO_ROOT}/spectaGATR/protocol)

# Library.
add_library(spectagatr STATIC
    src/publisher.cpp
    ${SPECTAGATR_PROTOCOL_DIR}/src/pose_line.cpp
    ${SPECTAGATR_PROTOCOL_DIR}/src/units.cpp
)
target_include_directories(spectagatr PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SPECTAGATR_PROTOCOL_DIR}/include
)
target_compile_features(spectagatr PUBLIC cxx_std_17)

find_package(Threads REQUIRED)

# Test files.
set(SPECTAGATR_TEST_SOURCES
    tests/pose_gtest.cpp
    tests/publisher_gtest.cpp
    tests/concurrency_gtest.cpp
)

add_executable(spectagatr_tests ${SPECTAGATR_TEST_SOURCES})
target_link_libraries(spectagatr_tests PRIVATE spectagatr Threads::Threads GTest::gtest_main)
gtest_discover_tests(spectagatr_tests)

# Own executable: it replaces the global operator new.
add_executable(spectagatr_allocation_tests tests/allocation_gtest.cpp)
target_link_libraries(spectagatr_allocation_tests PRIVATE spectagatr GTest::gtest_main)
gtest_discover_tests(spectagatr_allocation_tests)

if(NOT MSVC)
    target_compile_options(spectagatr PRIVATE -Wall -Wextra)
    target_compile_options(spectagatr_tests PRIVATE -Wall -Wextra)
    target_compile_options(spectagatr_allocation_tests PRIVATE -Wall -Wextra)
endif()
