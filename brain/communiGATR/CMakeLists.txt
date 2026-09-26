# communiGATR: brain link client, host fakes, tests.
# pros/ holds PROS-only sources and is never built here.

get_filename_component(GATR2_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)

# Library.
add_library(communigatr STATIC
    src/client.cpp
    src/driver.cpp
    ${GATR2_REPO_ROOT}/common/frame_codec.cpp
)
target_include_directories(communigatr PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GATR2_REPO_ROOT}
)
target_link_libraries(communigatr PUBLIC investigatr)
target_compile_features(communigatr PUBLIC cxx_std_17)

# Host fakes: fake Pi, half-duplex bus, test rig.
add_library(communigatr_fakes STATIC
    sim/fake_bus.cpp
    sim/fake_pi.cpp
    sim/link_rig.cpp
)
target_include_directories(communigatr_fakes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(communigatr_fakes PUBLIC communigatr)

# Test files.
set(COMMUNIGATR_TEST_SOURCES
    tests/fake_pi_gtest.cpp
    tests/client_gtest.cpp
    tests/client_timing_gtest.cpp
    tests/driver_gtest.cpp
    tests/equivalence_gtest.cpp
)

add_executable(communigatr_tests ${COMMUNIGATR_TEST_SOURCES})
target_link_libraries(communigatr_tests PRIVATE
    communigatr_fakes
    investigatr_sim
    GTest::gtest_main
)
gtest_discover_tests(communigatr_tests)

if(NOT MSVC)
    target_compile_options(communigatr PRIVATE -Wall -Wextra)
    target_compile_options(communigatr_fakes PRIVATE -Wall -Wextra)
    target_compile_options(communigatr_tests PRIVATE -Wall -Wextra)
endif()
