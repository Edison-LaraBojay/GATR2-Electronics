# spectaGATR wire format: portable C++17, no allocation on the encode path.
# The Brain publisher compiles these sources through brain/gatr2_brain.mk.

add_library(spectagatr_protocol STATIC
    src/pose_line.cpp
    src/units.cpp
)
target_include_directories(spectagatr_protocol PUBLIC include)
target_compile_options(spectagatr_protocol PRIVATE ${SPECTAGATR_WARNINGS})

if(SPECTAGATR_BUILD_TESTS)
    add_executable(spectagatr_protocol_tests
        tests/pose_line_gtest.cpp
    )
    target_link_libraries(spectagatr_protocol_tests PRIVATE spectagatr_protocol GTest::gtest_main)
    target_compile_options(spectagatr_protocol_tests PRIVATE ${SPECTAGATR_WARNINGS})
    gtest_discover_tests(spectagatr_protocol_tests)
endif()
