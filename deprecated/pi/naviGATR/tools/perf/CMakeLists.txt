# Inspection and viewer performance harness. Built from pi/naviGATR with
# -DNAVIGATR_BUILD_PERF_TOOLS=ON, or standalone against any naviGATR tree:
#   cmake -S tools/perf -B <dir> -DNAVIGATR_SOURCE_DIR=<naviGATR tree>
cmake_minimum_required(VERSION 3.16)

if(NOT TARGET navigatr_core)
    project(navigatr_perf CXX)
    set(NAVIGATR_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "naviGATR tree to measure")
    set(NAVIGATR_BUILD_TESTS OFF)
    enable_testing()
    add_subdirectory(${NAVIGATR_SOURCE_DIR} navigatr)
endif()

add_library(navigatr_perf_common STATIC perf_net.cpp perf_json.cpp perf_sys.cpp perf_analysis.cpp)
target_include_directories(navigatr_perf_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if(WIN32)
    target_link_libraries(navigatr_perf_common PUBLIC ws2_32)
endif()

add_executable(navigatr_inspect_bench inspect_bench.cpp)
target_link_libraries(navigatr_inspect_bench PRIVATE navigatr_perf_common navigatr_core)

add_executable(navigatr_cdp_bench cdp_bench.cpp)
target_link_libraries(navigatr_cdp_bench PRIVATE navigatr_perf_common)
target_compile_definitions(navigatr_cdp_bench PRIVATE
    NAVIGATR_PERF_PROBE_JS="${CMAKE_CURRENT_SOURCE_DIR}/perf_probe.js")

add_executable(navigatr_perf_report perf_report.cpp)
target_link_libraries(navigatr_perf_report PRIVATE navigatr_perf_common)

add_executable(navigatr_perf_selftest perf_selftest.cpp)
target_link_libraries(navigatr_perf_selftest PRIVATE navigatr_perf_common)

if(NOT MSVC)
    foreach(t navigatr_perf_common navigatr_inspect_bench navigatr_cdp_bench navigatr_perf_report
              navigatr_perf_selftest)
        target_compile_options(${t} PRIVATE -Wall -Wextra)
    endforeach()
endif()

# Tool checks: the estimator, and the report's labels on a fixture results
# folder (testdata/report_v2, an inspect/2-shaped run).
add_test(NAME perf_selftest COMMAND navigatr_perf_selftest)
set(PERF_FIXTURE "${CMAKE_CURRENT_SOURCE_DIR}/testdata/report_v2=after")
add_test(NAME perf_report_not_delivered_label COMMAND navigatr_perf_report ${PERF_FIXTURE})
set_tests_properties(perf_report_not_delivered_label PROPERTIES
    PASS_REGULAR_EXPRESSION "state messages not delivered, whole scenario \\(inspect/1: refused. inspect/2: replaced unsent or refused\\) \\| 229 \\|"
    FAIL_REGULAR_EXPRESSION "snapshots refused")
add_test(NAME perf_report_refused_row COMMAND navigatr_perf_report ${PERF_FIXTURE})
set_tests_properties(perf_report_refused_row PROPERTIES
    PASS_REGULAR_EXPRESSION "messages refused, all clients and channels \\| 0 \\|")
add_test(NAME perf_report_server_view COMMAND navigatr_perf_report ${PERF_FIXTURE})
set_tests_properties(perf_report_server_view PROPERTIES
    PASS_REGULAR_EXPRESSION "server view: `state` sent / replaced unsent / refused \\| 171 / 229 / 0 \\|")
add_test(NAME perf_report_minus_build COMMAND navigatr_perf_report ${PERF_FIXTURE})
set_tests_properties(perf_report_minus_build PROPERTIES
    PASS_REGULAR_EXPRESSION "minus p50 build time of that document, ms \\(queue \\+ socket \\+ read, estimate\\) \\| 836\\.94 \\|")
add_test(NAME perf_report_wide_offset COMMAND navigatr_perf_report ${PERF_FIXTURE})
set_tests_properties(perf_report_wide_offset PROPERTIES
    PASS_REGULAR_EXPRESSION "estimates too wide to use / all \\| 160 / 160 \\|")
add_test(NAME perf_report_net_throttle_flag COMMAND navigatr_perf_report ${PERF_FIXTURE})
set_tests_properties(perf_report_net_throttle_flag PROPERTIES
    PASS_REGULAR_EXPRESSION "100 kB/s requested. the page received 1100 kB/s over its WebSockets \\(median\\)\\. The throttle was NOT applied")
# older inspect/1 results (the baseline): no estimates, no inspect/2 counters
set(PERF_FIXTURE_V1 "${CMAKE_CURRENT_SOURCE_DIR}/testdata/report_v1=baseline")
add_test(NAME perf_report_v1_refused COMMAND navigatr_perf_report ${PERF_FIXTURE_V1})
set_tests_properties(perf_report_v1_refused PROPERTIES
    PASS_REGULAR_EXPRESSION "state messages not delivered, whole scenario \\(inspect/1: refused. inspect/2: replaced unsent or refused\\) \\| 346 \\|"
    FAIL_REGULAR_EXPRESSION "estimates too wide to use / all \\| [0-9]")
add_test(NAME perf_report_v1_minus_build COMMAND navigatr_perf_report ${PERF_FIXTURE_V1})
set_tests_properties(perf_report_v1_minus_build PROPERTIES
    PASS_REGULAR_EXPRESSION "minus p50 build time of that document, ms \\(queue \\+ socket \\+ read, estimate\\) \\| 0\\.86 \\|")
