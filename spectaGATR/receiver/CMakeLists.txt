# spectagatr laptop receiver. Include root: spectaGATR/ ("receiver/feed/pose_feed.h").
# OS code lives in platform/ only; every per-OS .cpp guards itself with #if.

find_package(Threads REQUIRED)

set(SPECTAGATR_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(SPECTAGATR_WEB_SRC ${SPECTAGATR_ROOT}/web)
set(SPECTAGATR_FIELDS_SRC ${SPECTAGATR_ROOT}/fields)

function(spectagatr_library name)
    add_library(${name} STATIC ${ARGN})
    target_include_directories(${name} PUBLIC ${SPECTAGATR_ROOT})
    target_compile_features(${name} PUBLIC cxx_std_17)
    target_compile_options(${name} PRIVATE ${SPECTAGATR_WARNINGS})
endfunction()

# Static on MinGW: runs without the ucrt64 runtime DLLs on PATH.
function(spectagatr_static_runtime target)
    if(MINGW)
        target_link_options(${target} PRIVATE -static)
    endif()
endfunction()

spectagatr_library(spectagatr_receiver_platform
    platform/browser.cpp
    platform/process.cpp
    platform/serial_port.cpp
    platform/serial_port_posix.cpp
    platform/serial_port_win32.cpp
    platform/stop_signal.cpp
    platform/usb_ports.cpp
    platform/usb_ports_linux.cpp
    platform/usb_ports_macos.cpp
    platform/usb_ports_win32.cpp
    platform/vex_ports.cpp
    platform/win32_text.cpp
)
target_link_libraries(spectagatr_receiver_platform PUBLIC Threads::Threads)
if(WIN32)
    target_link_libraries(spectagatr_receiver_platform PRIVATE setupapi cfgmgr32 ole32 shell32)
elseif(APPLE)
    target_link_libraries(spectagatr_receiver_platform PRIVATE
        "-framework IOKit" "-framework CoreFoundation")
endif()

# Bytes to poses to JSON: protocol only, no OS code.
spectagatr_library(spectagatr_receiver_feed
    feed/feed_json.cpp
    feed/feed_status.cpp
    feed/pose_feed.cpp
    feed/pose_tracker.cpp
)
target_link_libraries(spectagatr_receiver_feed PUBLIC spectagatr_protocol)

spectagatr_library(spectagatr_receiver_sources
    sources/brain_usb.cpp
    sources/cdc2.cpp
    sources/chunk.cpp
    sources/controller.cpp
    sources/replay.cpp
    sources/serial_source.cpp
    sources/source.cpp
    sources/vex_port.cpp
    sim/sim_source.cpp
    sim/trajectory.cpp
)
target_link_libraries(spectagatr_receiver_sources PUBLIC
    spectagatr_receiver_platform spectagatr_protocol)

spectagatr_library(spectagatr_receiver_server
    server/http_server.cpp
    server/static_site.cpp
)
target_link_libraries(spectagatr_receiver_server PUBLIC spectagatr_receiver_feed Threads::Threads)
if(WIN32)
    target_link_libraries(spectagatr_receiver_server PUBLIC ws2_32)
endif()

spectagatr_library(spectagatr_receiver_app
    app/asset_dirs.cpp
    app/command_line.cpp
    app/live_feed.cpp
    app/run_source.cpp
)
target_link_libraries(spectagatr_receiver_app PUBLIC
    spectagatr_receiver_server spectagatr_receiver_sources spectagatr_receiver_feed)

add_executable(spectagatr app/main.cpp)
target_link_libraries(spectagatr PRIVATE spectagatr_receiver_app)
target_compile_options(spectagatr PRIVATE ${SPECTAGATR_WARNINGS})
target_compile_definitions(spectagatr PRIVATE
    SPECTAGATR_SOURCE_WEB_DIR="${SPECTAGATR_WEB_SRC}"
    SPECTAGATR_SOURCE_FIELDS_DIR="${SPECTAGATR_FIELDS_SRC}")
spectagatr_static_runtime(spectagatr)

# web/ and fields/*.json next to the program, copied again on every build.
add_custom_target(spectagatr_assets ALL
    COMMAND ${CMAKE_COMMAND}
        -DWEB_SRC=${SPECTAGATR_WEB_SRC}
        -DFIELDS_SRC=${SPECTAGATR_FIELDS_SRC}
        -DDEST=$<TARGET_FILE_DIR:spectagatr>
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/copy_assets.cmake
    COMMENT "Copying spectaGATR web/ and fields/ next to spectagatr"
    VERBATIM)
add_dependencies(spectagatr_assets spectagatr)

# cmake --install <build> --prefix <dir>: bin/spectagatr, share/spectagatr/{web,fields}.
install(TARGETS spectagatr RUNTIME DESTINATION bin)
install(DIRECTORY ${SPECTAGATR_WEB_SRC}/ DESTINATION share/spectagatr/web
    OPTIONAL PATTERN "tests" EXCLUDE)
install(DIRECTORY ${SPECTAGATR_FIELDS_SRC}/ DESTINATION share/spectagatr/fields
    OPTIONAL FILES_MATCHING PATTERN "*.json")

if(SPECTAGATR_BUILD_TESTS)
    spectagatr_library(spectagatr_receiver_fakes
        sources/fakes/brain_stdio.cpp
        sources/fakes/fake_controller.cpp
        sources/fakes/fake_port.cpp
    )
    target_link_libraries(spectagatr_receiver_fakes PUBLIC
        spectagatr_receiver_sources spectagatr_protocol)

    add_executable(spectagatr_receiver_tests
        tests/asset_dirs_gtest.cpp
        tests/brain_usb_gtest.cpp
        tests/browser_gtest.cpp
        tests/cdc2_gtest.cpp
        tests/command_line_gtest.cpp
        tests/controller_gtest.cpp
        tests/fakes_gtest.cpp
        tests/feed_json_gtest.cpp
        tests/feed_status_gtest.cpp
        tests/http_server_gtest.cpp
        tests/pose_feed_gtest.cpp
        tests/pose_tracker_gtest.cpp
        tests/replay_gtest.cpp
        tests/serial_port_gtest.cpp
        tests/sim_gtest.cpp
        tests/static_site_gtest.cpp
        tests/stop_signal_gtest.cpp
        tests/usb_ports_gtest.cpp
        tests/vex_port_gtest.cpp
        tests/vex_ports_gtest.cpp
    )
    target_link_libraries(spectagatr_receiver_tests PRIVATE
        spectagatr_receiver_app spectagatr_receiver_fakes GTest::gtest_main)
    if(UNIX AND NOT APPLE)
        target_link_libraries(spectagatr_receiver_tests PRIVATE util) # openpty
    endif()
    target_compile_options(spectagatr_receiver_tests PRIVATE ${SPECTAGATR_WARNINGS})
    spectagatr_static_runtime(spectagatr_receiver_tests)
    gtest_discover_tests(spectagatr_receiver_tests PROPERTIES TIMEOUT 60)
endif()
