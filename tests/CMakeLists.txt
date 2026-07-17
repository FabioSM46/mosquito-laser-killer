find_package(GTest REQUIRED)

# NOTE: -Werror=unused-result is deliberately NOT relaxed here. It was previously
# disabled, which let 16 discarded [[nodiscard]] safety results through — including
# two tests whose only call was a dropped execute_cycle() result, so they asserted
# nothing at all. If a test discards a safety return, that is the bug.
add_compile_options(-Wno-unused-parameter)

# add_unit_test(<name> [EXTRA_LIBS...]) — every test gets the mocks, the src tree,
# and gtest/gmock. Production code comes from the real libraries rather than
# re-listed source files, so a test can never link a different build of a
# safety component than the binary ships.
function(add_unit_test name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(${name} PRIVATE
        mosquito_hal
        mosquito_safety
        mosquito_vision
        mosquito_control
        Eigen3::Eigen
        GTest::gtest
        GTest::gmock
        GTest::gtest_main
        pthread
    )
    gtest_discover_tests(${name})
endfunction()

#
# Unit Tests
#

add_unit_test(test_safety_guards unit/test_safety_guards.cpp)
add_unit_test(test_watchdog unit/test_watchdog.cpp)
add_unit_test(test_arm_switch unit/test_arm_switch.cpp)
add_unit_test(test_e_stop unit/test_e_stop.cpp)
add_unit_test(test_coordinate_mapper unit/test_coordinate_mapper.cpp)
add_unit_test(test_firing_controller unit/test_firing_controller.cpp)
add_unit_test(test_system_state unit/test_system_state.cpp)
add_unit_test(test_thread_safe_queue unit/test_thread_safe_queue.cpp)
add_unit_test(test_stereo_matcher unit/test_stereo_matcher.cpp)
add_unit_test(test_kalman_tracker unit/test_kalman_tracker.cpp)
add_unit_test(test_differential_galvo_driver unit/test_differential_galvo_driver.cpp)
add_unit_test(test_mcp4922 unit/test_mcp4922.cpp)
add_unit_test(test_detector unit/test_detector.cpp)
add_unit_test(test_signal_handling unit/test_signal_handling.cpp)
add_unit_test(test_config_validator unit/test_config_validator.cpp)
add_unit_test(test_print unit/test_print.cpp)

# Exercises the real control_step() from src/control/control_loop.cpp. This
# replaces test_control_arm_gating.cpp, which hand-copied main()'s gating logic
# into the test and therefore could only ever verify the copy.
add_unit_test(test_control_loop unit/test_control_loop.cpp)

#
# Stress Tests
#

add_unit_test(test_frame_flooding stress/test_frame_flooding.cpp)
add_unit_test(test_watchdog_jitter stress/test_watchdog_jitter.cpp)
add_unit_test(test_concurrent_shutdown stress/test_concurrent_shutdown.cpp)
add_unit_test(test_spi_backpressure stress/test_spi_backpressure.cpp)
