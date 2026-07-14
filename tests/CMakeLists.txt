find_package(GTest REQUIRED)

add_compile_options(-Wno-unused-parameter -Wno-error=unused-result)

set(SAFETY_SOURCES
    ${CMAKE_SOURCE_DIR}/src/safety/system_state.cpp
    ${CMAKE_SOURCE_DIR}/src/safety/watchdog.cpp
    ${CMAKE_SOURCE_DIR}/src/safety/bounding_box.cpp
)

set(CONTROL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/control/coordinate_mapper.cpp
    ${CMAKE_SOURCE_DIR}/src/control/firing_controller.cpp
)

add_executable(test_safety_guards
    unit/test_safety_guards.cpp
)
target_include_directories(test_safety_guards PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_safety_guards PRIVATE
    mosquito_hal
    Eigen3::Eigen
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_safety_guards)

add_executable(test_watchdog
    unit/test_watchdog.cpp
    ${SAFETY_SOURCES}
)
target_include_directories(test_watchdog PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_watchdog PRIVATE
    mosquito_hal
    Eigen3::Eigen
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_watchdog)

add_executable(test_coordinate_mapper
    unit/test_coordinate_mapper.cpp
    ${CONTROL_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/safety/bounding_box.cpp
)
target_include_directories(test_coordinate_mapper PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_coordinate_mapper PRIVATE
    mosquito_hal
    Eigen3::Eigen
    GTest::gtest
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_coordinate_mapper)

add_executable(test_firing_controller
    unit/test_firing_controller.cpp
    ${CONTROL_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/safety/bounding_box.cpp
)
target_include_directories(test_firing_controller PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_firing_controller PRIVATE
    mosquito_hal
    Eigen3::Eigen
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_firing_controller)

add_executable(test_system_state
    unit/test_system_state.cpp
    ${CMAKE_SOURCE_DIR}/src/safety/system_state.cpp
)
target_include_directories(test_system_state PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_system_state PRIVATE
    GTest::gtest
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_system_state)

add_executable(test_thread_safe_queue
    unit/test_thread_safe_queue.cpp
)
target_include_directories(test_thread_safe_queue PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_thread_safe_queue PRIVATE
    GTest::gtest
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_thread_safe_queue)

add_executable(test_stereo_matcher
    unit/test_stereo_matcher.cpp
)
target_include_directories(test_stereo_matcher PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_stereo_matcher PRIVATE
    mosquito_vision
    Eigen3::Eigen
    GTest::gtest
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_stereo_matcher)

add_executable(test_kalman_tracker
    unit/test_kalman_tracker.cpp
)
target_include_directories(test_kalman_tracker PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_kalman_tracker PRIVATE
    mosquito_vision
    Eigen3::Eigen
    GTest::gtest
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_kalman_tracker)

#
# Stress Tests
#

add_executable(test_frame_flooding
    stress/test_frame_flooding.cpp
)
target_include_directories(test_frame_flooding PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_frame_flooding PRIVATE
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    Eigen3::Eigen
    pthread
)
gtest_discover_tests(test_frame_flooding)

add_executable(test_watchdog_jitter
    stress/test_watchdog_jitter.cpp
    ${SAFETY_SOURCES}
)
target_include_directories(test_watchdog_jitter PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_watchdog_jitter PRIVATE
    mosquito_hal
    Eigen3::Eigen
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_watchdog_jitter)

add_executable(test_concurrent_shutdown
    stress/test_concurrent_shutdown.cpp
    ${SAFETY_SOURCES}
    ${CONTROL_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/safety/bounding_box.cpp
)
target_include_directories(test_concurrent_shutdown PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_concurrent_shutdown PRIVATE
    mosquito_hal
    Eigen3::Eigen
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_concurrent_shutdown)

add_executable(test_spi_backpressure
    stress/test_spi_backpressure.cpp
)
target_include_directories(test_spi_backpressure PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_spi_backpressure PRIVATE
    mosquito_hal
    Eigen3::Eigen
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    pthread
)
gtest_discover_tests(test_spi_backpressure)
