cmake_minimum_required(VERSION 3.25)
project(mosquito-laser-killer VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# An unset build type silently fell into the else() branch below and got
# -O3 -DNDEBUG with no trace of the decision. Same flags, said out loud.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    message(STATUS "CMAKE_BUILD_TYPE not set; defaulting to Release (-O3 -DNDEBUG)")
endif()

# Enforced minimum. 4.5 is the oldest version the code actually uses APIs
# from, and the documented deployment target (Raspberry Pi OS Bookworm) ships
# 4.6 — the "4.8+" once quoted in the docs was never satisfiable there.
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc calib3d video)

if(TARGET opencv_core AND NOT TARGET OpenCV::core)
    add_library(OpenCV::core ALIAS opencv_core)
    add_library(OpenCV::imgproc ALIAS opencv_imgproc)
    add_library(OpenCV::calib3d ALIAS opencv_calib3d)
    add_library(OpenCV::video ALIAS opencv_video)
endif()
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
pkg_check_modules(LIBGPIODCXX REQUIRED libgpiodcxx)

add_compile_options(
    -Wall -Wextra -Wpedantic
    -Werror -Werror=unused-result
    -Wno-unused-parameter
    -march=native
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-O0 -g3)
else()
    add_compile_options(-O3 -DNDEBUG)
endif()

set(CORE_SOURCES
    src/core/config_loader.cpp
)

set(HAL_SOURCES
    src/hal/gpio_impl.cpp
    src/hal/spi_impl.cpp
    src/hal/camera_impl.cpp
    src/hal/mcp4922.cpp
    src/hal/differential_galvo_driver.cpp
    src/hal/laser.cpp
)

set(SAFETY_SOURCES
    src/safety/system_state.cpp
    src/safety/watchdog.cpp
    src/safety/bounding_box.cpp
    src/safety/arm_switch.cpp
    src/safety/e_stop.cpp
    src/safety/signal_handler.cpp
    src/safety/config_validator.cpp
)

set(VISION_SOURCES
    src/vision/detector.cpp
    src/vision/stereo_matcher.cpp
    src/vision/tracker.cpp
    src/vision/multi_tracker.cpp
    src/vision/target_selector.cpp
)

set(CONTROL_SOURCES
    src/control/coordinate_mapper.cpp
    src/control/firing_controller.cpp
    src/control/control_loop.cpp
)

add_library(mosquito_core STATIC ${CORE_SOURCES})
target_include_directories(mosquito_core PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(mosquito_core PUBLIC
    yaml-cpp
    Eigen3::Eigen
)

add_library(mosquito_hal STATIC ${HAL_SOURCES})
target_include_directories(mosquito_hal PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${LIBGPIOD_INCLUDE_DIRS}
)
target_link_libraries(mosquito_hal PUBLIC
    ${LIBGPIOD_LIBRARIES}
    ${LIBGPIODCXX_LIBRARIES}
    OpenCV::core
    OpenCV::imgproc
    Eigen3::Eigen
)

add_library(mosquito_safety STATIC ${SAFETY_SOURCES})
target_include_directories(mosquito_safety PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(mosquito_safety PUBLIC
    mosquito_hal
    Eigen3::Eigen
)

add_library(mosquito_vision STATIC ${VISION_SOURCES})
target_include_directories(mosquito_vision PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(mosquito_vision PUBLIC
    OpenCV::core
    OpenCV::imgproc
    OpenCV::calib3d
    Eigen3::Eigen
)

add_library(mosquito_control STATIC ${CONTROL_SOURCES})
target_include_directories(mosquito_control PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(mosquito_control PUBLIC
    mosquito_hal
    mosquito_safety
    Eigen3::Eigen
)

add_executable(mosquito_laser_killer src/main.cpp)
target_link_libraries(mosquito_laser_killer PRIVATE
    mosquito_core
    mosquito_hal
    mosquito_safety
    mosquito_vision
    mosquito_control
    pthread
)

option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif()
