cmake_minimum_required(VERSION 3.25)
project(mosquito-laser-killer VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d video)

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

set(HAL_SOURCES
    src/hal/gpio_impl.cpp
    src/hal/spi_impl.cpp
    src/hal/camera_impl.cpp
    src/hal/mcp4922.cpp
    src/hal/differential_galvo_driver.cpp
    src/hal/laser.cpp
)

set(VISION_SOURCES
    src/vision/detector.cpp
    src/vision/stereo_matcher.cpp
    src/vision/tracker.cpp
)

set(CONTROL_SOURCES
    src/control/coordinate_mapper.cpp
    src/control/firing_controller.cpp
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
    Eigen3::Eigen
)

add_executable(mosquito_laser_killer src/main.cpp
    src/safety/system_state.cpp
    src/safety/watchdog.cpp
    src/safety/bounding_box.cpp
    src/safety/arm_switch.cpp
    src/safety/e_stop.cpp
)
target_link_libraries(mosquito_laser_killer PRIVATE
    mosquito_hal
    mosquito_vision
    mosquito_control
    yaml-cpp
    pthread
)

option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    find_package(GTest REQUIRED)
    add_subdirectory(tests)
endif()
