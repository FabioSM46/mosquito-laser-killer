#include "core/types.h"
#include "core/error.h"
#include "core/thread_safe_queue.h"

#include "hal/igpio.h"
#include "hal/ispi.h"
#include "hal/idac.h"
#include "hal/ilaser.h"
#include "hal/igalvo_driver.h"
#include "hal/gpio_impl.h"
#include "hal/spi_impl.h"
#include "hal/camera_impl.h"
#include "hal/mcp4922.h"
#include "hal/differential_galvo_driver.h"
#include "hal/laser.h"

#include "safety/e_stop.h"
#include "safety/system_state.h"
#include "safety/watchdog.h"
#include "safety/bounding_box.h"
#include "safety/arm_switch.h"
#include "safety/signal_handler.h"
#include "safety/config_validator.h"

#include "vision/detector.h"
#include "vision/stereo_matcher.h"
#include "vision/tracker.h"

#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"

#include "core/print.h"
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace {

std::atomic<bool> g_shutdown_requested{false};

auto load_config() -> SystemConfig {
    SystemConfig config{};

    try {
        YAML::Node yaml = YAML::LoadFile("config/system_config.yaml");

        if (yaml["settle_delay_ms"]) {
            config.settle_delay_ms = yaml["settle_delay_ms"].as<double>();
        }
        if (yaml["max_pulse_duration_ms"]) {
            config.max_pulse_duration_ms = yaml["max_pulse_duration_ms"].as<double>();
        }
        if (yaml["cooldown_seconds"]) {
            config.cooldown_seconds = yaml["cooldown_seconds"].as<double>();
        }
        if (yaml["watchdog_missed_threshold"]) {
            config.watchdog_missed_threshold = yaml["watchdog_missed_threshold"].as<uint32_t>();
        }
        if (yaml["frame_width"]) {
            config.frame_width = yaml["frame_width"].as<int>();
        }
        if (yaml["frame_height"]) {
            config.frame_height = yaml["frame_height"].as<int>();
        }
        if (yaml["target_fps"]) {
            config.target_fps = yaml["target_fps"].as<int>();
        }
        if (yaml["spi_device_x"]) {
            config.spi_device_x = yaml["spi_device_x"].as<std::string>();
        }
        if (yaml["spi_device_y"]) {
            config.spi_device_y = yaml["spi_device_y"].as<std::string>();
        }
        if (yaml["spi_speed_hz"]) {
            config.spi_speed_hz = yaml["spi_speed_hz"].as<int>();
        }
        if (yaml["dac_reference_voltage"]) {
            config.dac_ref_voltage = yaml["dac_reference_voltage"].as<double>();
        }
        if (yaml["laser_pin"]) {
            config.laser_pin = yaml["laser_pin"].as<unsigned int>();
        }
        if (yaml["arm_switch_pin"]) {
            config.arm_switch_pin = yaml["arm_switch_pin"].as<unsigned int>();
        }
        if (yaml["e_stop_pin"]) {
            config.e_stop_pin = yaml["e_stop_pin"].as<unsigned int>();
        }
        if (yaml["left_camera_device"] && yaml["left_camera_device"].as<std::string>() != "") {
            config.left_camera_device = yaml["left_camera_device"].as<std::string>();
        }
        if (yaml["right_camera_device"] && yaml["right_camera_device"].as<std::string>() != "") {
            config.right_camera_device = yaml["right_camera_device"].as<std::string>();
        }

        if (yaml["bounding_box"]) {
            auto bb = yaml["bounding_box"];
            config.bounding_box.x_min = bb["x_min"] ? bb["x_min"].as<double>() : -1.0;
            config.bounding_box.x_max = bb["x_max"] ? bb["x_max"].as<double>() : 1.0;
            config.bounding_box.y_min = bb["y_min"] ? bb["y_min"].as<double>() : -1.0;
            config.bounding_box.y_max = bb["y_max"] ? bb["y_max"].as<double>() : 1.0;
            config.bounding_box.z_min = bb["z_min"] ? bb["z_min"].as<double>() : 0.5;
            config.bounding_box.z_max = bb["z_max"] ? bb["z_max"].as<double>() : 5.0;
        }

        if (yaml["galvo_limits"]) {
            auto gl = yaml["galvo_limits"];
            config.galvo_limits.angle_x_min_deg = gl["angle_x_min_deg"]
                ? gl["angle_x_min_deg"].as<double>() : -15.0;
            config.galvo_limits.angle_x_max_deg = gl["angle_x_max_deg"]
                ? gl["angle_x_max_deg"].as<double>() : 15.0;
            config.galvo_limits.angle_y_min_deg = gl["angle_y_min_deg"]
                ? gl["angle_y_min_deg"].as<double>() : -15.0;
            config.galvo_limits.angle_y_max_deg = gl["angle_y_max_deg"]
                ? gl["angle_y_max_deg"].as<double>() : 15.0;
        }

        if (yaml["galvo_driver"]) {
            auto gd = yaml["galvo_driver"];
            config.galvo_driver.input_scale_v_per_deg = gd["input_scale_v_per_deg"]
                ? gd["input_scale_v_per_deg"].as<double>() : 0.33;
            config.galvo_driver.dac_max_diff_voltage = gd["dac_max_diff_voltage"]
                ? gd["dac_max_diff_voltage"].as<double>() : 5.0;
            config.galvo_driver.driver_input_voltage = gd["driver_input_voltage"]
                ? gd["driver_input_voltage"].as<double>() : 15.0;
        }

        if (yaml["camera_optics"]) {
            auto co = yaml["camera_optics"];
            config.camera_optics.lens_focal_length_mm = co["lens_focal_length_mm"]
                ? co["lens_focal_length_mm"].as<double>() : 3.0;
            config.camera_optics.image_sensor_width_mm = co["image_sensor_width_mm"]
                ? co["image_sensor_width_mm"].as<double>() : 3.84;
            config.camera_optics.image_sensor_height_mm = co["image_sensor_height_mm"]
                ? co["image_sensor_height_mm"].as<double>() : 2.4;
        }

        if (yaml["camera_controls"]) {
            auto cc = yaml["camera_controls"];
            config.camera_controls.exposure_auto = cc["exposure_auto"]
                ? cc["exposure_auto"].as<int>() : 1;
            config.camera_controls.exposure_absolute_us = cc["exposure_absolute_us"]
                ? cc["exposure_absolute_us"].as<int>() : 156;
            config.camera_controls.brightness = cc["brightness"]
                ? cc["brightness"].as<int>() : 0;
            config.camera_controls.gamma = cc["gamma"]
                ? cc["gamma"].as<int>() : 100;
            config.camera_controls.sharpness = cc["sharpness"]
                ? cc["sharpness"].as<int>() : 0;
            config.camera_controls.gain = cc["gain"]
                ? cc["gain"].as<int>() : 0;
        }

        if (yaml["stereo"]) {
            auto st = yaml["stereo"];
            config.stereo.baseline_m = st["baseline_m"]
                ? st["baseline_m"].as<double>() : 0.12;
            config.stereo.focal_length_px = st["focal_length_px"]
                ? st["focal_length_px"].as<double>() : 800.0;
            config.stereo.cx = st["cx"] ? st["cx"].as<double>() : 320.0;
            config.stereo.cy = st["cy"] ? st["cy"].as<double>() : 240.0;
        }

        println("[CONFIG] Loaded config/system_config.yaml");

    } catch (const YAML::Exception& e) {
        println(stderr, "[CONFIG] Failed to load config: {}. Using defaults.", e.what());
    }

    return config;
}

}

auto main(int argc, char* argv[]) -> int {
    (void)argc;
    (void)argv;

    println("===========================================");
    println("  Mosquito Laser Killer v1.0.0");
    println("  Stereoscopic Laser Targeting System");
    println("  Differential Galvo Driver (Dual-DAC)");
    println("===========================================");

    SignalHandler signal_handler;
    signal_handler.set_shutdown_callback([&] {
        println(stderr, "\n[SIGNAL] Shutdown requested");
        g_shutdown_requested.store(true, std::memory_order_release);
    });
    signal_handler.install();

    auto config = load_config();

    {
        auto validation_warnings = validate_engagement_volume(config);
        for (const auto& w : validation_warnings) {
            if (w.critical) {
                println(stderr, "[CONFIG] ERROR [{}]: {}", w.category, w.message);
            } else {
                println(stderr, "[CONFIG] WARNING [{}]: {}", w.category, w.message);
            }
        }
        if (has_critical_validation_errors(validation_warnings)) {
            println(stderr, "[CONFIG] Aborting: critical engagement-volume validation failed");
            return 1;
        }
    }

    SystemStateMachine state_machine;

    auto gpio_laser = std::make_unique<GpioImpl>(config.laser_pin);
    auto laser = std::make_unique<Laser>(std::move(gpio_laser), config.laser_pin,
                                         config.max_pulse_duration_ms);

    auto spi_x = std::make_unique<SpiImpl>(config.spi_device_x, config.spi_speed_hz);
    auto spi_y = std::make_unique<SpiImpl>(config.spi_device_y, config.spi_speed_hz);
    auto dac_x = std::make_unique<MCP4922>(std::move(spi_x));
    auto dac_y = std::make_unique<MCP4922>(std::move(spi_y));
    auto galvo = std::make_unique<DifferentialGalvoDriver>(std::move(dac_x), std::move(dac_y));

    println("[MAIN] Laser TTL on GPIO {}", config.laser_pin);
    println("[MAIN] SPI X-axis DAC: {} (CS0)", config.spi_device_x);
    println("[MAIN] SPI Y-axis DAC: {} (CS1)", config.spi_device_y);
    println("[MAIN] SPI speed: {} Hz", config.spi_speed_hz);

    auto gpio_arm = std::make_unique<GpioImpl>(config.arm_switch_pin);
    ArmSwitch arm_switch(std::move(gpio_arm));
    auto arm_init = arm_switch.initialize();
    if (!arm_init.has_value()) {
        println(stderr, "[MAIN] Arm switch init failed: {}",
                     to_string(arm_init.error()));
        (void)state_machine.transition(SystemState::SAFE_HALT);
        return 1;
    }
    println("[MAIN] Arm switch on GPIO {}", config.arm_switch_pin);

    auto gpio_estop = std::make_unique<GpioImpl>(config.e_stop_pin);
    EStop e_stop(std::move(gpio_estop));
    auto estop_init = e_stop.initialize();
    if (!estop_init.has_value()) {
        println(stderr, "[MAIN] E-stop init failed: {}",
                     to_string(estop_init.error()));
        (void)state_machine.transition(SystemState::SAFE_HALT);
        return 1;
    }
    println("[MAIN] E-stop on GPIO {}", config.e_stop_pin);

    if (!laser->is_initialized()) {
        println(stderr, "[MAIN] Laser hardware initialization failed");
        (void)state_machine.transition(SystemState::SAFE_HALT);
        return 1;
    }

    if (!galvo->is_initialized()) {
        println(stderr, "[MAIN] Galvo driver initialization failed");
        (void)state_machine.transition(SystemState::SAFE_HALT);
        return 1;
    }

    BoundingBox3D bounding_box(config.bounding_box);
    CoordinateMapper mapper(bounding_box, config.galvo_limits, config.dac_ref_voltage,
                            config.galvo_driver);
    FiringController firing_controller(*laser, *galvo, mapper,
        config.max_pulse_duration_ms,
        config.cooldown_seconds,
        config.settle_delay_ms);
    Watchdog watchdog(state_machine, *laser, *galvo,
        config.watchdog_missed_threshold, config.target_fps);

    Detector detector_left(config.frame_width, config.frame_height);
    Detector detector_right(config.frame_width, config.frame_height);
    StereoMatcher stereo_matcher(config.stereo);
    KalmanTracker tracker;

    ThreadSafeQueue<StereoFrame> frame_queue;
    ThreadSafeQueue<TargetCommand> target_queue;
    std::atomic<std::chrono::steady_clock::time_point> heartbeat{
        std::chrono::steady_clock::now()};

    auto init_ok = state_machine.transition(SystemState::IDLE);
    if (!init_ok) {
        println(stderr, "[MAIN] Failed to enter IDLE state");
        return 1;
    }
    println("[MAIN] System ready in IDLE state");

    // Capture thread may only set the atomic; control thread owns laser/state cleanup.
    auto request_system_halt = [&](const char* reason) {
        println(stderr, "[MAIN] System halt requested: {}", reason);
        g_shutdown_requested.store(true, std::memory_order_release);
    };

    println("[MAIN] Starting capture thread...");
    std::jthread capture_thread([&](std::stop_token stoken) {
        println("[CAPTURE] Thread started");
        uint64_t frame_id = 0;

        std::string left_dev = config.left_camera_device.empty()
            ? "/dev/video0" : config.left_camera_device;
        std::string right_dev = config.right_camera_device.empty()
            ? "/dev/video2" : config.right_camera_device;

        println("[CAPTURE] Left camera: {}", left_dev);
        println("[CAPTURE] Right camera: {}", right_dev);

        auto left_cam_ptr = std::make_unique<CameraImpl>(
            left_dev, config.frame_width, config.frame_height, config.target_fps,
            config.camera_controls);
        auto right_cam_ptr = std::make_unique<CameraImpl>(
            right_dev, config.frame_width, config.frame_height, config.target_fps,
            config.camera_controls);
        auto& left_cam = *left_cam_ptr;
        auto& right_cam = *right_cam_ptr;

        auto open_left = left_cam.open(0);
        if (!open_left.has_value()) {
            println(stderr, "[CAPTURE] Failed to open left camera: {}",
                         to_string(open_left.error()));
            request_system_halt("left camera open failed");
            return;
        }
        auto open_right = right_cam.open(1);
        if (!open_right.has_value()) {
            println(stderr, "[CAPTURE] Failed to open right camera: {}",
                         to_string(open_right.error()));
            request_system_halt("right camera open failed");
            return;
        }

        auto cycle_period = std::chrono::microseconds(1'000'000 / config.target_fps);

        while (!stoken.stop_requested() &&
               !g_shutdown_requested.load(std::memory_order_acquire) &&
               !signal_handler.is_shutdown_requested()) {
            auto cycle_start = std::chrono::steady_clock::now();

            StereoFrame frame;
            frame.frame_id = frame_id++;
            frame.timestamp = cycle_start;
            frame.left_frame.resize(static_cast<size_t>(config.frame_width) * config.frame_height);
            frame.right_frame.resize(static_cast<size_t>(config.frame_width) * config.frame_height);

            auto left_result = left_cam.capture(frame.left_frame.data(),
                                                 frame.left_frame.size());
            if (!left_result.has_value()) {
                println(stderr, "[CAPTURE] Left camera capture failed: {}",
                             to_string(left_result.error()));
                request_system_halt("left camera capture failed");
                break;
            }

            auto right_result = right_cam.capture(frame.right_frame.data(),
                                                    frame.right_frame.size());
            if (!right_result.has_value()) {
                println(stderr, "[CAPTURE] Right camera capture failed: {}",
                             to_string(right_result.error()));
                request_system_halt("right camera capture failed");
                break;
            }

            frame_queue.push(std::move(frame));

            auto cycle_end = std::chrono::steady_clock::now();
            auto elapsed = cycle_end - cycle_start;
            if (elapsed < cycle_period) {
                std::this_thread::sleep_for(cycle_period - elapsed);
            }
        }

        println("[CAPTURE] Thread exiting");
    });

    println("[MAIN] Starting processing thread...");
    std::jthread processing_thread([&](std::stop_token stoken) {
        println("[PROCESSING] Thread started");
        auto max_wait = std::chrono::milliseconds(16);

        while (!stoken.stop_requested() &&
               !g_shutdown_requested.load(std::memory_order_acquire) &&
               !signal_handler.is_shutdown_requested()) {
            auto frames = frame_queue.drain_all();

            std::optional<StereoFrame> latest_frame;
            if (!frames.empty()) {
                if (frames.size() > 1) {
                    println("[PROCESSING] Dropped {} stale frames",
                                 frames.size() - 1);
                }
                latest_frame = std::move(frames.back());
            } else {
                auto popped = frame_queue.pop(max_wait);
                if (popped.has_value()) {
                    latest_frame = std::move(popped.value());
                }
            }

            if (!latest_frame.has_value()) {
                continue;
            }

            auto& frame = latest_frame.value();
            heartbeat.store(std::chrono::steady_clock::now(), std::memory_order_release);

            auto left_detection = detector_left.detect(frame.left_frame.data(),
                                                          frame.left_frame.size());
            auto right_detection = detector_right.detect(frame.right_frame.data(),
                                                           frame.right_frame.size());

            TargetCommand cmd;
            cmd.frame_id = frame.frame_id;
            cmd.timestamp = frame.timestamp;

            auto target_3d = left_detection
                .and_then([&](const Pixel2D& left_pt) -> std::optional<Point3D> {
                    return right_detection.and_then([&](const Pixel2D& right_pt) {
                        return stereo_matcher.triangulate(left_pt, right_pt);
                    });
                });

            cmd.target_valid = target_3d.has_value();

            target_3d
                .and_then([&](const Point3D& pt) -> std::optional<Point3D> {
                    return tracker.update(pt, frame.timestamp);
                })
                .transform([&](const Point3D& filtered) {
                    cmd.target_position = filtered;
                    return filtered;
                })
                .or_else([&]() -> std::optional<Point3D> {
                    if (!cmd.target_valid) {
                        tracker.reset();
                    }
                    return std::nullopt;
                });

            target_queue.push(std::move(cmd));
        }

        println("[PROCESSING] Thread exiting");
    });

    println("[MAIN] Starting control thread...");
    std::jthread control_thread([&](std::stop_token stoken) {
        println("[CONTROL] Thread started");
        auto max_wait = std::chrono::milliseconds(16);
        auto cycle_period = std::chrono::microseconds(1'000'000 / config.target_fps);

        while (!stoken.stop_requested() &&
               !g_shutdown_requested.load(std::memory_order_acquire) &&
               !signal_handler.is_shutdown_requested()) {
            auto cycle_start = std::chrono::steady_clock::now();

            auto now = std::chrono::steady_clock::now();
            laser->enforce_max_pulse(now);
            if (!watchdog.check(now)) {
                println(stderr, "[CONTROL] Watchdog triggered, halting");
                firing_controller.emergency_stop();
                (void)state_machine.transition(SystemState::SAFE_HALT);
                break;
            }

            e_stop.update();
            if (e_stop.is_pressed()) {
                println(stderr, "[CONTROL] E-STOP PRESSED, halting");
                firing_controller.emergency_stop();
                (void)state_machine.transition(SystemState::SAFE_HALT);
                break;
            }

            arm_switch.update();
            bool is_armed = arm_switch.is_armed();
            auto cs = state_machine.current();

            // Structural arm gate: controller refuses targets/fire when disarmed.
            firing_controller.set_armed(is_armed);

            if (!is_armed && cs != SystemState::IDLE &&
                cs != SystemState::SAFE_HALT && cs != SystemState::COOLDOWN) {
                println("[CONTROL] Arm switch OFF, disarming");
                if (cs == SystemState::FIRING) {
                    (void)state_machine.transition(SystemState::COOLDOWN);
                } else {
                    (void)state_machine.transition(SystemState::IDLE);
                }
            }

            if (is_armed && cs == SystemState::IDLE) {
                println("[CONTROL] Arm switch ON, arming");
                (void)state_machine.transition(SystemState::ARMED);
            }

            auto cmd_opt = target_queue.pop(max_wait);

            watchdog.feed(heartbeat.load(std::memory_order_acquire));

            if (cmd_opt.has_value()) {
                auto& cmd = cmd_opt.value();
                auto current_state = state_machine.current();

                if (!is_armed) {
                    firing_controller.clear_target();
                } else if (current_state == SystemState::FIRING) {
                    // Hold aim while firing (motion blanking). Abort only on loss.
                    if (!cmd.target_valid || !cmd.target_position.has_value()) {
                        (void)state_machine.transition(SystemState::COOLDOWN);
                        firing_controller.clear_target();
                    }
                } else if (cmd.target_valid && cmd.target_position.has_value() &&
                           (current_state == SystemState::ARMED ||
                            current_state == SystemState::TRACKING)) {
                    if (current_state == SystemState::ARMED) {
                        (void)state_machine.transition(SystemState::TRACKING);
                    }
                    firing_controller.set_target(cmd.target_position.value());
                } else {
                    if (current_state == SystemState::TRACKING) {
                        (void)state_machine.transition(SystemState::ARMED);
                    }
                    firing_controller.clear_target();
                }
            }

            bool pulse_was_active = firing_controller.is_firing();
            const auto exec_now = std::chrono::steady_clock::now();
            bool pulse_ended = firing_controller.execute_cycle(exec_now);
            if (pulse_ended && state_machine.current() == SystemState::FIRING) {
                (void)state_machine.transition(SystemState::COOLDOWN);
            } else if (!pulse_was_active && firing_controller.is_firing() &&
                       state_machine.current() == SystemState::TRACKING) {
                (void)state_machine.transition(SystemState::FIRING);
            }

            if (state_machine.current() == SystemState::COOLDOWN &&
                !firing_controller.is_firing() &&
                firing_controller.may_fire()) {
                (void)state_machine.transition(SystemState::IDLE);
                if (is_armed) {
                    (void)state_machine.transition(SystemState::ARMED);
                }
            }

            if (state_machine.current() == SystemState::SAFE_HALT) {
                println(stderr, "[CONTROL] SAFE_HALT detected, breaking control loop");
                break;
            }

            auto cycle_end = std::chrono::steady_clock::now();
            auto elapsed = cycle_end - cycle_start;
            if (elapsed < cycle_period) {
                std::this_thread::sleep_for(cycle_period - elapsed);
            }
        }

        auto halt_result = laser->emergency_shutdown();
        if (!halt_result.has_value()) {
            println(stderr, "[CONTROL] Final emergency shutdown failed: {}",
                         to_string(halt_result.error()));
        }
        auto galvo_result = galvo->zero();
        if (!galvo_result.has_value()) {
            println(stderr, "[CONTROL] Final galvo zero failed: {}",
                         to_string(galvo_result.error()));
        }

        println("[CONTROL] Thread exiting");
    });

    capture_thread.join();
    processing_thread.join();
    control_thread.join();

    println("[MAIN] All threads joined. System shutdown complete.");
    println("[MAIN] Laser GPIO LOW, galvos at center (0V differential)");

    return 0;
}
