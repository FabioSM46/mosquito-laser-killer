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
#include "control/control_loop.h"

#include "core/print.h"
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace {

std::atomic<bool> g_shutdown_requested{false};

// Set when a worker thread halts the system because of a hardware fault, as
// opposed to an operator-requested shutdown. The capture thread may only touch
// atomics — the control thread owns the laser and the state machine — so a
// capture failure cannot reach SAFE_HALT by itself, and without this flag it
// would exit 0 and look identical to a clean Ctrl-C to a supervisor.
std::atomic<bool> g_hardware_fault{false};

namespace exit_code {
constexpr int ok = 0;
constexpr int config_error = 1;
constexpr int hardware_init_failed = 2;
constexpr int safe_halt = 3;
}

// Assigns only when the key is present, so types.h stays the single source of
// truth for defaults. The previous `node[k] ? node[k].as<T>() : <literal>` style
// duplicated every default at the call site, and one had already drifted (cy was
// 240 here versus 200 in types.h — a 40 px principal-point error).
template <typename T>
auto load_field(const YAML::Node& parent, const char* key, T& out) -> void {
    if (parent && parent[key]) {
        out = parent[key].as<T>();
    }
}

auto load_config() -> SystemConfig {
    SystemConfig config{};

    try {
        YAML::Node yaml = YAML::LoadFile("config/system_config.yaml");

        load_field(yaml, "settle_delay_ms", config.settle_delay_ms);
        load_field(yaml, "max_pulse_duration_ms", config.max_pulse_duration_ms);
        load_field(yaml, "cooldown_seconds", config.cooldown_seconds);
        load_field(yaml, "watchdog_timeout_ms", config.watchdog_timeout_ms);
        load_field(yaml, "watchdog_startup_grace_ms", config.watchdog_startup_grace_ms);
        load_field(yaml, "frame_width", config.frame_width);
        load_field(yaml, "frame_height", config.frame_height);
        load_field(yaml, "target_fps", config.target_fps);
        load_field(yaml, "spi_device_x", config.spi_device_x);
        load_field(yaml, "spi_device_y", config.spi_device_y);
        load_field(yaml, "spi_speed_hz", config.spi_speed_hz);
        load_field(yaml, "dac_reference_voltage", config.dac_ref_voltage);
        load_field(yaml, "laser_pin", config.laser_pin);
        load_field(yaml, "arm_switch_pin", config.arm_switch_pin);
        load_field(yaml, "e_stop_pin", config.e_stop_pin);
        load_field(yaml, "left_camera_device", config.left_camera_device);
        load_field(yaml, "right_camera_device", config.right_camera_device);

        auto bb = yaml["bounding_box"];
        load_field(bb, "x_min", config.bounding_box.x_min);
        load_field(bb, "x_max", config.bounding_box.x_max);
        load_field(bb, "y_min", config.bounding_box.y_min);
        load_field(bb, "y_max", config.bounding_box.y_max);
        load_field(bb, "z_min", config.bounding_box.z_min);
        load_field(bb, "z_max", config.bounding_box.z_max);

        auto gl = yaml["galvo_limits"];
        load_field(gl, "angle_x_min_deg", config.galvo_limits.angle_x_min_deg);
        load_field(gl, "angle_x_max_deg", config.galvo_limits.angle_x_max_deg);
        load_field(gl, "angle_y_min_deg", config.galvo_limits.angle_y_min_deg);
        load_field(gl, "angle_y_max_deg", config.galvo_limits.angle_y_max_deg);

        auto gd = yaml["galvo_driver"];
        load_field(gd, "input_scale_v_per_deg", config.galvo_driver.input_scale_v_per_deg);
        load_field(gd, "dac_max_diff_voltage", config.galvo_driver.dac_max_diff_voltage);
        load_field(gd, "driver_input_voltage", config.galvo_driver.driver_input_voltage);

        auto co = yaml["camera_optics"];
        load_field(co, "lens_focal_length_mm", config.camera_optics.lens_focal_length_mm);
        load_field(co, "image_sensor_width_mm", config.camera_optics.image_sensor_width_mm);
        load_field(co, "image_sensor_height_mm", config.camera_optics.image_sensor_height_mm);

        auto cc = yaml["camera_controls"];
        load_field(cc, "exposure_auto", config.camera_controls.exposure_auto);
        load_field(cc, "exposure_absolute_us", config.camera_controls.exposure_absolute_us);
        load_field(cc, "brightness", config.camera_controls.brightness);
        load_field(cc, "gamma", config.camera_controls.gamma);
        load_field(cc, "sharpness", config.camera_controls.sharpness);
        load_field(cc, "gain", config.camera_controls.gain);

        auto st = yaml["stereo"];
        load_field(st, "baseline_m", config.stereo.baseline_m);
        load_field(st, "focal_length_px", config.stereo.focal_length_px);
        load_field(st, "cx", config.stereo.cx);
        load_field(st, "cy", config.stereo.cy);

        auto det = yaml["detection"];
        load_field(det, "threshold", config.detection.threshold);
        load_field(det, "min_blob_area_px", config.detection.min_blob_area_px);
        load_field(det, "max_blob_area_px", config.detection.max_blob_area_px);
        load_field(det, "max_blobs", config.detection.max_blobs);
        load_field(det, "epipolar_tolerance_px", config.detection.epipolar_tolerance_px);
        load_field(det, "target_size_m", config.detection.target_size_m);

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

    // Before any thread starts: the control thread must never block on a log
    // write while it owns a live laser pulse.
    mlk_log::log_init();

    println("===========================================");
    println("  Mosquito Laser Killer v1.0.0");
    println("  Stereoscopic Laser Targeting System");
    println("  Differential Galvo Driver (Dual-DAC)");
    println("===========================================");

    SignalHandler signal_handler;
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
            return exit_code::config_error;
        }
    }

    SystemStateMachine state_machine;

    // Declaration order is load-bearing: destruction runs in reverse, and the
    // laser pin must be forced LOW before the galvos are commanded anywhere.
    // Declaring the laser last makes ~Laser the FIRST of these to run.
    auto spi_x = std::make_unique<SpiImpl>(config.spi_device_x, config.spi_speed_hz);
    auto spi_y = std::make_unique<SpiImpl>(config.spi_device_y, config.spi_speed_hz);
    auto dac_x = std::make_unique<MCP4922>(std::move(spi_x));
    auto dac_y = std::make_unique<MCP4922>(std::move(spi_y));
    auto galvo = std::make_unique<DifferentialGalvoDriver>(std::move(dac_x), std::move(dac_y));

    auto gpio_laser = std::make_unique<GpioImpl>(config.laser_pin);
    auto laser = std::make_unique<Laser>(std::move(gpio_laser), config.laser_pin,
                                         config.max_pulse_duration_ms);

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
        return exit_code::hardware_init_failed;
    }
    println("[MAIN] Arm switch on GPIO {}", config.arm_switch_pin);

    auto gpio_estop = std::make_unique<GpioImpl>(config.e_stop_pin);
    EStop e_stop(std::move(gpio_estop));
    auto estop_init = e_stop.initialize();
    if (!estop_init.has_value()) {
        println(stderr, "[MAIN] E-stop init failed: {}",
                     to_string(estop_init.error()));
        (void)state_machine.transition(SystemState::SAFE_HALT);
        return exit_code::hardware_init_failed;
    }
    println("[MAIN] E-stop on GPIO {}", config.e_stop_pin);

    if (!laser->is_initialized()) {
        println(stderr, "[MAIN] Laser hardware initialization failed");
        (void)state_machine.transition(SystemState::SAFE_HALT);
        return exit_code::hardware_init_failed;
    }

    if (!galvo->is_initialized()) {
        println(stderr, "[MAIN] Galvo driver initialization failed");
        (void)state_machine.transition(SystemState::SAFE_HALT);
        return exit_code::hardware_init_failed;
    }

    BoundingBox3D bounding_box(config.bounding_box);
    CoordinateMapper mapper(bounding_box, config.galvo_limits, config.dac_ref_voltage,
                            config.galvo_driver);
    FiringController firing_controller(*laser, *galvo, mapper,
        config.max_pulse_duration_ms,
        config.cooldown_seconds,
        config.settle_delay_ms,
        std::chrono::steady_clock::now());
    Watchdog watchdog(state_machine, *laser, *galvo,
        std::chrono::milliseconds(static_cast<long>(config.watchdog_timeout_ms)),
        std::chrono::milliseconds(static_cast<long>(config.watchdog_startup_grace_ms)));

    Detector detector_left(config.frame_width, config.frame_height, config.detection);
    Detector detector_right(config.frame_width, config.frame_height, config.detection);
    StereoMatcher stereo_matcher(config.stereo, config.detection, config.bounding_box);
    KalmanTracker tracker;

    ThreadSafeQueue<StereoFrame> frame_queue;
    ThreadSafeQueue<TargetCommand> target_queue;

    // Starts at the sentinel, not at now(): the control thread forwards this value
    // to the watchdog every cycle, and a real timestamp here would look like a
    // heartbeat from a processing thread that has not yet run.
    std::atomic<std::chrono::steady_clock::time_point> heartbeat{
        std::chrono::steady_clock::time_point::min()};

    auto init_ok = state_machine.transition(SystemState::IDLE);
    if (!init_ok) {
        println(stderr, "[MAIN] Failed to enter IDLE state");
        return exit_code::hardware_init_failed;
    }
    println("[MAIN] System ready in IDLE state");

    // Capture thread may only set atomics; control thread owns laser/state cleanup.
    auto request_system_halt = [&](const char* reason) {
        println(stderr, "[MAIN] System halt requested: {}", reason);
        g_hardware_fault.store(true, std::memory_order_release);
        g_shutdown_requested.store(true, std::memory_order_release);
    };

    println("[MAIN] Starting capture thread...");
    std::jthread capture_thread([&](std::stop_token stoken) {
        println("[CAPTURE] Thread started");
        uint64_t frame_id = 0;

        std::string left_dev = config.left_camera_device;
        std::string right_dev = config.right_camera_device;

        if (left_dev.empty() || right_dev.empty()) {
            println(stderr, "[CAPTURE] Camera device paths are not configured. "
                            "Use stable /dev/v4l/by-path/ symlinks in config/system_config.yaml");
            request_system_halt("camera device paths not configured");
            return;
        }

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
        const size_t frame_bytes =
            static_cast<size_t>(config.frame_width) * config.frame_height;

        while (!stoken.stop_requested() &&
               !g_shutdown_requested.load(std::memory_order_acquire) &&
               !signal_handler.is_shutdown_requested()) {
            auto cycle_start = std::chrono::steady_clock::now();

            StereoFrame frame;
            frame.frame_id = frame_id++;
            frame.timestamp = cycle_start;
            frame.left_frame.resize(frame_bytes);
            frame.right_frame.resize(frame_bytes);

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

            auto left_blobs = detector_left.detect_blobs(frame.left_frame.data(),
                                                         frame.left_frame.size());
            auto right_blobs = detector_right.detect_blobs(frame.right_frame.data(),
                                                           frame.right_frame.size());

            TargetCommand cmd;
            cmd.frame_id = frame.frame_id;
            cmd.timestamp = frame.timestamp;

            // Correspondence is established per blob and validated against the
            // epipolar constraint; an ambiguous scene yields no target.
            auto target_3d = stereo_matcher.match(left_blobs, right_blobs);

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
        auto cycle_period = std::chrono::microseconds(1'000'000 / config.target_fps);

        ControlDeps deps{state_machine, firing_controller, watchdog,
                         arm_switch, e_stop, *laser};

        while (!stoken.stop_requested() &&
               !g_shutdown_requested.load(std::memory_order_acquire) &&
               !signal_handler.is_shutdown_requested()) {
            auto cycle_start = std::chrono::steady_clock::now();

            // Take the freshest command and drop the rest, for the same reason the
            // processing thread drops stale frames: acting on a queued command
            // aims at a position the target has already left. A blocking pop here
            // would also widen the interval between max-pulse checks.
            std::optional<TargetCommand> cmd;
            auto commands = target_queue.drain_all();
            if (!commands.empty()) {
                cmd = std::move(commands.back());
            }

            auto outcome = control_step(deps, cmd,
                                        heartbeat.load(std::memory_order_acquire),
                                        cycle_start);
            if (outcome == ControlOutcome::Halt) {
                // Every halt path must stop the other threads, or the joins below
                // block forever.
                g_shutdown_requested.store(true, std::memory_order_release);
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

    const auto final_state = state_machine.current();
    if (final_state == SystemState::SAFE_HALT) {
        println(stderr, "[MAIN] Shutdown complete in SAFE_HALT — a safety interlock "
                "fired. Review the log above before restarting.");
        mlk_log::log_shutdown();
        return exit_code::safe_halt;
    }

    // A capture-thread hardware fault never reaches SAFE_HALT (that thread may
    // only set atomics), so without this check it would fall through to exit 0 and
    // a supervisor with Restart=on-failure would leave the system dead.
    if (g_hardware_fault.load(std::memory_order_acquire)) {
        println(stderr, "[MAIN] Shutdown complete after a hardware fault. "
                "Review the log above before restarting.");
        mlk_log::log_shutdown();
        return exit_code::hardware_init_failed;
    }

    println("[MAIN] All threads joined. System shutdown complete.");
    mlk_log::log_shutdown();
    return exit_code::ok;
}
