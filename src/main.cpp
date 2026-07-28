#include "core/types.h"
#include "core/error.h"
#include "core/config_loader.h"
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
#include "vision/multi_tracker.h"
#include "vision/target_selector.h"

#include "control/coordinate_mapper.h"
#include "control/firing_controller.h"
#include "control/control_loop.h"

#include "core/print.h"
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

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

// Runs log_shutdown() on EVERY exit path from main, including the early
// hardware/config failure returns: O_NONBLOCK is a property of the shared open
// file description, so skipping the restore hands EAGAIN back to whatever owns
// our stdout (an interactive shell, a supervisor). Declared right after
// log_init() so it is destroyed last, after every logger has gone quiet.
struct LogSessionGuard {
    ~LogSessionGuard() { mlk_log::log_shutdown(); }
};

}

auto main(int argc, char* argv[]) -> int {
    // Before any thread starts: the control thread must never block on a log
    // write while it owns a live laser pulse.
    mlk_log::log_init();
    LogSessionGuard log_guard;

    println("===========================================");
    println("  Mosquito Laser Killer v1.0.0");
    println("  Stereoscopic Laser Targeting System");
    println("  Differential Galvo Driver (Dual-DAC)");
    println("===========================================");

    SignalHandler signal_handler;
    signal_handler.install();

    // Fail closed on any config problem: a laser system must never run on a
    // config the operator did not review. The path is relative to the working
    // directory unless overridden by the first argument.
    const std::string config_path =
        (argc > 1) ? argv[1] : "config/system_config.yaml";
    auto config_result = load_config(config_path);
    if (!config_result.has_value()) {
        println(stderr, "[CONFIG] FATAL: {}", config_result.error());
        println(stderr, "[CONFIG] Fix the file (or pass its path as the first "
                        "argument) and restart.");
        return exit_code::config_error;
    }
    auto config = std::move(config_result.value());
    println("[CONFIG] Loaded {}", config_path);

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
    MultiTracker multi_tracker(config.tracking);
    TargetSelector target_selector;

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

        if (left_dev == right_dev) {
            println(stderr, "[CAPTURE] left_camera_device and right_camera_device "
                            "are the same path ({}) — stereo needs two distinct "
                            "cameras", left_dev);
            request_system_halt("camera device paths identical");
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
            frame.left_timestamp = left_cam.last_frame_timestamp();

            auto right_result = right_cam.capture(frame.right_frame.data(),
                                                    frame.right_frame.size());
            if (!right_result.has_value()) {
                println(stderr, "[CAPTURE] Right camera capture failed: {}",
                             to_string(right_result.error()));
                request_system_halt("right camera capture failed");
                break;
            }
            frame.right_timestamp = right_cam.last_frame_timestamp();

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

        // Stereo-skew watermark. The cameras free-run without hardware sync,
        // so a pair can be up to one frame period apart; for a laterally
        // moving target the skew biases z by roughly z·v·Δt/baseline (§4.12).
        // Log the worst case periodically so a degrading rig shows up in the
        // record instead of silently widening the depth error.
        auto max_skew = std::chrono::steady_clock::duration::zero();
        uint64_t skew_frames = 0;

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

            const auto skew =
                std::chrono::abs(frame.right_timestamp - frame.left_timestamp);
            if (skew > max_skew) {
                max_skew = skew;
            }
            if (++skew_frames % 512 == 0) {
                println("[PROCESSING] Stereo skew watermark: {}us over the last "
                        "512 frames",
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            max_skew).count());
                max_skew = std::chrono::steady_clock::duration::zero();
            }

            auto left_blobs = detector_left.detect_blobs(frame.left_frame.data(),
                                                         frame.left_frame.size());
            auto right_blobs = detector_right.detect_blobs(frame.right_frame.data(),
                                                           frame.right_frame.size());

            TargetCommand cmd;
            cmd.frame_id = frame.frame_id;
            cmd.timestamp = frame.timestamp;

            // Correspondence is established per blob and validated against the
            // epipolar constraint; an ambiguous cluster yields no target from
            // that cluster, while clean pairs elsewhere in the frame survive.
            auto targets_3d = stereo_matcher.match_all(left_blobs, right_blobs);

            // Tracks coast through brief detection gaps on their Kalman
            // predictions (bounded by k_max_predict_horizon_s) instead of
            // throwing the velocity estimates away; a track past the horizon
            // is deleted — fail closed. Only confirmed, plausibly-flying
            // tracks are engageable.
            auto tracks = multi_tracker.update(targets_3d, frame.timestamp);

            // One laser, one galvo: pick the sticky/nearest engageable target.
            // The control thread and firing path below are unchanged — they
            // still see exactly one TargetCommand per frame.
            auto chosen = target_selector.select(tracks);

            cmd.target_valid = chosen.has_value();
            if (chosen.has_value()) {
                cmd.target_position = chosen->position;
            }

            target_queue.push(std::move(cmd));
        }

        println("[PROCESSING] Thread exiting");
    });

    println("[MAIN] Starting control thread...");
    std::jthread control_thread([&](std::stop_token stoken) {
        println("[CONTROL] Thread started");
        // Fixed period (see control_loop.h): the safety latencies this cycle
        // bounds must not scale with the camera's target_fps.
        constexpr auto cycle_period = k_control_period;

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
        return exit_code::safe_halt;
    }

    // A capture-thread hardware fault never reaches SAFE_HALT (that thread may
    // only set atomics), so without this check it would fall through to exit 0 and
    // a supervisor with Restart=on-failure would leave the system dead.
    if (g_hardware_fault.load(std::memory_order_acquire)) {
        println(stderr, "[MAIN] Shutdown complete after a hardware fault. "
                "Review the log above before restarting.");
        return exit_code::hardware_init_failed;
    }

    println("[MAIN] All threads joined. System shutdown complete.");
    return exit_code::ok;
}
