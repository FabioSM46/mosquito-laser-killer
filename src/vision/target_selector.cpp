#include "vision/target_selector.h"
#include "core/print.h"

auto TargetSelector::select(const std::vector<TrackedTarget>& targets)
    -> std::optional<TrackedTarget> {
    if (engaged_id_.has_value()) {
        for (const auto& t : targets) {
            if (t.id == engaged_id_.value()) {
                if (t.engageable) {
                    return t;
                }
                println("[SELECTOR] Target {} no longer engageable (hits={}, "
                        "misses={}), releasing",
                        t.id, t.hits, t.misses);
                engaged_id_.reset();
                break;
            }
        }
        if (engaged_id_.has_value()) {
            println("[SELECTOR] Target {} lost", engaged_id_.value());
            engaged_id_.reset();
        }
    }

    // Nearest engageable target: smallest z closes the engagement fastest and
    // gives the largest, most reliable blob.
    const TrackedTarget* best = nullptr;
    for (const auto& t : targets) {
        if (!t.engageable) {
            continue;
        }
        if (best == nullptr || t.position.z < best->position.z) {
            best = &t;
        }
    }
    if (best == nullptr) {
        return std::nullopt;
    }

    engaged_id_ = best->id;
    println("[SELECTOR] Engaging target {} at ({:.3f}, {:.3f}, {:.3f}) m",
            best->id, best->position.x, best->position.y, best->position.z);
    return *best;
}

auto TargetSelector::reset() -> void {
    engaged_id_.reset();
}
