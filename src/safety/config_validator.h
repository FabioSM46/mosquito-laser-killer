#pragma once

#include "core/types.h"
#include <string>
#include <vector>

struct ValidationWarning {
    std::string category;
    std::string message;
    bool critical{false};
};

[[nodiscard]] auto validate_engagement_volume(const SystemConfig& config)
    -> std::vector<ValidationWarning>;

[[nodiscard]] auto has_critical_validation_errors(
    const std::vector<ValidationWarning>& warnings) -> bool;

[[nodiscard]] auto horizontal_fov_deg(const SystemConfig::CameraOptics& optics) -> double;

[[nodiscard]] auto vertical_fov_deg(const SystemConfig::CameraOptics& optics) -> double;
