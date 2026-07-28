#pragma once

#include "core/types.h"
#include <expected>
#include <string>

// Loads the runtime configuration from a YAML file, fail-closed.
//
// A missing KEY leaves the corresponding SystemConfig member at its types.h
// default — types.h is the single source of truth for defaults. But a FILE
// that cannot be read or parsed, or a key whose value cannot convert to its
// target type, is an error, not a fallback: continuing on a partial or
// default config runs a Class 4 laser with settings the operator never
// reviewed, and the previous "log and use defaults" behaviour produced a
// MIXED config on a mid-file parse error (keys before the throw kept their
// YAML values, everything after silently reverted). The returned error names
// the absolute path that was attempted so a wrong working directory is
// diagnosable from the log alone.
[[nodiscard]] auto load_config(const std::string& path)
    -> std::expected<SystemConfig, std::string>;
