#pragma once
/// @file DefaultLimits.hpp
/// @brief Per-system default perturbation limits based on real-world physics.

#include "DestabilizerTypes.hpp"
#include <vector>
#include <string>

namespace Destabilizer {

/// Get the default perturbation channels for a system by its ID.
/// Returns configured channels with real-world-based default limits.
std::vector<PerturbationChannel> getDefaultChannels(int systemId);

/// Get a human-readable rationale string for a given system's defaults.
std::string getDefaultRationale(int systemId);

/// Reset a channel's constraints to the system default.
ChannelConstraints getDefaultConstraints(int systemId, int channelIndex);

} // namespace Destabilizer
