#pragma once
/**
 * @file DefaultLimits.hpp
 * @brief Per-system default perturbation limits based on real-world physics.
 */

#include "DestabilizerTypes.hpp"
#include <vector>
#include <string>

namespace Destabilizer {

/**
 * Get the default perturbation channels for a system by its ID.
 * Returns configured channels with real-world-based default limits.
 *
 * @param systemId  Simulation system ID.
 * @return channel list with defaults; empty for unknown IDs.
 */
std::vector<PerturbationChannel> getDefaultChannels(int systemId);

/**
 * @brief Get a human-readable rationale string for a given system's defaults.
 * @param systemId  Simulation system ID.
 * @return rationale text.
 */
std::string getDefaultRationale(int systemId);

/**
 * @brief Reset a channel's constraints to the system default.
 * @param systemId      Simulation system ID.
 * @param channelIndex  Channel index within the system's channel list.
 * @return default constraints for that channel.
 */
ChannelConstraints getDefaultConstraints(int systemId, int channelIndex);

} // namespace Destabilizer
