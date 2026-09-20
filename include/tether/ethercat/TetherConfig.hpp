#pragma once

/**
 * @file TetherConfig.hpp
 * @brief Deprecated alias for EtherCATConfig.hpp.
 *
 * This header used to contain the EtherCAT stack configuration. It was
 * renamed to EtherCATConfig.hpp to remove the ambiguity with the generated
 * top-level tether/TetherConfig.hpp (a different file that only exists in
 * the build tree). New code should include
 * "tether/ethercat/EtherCATConfig.hpp"; this forwarding header is kept for
 * backward compatibility with existing consumers.
 */

#include "tether/ethercat/EtherCATConfig.hpp"
