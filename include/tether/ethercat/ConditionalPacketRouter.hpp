/**
 * @file ConditionalPacketRouter.hpp
 * @brief Backward-compatibility redirect to TransactionRouter.hpp
 *
 * ConditionalPacketRouter has been replaced by TransactionRouter.
 * This header exists solely to keep existing #includes compiling.
 * The `ConditionalPacketRouter` name is available as a type alias.
 */
#pragma once
#include "tether/ethercat/TransactionRouter.hpp"
