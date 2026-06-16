/**
 * @file dc_init.cpp
 * @brief EtherCAT Distributed Clock (DC) — formerly held a global singleton.
 *
 * All DC state is now managed by DCManager (owned by Master).
 * This file is kept only for the platform time-source weak symbols that
 * live in dc_time_source.cpp and are declared in DC.hpp.
 */
