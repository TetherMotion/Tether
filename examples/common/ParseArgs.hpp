#pragma once

#include <argparse/argparse.hpp>
#include <iostream>
#include <stdexcept>

namespace Tether::Examples {

/// Parse `argc`/`argv`; on failure prints the error and usage to stderr
/// and returns false — replaces the repeated
/// `try { program.parse_args } catch { err + usage; return 1; }` block.
/// Call sites: `if (!parseArgs(program, argc, argv)) return 1;`
///
/// Header-only so tools that do not link tether_examples_common can use it.
inline bool parseArgs(argparse::ArgumentParser& program, int argc,
                      char** argv) {
    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return false;
    }
    return true;
}

} // namespace Tether::Examples
