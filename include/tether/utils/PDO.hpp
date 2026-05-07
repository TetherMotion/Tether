#pragma once

#include <cstdint>
#include <iterator>
#include <type_traits>

namespace EtherCAT {
namespace Utils {

/// Generic PDO lookup helper.
///
/// Iterates over a span of pointers to PDO-like objects, checking the
/// `index` member of each entry and returning the first match.  This
/// function is constexpr and can be used in compile-time contexts.
///
/// 	PDOType  type of object stored in the span; must expose a `uint16_t
/// 	index` member.
/// 	list     span containing pointers to PDOType instances.
/// 	idx      index value to search for.
///
/// 	Returns a pointer to the matching entry or nullptr if none was found.
template <typename Range>
inline constexpr auto findPDOByIndex(const Range& list, uint16_t idx) noexcept
{
    // Deduce pointer type from the range elements.
    // The range is expected to contain pointers to objects exposing `uint16_t index`.
    using Ptr = std::remove_cv_t<std::remove_reference_t<decltype(*std::begin(list))>>;
    static_assert(std::is_pointer_v<Ptr>, "findPDOByIndex expects a range of pointers");

    for (Ptr p : list) {
        if (p && p->index == idx) {
            return p;
        }
    }
    return static_cast<Ptr>(nullptr);
}

} // namespace Utils
} // namespace EtherCAT
