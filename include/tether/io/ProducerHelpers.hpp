/**
 * @file ProducerHelpers.hpp
 * @brief Producer-side conveniences for Tether IO servers: clock/log
 *        defaults, atomic-shadowed params, no-argument actions, and
 *        callback string signals.
 *
 * The registry API is deliberately low-level (read/write function
 * pointers on entry structs). These factories cover the three shapes
 * nearly every producer writes by hand:
 *
 *   atomicParam<T>()  — a RW param backed by std::atomic<T>, with an
 *                       optional hook fired after each write.
 *   makeAction()      — a parameterless function entry wrapping a
 *                       void() / FunctionCallResult() body.
 *   makeStringSignal()— a read-only String signal whose value a callback
 *                       produces on each read (NUL-terminated wire form).
 *
 * Plus the two defaults ServerConfig wants: steadyClockUs() matches
 * TimestampFn, stderrLog() matches LogFn.
 *
 * Lifetime: entry callbacks capture their storage/callbacks by
 * reference — the atomics and hook targets must outlive the Registry.
 */
#pragma once

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <string_view>
#include <type_traits>

#include "tether/io/Function.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Session.hpp"

namespace tether::io {

// ---------------------------------------------------------------------------
// ServerConfig defaults
// ---------------------------------------------------------------------------

/**
 * Steady-clock µs timestamp — drop-in ServerConfig::timestampFn.
 * @return microseconds since an arbitrary epoch (monotonic).
 */
inline uint64_t steadyClockUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

/**
 * @brief stderr logger — drop-in ServerConfig::logFn.
 * @param tag  Component tag.
 * @param fmt  printf-style format string.
 * @param ...  Format arguments.
 */
inline void stderrLog(const char* tag, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::fprintf(stderr, "[%s] ", tag ? tag : "tether");
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
}

// ---------------------------------------------------------------------------
// Atomic-shadowed params
// ---------------------------------------------------------------------------

/**
 * Maps a C++ scalar to its wire ValueType.
 * @tparam T  Scalar type.
 * @return matching ValueType.
 * @tparam T fixed-width scalar (bool, integers, float, double).
 */
template <typename T>
constexpr ValueType valueTypeOf() {
    if constexpr (std::is_same_v<T, bool>)     return ValueType::Bool;
    else if constexpr (std::is_same_v<T, uint8_t>)  return ValueType::U8;
    else if constexpr (std::is_same_v<T, uint16_t>) return ValueType::U16;
    else if constexpr (std::is_same_v<T, uint32_t>) return ValueType::U32;
    else if constexpr (std::is_same_v<T, uint64_t>) return ValueType::U64;
    else if constexpr (std::is_same_v<T, int8_t>)   return ValueType::I8;
    else if constexpr (std::is_same_v<T, int16_t>)  return ValueType::I16;
    else if constexpr (std::is_same_v<T, int32_t>)  return ValueType::I32;
    else if constexpr (std::is_same_v<T, int64_t>)  return ValueType::I64;
    else if constexpr (std::is_same_v<T, float>)    return ValueType::F32;
    else if constexpr (std::is_same_v<T, double>)   return ValueType::F64;
    else static_assert(!sizeof(T*),
        "valueTypeOf: no ValueType mapping for this type");
}

/**
 * A RW param backed by an atomic shadow: reads load, writes store then
 * fire `onWrite` (e.g. to push the new value into the control loop).
 * @tparam T fixed-width scalar — the ValueType is derived from it.
 * @param id entry id (see entryId()).
 * @param name dotted entry name.
 * @param group catalog group label.
 * @param schema schema ref for the entry's value.
 * @param schemaSlot slot of that schema in the installed catalog.
 * @param slot atomic storage; must outlive the registered entry.
 * @param onWrite hook invoked with the stored value after each write;
 *        empty disables it (the param is still writable).
 * @return the ParamEntry to hand to Registry::addParam.
 */
template <typename T, typename Hook = std::function<void(T)>>
ParamEntry atomicParam(uint64_t id, std::string_view name,
                       std::string_view group, const SchemaRef& schema,
                       uint32_t schemaSlot, std::atomic<T>& slot,
                       Hook&& onWrite = {}) {
    static_assert(std::is_trivially_copyable_v<T>,
                  "atomicParam requires a trivially-copyable value type");
    ParamEntry e;
    e.id = id;
    e.name = std::string(name);
    e.group = std::string(group);
    e.valueType = valueTypeOf<T>();
    e.schema = schema;
    e.schemaSlot = schemaSlot;
    e.readFn = [&slot](void* dest) {
        const T v = slot.load(std::memory_order_relaxed);
        std::memcpy(dest, &v, sizeof(T));
    };
    e.writeFn = [&slot,
                 onWrite = std::function<void(T)>(
                     std::forward<Hook>(onWrite))](const void* src) {
        T v;
        std::memcpy(&v, src, sizeof(T));
        slot.store(v, std::memory_order_relaxed);
        if (onWrite) onWrite(v);
    };
    return e;
}

// ---------------------------------------------------------------------------
// No-argument actions
// ---------------------------------------------------------------------------

/**
 * A parameterless function entry that runs `body` and reports success.
 * @param id entry id (see entryId()).
 * @param name dotted function name.
 * @param group catalog group label.
 * @param body action to run on invocation (must return void — use the
 *        FunctionCallResult overload for actions that can fail).
 * @return the FunctionEntry to hand to Registry::addFunction.
 */
template <typename F>
    requires std::is_void_v<std::invoke_result_t<F>>
FunctionEntry makeAction(uint64_t id, std::string_view name,
                         std::string_view group, F&& body) {
    FunctionEntry e;
    e.id = id;
    e.name = std::string(name);
    e.group = std::string(group);
    e.callback = [body = std::forward<F>(body)](const std::vector<FunctionArgument>&) {
        FunctionCallResult r;
        body();
        r.success = true;
        return r;
    };
    return e;
}

/**
 * A parameterless function entry whose body controls the result —
 * for actions that can fail with a message.
 * @param id entry id (see entryId()).
 * @param name dotted function name.
 * @param group catalog group label.
 * @param body action to run; its result is returned to the caller.
 * @return the FunctionEntry to hand to Registry::addFunction.
 */
inline FunctionEntry makeAction(uint64_t id, std::string_view name,
                                std::string_view group,
                                std::function<FunctionCallResult()> body) {
    FunctionEntry e;
    e.id = id;
    e.name = std::string(name);
    e.group = std::string(group);
    e.callback = [body = std::move(body)](const std::vector<FunctionArgument>&) {
        return body ? body() : FunctionCallResult{};
    };
    return e;
}

// ---------------------------------------------------------------------------
// Callback string signals
// ---------------------------------------------------------------------------

/**
 * A read-only String signal whose wire value `fn` produces on each
 * read, NUL-terminated (the String schema-value encoding) and capped
 * at `maxValueSize` bytes.
 * @param id entry id (see entryId()).
 * @param name dotted signal name.
 * @param group catalog group label.
 * @param schema schema ref for the string value.
 * @param schemaSlot slot of that schema in the installed catalog.
 * @param fn value producer; must outlive the registered entry.
 * @param maxValueSize declared wire cap (bytes incl. NUL).
 * @return the SignalEntry to hand to Registry::addSignal.
 */
inline SignalEntry makeStringSignal(uint64_t id, std::string_view name,
                                    std::string_view group,
                                    const SchemaRef& schema,
                                    uint32_t schemaSlot,
                                    std::function<std::string()> fn,
                                    uint16_t maxValueSize = 4096) {
    SignalEntry e;
    e.id = id;
    e.name = std::string(name);
    e.group = std::string(group);
    e.valueType = ValueType::String;
    e.schema = schema;
    e.schemaSlot = schemaSlot;
    e.maxValueSize = maxValueSize;
    e.varReadFn = [fn = std::move(fn)](void* dest, size_t maxLen) -> size_t {
        const std::string s = fn ? fn() : std::string{};
        const size_t n = std::min(maxLen, s.size() + 1);
        std::memcpy(dest, s.c_str(), n);
        if (n) static_cast<uint8_t*>(dest)[n - 1] = 0;
        return n;
    };
    return e;
}

}  // namespace tether::io
