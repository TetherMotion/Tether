#pragma once

/**
 * @file CiA402Tuning.hpp
 * @brief Generic access to CiA 402 (and vendor) tuning parameters over SDO
 *
 * Thin typed facade on top of CoEManager::readEntry/writeEntry.  A tuning
 * parameter is any object-dictionary entry (index/subindex/data_type from a
 * RegisterEntry table — Parameters60xx for standard objects, the vendor
 * Registers:: tables for drive-specific ones like the SOMANET 0x2010-0x2012
 * controller gains).
 *
 * Typical use:
 *   CiA402Tuning tuning{sdo_mgr, {.timeout_ms = 5000}};
 *   auto kp   = tuning.read(Obj2012::PositionControllerKp);   // as double
 *   auto rows = tuning.readAll(Obj2012::kRegisterList);       // snapshot
 *   (void)tuning.write(Obj2012::PositionControllerKp, 0.006);
 *   auto diff = CiA402Tuning::diff(rows_a, rows_b);           // compare drives
 */

#include <bit>
#include <cmath>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/profiles/cia402/60xx-Parameters.hpp"

namespace EtherCAT {
namespace Drives {

class CiA402Tuning {
public:
    using Entry = ObjectDictionary::ObjectDictionaryEntry;
    using D = ObjectDictionary::ObjectDictionaryDataType;
    /// Any container of `const Entry*` (Parameters60xx::kRegisterList
    /// array, Registers::Synapticon RegisterList vectors, ad-hoc lists).
    using EntryList = std::span<const Entry* const>;

    /// One read outcome: the entry queried and its raw bits (or the error).
    struct Result {
        const Entry* entry = nullptr;
        CoE::CoEResult<uint64_t> bits = std::unexpected(
            CoE::CoEError{CoE::CoEErrorCode::InternalError});
        bool ok() const { return bits.has_value(); }
    };

    explicit CiA402Tuning(CoE::CoEManager& sdo,
                          CoE::CoETransactionOptions opts = {})
        : sdo_(sdo), opts_(opts) {}

    // ----- Raw access ----------------------------------------------------

    /// Read the entry at its declared subindex.
    CoE::CoEResult<uint64_t> readBits(const Entry& e) {
        return sdo_.readEntry(e, opts_);
    }
    /// Read a record subindex overriding entry.subindex.
    CoE::CoEResult<uint64_t> readBits(const Entry& e, uint8_t subindex) {
        return sdo_.readEntry(e, subindex, opts_);
    }

    /// Write raw bits (as produced by encode()) at the declared subindex.
    CoE::CoEResult<void> writeBits(const Entry& e, uint64_t bits) {
        return sdo_.writeEntry(e, bits, opts_);
    }
    CoE::CoEResult<void> writeBits(const Entry& e, uint8_t subindex,
                                   uint64_t bits) {
        return sdo_.writeEntry(e, subindex, bits, opts_);
    }

    // ----- Typed access ----------------------------------------------------

    /// Read and decode per entry.data_type (signed/unsigned/Real32/Real64
    /// all collapse to double — lossless for every width except U64/I64
    /// extremes).
    CoE::CoEResult<double> read(const Entry& e) {
        auto r = readBits(e);
        if (!r) return std::unexpected(r.error());
        return asDouble(e, *r);
    }
    CoE::CoEResult<double> read(const Entry& e, uint8_t subindex) {
        auto r = readBits(e, subindex);
        if (!r) return std::unexpected(r.error());
        return asDouble(e, *r);
    }

    /// Write a numeric value encoded per entry.data_type — Real32/Real64
    /// are bit-cast, integer types are rounded to the nearest whole.
    CoE::CoEResult<void> write(const Entry& e, double value) {
        return writeBits(e, encode(e, value));
    }
    CoE::CoEResult<void> write(const Entry& e, uint8_t subindex,
                               double value) {
        return writeBits(e, subindex, encode(e, value));
    }

    /// Whole-list snapshot: preserves order so two drives' snapshots align
    /// entry-for-entry for diff().
    std::vector<Result> readAll(EntryList entries) {
        std::vector<Result> out;
        out.reserve(entries.size());
        for (const Entry* e : entries) {
            Result r{.entry = e, .bits = readBits(*e)};
            out.push_back(std::move(r));
        }
        return out;
    }

    // ----- Decoding / encoding helpers ------------------------------------

    static int64_t asInt(const Entry& e, uint64_t bits) {
        // readEntry already sign-extends signed types.
        return static_cast<int64_t>(bits);
    }
    static uint64_t asUInt(const Entry& e, uint64_t bits) { return bits; }
    static float asF32(const Entry&, uint64_t bits) {
        return std::bit_cast<float>(static_cast<uint32_t>(bits));
    }
    static double asF64(const Entry&, uint64_t bits) {
        return std::bit_cast<double>(bits);
    }

    static bool isRealType(D dt) {
        return dt == D::Real32 || dt == D::Real64;
    }
    static bool isSignedType(D dt) {
        switch (dt) {
            case D::Integer8:  case D::Integer16: case D::Integer24:
            case D::Integer32: case D::Integer40: case D::Integer48:
            case D::Integer56: case D::Integer64:
                return true;
            default:
                return false;
        }
    }

    static double asDouble(const Entry& e, uint64_t bits) {
        if (e.data_type == D::Real32) return asF32(e, bits);
        if (e.data_type == D::Real64) return asF64(e, bits);
        if (isSignedType(e.data_type))
            return static_cast<double>(asInt(e, bits));
        return static_cast<double>(bits);
    }

    /// Encode a numeric value into the raw bits writeEntry expects.
    static uint64_t encode(const Entry& e, double value) {
        if (e.data_type == D::Real32)
            return std::bit_cast<uint32_t>(static_cast<float>(value));
        if (e.data_type == D::Real64)
            return std::bit_cast<uint64_t>(value);
        if (isSignedType(e.data_type))
            return static_cast<uint64_t>(
                static_cast<int64_t>(std::llround(value)));
        return static_cast<uint64_t>(std::llround(value));
    }

    /// Human-readable value honoring the declared type (floats get %g).
    static std::string format(const Entry& e, uint64_t bits) {
        if (e.data_type == D::Real32)
            return std::format("{:.6g}", asF32(e, bits));
        if (e.data_type == D::Real64)
            return std::format("{:.6g}", asF64(e, bits));
        if (isSignedType(e.data_type))
            return std::format("{}", asInt(e, bits));
        return std::format("{}", bits);
    }

    static std::string formatResult(const Result& r) {
        if (!r.ok()) return "ERR";
        return format(*r.entry, *r.bits);
    }

    // ----- Comparison -----------------------------------------------------

    /// Compare two aligned snapshots (same entry list order — e.g. two
    /// drives read with readAll over the same RegisterList).  Returns one
    /// line per differing or unresolved entry; empty means identical.
    static std::vector<std::string> diff(const std::vector<Result>& a,
                                         const std::vector<Result>& b) {
        std::vector<std::string> out;
        const size_t n = std::min(a.size(), b.size());
        for (size_t i = 0; i < n; ++i) {
            const Entry* e = a[i].entry ? a[i].entry : b[i].entry;
            const char* name = e ? e->name : "?";
            if (!a[i].ok() || !b[i].ok()) {
                if (a[i].ok() != b[i].ok()) {
                    out.push_back(std::format(
                        "{}: {} vs {}", name,
                        formatResult(a[i]), formatResult(b[i])));
                }
                continue;
            }
            if (*a[i].bits != *b[i].bits) {
                out.push_back(std::format(
                    "{}: {} vs {}", name,
                    format(*e, *a[i].bits), format(*e, *b[i].bits)));
            }
        }
        return out;
    }

    // ----- Curated CiA 402 tuning set --------------------------------------
    /// Standard objects that shape how a drive interprets position/velocity
    /// commands: encoder resolution, gear ratio, feed constant, limits,
    /// polarity, following-error window, profile speeds.  Vendor loop gains
    /// live in the drive's Registers:: tables (same API).
    static const Entry* const* standardTuningEntries() {
        namespace P = CiA402::Parameters60xx;
        static const Entry* const k[] = {
            &P::EncoderIncrements,
            &P::MotorRevolutions,
            &P::GearRatioMotorRevolutions,
            &P::GearRatioShaftRevolutions,
            &P::FeedConstantFeed,
            &P::FeedConstantShaftRevolutions,
            &P::MinSoftwarePositionLimit,
            &P::MaxSoftwarePositionLimit,
            &P::MaxSpeed,              // 0x607F max profile velocity
            &P::MaxMotorSpeed,         // 0x6080
            &P::FollowingErrorWindow,  // 0x6065
            &P::ReferencePolarity,     // 0x607E
            &P::HomeOffset,            // 0x607C
        };
        return k;
    }
    static constexpr size_t standardTuningCount() { return 13; }
    static EntryList standardTuning() {
        return {standardTuningEntries(), standardTuningCount()};
    }

private:
    CoE::CoEManager& sdo_;
    CoE::CoETransactionOptions opts_;
};

} // namespace Drives
} // namespace EtherCAT
