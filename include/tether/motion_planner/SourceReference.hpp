/**
 * @file SourceReference.hpp
 * @brief Source Reference and Traceability System
 *
 * @details
 * This file provides the traceability infrastructure that enables any data
 * structure in the motion planning system to reference back to its originating
 * G-code line(s).
 *
 * ## Traceability Chain
 *
 * ```
 * GCodeLine (source file, line number)
 *     ↓ (reference stored)
 * MotionSegment (parsed representation)
 *     ↓ (reference stored)
 * BezierSegment (geometric path)
 *     ↓ (reference stored)
 * ProfileSegment (velocity profile)
 *     ↓ (reference stored)
 * MotionPlan (final evaluable plan)
 * ```
 *
 * This enables:
 * 1. Error reporting with G-code line numbers
 * 2. Debugging and visualization
 * 3. Reverse traversal for lookbehind operations
 *
 * @see MotionSegment.hpp
 * @see BezierCurve.hpp
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include <memory>

namespace MotionPlanner {

// ============================================================================
// Source File Identifier
// ============================================================================

/**
 * @brief Identifies a source file
 *
 * For most G-code programs, there is a single source file. However, this
 * supports multi-file programs (e.g., with file inclusions).
 */
struct SourceFile {
    /// Unique identifier for this file
    uint32_t id = 0;
    
    /// File path or name
    std::string path;
    
    /// Optional description or label
    std::string description;

    SourceFile() = default;
    
    explicit SourceFile(std::string filePath, uint32_t fileId = 0)
        : id(fileId), path(std::move(filePath)) {}

    bool operator==(const SourceFile& other) const {
        return id == other.id && path == other.path;
    }
};

// ============================================================================
// G-Code Line Reference
// ============================================================================

/**
 * @brief Reference to a specific G-code line
 *
 * This is the fundamental unit of traceability - a pointer back to the
 * exact source line that generated a piece of motion data.
 */
struct GCodeLineRef {
    /// Source file (can be shared across many references)
    std::shared_ptr<SourceFile> sourceFile;
    
    /// 1-based line number in source file
    uint32_t lineNumber = 0;
    
    /// 1-based column (optional, for specific word references)
    uint16_t column = 0;
    
    /// Original G-code text (optional, for debugging)
    std::string originalText;
    
    /// Block number (N word value, if present)
    std::optional<uint32_t> blockNumber;
    
    /// Comment from this line (if any)
    std::string comment;

    GCodeLineRef() = default;

    GCodeLineRef(std::shared_ptr<SourceFile> file, uint32_t line)
        : sourceFile(std::move(file)), lineNumber(line) {}

    GCodeLineRef(std::shared_ptr<SourceFile> file, uint32_t line, std::string text)
        : sourceFile(std::move(file)), lineNumber(line), originalText(std::move(text)) {}

    /**
     * @brief Check if this is a valid reference
     */
    bool isValid() const noexcept {
        return lineNumber > 0;
    }

    /**
     * @brief Get a human-readable location string
     *
     * @return String like "file.nc:42" or "line 42"
     */
    std::string locationString() const {
        std::string result;
        if (sourceFile && !sourceFile->path.empty()) {
            result = sourceFile->path + ":";
        }
        result += std::to_string(lineNumber);
        if (column > 0) {
            result += ":" + std::to_string(column);
        }
        return result;
    }

    bool operator==(const GCodeLineRef& other) const {
        bool sameFile = (!sourceFile && !other.sourceFile) ||
                       (sourceFile && other.sourceFile && *sourceFile == *other.sourceFile);
        return sameFile && lineNumber == other.lineNumber;
    }

    bool operator<(const GCodeLineRef& other) const {
        if (sourceFile && other.sourceFile) {
            if (sourceFile->id != other.sourceFile->id) {
                return sourceFile->id < other.sourceFile->id;
            }
        }
        return lineNumber < other.lineNumber;
    }
};

// ============================================================================
// Source Reference (Multi-Line)
// ============================================================================

/**
 * @brief Reference to one or more source G-code lines
 *
 * Some motion plan elements may derive from multiple G-code lines
 * (e.g., corner blends combine two adjacent motion commands).
 */
class SourceReference {
public:
    /// Reference type
    enum class Type : uint8_t {
        None,           ///< No source (synthetic/computed)
        Empty = None,   ///< Legacy name for None
        Single,         ///< Single G-code line
        Multiple,       ///< Multiple contributing lines
        Range,          ///< Contiguous range of lines
        Synthetic       ///< Generated (not from source)
    };

    // ========================================================================
    // Constructors
    // ========================================================================

    /**
     * @brief Default constructor - no source reference
     */
    SourceReference() = default;

    /**
     * @brief Single line reference
     */
    explicit SourceReference(GCodeLineRef ref)
        : type_(Type::Single), primaryRef_(std::move(ref)) {}

    /**
     * @brief Multiple line references
     */
    explicit SourceReference(std::vector<GCodeLineRef> refs)
        : type_(refs.size() == 1 ? Type::Single : Type::Multiple)
        , additionalRefs_(std::move(refs)) {
        if (!additionalRefs_.empty()) {
            primaryRef_ = additionalRefs_.front();
        }
    }

    /**
     * @brief Range of contiguous lines
     */
    SourceReference(GCodeLineRef start, GCodeLineRef end)
        : type_(Type::Range), primaryRef_(std::move(start)), endRef_(std::move(end)) {}

    /**
     * @brief Create synthetic reference with description
     */
    static SourceReference synthetic(std::string description) {
        SourceReference ref;
        ref.type_ = Type::Synthetic;
        ref.description_ = std::move(description);
        return ref;
    }

    /**
     * @brief Backwards-compatible helpers for tests and older callers
     */
    static SourceReference fromLine(uint32_t lineNumber, std::shared_ptr<SourceFile> file = nullptr, std::string text = "") {
        GCodeLineRef ref(std::move(file), lineNumber, std::move(text));
        return SourceReference(std::move(ref));
    }

    static SourceReference multiple(const std::vector<SourceReference>& refs) {
        // Collect GCodeLineRef objects from provided SourceReference objects
        std::vector<GCodeLineRef> collected;
        for (const auto& r : refs) {
            auto v = r.allRefs();
            collected.insert(collected.end(), v.begin(), v.end());
        }
        return SourceReference(std::move(collected));
    }

    static SourceReference range(uint32_t startLine, uint32_t endLine, std::shared_ptr<SourceFile> file = nullptr) {
        GCodeLineRef s(std::move(file), startLine);
        GCodeLineRef e(s.sourceFile, endLine);
        return SourceReference(std::move(s), std::move(e));
    }

    // ========================================================================
    // Properties
    // ========================================================================

    /**
     * @brief Get reference type
     */
    Type type() const noexcept { return type_; }

    /**
     * @brief Convenience: legacy accessor returning primary line number
     */
    uint32_t lineNumber() const noexcept { return primaryRef_.lineNumber; }

    /**
     * @brief Convenience: legacy accessor returning raw pointer to source file
     */
    const SourceFile* sourceFile() const noexcept { return primaryRef_.sourceFile.get(); }

    /**
     * @brief Convenience: start line for range type
     */
    uint32_t startLine() const noexcept { return primaryRef_.lineNumber; }

    /**
     * @brief Convenience: end line for range type
     */
    uint32_t endLine() const noexcept { return endRef_.lineNumber; }

    /**
     * @brief Check if reference is valid
     */
    bool isValid() const noexcept {
        return type_ != Type::None || !description_.empty();
    }

    /**
     * @brief Check if this is a synthetic (computed) reference
     */
    bool isSynthetic() const noexcept {
        return type_ == Type::Synthetic;
    }

    /**
     * @brief Get primary line reference (first/most relevant line)
     */
    const GCodeLineRef& primary() const noexcept {
        return primaryRef_;
    }

    /**
     * @brief Get all contributing line references
     */
    std::vector<GCodeLineRef> allRefs() const {
        if (type_ == Type::Multiple) {
            return additionalRefs_;
        } else if (type_ == Type::Single || type_ == Type::Range) {
            return {primaryRef_};
        }
        return {};
    }

    /**
     * @brief Get number of contributing lines
     */
    size_t lineCount() const noexcept {
        switch (type_) {
            case Type::Single: return 1;
            case Type::Multiple: return additionalRefs_.size();
            case Type::Range:
                return endRef_.lineNumber - primaryRef_.lineNumber + 1;
            default: return 0;
        }
    }

    /**
     * @brief Get description (for synthetic references)
     */
    const std::string& description() const noexcept {
        return description_;
    }

    /**
     * @brief Set description
     */
    void setDescription(std::string desc) {
        description_ = std::move(desc);
    }

    // ========================================================================
    // Operations
    // ========================================================================

    /**
     * @brief Merge with another reference
     *
     * Creates a new reference that includes all lines from both.
     */
    SourceReference merge(const SourceReference& other) const {
        if (!isValid()) return other;
        if (!other.isValid()) return *this;

        std::vector<GCodeLineRef> combined = allRefs();
        auto otherRefs = other.allRefs();
        combined.insert(combined.end(), otherRefs.begin(), otherRefs.end());

        // Remove duplicates
        std::sort(combined.begin(), combined.end());
        combined.erase(std::unique(combined.begin(), combined.end()), combined.end());

        return SourceReference(std::move(combined));
    }

    /**
     * @brief Get human-readable location string
     */
    std::string locationString() const {
        switch (type_) {
            case Type::None:
                return "<no source>";
            case Type::Single:
                return primaryRef_.locationString();
            case Type::Multiple: {
                std::string result;
                for (size_t i = 0; i < additionalRefs_.size() && i < 3; ++i) {
                    if (i > 0) result += ", ";
                    result += additionalRefs_[i].locationString();
                }
                if (additionalRefs_.size() > 3) {
                    result += " (+" + std::to_string(additionalRefs_.size() - 3) + " more)";
                }
                return result;
            }
            case Type::Range:
                return primaryRef_.locationString() + "-" +
                       std::to_string(endRef_.lineNumber);
            case Type::Synthetic:
                return "<synthetic: " + description_ + ">";
        }
        return "";
    }

    /**
     * @brief Get original G-code text (from primary reference)
     */
    std::string originalText() const {
        return primaryRef_.originalText;
    }

    /**
     * @brief Check if this reference contains a specific line
     */
    bool containsLine(uint32_t lineNum) const {
        switch (type_) {
            case Type::Single:
                return primaryRef_.lineNumber == lineNum;
            case Type::Multiple:
                for (const auto& ref : additionalRefs_) {
                    if (ref.lineNumber == lineNum) return true;
                }
                return false;
            case Type::Range:
                return lineNum >= primaryRef_.lineNumber &&
                       lineNum <= endRef_.lineNumber;
            default:
                return false;
        }
    }

private:
    Type type_ = Type::None;
    GCodeLineRef primaryRef_;
    GCodeLineRef endRef_;  // For range type
    std::vector<GCodeLineRef> additionalRefs_;  // For multiple type
    std::string description_;  // For synthetic/description
};

// ============================================================================
// Source Reference Builder
// ============================================================================

/**
 * @brief Helper class for building source references
 *
 * Useful when constructing references from parsed G-code blocks.
 */
class SourceReferenceBuilder {
public:
    /**
     * @brief Set the source file for all subsequent references
     */
    void setSourceFile(std::shared_ptr<SourceFile> file) {
        currentFile_ = std::move(file);
    }

    /**
     * @brief Create a reference for a single line
     */
    SourceReference forLine(uint32_t lineNumber, std::string text = "") {
        GCodeLineRef ref(currentFile_, lineNumber, std::move(text));
        return SourceReference(std::move(ref));
    }

    /**
     * @brief Create a reference for a range of lines
     */
    SourceReference forRange(uint32_t startLine, uint32_t endLine) {
        GCodeLineRef start(currentFile_, startLine);
        GCodeLineRef end(currentFile_, endLine);
        return SourceReference(std::move(start), std::move(end));
    }

    /**
     * @brief Create a reference combining multiple sources
     */
    SourceReference combine(const SourceReference& a, const SourceReference& b) {
        return a.merge(b);
    }

    /**
     * @brief Get the current source file
     */
    std::shared_ptr<SourceFile> currentSourceFile() const {
        return currentFile_;
    }

private:
    std::shared_ptr<SourceFile> currentFile_;
};

// ============================================================================
// Traceable Interface
// ============================================================================

/**
 * @brief Interface for types that have source traceability
 *
 * Any class that can trace back to source G-code should implement this.
 */
class Traceable {
public:
    virtual ~Traceable() = default;

    /**
     * @brief Get the source reference
     */
    virtual const SourceReference& sourceRef() const = 0;

    /**
     * @brief Check if this has valid source reference
     */
    bool hasSourceRef() const {
        return sourceRef().isValid();
    }

    /**
     * @brief Get location string for error messages
     */
    std::string locationString() const {
        return sourceRef().locationString();
    }
};

}  // namespace MotionPlanner
