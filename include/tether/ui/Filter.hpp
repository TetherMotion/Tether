#pragma once

/**
 * @file Filter.hpp
 * @brief Multi-facet record filter for table/tree view models.
 *
 * UI-agnostic (no ncurses): the same filter drives terminal tables today
 * and can back HTTP query parameters for a web UI later.
 *
 * Expression syntax (whitespace-separated terms, AND-combined):
 *   field=value      field contains value (case-insensitive substring)
 *   !field=value     field does NOT contain value (wins over positives)
 *   field>v / <v     numeric comparison on the field value
 *   text=value       substring over the row's joined cells
 *   word             bare word — substring over all cells AND field values
 *
 * Example: "state=op !name=coupler text=error idx<4"
 */

#include <algorithm>
#include <cctype>
#include <charconv>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace Tether {
namespace UI {

/// Named fields attached to a row/record — what `field=` terms match on.
struct FieldSet {
    std::map<std::string, std::string> fields;

    const std::string* find(std::string_view name) const {
        const std::string key{name};
        const auto it = fields.find(key);
        return it == fields.end() ? nullptr : &it->second;
    }
};

class RecordFilter {
public:
    struct Term {
        std::string field;
        std::string value;
        char op = '=';      ///< '=' contains, '>' numeric-gt, '<' numeric-lt
        bool negated = false;
    };

    static std::string lower(std::string_view s) {
        std::string out{s};
        std::ranges::transform(out, out.begin(),
                               [](unsigned char c) { return std::tolower(c); });
        return out;
    }

    static bool containsCi(std::string_view hay, std::string_view needle) {
        return lower(hay).find(lower(needle)) != std::string::npos;
    }

    /// Parse an expression string.  Always succeeds — unrecognized terms
    /// degrade to substring matches so a partial expression still filters.
    static RecordFilter parse(std::string_view expr) {
        RecordFilter f;
        std::istringstream in{std::string{expr}};
        std::string tok;
        while (in >> tok) {
            bool negated = false;
            if (tok.starts_with('!')) {
                negated = true;
                tok.erase(0, 1);
            }
            const auto eq = tok.find('=');
            const auto gt = tok.find('>');
            const auto lt = tok.find('<');
            if (eq != std::string::npos) {
                std::string field = tok.substr(0, eq);
                std::string value = tok.substr(eq + 1);
                if (field == "text" || field == "contains" || field == "grep") {
                    f.substrings_.push_back(value);
                } else {
                    f.terms_.push_back({.field = std::move(field),
                                        .value = std::move(value),
                                        .op = '=', .negated = negated});
                }
            } else if (gt != std::string::npos || lt != std::string::npos) {
                const auto p = std::min(gt, lt);
                f.terms_.push_back({.field = tok.substr(0, p),
                                    .value = tok.substr(p + 1),
                                    .op = tok[p], .negated = negated});
            } else if (!tok.empty()) {
                if (negated) {
                    f.neg_substrings_.push_back(tok);
                } else {
                    f.substrings_.push_back(std::move(tok));
                }
            }
            f.active_ = true;
        }
        return f;
    }

    /// `haystack` is the row's pre-joined cell text (for `text=`/bare words).
    bool matches(const FieldSet& fs, std::string_view haystack) const {
        if (!active_) return true;
        for (const auto& t : terms_) {
            const std::string* v = fs.find(t.field);
            bool hit = false;
            if (v) {
                if (t.op == '=') {
                    hit = containsCi(*v, t.value);
                } else {
                    double lhs = 0, rhs = 0;
                    const char* b = v->data() + (v->starts_with("0x") ? 2 : 0);
                    const auto r1 = std::from_chars(b, v->data() + v->size(), lhs);
                    const auto r2 = std::from_chars(t.value.data(),
                        t.value.data() + t.value.size(), rhs);
                    if (r1.ec == std::errc() && r2.ec == std::errc()) {
                        hit = t.op == '>' ? lhs > rhs : lhs < rhs;
                    }
                }
            }
            if (hit == t.negated) return false;   // exclude wins
        }
        for (const auto& s : substrings_) {
            if (!containsCi(haystack, s) && !anyFieldContains(fs, s))
                return false;
        }
        for (const auto& s : neg_substrings_) {
            if (containsCi(haystack, s) || anyFieldContains(fs, s))
                return false;
        }
        return true;
    }

    bool active() const { return active_; }
    void clear() { *this = RecordFilter{}; }

    /// Compact human-readable summary for status lines.
    std::string describe() const {
        if (!active_) return {};
        std::string out;
        for (const auto& t : terms_) {
            if (!out.empty()) out += ' ';
            if (t.negated) out += '!';
            out += t.field;
            out += t.op;
            out += t.value;
        }
        for (const auto& s : substrings_) { if (!out.empty()) out += ' '; out += s; }
        for (const auto& s : neg_substrings_) { if (!out.empty()) out += ' '; out += '!'; out += s; }
        return out;
    }

private:
    static bool anyFieldContains(const FieldSet& fs, std::string_view needle) {
        return std::ranges::any_of(fs.fields, [&](const auto& kv) {
            return containsCi(kv.second, needle);
        });
    }

    bool active_ = false;
    std::vector<Term> terms_;
    std::vector<std::string> substrings_;
    std::vector<std::string> neg_substrings_;
};

} // namespace UI
} // namespace Tether
