/**
 * @file KlippyUdsServerJson.cpp
 * @brief JSON utilities, parser, and UDS connection implementation
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"
#include "tether/klipper/klippy/AdvancedObjects.hpp"
#include "UdsConnection_internal.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <netinet/in.h>
#include <set>
#include <signal.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace tether::klipper::klippy {

// ============================================================================
// JsonValue implementation
// ============================================================================

std::string JsonValue::dump() const {
    std::ostringstream os;
    switch (type_) {
        case Type::Null:   os << "null"; break;
        case Type::Bool:   os << (bool_ ? "true" : "false"); break;
        case Type::Int:    os << int_; break;
        case Type::Double:
            os << std::setprecision(17) << double_;
            break;
        case Type::String: {
            os << '"';
            for (char c : str_) {
                switch (c) {
                    case '"':  os << "\\\""; break;
                    case '\\': os << "\\\\"; break;
                    case '\n': os << "\\n"; break;
                    case '\r': os << "\\r"; break;
                    case '\t': os << "\\t"; break;
                    case '\b': os << "\\b"; break;
                    case '\f': os << "\\f"; break;
                    default:
                        if (static_cast<unsigned char>(c) < 0x20) {
                            os << "\\u" << std::hex << std::setw(4)
                               << std::setfill('0') << static_cast<int>(c);
                            os << std::dec;
                        } else {
                            os << c;
                        }
                }
            }
            os << '"';
            break;
        }
        case Type::Array: {
            os << '[';
            for (size_t i = 0; i < arr_.size(); ++i) {
                if (i) os << ',';
                os << arr_[i].dump();
            }
            os << ']';
            break;
        }
        case Type::Object: {
            os << '{';
            bool first = true;
            for (const auto& [k, v] : obj_) {
                if (!first) os << ',';
                first = false;
                os << '"' << k << "\":" << v.dump();
            }
            os << '}';
            break;
        }
    }
    return os.str();
}

namespace {

/// @brief Simple recursive-descent JSON parser.
class JsonParser {
public:
    explicit JsonParser(std::string_view s) : s_(s), pos_(0) {}

    std::optional<JsonValue> parse() {
        skipWs();
        auto v = parseValue();
        skipWs();
        return v;
    }

private:
    std::string_view s_;
    size_t pos_;

    void skipWs() {
        while (pos_ < s_.size() &&
               (s_[pos_] == ' ' || s_[pos_] == '\t' ||
                s_[pos_] == '\n' || s_[pos_] == '\r'))
            ++pos_;
    }

    char peek() {
        return pos_ < s_.size() ? s_[pos_] : '\0';
    }

    char get() {
        return pos_ < s_.size() ? s_[pos_++] : '\0';
    }

    std::optional<JsonValue> parseValue() {
        skipWs();
        char c = peek();
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == '"') return parseString();
        if (c == 't' || c == 'f') return parseBool();
        if (c == 'n') return parseNull();
        if (c == '-' || (c >= '0' && c <= '9')) return parseNumber();
        return std::nullopt;
    }

    std::optional<JsonValue> parseObject() {
        get(); // consume '{'
        std::map<std::string, JsonValue> obj;
        skipWs();
        if (peek() == '}') { get(); return JsonValue(obj); }
        while (true) {
            skipWs();
            if (peek() != '"') return std::nullopt;
            auto key = parseString();
            if (!key) return std::nullopt;
            skipWs();
            if (get() != ':') return std::nullopt;
            auto val = parseValue();
            if (!val) return std::nullopt;
            obj[key->asString()] = std::move(*val);
            skipWs();
            char c = get();
            if (c == '}') break;
            if (c != ',') return std::nullopt;
        }
        return JsonValue(std::move(obj));
    }

    std::optional<JsonValue> parseArray() {
        get(); // consume '['
        std::vector<JsonValue> arr;
        skipWs();
        if (peek() == ']') { get(); return JsonValue(arr); }
        while (true) {
            auto val = parseValue();
            if (!val) return std::nullopt;
            arr.push_back(std::move(*val));
            skipWs();
            char c = get();
            if (c == ']') break;
            if (c != ',') return std::nullopt;
        }
        return JsonValue(std::move(arr));
    }

    std::optional<JsonValue> parseString() {
        get(); // consume '"'
        std::string s;
        while (pos_ < s_.size()) {
            char c = get();
            if (c == '"') return JsonValue(std::move(s));
            if (c == '\\') {
                char esc = get();
                switch (esc) {
                    case '"': s += '"'; break;
                    case '\\': s += '\\'; break;
                    case '/': s += '/'; break;
                    case 'n': s += '\n'; break;
                    case 'r': s += '\r'; break;
                    case 't': s += '\t'; break;
                    case 'b': s += '\b'; break;
                    case 'f': s += '\f'; break;
                    case 'u': {
                        if (pos_ + 4 > s_.size()) return std::nullopt;
                        unsigned int cp = 0;
                        for (int i = 0; i < 4; ++i) {
                            char h = get();
                            cp <<= 4;
                            if (h >= '0' && h <= '9') cp |= h - '0';
                            else if (h >= 'a' && h <= 'f') cp |= h - 'a' + 10;
                            else if (h >= 'A' && h <= 'F') cp |= h - 'A' + 10;
                            else return std::nullopt;
                        }
                        // Simple UTF-8 encoding
                        if (cp < 0x80) s += static_cast<char>(cp);
                        else if (cp < 0x800) {
                            s += static_cast<char>(0xC0 | (cp >> 6));
                            s += static_cast<char>(0x80 | (cp & 0x3F));
                        } else {
                            s += static_cast<char>(0xE0 | (cp >> 12));
                            s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                            s += static_cast<char>(0x80 | (cp & 0x3F));
                        }
                        break;
                    }
                    default: return std::nullopt;
                }
            } else {
                s += c;
            }
        }
        return std::nullopt;
    }

    std::optional<JsonValue> parseBool() {
        if (s_.substr(pos_, 4) == "true") { pos_ += 4; return JsonValue(true); }
        if (s_.substr(pos_, 5) == "false") { pos_ += 5; return JsonValue(false); }
        return std::nullopt;
    }

    std::optional<JsonValue> parseNull() {
        if (s_.substr(pos_, 4) == "null") { pos_ += 4; return JsonValue(); }
        return std::nullopt;
    }

    std::optional<JsonValue> parseNumber() {
        size_t start = pos_;
        if (peek() == '-') get();
        while (peek() >= '0' && peek() <= '9') get();
        bool isDouble = false;
        if (peek() == '.') { isDouble = true; get(); while (peek() >= '0' && peek() <= '9') get(); }
        if (peek() == 'e' || peek() == 'E') {
            isDouble = true; get();
            if (peek() == '+' || peek() == '-') get();
            while (peek() >= '0' && peek() <= '9') get();
        }
        std::string numStr(s_.substr(start, pos_ - start));
        try {
            if (isDouble) return JsonValue(std::stod(numStr));
            return JsonValue(static_cast<int64_t>(std::stoll(numStr)));
        } catch (...) { return std::nullopt; }
    }
};

} // anonymous namespace

std::optional<JsonValue> JsonValue::parse(std::string_view json) {
    JsonParser p(json);
    return p.parse();
}

const JsonValue* JsonValue::find(std::string_view key) const {
    if (type_ != Type::Object) return nullptr;
    auto it = obj_.find(std::string(key));
    if (it == obj_.end()) return nullptr;
    return &it->second;
}

// UdsConnection is defined in UdsConnection_internal.hpp
// (included by all split KlippyUdsServer*.cpp files)

} // namespace tether::klipper::klippy
