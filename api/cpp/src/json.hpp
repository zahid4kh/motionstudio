// A small JSON value with a parser and a writer: enough for the API's
// messages. Private to the library.
#pragma once

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace motionstudio::detail {

class Json {
public:
    using Array  = std::vector<Json>;
    using Object = std::map<std::string, Json>;

    Json() = default;
    Json(std::nullptr_t) {}
    Json(bool b) : v_(b) {}
    Json(int n) : v_(double(n)) {}
    Json(double n) : v_(n) {}
    Json(const char* s) : v_(std::string(s)) {}
    Json(std::string s) : v_(std::move(s)) {}
    Json(Array a) : v_(std::move(a)) {}
    Json(Object o) : v_(std::move(o)) {}

    bool isNull() const   { return std::holds_alternative<std::nullptr_t>(v_); }
    bool isBool() const   { return std::holds_alternative<bool>(v_); }
    bool isNumber() const { return std::holds_alternative<double>(v_); }
    bool isString() const { return std::holds_alternative<std::string>(v_); }
    bool isArray() const  { return std::holds_alternative<Array>(v_); }
    bool isObject() const { return std::holds_alternative<Object>(v_); }

    double number(double fallback = 0) const {
        if (isNumber()) return std::get<double>(v_);
        if (isBool()) return std::get<bool>(v_) ? 1 : 0;
        return fallback;
    }
    bool boolean() const { return isBool() ? std::get<bool>(v_) : number() != 0; }
    std::string str() const { return isString() ? std::get<std::string>(v_) : std::string(); }
    const Array& array() const { static const Array empty; return isArray() ? std::get<Array>(v_) : empty; }
    const Object& object() const { static const Object empty; return isObject() ? std::get<Object>(v_) : empty; }

    bool has(const std::string& key) const { return isObject() && object().count(key) > 0; }
    const Json& operator[](const std::string& key) const {
        static const Json none;
        if (!isObject()) return none;
        auto it = std::get<Object>(v_).find(key);
        return it == std::get<Object>(v_).end() ? none : it->second;
    }
    Json& set(const std::string& key, Json value) {
        if (!isObject()) v_ = Object{};
        std::get<Object>(v_)[key] = std::move(value);
        return *this;
    }

    std::string dump() const {
        std::string out;
        write(out);
        return out;
    }

    static Json parse(const std::string& text) {
        size_t i = 0;
        Json j = parseValue(text, i);
        skip(text, i);
        if (i != text.size()) throw std::runtime_error("JSON: trailing characters");
        return j;
    }

private:
    void write(std::string& out) const {
        if (isNull()) out += "null";
        else if (isBool()) out += std::get<bool>(v_) ? "true" : "false";
        else if (isNumber()) {
            const double d = std::get<double>(v_);
            if (!std::isfinite(d)) { out += "null"; return; }
            char buf[32];
            std::snprintf(buf, sizeof buf, "%.17g", d);
            out += buf;
        } else if (isString()) writeString(out, std::get<std::string>(v_));
        else if (isArray()) {
            out += '[';
            bool first = true;
            for (const Json& j : std::get<Array>(v_)) { if (!first) out += ','; first = false; j.write(out); }
            out += ']';
        } else {
            out += '{';
            bool first = true;
            for (const auto& [k, j] : std::get<Object>(v_)) {
                if (!first) out += ',';
                first = false;
                writeString(out, k);
                out += ':';
                j.write(out);
            }
            out += '}';
        }
    }
    static void writeString(std::string& out, const std::string& s) {
        out += '"';
        for (unsigned char c : s) {
            switch (c) {
                case '"':  out += "\\\""; break;
                case '\\': out += "\\\\"; break;
                case '\n': out += "\\n"; break;
                case '\r': out += "\\r"; break;
                case '\t': out += "\\t"; break;
                default:
                    if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); out += b; }
                    else out += char(c);
            }
        }
        out += '"';
    }

    static void skip(const std::string& s, size_t& i) {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
    }
    [[noreturn]] static void bad(const char* what) { throw std::runtime_error(std::string("JSON: ") + what); }

    static Json parseValue(const std::string& s, size_t& i) {
        skip(s, i);
        if (i >= s.size()) bad("unexpected end");
        const char c = s[i];
        if (c == '{') {
            ++i;
            Object o;
            skip(s, i);
            if (i < s.size() && s[i] == '}') { ++i; return o; }
            while (true) {
                skip(s, i);
                if (i >= s.size() || s[i] != '"') bad("expected a key");
                std::string key = parseString(s, i);
                skip(s, i);
                if (i >= s.size() || s[i] != ':') bad("expected ':'");
                ++i;
                o[key] = parseValue(s, i);
                skip(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == '}') { ++i; return o; }
                bad("expected ',' or '}'");
            }
        }
        if (c == '[') {
            ++i;
            Array a;
            skip(s, i);
            if (i < s.size() && s[i] == ']') { ++i; return a; }
            while (true) {
                a.push_back(parseValue(s, i));
                skip(s, i);
                if (i < s.size() && s[i] == ',') { ++i; continue; }
                if (i < s.size() && s[i] == ']') { ++i; return a; }
                bad("expected ',' or ']'");
            }
        }
        if (c == '"') return parseString(s, i);
        if (s.compare(i, 4, "true") == 0)  { i += 4; return true; }
        if (s.compare(i, 5, "false") == 0) { i += 5; return false; }
        if (s.compare(i, 4, "null") == 0)  { i += 4; return nullptr; }
        size_t end = i;
        while (end < s.size() && (std::isdigit(static_cast<unsigned char>(s[end])) || s[end] == '-' || s[end] == '+'
                                  || s[end] == '.' || s[end] == 'e' || s[end] == 'E')) ++end;
        if (end == i) bad("unexpected character");
        const double d = std::strtod(s.substr(i, end - i).c_str(), nullptr);
        i = end;
        return d;
    }

    static void putUtf8(std::string& out, unsigned cp) {
        if (cp < 0x80) out += char(cp);
        else if (cp < 0x800) { out += char(0xC0 | (cp >> 6)); out += char(0x80 | (cp & 0x3F)); }
        else if (cp < 0x10000) { out += char(0xE0 | (cp >> 12)); out += char(0x80 | ((cp >> 6) & 0x3F)); out += char(0x80 | (cp & 0x3F)); }
        else { out += char(0xF0 | (cp >> 18)); out += char(0x80 | ((cp >> 12) & 0x3F)); out += char(0x80 | ((cp >> 6) & 0x3F)); out += char(0x80 | (cp & 0x3F)); }
    }

    static std::string parseString(const std::string& s, size_t& i) {
        ++i;   // opening quote
        std::string out;
        while (i < s.size() && s[i] != '"') {
            char c = s[i++];
            if (c != '\\') { out += c; continue; }
            if (i >= s.size()) bad("bad escape");
            c = s[i++];
            switch (c) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'u': {
                    if (i + 4 > s.size()) bad("bad \\u escape");
                    unsigned cp = unsigned(std::stoul(s.substr(i, 4), nullptr, 16));
                    i += 4;
                    if (cp >= 0xD800 && cp < 0xDC00 && i + 6 <= s.size() && s[i] == '\\' && s[i + 1] == 'u') {
                        const unsigned lo = unsigned(std::stoul(s.substr(i + 2, 4), nullptr, 16));
                        i += 6;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    putUtf8(out, cp);
                    break;
                }
                default: out += c;   // \" \\ \/
            }
        }
        if (i >= s.size()) bad("unterminated string");
        ++i;   // closing quote
        return out;
    }

    std::variant<std::nullptr_t, bool, double, std::string, Array, Object> v_ = nullptr;
};

} // namespace motionstudio::detail
