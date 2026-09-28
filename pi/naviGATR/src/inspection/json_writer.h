// json_writer.h
// Minimal streaming JSON writer for the inspection documents. Strings are
// escaped, non-finite numbers become null (a NaN in a document is a bug,
// not a value), and the writer inserts commas so callers only name keys
// and values. No parsing, no dependency.

#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace navigatr
{

// Byte ranges [begin, end) of a document.
using JsonSpans = std::vector<std::pair<std::size_t, std::size_t>>;

class JsonWriter
{
public:
    JsonWriter() { out_.reserve(4096); }

    // Identity checks: while set, the byte ranges of values that change on
    // every build (clocks, ages, counters) are recorded here, so a hash of
    // the rest tells whether the content changed. Off (null) by default.
    void recordVarying(JsonSpans* spans) { varying_ = spans; }
    std::size_t mark() const { return out_.size(); }
    // Everything written since from is varying.
    void varying(std::size_t from) {
        if (varying_ != nullptr && out_.size() > from) {
            varying_->emplace_back(from, out_.size());
        }
    }
    template <typename T>
    void varyingField(const char* k, const T& v) {
        const std::size_t from = mark();
        field(k, v);
        varying(from);
    }

    void beginObject() {
        separate();
        out_ += '{';
        first_.push_back(true);
    }
    void endObject() {
        out_ += '}';
        first_.pop_back();
        afterValue();
    }
    void beginArray() {
        separate();
        out_ += '[';
        first_.push_back(true);
    }
    void endArray() {
        out_ += ']';
        first_.pop_back();
        afterValue();
    }

    void key(const char* k) {
        separate();
        appendString(k);
        out_ += ':';
        pending_key_ = true;
    }
    void key(const std::string& k) { key(k.c_str()); }

    void value(const char* s) {
        separate();
        appendString(s);
        afterValue();
    }
    void value(const std::string& s) { value(s.c_str()); }
    void value(bool b) {
        separate();
        out_ += b ? "true" : "false";
        afterValue();
    }
    // Fixed-width integers may alias long or long long, depending on the ABI.
    // One integral overload handles either without duplicate declarations.
    template <typename Integer,
              std::enable_if_t<std::is_integral_v<Integer> &&
                                   !std::is_same_v<Integer, bool>, int> = 0>
    void value(Integer v) {
        separate();
        out_ += std::to_string(v);
        afterValue();
    }
    void value(double v) {
        separate();
        if (!std::isfinite(v)) {
            out_ += "null";
        } else {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.6g", v);
            // %g may print an integer-looking value; JSON accepts both
            out_ += buf;
        }
        afterValue();
    }
    void null() {
        separate();
        out_ += "null";
        afterValue();
    }
    // A value the caller already holds as valid JSON text (a checked number
    // token, a nested document). Nothing is escaped or checked here.
    void raw(const std::string& json) {
        separate();
        out_ += json;
        afterValue();
    }

    // Convenience: key + value in one call.
    template <typename T>
    void field(const char* k, const T& v) {
        key(k);
        value(v);
    }
    void fieldNull(const char* k) {
        key(k);
        null();
    }

    const std::string& str() const { return out_; }
    std::string        take() { return std::move(out_); }

private:
    void separate() {
        if (pending_key_) {
            pending_key_ = false;
            return;
        }
        if (first_.empty()) {
            return;
        }
        if (!first_.back()) {
            out_ += ',';
        }
        first_.back() = false;
    }
    void afterValue() {}

    void appendString(const char* s) {
        out_ += '"';
        for (const unsigned char* p = reinterpret_cast<const unsigned char*>(s); *p != 0; ++p) {
            const unsigned char c = *p;
            switch (c) {
            case '"': out_ += "\\\""; break;
            case '\\': out_ += "\\\\"; break;
            case '\n': out_ += "\\n"; break;
            case '\r': out_ += "\\r"; break;
            case '\t': out_ += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out_ += buf;
                } else {
                    out_ += static_cast<char>(c);
                }
            }
        }
        out_ += '"';
    }

    std::string       out_;
    std::vector<bool> first_;
    bool              pending_key_ = false;
    JsonSpans*        varying_     = nullptr;
};

} // namespace navigatr
