// perf_json.cpp

#include "perf_json.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace navigatr
{
namespace perf
{

namespace
{

struct Parser {
    const std::string& s;
    std::size_t        i = 0;
    std::string        err;

    void skip() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) {
            ++i;
        }
    }

    bool fail(const char* what) {
        if (err.empty()) {
            err = std::string(what) + " at " + std::to_string(i);
        }
        return false;
    }

    static void putUtf8(std::string& out, unsigned cp) {
        if (cp < 0x80) {
            out.push_back(static_cast<char>(cp));
        } else if (cp < 0x800) {
            out.push_back(static_cast<char>(0xc0 | (cp >> 6)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
        } else if (cp < 0x10000) {
            out.push_back(static_cast<char>(0xe0 | (cp >> 12)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
        } else {
            out.push_back(static_cast<char>(0xf0 | (cp >> 18)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3f)));
            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3f)));
            out.push_back(static_cast<char>(0x80 | (cp & 0x3f)));
        }
    }

    bool hex4(unsigned& v) {
        if (i + 4 > s.size()) {
            return false;
        }
        v = 0;
        for (int k = 0; k < 4; ++k) {
            const char c = s[i++];
            v <<= 4;
            if (c >= '0' && c <= '9') {
                v |= static_cast<unsigned>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                v |= static_cast<unsigned>(c - 'a' + 10);
            } else if (c >= 'A' && c <= 'F') {
                v |= static_cast<unsigned>(c - 'A' + 10);
            } else {
                return false;
            }
        }
        return true;
    }

    bool string(std::string& out) {
        if (i >= s.size() || s[i] != '"') {
            return fail("expected string");
        }
        ++i;
        out.clear();
        while (i < s.size()) {
            const char c = s[i++];
            if (c == '"') {
                return true;
            }
            if (c != '\\') {
                out.push_back(c);
                continue;
            }
            if (i >= s.size()) {
                break;
            }
            const char e = s[i++];
            switch (e) {
            case '"': out.push_back('"'); break;
            case '\\': out.push_back('\\'); break;
            case '/': out.push_back('/'); break;
            case 'b': out.push_back('\b'); break;
            case 'f': out.push_back('\f'); break;
            case 'n': out.push_back('\n'); break;
            case 'r': out.push_back('\r'); break;
            case 't': out.push_back('\t'); break;
            case 'u': {
                unsigned cp = 0;
                if (!hex4(cp)) {
                    return fail("bad \\u escape");
                }
                if (cp >= 0xd800 && cp < 0xdc00 && i + 6 <= s.size() && s[i] == '\\' &&
                    s[i + 1] == 'u') {
                    i += 2;
                    unsigned lo = 0;
                    if (!hex4(lo)) {
                        return fail("bad surrogate");
                    }
                    cp = 0x10000 + ((cp - 0xd800) << 10) + (lo - 0xdc00);
                }
                putUtf8(out, cp);
                break;
            }
            default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }

    bool value(Json& v, int depth) {
        if (depth > 200) {
            return fail("too deep");
        }
        skip();
        if (i >= s.size()) {
            return fail("unexpected end");
        }
        const char c = s[i];
        if (c == '{') {
            ++i;
            v.type = Json::Type::kObject;
            skip();
            if (i < s.size() && s[i] == '}') {
                ++i;
                return true;
            }
            for (;;) {
                skip();
                std::string key;
                if (!string(key)) {
                    return false;
                }
                skip();
                if (i >= s.size() || s[i] != ':') {
                    return fail("expected ':'");
                }
                ++i;
                v.fields.emplace_back(std::move(key), Json{});
                if (!value(v.fields.back().second, depth + 1)) {
                    return false;
                }
                skip();
                if (i < s.size() && s[i] == ',') {
                    ++i;
                    continue;
                }
                if (i < s.size() && s[i] == '}') {
                    ++i;
                    return true;
                }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            ++i;
            v.type = Json::Type::kArray;
            skip();
            if (i < s.size() && s[i] == ']') {
                ++i;
                return true;
            }
            for (;;) {
                v.items.emplace_back();
                if (!value(v.items.back(), depth + 1)) {
                    return false;
                }
                skip();
                if (i < s.size() && s[i] == ',') {
                    ++i;
                    continue;
                }
                if (i < s.size() && s[i] == ']') {
                    ++i;
                    return true;
                }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') {
            v.type = Json::Type::kString;
            return string(v.str);
        }
        if (s.compare(i, 4, "true") == 0) {
            v.type = Json::Type::kBool;
            v.b    = true;
            i += 4;
            return true;
        }
        if (s.compare(i, 5, "false") == 0) {
            v.type = Json::Type::kBool;
            i += 5;
            return true;
        }
        if (s.compare(i, 4, "null") == 0) {
            v.type = Json::Type::kNull;
            i += 4;
            return true;
        }
        const char* begin = s.c_str() + i;
        char*       end   = nullptr;
        v.number          = std::strtod(begin, &end);
        if (end == begin) {
            return fail("bad value");
        }
        v.type = Json::Type::kNumber;
        i += static_cast<std::size_t>(end - begin);
        return true;
    }
};

void write(const Json& v, std::string& out) {
    switch (v.type) {
    case Json::Type::kNull: out += "null"; break;
    case Json::Type::kBool: out += v.b ? "true" : "false"; break;
    case Json::Type::kNumber: {
        char buf[40];
        if (std::isfinite(v.number)) {
            std::snprintf(buf, sizeof(buf), "%.10g", v.number);
        } else {
            std::snprintf(buf, sizeof(buf), "null");
        }
        out += buf;
        break;
    }
    case Json::Type::kString: out += jsonQuote(v.str); break;
    case Json::Type::kArray:
        out.push_back('[');
        for (std::size_t k = 0; k < v.items.size(); ++k) {
            if (k > 0) {
                out.push_back(',');
            }
            write(v.items[k], out);
        }
        out.push_back(']');
        break;
    case Json::Type::kObject:
        out.push_back('{');
        for (std::size_t k = 0; k < v.fields.size(); ++k) {
            if (k > 0) {
                out.push_back(',');
            }
            out += jsonQuote(v.fields[k].first);
            out.push_back(':');
            write(v.fields[k].second, out);
        }
        out.push_back('}');
        break;
    }
}

double quantile(const std::vector<double>& sorted, double q) {
    if (sorted.empty()) {
        return 0.0;
    }
    // nearest rank, so a reported percentile is a value that occurred
    const double      rank = std::ceil(q * static_cast<double>(sorted.size()));
    const std::size_t idx  = static_cast<std::size_t>(std::max(1.0, rank)) - 1;
    return sorted[std::min(idx, sorted.size() - 1)];
}

std::string num(double v) {
    char buf[40];
    if (!std::isfinite(v)) {
        return "null";
    }
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    return buf;
}

} // namespace

const Json* Json::get(const std::string& key) const {
    if (type != Type::kObject) {
        return nullptr;
    }
    for (const auto& kv : fields) {
        if (kv.first == key) {
            return &kv.second;
        }
    }
    return nullptr;
}

const Json* Json::at(std::initializer_list<const char*> path) const {
    const Json* v = this;
    for (const char* key : path) {
        v = v->get(key);
        if (v == nullptr) {
            return nullptr;
        }
    }
    return v;
}

bool parseJson(const std::string& text, Json& out, std::string* err) {
    out = Json{};
    Parser p{text, 0, std::string()};
    if (!p.value(out, 0)) {
        if (err != nullptr) {
            *err = p.err;
        }
        return false;
    }
    return true;
}

std::string toJson(const Json& v) {
    std::string out;
    write(v, out);
    return out;
}

std::string jsonQuote(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
                out += buf;
            } else {
                out.push_back(c);
            }
        }
    }
    out.push_back('"');
    return out;
}

Summary summarize(std::vector<double> values) {
    // a missing field reads as NaN; it is not a sample (and breaks sort)
    values.erase(std::remove_if(values.begin(), values.end(),
                                [](double v) { return !std::isfinite(v); }),
                 values.end());
    Summary s;
    s.n = values.size();
    if (values.empty()) {
        return s;
    }
    std::sort(values.begin(), values.end());
    double sum = 0.0;
    for (double v : values) {
        sum += v;
    }
    s.min  = values.front();
    s.max  = values.back();
    s.mean = sum / static_cast<double>(values.size());
    s.p50  = quantile(values, 0.50);
    s.p95  = quantile(values, 0.95);
    s.p99  = quantile(values, 0.99);
    return s;
}

std::string summaryJson(const Summary& s) {
    return "{\"n\":" + std::to_string(s.n) + ",\"min\":" + num(s.min) + ",\"p50\":" + num(s.p50) +
           ",\"p95\":" + num(s.p95) + ",\"p99\":" + num(s.p99) + ",\"max\":" + num(s.max) +
           ",\"mean\":" + num(s.mean) + "}";
}

} // namespace perf
} // namespace navigatr
