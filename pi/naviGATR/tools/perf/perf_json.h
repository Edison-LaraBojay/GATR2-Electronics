// perf_json.h
// A small JSON reader for the perf tools (inspection messages, CDP replies)
// and the summary statistics they print.

#pragma once
#include <cstddef>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace navigatr
{
namespace perf
{

struct Json {
    enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

    Type                                      type   = Type::kNull;
    bool                                      b      = false;
    double                                    number = 0.0;
    std::string                               str;
    std::vector<Json>                         items;
    std::vector<std::pair<std::string, Json>> fields;

    const Json* get(const std::string& key) const;
    // nested object lookup, null when any step is missing
    const Json* at(std::initializer_list<const char*> path) const;

    bool        isNumber() const { return type == Type::kNumber; }
    double      numberOr(double def) const { return type == Type::kNumber ? number : def; }
    std::string stringOr(const std::string& def) const {
        return type == Type::kString ? str : def;
    }
};

bool parseJson(const std::string& text, Json& out, std::string* err = nullptr);

// Compact JSON text for a value (for echoing sub-objects into reports).
std::string toJson(const Json& v);

std::string jsonQuote(const std::string& s);

// median/p95/max etc. of a sample set; NaN/inf are not samples; empty sets report n = 0
struct Summary {
    std::size_t n = 0;
    double      min = 0, p50 = 0, p95 = 0, p99 = 0, max = 0, mean = 0;
};

Summary summarize(std::vector<double> values);

// {"n":..,"min":..,"p50":..,"p95":..,"p99":..,"max":..,"mean":..}
std::string summaryJson(const Summary& s);

} // namespace perf
} // namespace navigatr
