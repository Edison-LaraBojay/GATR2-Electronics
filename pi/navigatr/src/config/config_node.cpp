// config_node.cpp

#include "config/config_node.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "tinyxml2/tinyxml2.h"

namespace navigatr
{

namespace
{

std::string badValue(const ConfigNode& node, const char* key, const char* raw) {
    return node.path() + " attribute " + key + ": invalid value \"" + raw + "\"";
}

} // namespace

const char* ConfigNode::name() const { return e_ == nullptr ? "" : e_->Name(); }

std::string ConfigNode::path() const {
    if (e_ == nullptr) {
        return "<none>";
    }
    std::vector<std::string> parts;
    for (const tinyxml2::XMLElement* e = e_; e != nullptr;
         e                             = e->Parent() != nullptr ? e->Parent()->ToElement()
                                                                : nullptr) {
        std::string part = e->Name();
        const char* id   = e->Attribute("id");
        if (id != nullptr) {
            part += std::string("[@id='") + id + "']";
        }
        parts.push_back(std::move(part));
    }
    std::string out;
    for (auto it = parts.rbegin(); it != parts.rend(); ++it) {
        out += "/" + *it;
    }
    return out;
}

bool ConfigNode::hasAttr(const char* key) const {
    return e_ != nullptr && e_->Attribute(key) != nullptr;
}

std::string ConfigNode::attr(const char* key, const std::string& def) const {
    if (e_ == nullptr) {
        return def;
    }
    const char* v = e_->Attribute(key);
    return v == nullptr ? def : std::string(v);
}

bool ConfigNode::getDouble(const char* key, double def, double& out,
                           std::string& err) const {
    out             = def;
    const char* raw = e_ == nullptr ? nullptr : e_->Attribute(key);
    if (raw == nullptr) {
        return true;
    }
    char*        end = nullptr;
    const double v   = std::strtod(raw, &end);
    if (end == raw || *end != '\0' || !std::isfinite(v)) {
        err = badValue(*this, key, raw);
        return false;
    }
    out = v;
    return true;
}

bool ConfigNode::getInt(const char* key, long def, long& out, std::string& err) const {
    out             = def;
    const char* raw = e_ == nullptr ? nullptr : e_->Attribute(key);
    if (raw == nullptr) {
        return true;
    }
    char*      end = nullptr;
    errno          = 0;
    const long v   = std::strtol(raw, &end, 10);
    if (end == raw || *end != '\0' || errno == ERANGE) {
        err = badValue(*this, key, raw);
        return false;
    }
    out = v;
    return true;
}

bool ConfigNode::getBool(const char* key, bool def, bool& out, std::string& err) const {
    out             = def;
    const char* raw = e_ == nullptr ? nullptr : e_->Attribute(key);
    if (raw == nullptr) {
        return true;
    }
    const std::string s(raw);
    if (s == "true" || s == "1") {
        out = true;
        return true;
    }
    if (s == "false" || s == "0") {
        out = false;
        return true;
    }
    err = badValue(*this, key, raw);
    return false;
}

bool ConfigNode::requireDouble(const char* key, double& out, std::string& err) const {
    if (!hasAttr(key)) {
        err = path() + ": missing required attribute " + key;
        return false;
    }
    return getDouble(key, 0.0, out, err);
}

bool ConfigNode::requireInt(const char* key, long& out, std::string& err) const {
    if (!hasAttr(key)) {
        err = path() + ": missing required attribute " + key;
        return false;
    }
    return getInt(key, 0, out, err);
}

bool ConfigNode::requireAttr(const char* key, std::string& out, std::string& err) const {
    if (!hasAttr(key)) {
        err = path() + ": missing required attribute " + key;
        return false;
    }
    out = attr(key);
    if (out.empty()) {
        err = path() + ": attribute " + key + " must not be empty";
        return false;
    }
    return true;
}

ConfigNode ConfigNode::child(const char* child_name) const {
    return e_ == nullptr ? ConfigNode{} : ConfigNode{e_->FirstChildElement(child_name)};
}

ConfigNode ConfigNode::next(const char* sibling_name) const {
    return e_ == nullptr ? ConfigNode{} : ConfigNode{e_->NextSiblingElement(sibling_name)};
}

} // namespace navigatr
