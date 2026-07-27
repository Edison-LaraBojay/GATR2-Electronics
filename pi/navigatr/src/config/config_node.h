// config_node.h
// Non-owning view of one XML element. Generic builders read only routing
// attributes (element name, id, type); everything below a selected
// implementation is handed over as a ConfigNode and parsed by that
// implementation alone. Views are valid only while the document is loaded,
// i.e. during initialization; implementations copy what they keep.
//
// Numeric and boolean getters are strict: an absent attribute yields the
// default, while malformed text, NaN, and infinity are configuration errors,
// never silent defaults. Errors carry the element's XML path.

#pragma once
#include <string>

namespace tinyxml2
{
class XMLElement;
} // namespace tinyxml2

namespace navigatr
{

class ConfigNode
{
public:
    ConfigNode() = default;
    explicit ConfigNode(const tinyxml2::XMLElement* e) : e_(e) {}

    bool        valid() const { return e_ != nullptr; }
    const char* name() const;

    // XPath-like location for error messages, e.g.
    // /System/Pipeline/Preprocessing/Preprocessor[@id='tracking_motion']
    std::string path() const;

    bool        hasAttr(const char* key) const;
    std::string attr(const char* key, const std::string& def = "") const;

    // False and err when the attribute is present but not parseable, not
    // finite, or (for bools) not one of true/false/1/0.
    bool getDouble(const char* key, double def, double& out, std::string& err) const;
    bool getInt(const char* key, long def, long& out, std::string& err) const;
    bool getBool(const char* key, bool def, bool& out, std::string& err) const;

    // First child element, optionally by name. Invalid node when absent.
    ConfigNode child(const char* child_name = nullptr) const;

    // Next sibling element, optionally by name. Invalid node when exhausted.
    ConfigNode next(const char* sibling_name = nullptr) const;

    // Children with a given name, in document order.
    template <typename Fn>
    void forEach(const char* child_name, Fn fn) const {
        for (ConfigNode c = child(child_name); c.valid(); c = c.next(child_name)) {
            fn(c);
        }
    }

private:
    const tinyxml2::XMLElement* e_ = nullptr;
};

} // namespace navigatr
