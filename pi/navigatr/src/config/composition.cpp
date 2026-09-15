// composition.cpp

#include "config/composition.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "tinyxml2/tinyxml2.h"

namespace navigatr
{
namespace
{

bool readFileBytes(const std::string& path, std::string& out, std::string& err) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        err = "cannot open " + path;
        return false;
    }
    std::ostringstream bytes;
    bytes << file.rdbuf();
    if (file.bad()) {
        err = "cannot read " + path;
        return false;
    }
    out = bytes.str();
    return true;
}

bool endsWith(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string chainText(const std::vector<std::string>& chain) {
    std::string result;
    for (const auto& path : chain) {
        if (!result.empty()) result += " -> ";
        result += path;
    }
    return result;
}

// Only framework-owned positions reserve `file` for XML composition.
// Resource/sensor implementation children (Device, Source, Serial, etc.)
// are opaque: their file attributes retain their implementation meaning.
enum class Section {
    kOpaque, kSystem, kConfiguration, kRobot, kField, kResources, kSensors,
    kPipeline, kLocalization, kStage,
};

struct ChildSection {
    bool include = false;
    Section kind = Section::kOpaque;
};

ChildSection childSection(Section parent, const std::string& name) {
    switch (parent) {
    case Section::kSystem:
        if (name == "Resources") return {true, Section::kResources};
        if (name == "Sensors") return {true, Section::kSensors};
        if (name == "Pipeline") return {true, Section::kPipeline};
        if (name == "Loop" || name == "Inspection") return {true, Section::kOpaque};
        break;
    case Section::kConfiguration:
        if (name == "Robot") return {true, Section::kRobot};
        if (name == "Field") return {true, Section::kField};
        if (name == "Pipeline") return {true, Section::kPipeline};
        if (name == "Loop" || name == "Inspection") return {true, Section::kOpaque};
        break;
    case Section::kRobot:
        if (name == "Resources") return {true, Section::kResources};
        if (name == "Sensors") return {true, Section::kSensors};
        break;
    case Section::kField:
    case Section::kResources:
        if (name == "Resource") return {true, Section::kOpaque};
        break;
    case Section::kSensors:
        if (name == "Sensor") return {true, Section::kOpaque};
        break;
    case Section::kPipeline:
        // The slot builder validates names/types. This boundary also works
        // for implementation-selected subpipelines and their registered slots.
        return {true, name == "Localization" ? Section::kLocalization : Section::kStage};
    case Section::kLocalization:
        if (name == "Observation" || name == "Estimator") return {true, Section::kStage};
        if (name == "History" || name == "InitialPlacement") return {true, Section::kOpaque};
        break;
    case Section::kStage:
        if (name == "Pipeline") return {true, Section::kPipeline};
        break;
    case Section::kOpaque:
        break;
    }
    return {};
}

struct Location {
    std::string file;
    int line = 0;
};

bool rejectUnresolvedIncludes(const tinyxml2::XMLElement& node, Section section,
                              const std::string& path, std::string& err) {
    if (node.Attribute("file") != nullptr) {
        err = path + ": unresolved XML file reference; use System::buildFromFile so "
              "relative references resolve against their containing XML file";
        return false;
    }
    for (const auto* child = node.FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        const auto slot = childSection(section, child->Name());
        if (slot.include && !rejectUnresolvedIncludes(*child, slot.kind,
                                                       path + "/" + child->Name(), err)) {
            return false;
        }
    }
    return true;
}

class Resolver {
public:
    explicit Resolver(ResolvedConfiguration& output) : output_(output) {}

    bool resolve(const std::string& path, tinyxml2::XMLDocument& document,
                 std::string& err) {
        std::string normalized;
        if (!load(path, document, normalized, err)) return false;
        auto* root = document.RootElement();
        if (root == nullptr) return fail(normalized + ": empty document", err);
        const std::string name = root->Name();
        if (name != "Configuration" && name != "System") {
            return fail(normalized + ": root element must be Configuration or System", err);
        }
        const Section section = name == "Configuration" ? Section::kConfiguration : Section::kSystem;
        return expand(root, section, normalized, err);
    }

    Location location(const tinyxml2::XMLElement* node) const {
        const auto found = origins_.find(node);
        return found == origins_.end() ? Location{"configuration", node->GetLineNum()} : found->second;
    }

    std::string where(const tinyxml2::XMLElement* node) const {
        const auto source = location(node);
        return source.file + ":" + std::to_string(source.line);
    }

private:
    bool fail(const std::string& message, std::string& err) const {
        err = message + " (include chain: " + chainText(active_) + ")";
        return false;
    }

    void remember(const tinyxml2::XMLElement* source, tinyxml2::XMLElement* copy,
                  const std::string& file) {
        const auto found = origins_.find(source);
        origins_[copy] = source == copy || found == origins_.end()
                             ? Location{file, source->GetLineNum()} : found->second;
        auto* destination = copy->FirstChildElement();
        for (const auto* child = source->FirstChildElement(); child != nullptr;
             child = child->NextSiblingElement(), destination = destination->NextSiblingElement()) {
            remember(child, destination, file);
        }
    }

    bool load(const std::string& requested, tinyxml2::XMLDocument& document,
              std::string& normalized, std::string& err) {
        std::error_code error;
        const auto canonical = std::filesystem::weakly_canonical(requested, error);
        if (error) return fail("cannot resolve " + requested + ": " + error.message(), err);
        normalized = canonical.generic_string();
        const auto same = [&](const std::string& previous) {
            if (previous == normalized) return true;
            std::error_code equivalent_error;
            return std::filesystem::equivalent(previous, normalized, equivalent_error) &&
                   !equivalent_error;
        };
        for (const auto& active : active_) {
            if (same(active)) return fail("cyclic include of " + normalized, err);
        }
        for (const auto& seen : output_.files) {
            if (same(seen)) return fail("file " + normalized + " is included more than once", err);
        }
        active_.push_back(normalized);
        if (endsWith(normalized, ".xml.in")) {
            return fail("file " + normalized + " is an intentionally non-runnable template", err);
        }
        std::string bytes;
        if (!readFileBytes(normalized, bytes, err)) return fail(err, err);
        if (document.Parse(bytes.c_str()) != tinyxml2::XML_SUCCESS) {
            return fail(normalized + ":" + std::to_string(document.ErrorLineNum()) +
                        ": cannot parse xml: " + document.ErrorStr(), err);
        }
        output_.files.push_back(normalized);
        output_.digest = contentDigest(output_.digest, normalized);
        output_.digest = contentDigest(output_.digest, bytes);
        if (document.RootElement() != nullptr) {
            remember(document.RootElement(), document.RootElement(), normalized);
        }
        return true;
    }

    bool checkReference(const tinyxml2::XMLElement* node, std::string& err) const {
        for (const auto* attr = node->FirstAttribute(); attr != nullptr; attr = attr->Next()) {
            const std::string name = attr->Name();
            if (name != "file" && name != "calibration_status") {
                return fail(where(node) + ": " + node->Name() +
                            " file reference cannot also specify " + name +
                            "; put configuration attributes in the referenced root", err);
            }
        }
        for (const auto* child = node->FirstChild(); child != nullptr; child = child->NextSibling()) {
            if (child->ToComment() != nullptr) continue;
            if (const auto* text = child->ToText()) {
                if (std::string(text->Value()).find_first_not_of(" \t\r\n") == std::string::npos) continue;
            }
            return fail(where(node) + ": " + node->Name() +
                        " file reference cannot also contain inline content", err);
        }
        if (std::string(node->Attribute("file")).empty()) {
            return fail(where(node) + ": " + node->Name() + " needs a nonempty file", err);
        }
        return true;
    }

    bool expand(tinyxml2::XMLElement*& node, Section section, const std::string& file,
                std::string& err) {
        if (node->Attribute("file") != nullptr) {
            if (!checkReference(node, err)) return false;
            const std::string expected = node->Name();
            const std::filesystem::path reference(node->Attribute("file"));
            const auto requested = reference.is_absolute()
                                       ? reference : std::filesystem::path(file).parent_path() / reference;
            tinyxml2::XMLDocument fragment;
            std::string resolved;
            if (!load(requested.string(), fragment, resolved, err)) return false;
            const auto* root = fragment.RootElement();
            if (root == nullptr) return fail(resolved + ": empty document", err);
            const std::string actual = root->Name();
            const bool field_resource = expected == "Field" && actual == "Resource";
            if (actual != expected && !field_resource) {
                return fail(resolved + ": root element must be " + expected +
                            (expected == "Field" ? " or Resource" : "") +
                            ", found " + actual, err);
            }
            auto* replacement = root->DeepClone(node->GetDocument())->ToElement();
            remember(root, replacement, resolved);
            auto* parent = node->Parent();
            parent->InsertAfterChild(node, replacement);
            parent->DeleteChild(node);
            node = replacement;
            // A legacy Field reference can contain a single Resource. Wrap
            // it in Field so the profile's structure remains explicit.
            if (field_resource) {
                auto* wrapper = node->GetDocument()->NewElement("Field");
                origins_[wrapper] = location(node);
                parent->InsertAfterChild(node, wrapper);
                wrapper->InsertEndChild(node);
                node = wrapper;
            }
            const bool ok = expand(node, section, resolved, err);
            active_.pop_back();
            return ok;
        }
        for (auto* child = node->FirstChildElement(); child != nullptr;) {
            auto* next = child->NextSiblingElement();
            const auto child_section = childSection(section, child->Name());
            if (child_section.include && !expand(child, child_section.kind, file, err)) return false;
            child = next;
        }
        return true;
    }

    ResolvedConfiguration& output_;
    std::vector<std::string> active_;
    std::unordered_map<const tinyxml2::XMLElement*, Location> origins_;
};

bool rejectTokens(const tinyxml2::XMLElement* element, const Resolver& resolver,
                  std::string& err) {
    for (const auto* attr = element->FirstAttribute(); attr != nullptr; attr = attr->Next()) {
        // Human annotations have no runtime meaning, including template-like text.
        if (std::string(attr->Name()) == "calibration_status") continue;
        const std::string value = attr->Value();
        const auto first = value.find('@');
        if (first != std::string::npos && value.find('@', first + 1) != std::string::npos) {
            err = resolver.where(element) + ": element " + element->Name() + " attribute " +
                  attr->Name() + " still holds template placeholder \"" + value +
                  "\"; measure and configure the value";
            return false;
        }
    }
    for (const auto* child = element->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        if (!rejectTokens(child, resolver, err)) return false;
    }
    return true;
}

bool noteId(std::vector<std::pair<std::string, std::string>>& seen,
            const std::string& kind, const tinyxml2::XMLElement* element,
            const Resolver& resolver, std::string& err) {
    const char* id = element->Attribute("id");
    const auto file = resolver.location(element).file;
    if (id == nullptr || std::string(id).empty()) {
        err = resolver.where(element) + ": " + kind + " needs id";
        return false;
    }
    for (const auto& previous : seen) {
        if (previous.first == id) {
            err = kind + " id " + id + " is declared in both " + previous.second + " and " + file;
            return false;
        }
    }
    seen.emplace_back(id, file);
    return true;
}

bool composeProfile(const tinyxml2::XMLElement* root, const Resolver& resolver,
                    tinyxml2::XMLDocument& merged, ResolvedConfiguration& output,
                    std::string& err) {
    const char* id = root->Attribute("id");
    if (id == nullptr || std::string(id).empty()) {
        err = resolver.where(root) + ": Configuration needs id";
        return false;
    }
    output.id = id;
    auto* system = merged.NewElement("System");
    merged.InsertEndChild(system);
    auto* resources = merged.NewElement("Resources");
    auto* sensors = merged.NewElement("Sensors");
    system->InsertEndChild(resources);
    system->InsertEndChild(sensors);
    std::vector<std::pair<std::string, std::string>> resource_ids, sensor_ids;
    std::vector<std::string> singular;
    bool have_robot = false, have_pipeline = false;

    const auto copyItem = [&](const tinyxml2::XMLElement* item, const char* kind,
                              tinyxml2::XMLElement* destination,
                              std::vector<std::pair<std::string, std::string>>& ids) {
        if (std::string(item->Name()) != kind) {
            err = resolver.where(item) + ": expected " + kind + ", found " + item->Name();
            return false;
        }
        if (!noteId(ids, kind, item, resolver, err)) return false;
        destination->InsertEndChild(item->DeepClone(&merged));
        return true;
    };
    for (const auto* child = root->FirstChildElement(); child != nullptr;
         child = child->NextSiblingElement()) {
        const std::string name = child->Name();
        if (name == "Robot") {
            have_robot = true;
            for (const auto* section = child->FirstChildElement(); section != nullptr;
                 section = section->NextSiblingElement()) {
                const std::string section_name = section->Name();
                if (section_name != "Resources" && section_name != "Sensors") {
                    err = resolver.where(section) + ": Robot has unknown element " + section_name;
                    return false;
                }
                for (const auto* item = section->FirstChildElement(); item != nullptr;
                     item = item->NextSiblingElement()) {
                    if (section_name == "Resources") {
                        if (!copyItem(item, "Resource", resources, resource_ids)) return false;
                    } else if (!copyItem(item, "Sensor", sensors, sensor_ids)) return false;
                }
            }
        } else if (name == "Field") {
            for (const auto* item = child->FirstChildElement(); item != nullptr;
                 item = item->NextSiblingElement()) {
                if (!copyItem(item, "Resource", resources, resource_ids)) return false;
            }
        } else if (name == "ConfigurationName" || name == "Loop" ||
                   name == "Inspection" || name == "Pipeline") {
            for (const auto& previous : singular) {
                if (previous == name) {
                    err = resolver.where(child) + ": more than one " + name;
                    return false;
                }
            }
            singular.push_back(name);
            if (name == "ConfigurationName") {
                if (child->GetText() != nullptr) output.name = child->GetText();
            } else {
                system->InsertEndChild(child->DeepClone(&merged));
                if (name == "Pipeline") have_pipeline = true;
            }
        } else {
            err = resolver.where(child) + ": Configuration has unknown element " + name;
            return false;
        }
    }
    if (!have_robot || !have_pipeline) {
        err = resolver.where(root) + ": Configuration needs at least one Robot and one Pipeline";
        return false;
    }
    return true;
}

} // namespace

uint64_t contentDigest(uint64_t seed, const std::string& bytes) {
    uint64_t hash = seed == 0 ? 1469598103934665603ull : seed;
    for (const char byte : bytes) {
        hash ^= static_cast<unsigned char>(byte);
        hash *= 1099511628211ull;
    }
    return hash;
}

bool requireResolvedSystem(const tinyxml2::XMLElement& root, std::string& err) {
    return rejectUnresolvedIncludes(root, Section::kSystem, "/System", err);
}

bool resolveConfiguration(const std::string& path, ResolvedConfiguration& out,
                          std::string& err) {
    out = ResolvedConfiguration{};
    err.clear();
    ResolvedConfiguration candidate;
    Resolver resolver(candidate);
    tinyxml2::XMLDocument document;
    if (!resolver.resolve(path, document, err)) return false;
    const auto* root = document.RootElement();
    if (!rejectTokens(root, resolver, err)) return false;
    tinyxml2::XMLDocument merged;
    if (std::string(root->Name()) == "Configuration") {
        if (!composeProfile(root, resolver, merged, candidate, err)) return false;
    } else {
        candidate.id = path; // preserve plain-System identity
        merged.InsertEndChild(root->DeepClone(&merged));
    }
    tinyxml2::XMLPrinter printer;
    merged.Print(&printer);
    candidate.xml = printer.CStr();
    out = std::move(candidate);
    return true;
}

} // namespace navigatr
