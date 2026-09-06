// composition.cpp

#include "config/composition.h"

#include <fstream>
#include <sstream>
#include <utility>
#include <vector>

#include "tinyxml2/tinyxml2.h"

namespace navigatr
{

namespace
{

bool readFileBytes(const std::string& path, std::string& out, std::string& err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        err = "cannot open " + path;
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

std::string dirName(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

// Lexical normalization so the same file included through different
// relative spellings is recognized as a repeat.
std::string normalizePath(const std::string& raw) {
    std::string s = raw;
    for (auto& c : s) {
        if (c == '\\') {
            c = '/';
        }
    }
    std::vector<std::string> parts;
    std::string              piece;
    std::istringstream       stream(s);
    const bool               absolute = !s.empty() && s[0] == '/';
    while (std::getline(stream, piece, '/')) {
        if (piece.empty() || piece == ".") {
            continue;
        }
        if (piece == ".." && !parts.empty() && parts.back() != "..") {
            parts.pop_back();
            continue;
        }
        parts.push_back(piece);
    }
    std::string out = absolute ? "/" : "";
    for (std::size_t i = 0; i < parts.size(); ++i) {
        out += parts[i];
        if (i + 1 < parts.size()) {
            out += "/";
        }
    }
    return out;
}

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string chainText(const std::vector<std::string>& chain) {
    std::string out;
    for (const auto& f : chain) {
        if (!out.empty()) {
            out += " -> ";
        }
        out += f;
    }
    return out;
}

// A template placeholder anywhere in a fragment makes the whole profile
// non-runnable; report the exact element and attribute.
bool rejectTokens(const tinyxml2::XMLElement* e, const std::string& file,
                  std::string& err) {
    for (const auto* attr = e->FirstAttribute(); attr != nullptr; attr = attr->Next()) {
        const std::string value = attr->Value();
        const auto        first = value.find('@');
        if (first != std::string::npos && value.find('@', first + 1) != std::string::npos) {
            err = file + ":" + std::to_string(e->GetLineNum()) + ": element " +
                  e->Name() + " attribute " + attr->Name() +
                  " still holds template placeholder \"" + value +
                  "\"; measure and configure the value";
            return false;
        }
    }
    for (const auto* c = e->FirstChildElement(); c != nullptr;
         c            = c->NextSiblingElement()) {
        if (!rejectTokens(c, file, err)) {
            return false;
        }
    }
    return true;
}

struct LoadedFragment {
    tinyxml2::XMLDocument doc;
    std::string           path;   // as given, for messages
};

bool loadFragment(const std::string& path, const std::vector<std::string>& chain,
                  std::vector<std::string>& visited, LoadedFragment& out,
                  uint64_t& digest, std::vector<std::string>& files, std::string& err) {
    const auto normalized = normalizePath(path);
    for (const auto& seen : visited) {
        if (seen == normalized) {
            err = "file " + path + " is included more than once (include chain: " +
                  chainText(chain) + ")";
            return false;
        }
    }
    if (endsWith(normalized, ".xml.in")) {
        err = "file " + path + " is an intentionally non-runnable template "
              "(include chain: " + chainText(chain) + ")";
        return false;
    }
    std::string bytes;
    if (!readFileBytes(path, bytes, err)) {
        err += " (include chain: " + chainText(chain) + ")";
        return false;
    }
    if (out.doc.Parse(bytes.c_str()) != tinyxml2::XML_SUCCESS) {
        err = path + ":" + std::to_string(out.doc.ErrorLineNum()) +
              ": cannot parse xml: " + out.doc.ErrorStr();
        return false;
    }
    visited.push_back(normalized);
    files.push_back(normalized);
    digest = contentDigest(digest, normalized);
    digest = contentDigest(digest, bytes);
    out.path = path;
    return true;
}

// Registers an id under a section, failing on a cross-file collision with
// both files named.
bool noteId(std::vector<std::pair<std::string, std::string>>& seen,
            const std::string& kind, const std::string& id, const std::string& file,
            std::string& err) {
    for (const auto& entry : seen) {
        if (entry.first == id) {
            err = kind + " id " + id + " is declared in both " + entry.second +
                  " and " + file;
            return false;
        }
    }
    seen.emplace_back(id, file);
    return true;
}

} // namespace

uint64_t contentDigest(uint64_t seed, const std::string& bytes) {
    uint64_t h = seed == 0 ? 1469598103934665603ull : seed;
    for (const char c : bytes) {
        h ^= static_cast<unsigned char>(c);
        h *= 1099511628211ull;
    }
    return h;
}

bool isComposedConfiguration(const std::string& path, bool& composed, std::string& err) {
    std::string bytes;
    if (!readFileBytes(path, bytes, err)) {
        return false;
    }
    tinyxml2::XMLDocument doc;
    if (doc.Parse(bytes.c_str()) != tinyxml2::XML_SUCCESS) {
        err = path + ":" + std::to_string(doc.ErrorLineNum()) +
              ": cannot parse xml: " + doc.ErrorStr();
        return false;
    }
    const auto* root = doc.RootElement();
    if (root == nullptr) {
        err = path + ": empty document";
        return false;
    }
    composed = std::string(root->Name()) == "Configuration";
    return true;
}

bool resolveConfiguration(const std::string& path, ResolvedConfiguration& out,
                          std::string& err) {
    std::vector<std::string> visited;
    std::vector<std::string> chain{path};

    LoadedFragment config;
    if (!loadFragment(path, chain, visited, config, out.digest, out.files, err)) {
        return false;
    }
    const auto* root = config.doc.RootElement();
    if (root == nullptr || std::string(root->Name()) != "Configuration") {
        err = path + ": root element must be Configuration";
        return false;
    }
    if (!rejectTokens(root, path, err)) {
        return false;
    }
    const char* id = root->Attribute("id");
    if (id == nullptr || std::string(id).empty()) {
        err = path + ":" + std::to_string(root->GetLineNum()) +
              ": Configuration needs id";
        return false;
    }
    out.id = id;

    // Strict framework-owned schema for the profile document itself.
    const tinyxml2::XMLElement* name_e     = nullptr;
    const tinyxml2::XMLElement* loop_e     = nullptr;
    const tinyxml2::XMLElement* pipeline_e = nullptr;
    std::vector<const tinyxml2::XMLElement*> robot_es;
    std::vector<const tinyxml2::XMLElement*> field_es;
    for (const auto* c = root->FirstChildElement(); c != nullptr;
         c            = c->NextSiblingElement()) {
        const std::string name = c->Name();
        const auto        once = [&](const tinyxml2::XMLElement*& slot) {
            if (slot != nullptr) {
                err = path + ":" + std::to_string(c->GetLineNum()) +
                      ": more than one " + name;
                return false;
            }
            slot = c;
            return true;
        };
        if (name == "ConfigurationName") {
            if (!once(name_e)) {
                return false;
            }
        } else if (name == "Loop") {
            if (!once(loop_e)) {
                return false;
            }
        } else if (name == "Robot") {
            robot_es.push_back(c);
        } else if (name == "Pipeline") {
            if (!once(pipeline_e)) {
                return false;
            }
        } else if (name == "Field") {
            field_es.push_back(c);
        } else {
            err = path + ":" + std::to_string(c->GetLineNum()) +
                  ": Configuration has unknown element " + name;
            return false;
        }
    }
    if (robot_es.empty() || pipeline_e == nullptr) {
        err = path + ": Configuration needs at least one Robot and one Pipeline "
              "reference";
        return false;
    }
    if (name_e != nullptr && name_e->GetText() != nullptr) {
        out.name = name_e->GetText();
    }

    const auto fragmentPath = [&](const tinyxml2::XMLElement* e,
                                  std::string& resolved) {
        const char* file = e->Attribute("file");
        if (file == nullptr || std::string(file).empty()) {
            err = path + ":" + std::to_string(e->GetLineNum()) + ": " + e->Name() +
                  " needs file";
            return false;
        }
        resolved = dirName(path) + "/" + file;
        return true;
    };

    tinyxml2::XMLDocument merged;
    auto* system = merged.NewElement("System");
    merged.InsertEndChild(system);
    if (loop_e != nullptr) {
        system->InsertEndChild(loop_e->DeepClone(&merged));
    }
    auto* resources = merged.NewElement("Resources");
    system->InsertEndChild(resources);
    auto* sensors = merged.NewElement("Sensors");
    system->InsertEndChild(sensors);

    std::vector<std::pair<std::string, std::string>> resource_ids;
    std::vector<std::pair<std::string, std::string>> sensor_ids;

    const auto cloneResource = [&](const tinyxml2::XMLElement* r,
                                   const std::string& file) {
        const char* rid = r->Attribute("id");
        if (rid == nullptr || std::string(rid).empty()) {
            err = file + ":" + std::to_string(r->GetLineNum()) + ": Resource needs id";
            return false;
        }
        if (!noteId(resource_ids, "Resource", rid, file, err)) {
            return false;
        }
        resources->InsertEndChild(r->DeepClone(&merged));
        return true;
    };

    // Robot fragments: Resources and Sensors. A robot description may be
    // split (base hardware plus optional add-ons such as a camera stack);
    // ids must still be unique across every fragment.
    for (const auto* robot_e : robot_es) {
        std::string robot_path;
        if (!fragmentPath(robot_e, robot_path)) {
            return false;
        }
        chain.push_back(robot_path);
        LoadedFragment robot;
        if (!loadFragment(robot_path, chain, visited, robot, out.digest, out.files,
                          err)) {
            return false;
        }
        const auto* rroot = robot.doc.RootElement();
        if (rroot == nullptr || std::string(rroot->Name()) != "Robot") {
            err = robot_path + ": root element must be Robot";
            return false;
        }
        if (!rejectTokens(rroot, robot_path, err)) {
            return false;
        }
        for (const auto* c = rroot->FirstChildElement(); c != nullptr;
             c            = c->NextSiblingElement()) {
            const std::string name = c->Name();
            if (name == "Resources") {
                for (const auto* r = c->FirstChildElement(); r != nullptr;
                     r            = r->NextSiblingElement()) {
                    if (std::string(r->Name()) != "Resource") {
                        err = robot_path + ":" + std::to_string(r->GetLineNum()) +
                              ": Resources has unknown element " + r->Name();
                        return false;
                    }
                    if (!cloneResource(r, robot_path)) {
                        return false;
                    }
                }
            } else if (name == "Sensors") {
                for (const auto* sn = c->FirstChildElement(); sn != nullptr;
                     sn            = sn->NextSiblingElement()) {
                    if (std::string(sn->Name()) != "Sensor") {
                        err = robot_path + ":" + std::to_string(sn->GetLineNum()) +
                              ": Sensors has unknown element " + sn->Name();
                        return false;
                    }
                    const char* sid = sn->Attribute("id");
                    if (sid == nullptr || std::string(sid).empty()) {
                        err = robot_path + ":" + std::to_string(sn->GetLineNum()) +
                              ": Sensor needs id";
                        return false;
                    }
                    if (!noteId(sensor_ids, "Sensor", sid, robot_path, err)) {
                        return false;
                    }
                    sensors->InsertEndChild(sn->DeepClone(&merged));
                }
            } else {
                err = robot_path + ":" + std::to_string(c->GetLineNum()) +
                      ": Robot has unknown element " + name;
                return false;
            }
        }
        chain.pop_back();
    }

    // Field fragments: a single Resource root or a Field of Resources.
    for (const auto* field_ref : field_es) {
        std::string field_path;
        if (!fragmentPath(field_ref, field_path)) {
            return false;
        }
        chain.push_back(field_path);
        LoadedFragment field;
        if (!loadFragment(field_path, chain, visited, field, out.digest, out.files,
                          err)) {
            return false;
        }
        const auto* froot = field.doc.RootElement();
        if (froot == nullptr) {
            err = field_path + ": empty document";
            return false;
        }
        if (!rejectTokens(froot, field_path, err)) {
            return false;
        }
        const std::string fname = froot->Name();
        if (fname == "Resource") {
            if (!cloneResource(froot, field_path)) {
                return false;
            }
        } else if (fname == "Field") {
            for (const auto* r = froot->FirstChildElement(); r != nullptr;
                 r            = r->NextSiblingElement()) {
                if (std::string(r->Name()) != "Resource") {
                    err = field_path + ":" + std::to_string(r->GetLineNum()) +
                          ": Field has unknown element " + r->Name();
                    return false;
                }
                if (!cloneResource(r, field_path)) {
                    return false;
                }
            }
        } else {
            err = field_path + ": root element must be Resource or Field";
            return false;
        }
        chain.pop_back();
    }

    // Pipeline fragment.
    {
        std::string pipeline_path;
        if (!fragmentPath(pipeline_e, pipeline_path)) {
            return false;
        }
        chain.push_back(pipeline_path);
        LoadedFragment pipeline;
        if (!loadFragment(pipeline_path, chain, visited, pipeline, out.digest,
                          out.files, err)) {
            return false;
        }
        const auto* proot = pipeline.doc.RootElement();
        if (proot == nullptr || std::string(proot->Name()) != "Pipeline") {
            err = pipeline_path + ": root element must be Pipeline";
            return false;
        }
        if (!rejectTokens(proot, pipeline_path, err)) {
            return false;
        }
        system->InsertEndChild(proot->DeepClone(&merged));
        chain.pop_back();
    }

    tinyxml2::XMLPrinter printer;
    merged.Print(&printer);
    out.xml = printer.CStr();
    return true;
}

} // namespace navigatr
