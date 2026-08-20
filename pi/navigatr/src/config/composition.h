// composition.h
// Multi-file configuration resolution. A profile is one <Configuration>
// document naming its parts by file; paths resolve relative to the
// referencing file, every fragment root is validated, and the result is one
// merged <System> document plus a stable identity:
//
//   <Configuration id="override_three_wheel_diagnostic">
//       <ConfigurationName>Override three-wheel diagnostic</ConfigurationName>
//       <Loop rate_hz="100"/>
//       <Robot file="../../shared/robots/gatr2_as5047_bno08x.xml"/>
//       <Field file="../field.xml"/>
//       <Pipeline file="../../shared/pipelines/three_wheel_bno08x_camera_diagnostic.xml"/>
//   </Configuration>
//
// Fragment roots: Robot (Resources + Sensors), Field (a <Resource> or a
// <Field> of Resources), Pipeline (a <Pipeline>). Missing files, repeated
// includes, template files (.xml.in or any @TOKEN@ placeholder), and
// cross-file duplicate resource or sensor ids fail with the include chain;
// nothing merges silently. The resolved configuration carries the declared
// id and an FNV-1a digest over every contributing file, so the running
// profile is identifiable exactly.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace navigatr
{

struct ResolvedConfiguration {
    std::string id;     // Configuration id, or the file name for a plain System
    std::string name;   // ConfigurationName text, may be empty
    uint64_t    digest = 0;
    std::vector<std::string> files;   // every contributing file, resolution order
    std::string xml;                  // the merged <System> document
};

// True when the file's root element is <Configuration> (a composed profile)
// rather than a plain <System>. False with err when the file cannot be read
// or parsed at all.
bool isComposedConfiguration(const std::string& path, bool& composed, std::string& err);

// Resolves a <Configuration> file into one merged System document. False
// and err (with file, line, and include chain) on any violation.
bool resolveConfiguration(const std::string& path, ResolvedConfiguration& out,
                          std::string& err);

// FNV-1a 64 content digest helper, exposed for plain System files.
uint64_t contentDigest(uint64_t seed, const std::string& bytes);

} // namespace navigatr
