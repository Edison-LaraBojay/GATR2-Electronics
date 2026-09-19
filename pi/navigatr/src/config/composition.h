// composition.h
// Recursive configuration resolution for both <System> and composed
// <Configuration> documents. A framework section can contain inline XML or
// file="..." referencing a document with the same root. Relative paths are
// relative to the containing file; absolute paths are accepted.
//
//   <Configuration id="override_three_wheel_diagnostic">
//       <ConfigurationName>Override three-wheel diagnostic</ConfigurationName>
//       <Loop rate_hz="100"/>
//       <Inspection enabled="true" port="8765"/>          optional
//       <Robot file="../../shared/robots/gatr2_as5047_imu.xml"/>
//       <Field file="../field.xml"/>
//       <Pipeline file="../../shared/pipelines/three_wheel_imu_camera_diagnostic.xml"/>
//   </Configuration>
//
// Include positions: System/Robot Resources and Sensors; individual Resource
// and Sensor entries; Pipeline and its stage children; Localization and its
// Observation/Estimator/History/InitialPlacement children; a stage's nested
// Pipeline; System/Configuration Loop and Inspection; Configuration Robot,
// Field and Pipeline. Resource/sensor implementation options are opaque, so
// Device file="..." remains a device filename rather than an XML include.
//
// A reference contains only file, comments, and optionally an uninterpreted
// calibration_status annotation. Configuration attrs/options belong in the
// referenced root; file plus inline content/overrides is rejected. The
// existing Configuration Field reference also accepts a single Resource
// root. Cycles, repeated includes, missing/wrong roots, .xml.in templates and
// unresolved value placeholders fail; calibration_status text is ignored.
// The digest covers every contributing file's canonical path and contents.

#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace tinyxml2 { class XMLElement; }

namespace navigatr
{

struct ResolvedConfiguration {
    std::string id;     // Configuration id, or the file name for a plain System
    std::string name;   // ConfigurationName text, may be empty
    uint64_t    digest = 0;
    std::vector<std::string> files;   // every contributing file, resolution order
    std::string xml;                  // the merged <System> document
};

// Resolves a System or Configuration file into one merged System document.
// On failure out is cleared and err identifies the invalid reference/schema.
bool resolveConfiguration(const std::string& path, ResolvedConfiguration& out,
                          std::string& err);

// String-based construction has no containing-file base. Reject unresolved
// references at the same framework-owned positions used by the resolver;
// implementation-owned file options remain opaque.
bool requireResolvedSystem(const tinyxml2::XMLElement& root, std::string& err);

// FNV-1a 64 content digest helper.
uint64_t contentDigest(uint64_t seed, const std::string& bytes);

} // namespace navigatr
