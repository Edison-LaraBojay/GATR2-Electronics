// e5_probe.cpp
// Step 5 build probe: System::buildFromString over mutations of the all-noop
// System, printing the exact error text, then a valid run's diagnostics
// labels and slot types. Built once against the pre-step-5 tree and library,
// once against the step-5 tree and library; the outputs must be identical.

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "core/system.h"
#include "pipeline/register_all.h"

using namespace navigatr;

namespace
{

const char* kAllNoop = R"(
<System>
    <Pipeline>
        <CommandCollection type="noop"/>
        <Localization><Estimator type="noop"/></Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="noop"/>
    </Pipeline>
</System>
)";

std::string replaced(std::string xml, const std::string& from, const std::string& to) {
    const auto pos = xml.find(from);
    if (pos == std::string::npos) {
        return "PROBE: pattern not found: " + from;
    }
    xml.replace(pos, from.size(), to);
    return xml;
}

struct Case {
    const char* name;
    std::string xml;
};

void report(const Case& c) {
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    std::unique_ptr<System> system = System::buildFromString(c.xml.c_str(), functions, err);
    if (system == nullptr) {
        std::printf("%-28s FAIL %s\n", c.name, err.c_str());
        return;
    }
    std::printf("%-28s OK commands=%s warnings=%zu\n", c.name, system->commandsType().c_str(),
                system->warnings().size());
    for (int i = 0; i < 3; ++i) {
        system->step(hostTime(10 * (i + 1)));
    }
    for (const auto& kv : system->diagnostics().functions) {
        std::printf("    est %-40s runs %u ok %u no_data %u fault %u\n", kv.first.c_str(),
                    kv.second.runs, kv.second.ok, kv.second.no_data, kv.second.fault);
    }
    for (const auto& kv : system->fieldDiagnostics().functions) {
        std::printf("    fld %-40s runs %u ok %u no_data %u fault %u\n", kv.first.c_str(),
                    kv.second.runs, kv.second.ok, kv.second.no_data, kv.second.fault);
    }
    system->reset();
    system->step(hostTime(100));
    std::printf("    after reset: cycle %llu resets %llu est_cycles %llu\n",
                static_cast<unsigned long long>(system->cycle()),
                static_cast<unsigned long long>(system->resetCount()),
                static_cast<unsigned long long>(system->diagnostics().cycles));
}

} // namespace

int main() {
    const std::string base = kAllNoop;
    const std::string cc   = "<CommandCollection type=\"noop\"/>";
    const std::string loc  = "<Localization><Estimator type=\"noop\"/></Localization>";
    const std::string we   = "<WorldEstimation><Estimator id=\"none\" type=\"noop\"/></WorldEstimation>";
    const std::string tr   = "<TargetResolution type=\"noop\"/>";
    const std::string pub  = "<Publishing type=\"noop\"/>";

    const std::vector<Case> cases = {
        {"valid", base},
        {"not_xml", "<System"},
        {"root_not_system", "<Nope/>"},
        {"root_unknown_child", replaced(base, "<Pipeline>", "<Bogus/><Pipeline>")},
        {"root_duplicate_loop",
         replaced(base, "<Pipeline>", "<Loop rate_hz=\"50\"/><Loop rate_hz=\"50\"/><Pipeline>")},
        {"loop_zero", replaced(base, "<Pipeline>", "<Loop rate_hz=\"0\"/><Pipeline>")},
        {"loop_text", replaced(base, "<Pipeline>", "<Loop rate_hz=\"abc\"/><Pipeline>")},
        {"loop_50", replaced(base, "<Pipeline>", "<Loop rate_hz=\"50\"/><Pipeline>")},
        {"inspection_bad", replaced(base, "<Pipeline>", "<Inspection enabled=\"maybe\"/><Pipeline>")},
        {"capture_bad", replaced(base, "<Pipeline>", "<Capture rolling_s=\"-1\"/><Pipeline>")},
        {"no_pipeline", "<System><Loop rate_hz=\"50\"/></System>"},
        {"pipeline_unknown", replaced(base, cc, cc + "<Perceptron/>")},
        {"pipeline_duplicate_cc", replaced(base, cc, cc + cc)},
        {"no_cc", replaced(base, cc, "")},
        {"cc_untyped", replaced(base, cc, "<CommandCollection/>")},
        {"cc_unknown_type", replaced(base, cc, "<CommandCollection type=\"quantum\"/>")},
        {"cc_wrong_signature", replaced(base, cc, "<CommandCollection type=\"planar_motion_integrator\"/>")},
        {"cc_brain_link_no_serial", replaced(base, cc, "<CommandCollection type=\"brain_link\"/>")},
        {"no_loc", replaced(base, loc, "")},
        {"no_loc_bad_cc", replaced(replaced(base, loc, ""), cc, "<CommandCollection type=\"quantum\"/>")},
        {"loc_unknown_est", replaced(base, loc, "<Localization><Estimator type=\"quantum\"/></Localization>")},
        {"no_we", replaced(base, we, "")},
        {"no_we_bad_tr", replaced(replaced(base, we, ""), tr, "<TargetResolution type=\"quantum\"/>")},
        {"we_two_estimators",
         replaced(base, we,
                  "<WorldEstimation><Estimator id=\"a\" type=\"noop\"/><Estimator id=\"b\" "
                  "type=\"noop\"/></WorldEstimation>")},
        {"no_tr", replaced(base, tr, "")},
        {"tr_untyped", replaced(base, tr, "<TargetResolution/>")},
        {"tr_unknown_type", replaced(base, tr, "<TargetResolution type=\"quantum\"/>")},
        {"no_pub", replaced(base, pub, "")},
        {"pub_untyped", replaced(base, pub, "<Publishing type=\"\"/>")},
        {"pub_unknown_type", replaced(base, pub, "<Publishing type=\"quantum\"/>")},
        {"pub_brain_link_no_serial", replaced(base, pub, "<Publishing type=\"brain_link\"/>")},
        {"tr_configured_no_targets", replaced(base, tr, "<TargetResolution type=\"configured_targets\"/>")},
    };
    for (const Case& c : cases) {
        report(c);
    }
    return 0;
}
