// field_documents_gtest.cpp
// The brain link field documents: the map document built from a field
// definition (layout, flags, wire rounding, identity, errors), the estimate
// records (nominal, observed under the current anchor, epoch mismatch), the
// snapshot rule (content change plus period, three retained), and chunked
// READ_DOC over a memory link: the Override field, a field with another
// object count from inline XML, Stale after replacement, refusals, the
// publisher's Field configuration, and a whole Pi following its anchor.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "translaGATR/frame_codec.h"
#include "translaGATR/link_documents.h"
#include "impl/publishing/field_documents.h"
#include "impl/resources/serial_links.h"
#include "math/angles.h"
#include "runtime/register_all.h"
#include "runtime/sensor_catalog.h"
#include "runtime/system.h"
#include "tinyxml2/tinyxml2.h"

using namespace navigatr;

namespace
{

const std::string kConfigDir = NAVIGATR_CONFIG_DIR;
const std::string kOverride  = kConfigDir + "/override/field.xml";

bool parseInline(const std::string& xml, FieldMap& map, std::string& err) {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS) {
        err = doc.ErrorStr();
        return false;
    }
    return parseFieldMap(ConfigNode{doc.RootElement()}, map, err);
}

FieldMapDocument overrideDocument() {
    FieldMap         map;
    FieldMapDocument doc;
    std::string      err;
    EXPECT_TRUE(loadFieldMapFile(kOverride, map, err)) << err;
    EXPECT_TRUE(buildFieldMapDocument(map, doc, err)) << err;
    return doc;
}

// landmarks goal_0.. with ids 1.., obstacles block_0.. with ids 1000.., all
// with boxes, on a 3.6 m field
std::string fieldXml(int landmarks, int obstacles, const std::string& id = "field") {
    std::string xml = "<Resource id=\"" + id + "\" type=\"field_map\" revision=\"3\">\n"
                      "  <Boundary min_x_m=\"0\" min_y_m=\"0\" max_x_m=\"3.6\" max_y_m=\"3.6\"/>\n";
    for (int i = 0; i < landmarks; ++i) {
        const double x = 0.2 + 0.08 * (i % 40);
        const double y = 0.3 + 0.5 * (i / 40);
        xml += "  <Landmark id=\"goal_" + std::to_string(i) + "\" wire_id=\"" +
               std::to_string(i + 1) + "\">\n    <NominalPose x_m=\"" + std::to_string(x) +
               "\" y_m=\"" + std::to_string(y) + "\" heading_deg=\"" + std::to_string(i % 7 * 10) +
               "\"/>\n    <CollisionBox x_m=\"0\" y_m=\"0\" size_x_m=\"0.05\" size_y_m=\"0.05\"/>\n"
               "  </Landmark>\n";
    }
    for (int j = 0; j < obstacles; ++j) {
        xml += "  <Obstacle id=\"block_" + std::to_string(j) + "\" wire_id=\"" +
               std::to_string(1000 + j) + "\" x_m=\"" + std::to_string(0.3 + 0.2 * j) +
               "\" y_m=\"3.0\" heading_deg=\"30\">\n    <CollisionBox x_m=\"0.01\" y_m=\"-0.02\" "
               "size_x_m=\"0.1\" size_y_m=\"0.04\" yaw_deg=\"15\"/>\n  </Obstacle>\n";
    }
    return xml + "</Resource>";
}

FieldObjectState observedAt(const Pose2D& T_odom, uint64_t epoch, int64_t seen_ms) {
    FieldObjectState s;
    s.valid          = true;
    s.source         = EstimateSource::kObserved;
    s.T_odom_object  = T_odom;
    s.odometry_epoch = epoch;
    s.lastObservedAt = hostTime(seen_ms);
    return s;
}

translagatr::FieldEstimateRecord recordFor(const std::vector<translagatr::FieldEstimateRecord>& records,
                                     uint16_t id) {
    for (const translagatr::FieldEstimateRecord& r : records) {
        if (r.object_id == id) {
            return r;
        }
    }
    ADD_FAILURE() << "no record " << id;
    return {};
}

// The newest estimate through FieldDocuments::read, chunk by chunk.
std::vector<uint8_t> wholeEstimate(const FieldDocuments& docs) {
    std::vector<uint8_t> bytes;
    translagatr::BrainReply    reply;
    for (uint16_t offset = 0;;) {
        if (docs.read(translagatr::kDocFieldEstimate, docs.estimateId(), offset, translagatr::kDocChunkMax,
                      reply) != translagatr::kResultOk) {
            ADD_FAILURE() << "estimate read refused at " << offset;
            return bytes;
        }
        bytes.insert(bytes.end(), reply.data, reply.data + reply.data_len);
        offset = static_cast<uint16_t>(offset + reply.data_len);
        if (offset >= reply.doc_total_len) {
            return bytes;
        }
    }
}

// ---- wire --------------------------------------------------------------------

std::vector<uint8_t> requestBytes(const translagatr::BrainRequest& r) {
    std::vector<uint8_t> buf(translagatr::kMaxFrameLen);
    buf.resize(translagatr::encodeBrainRequest(r, buf.data(), translagatr::kMaxFrameLen));
    EXPECT_FALSE(buf.empty());
    return buf;
}

translagatr::BrainRequest request(uint8_t op, uint32_t session, uint16_t rid) {
    translagatr::BrainRequest r;
    r.op         = op;
    r.session    = session;
    r.request_id = rid;
    return r;
}

translagatr::BrainRequest readRequest(uint32_t session, uint16_t rid, uint8_t kind, uint32_t doc_id,
                                uint16_t offset, uint8_t max_len = translagatr::kDocChunkMax) {
    translagatr::BrainRequest r = request(translagatr::kOpReadDoc, session, rid);
    r.doc_kind            = kind;
    r.doc_id              = doc_id;
    r.doc_offset          = offset;
    r.max_len             = max_len;
    return r;
}

std::vector<translagatr::BrainReply> takeReplies(MemoryLink& link) {
    std::vector<translagatr::BrainReply> out;
    translagatr::FrameReader             reader;
    for (uint8_t b : link.output().takeAll()) {
        if (!reader.push(b)) {
            continue;
        }
        do {
            translagatr::BrainReply reply;
            EXPECT_TRUE(translagatr::decodeBrainReply(reader.frame(), reader.frameLen(), reply));
            out.push_back(reply);
        } while (reader.next());
    }
    return out;
}

using Exchange = std::function<translagatr::BrainReply(const translagatr::BrainRequest&)>;

struct Assembled {
    uint8_t              result = translagatr::kResultOk;   // first refusal
    uint32_t             id     = 0;
    uint32_t             crc    = 0;
    int                  chunks = 0;
    std::vector<uint8_t> bytes;
};

// The whole document in kDocChunkMax chunks, pinned to the id of the first.
Assembled readWhole(const Exchange& one, uint32_t session, uint16_t& rid, uint8_t kind,
                    uint32_t doc_id = 0) {
    Assembled a;
    uint16_t  total  = 0;
    uint16_t  offset = 0;
    do {
        const translagatr::BrainReply reply =
            one(readRequest(session, rid++, kind, a.chunks == 0 ? doc_id : a.id, offset));
        if (reply.result != translagatr::kResultOk) {
            a.result = reply.result;
            return a;
        }
        if (a.chunks == 0) {
            a.id  = reply.doc_id;
            a.crc = reply.doc_crc32;
            total = reply.doc_total_len;
        }
        EXPECT_EQ(reply.doc_kind, kind);
        EXPECT_EQ(reply.doc_id, a.id);
        EXPECT_EQ(reply.doc_crc32, a.crc);
        EXPECT_EQ(reply.doc_total_len, total);
        EXPECT_EQ(reply.doc_offset, offset);
        EXPECT_GT(reply.data_len, 0u);
        a.bytes.insert(a.bytes.end(), reply.data, reply.data + reply.data_len);
        offset = static_cast<uint16_t>(offset + reply.data_len);
        ++a.chunks;
        if (reply.data_len == 0) {
            break;
        }
    } while (offset < total);
    EXPECT_EQ(translagatr::crc32(a.bytes.data(), static_cast<uint32_t>(a.bytes.size())), a.crc);
    return a;
}

// ---- slot harness ----------------------------------------------------------

// brain_link commands and publishing over one memory link, with a field_map
// resource and hand-built robot and field state.
struct DocHarness {
    FunctionRegistry          functions;
    std::vector<std::string>  warnings;
    tinyxml2::XMLDocument     resources_doc;
    tinyxml2::XMLDocument     field_doc;
    tinyxml2::XMLDocument     doc;
    ResourceStore             store;
    SensorCatalog             catalog;
    SlotInitializationContext context;
    MemoryLink*               brain    = nullptr;
    int64_t                   clock_us = 1000000;
    int64_t                   now_ms   = 1000;
    std::string               build_error;

    std::unique_ptr<Commands>   commands;
    std::unique_ptr<Publishing> publisher;

    SensorMap          results;
    LocalizationStatus localization;
    ObservationMap     observations;
    AssociationMap     associations;
    RobotState         robot;
    FieldState         field;
    CommandState       command;
    TargetState        target;
    Diagnostics        diagnostics;

    uint32_t session = 0;
    uint16_t rid     = 1;

    // field_xml is a field_map Resource element, or a file path with from_file
    DocHarness(const std::string& field_xml, const std::string& field_element,
               bool from_file = false) {
        register_resources(functions);
        register_commands(functions);
        register_publishers(functions);

        EXPECT_EQ(resources_doc.Parse(
                      R"(<Resources><Resource id="brain_uart" type="memory_link"/></Resources>)"),
                  tinyxml2::XML_SUCCESS);
        const tinyxml2::XMLError loaded = from_file ? field_doc.LoadFile(field_xml.c_str())
                                                    : field_doc.Parse(field_xml.c_str());
        EXPECT_EQ(loaded, tinyxml2::XML_SUCCESS);
        ResourceStoreBuilder builder(functions, &warnings);
        std::string          err;
        bool                 ok = builder.index(ConfigNode{field_doc.RootElement()}, err);
        ConfigNode{resources_doc.RootElement()}.forEach("Resource", [&](const ConfigNode& r) {
            if (ok) {
                ok = builder.index(r, err);
            }
        });
        EXPECT_TRUE(ok && builder.buildAll(err)) << err;
        store = builder.take();

        auto link = store.require<SerialLink>(ResourceId{"brain_uart"}, err);
        brain     = dynamic_cast<MemoryLink*>(link.get());
        EXPECT_NE(brain, nullptr);
        brain->setClock([this] { return clock_us; });

        context.resources       = &store;
        context.sensors         = &catalog;
        context.functions       = &functions;
        context.commands_type   = FunctionKey{"brain_link"};
        context.commands_serial = ResourceId{"brain_uart"};

        commands = make<CommandsMakeFunction>(
            R"(<CommandCollection type="brain_link"><Serial resource_id="brain_uart"/></CommandCollection>)",
            "brain_link", err);
        EXPECT_NE(commands, nullptr) << err;
        const std::string publishing =
            R"(<Publishing type="brain_link"><Serial resource_id="brain_uart"/>)" + field_element +
            "</Publishing>";
        publisher = make<PublishingMakeFunction>(publishing.c_str(), "brain_link", build_error);
        if (publisher != nullptr) {
            cycle();   // the first drain never replies
        }
    }

    template <typename MakeFunction>
    typename MakeFunction::result_type make(const char* xml, const char* key,
                                           std::string& err) {
        doc.Clear();
        EXPECT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);
        const MakeFunction* factory = functions.find<MakeFunction>(FunctionKey{key}, err);
        EXPECT_NE(factory, nullptr) << err;
        return (*factory)(ConfigNode{doc.RootElement()}, context, err);
    }

    std::vector<translagatr::BrainReply> cycle() {
        clock_us += 5000;
        now_ms += 5;
        command = commands->run({command, hostTime(now_ms), &diagnostics}).command;
        publisher->run({results, observations, associations, robot, localization, field,
                        command, target, hostTime(now_ms), &diagnostics});
        return takeReplies(*brain);
    }

    translagatr::BrainReply one(const translagatr::BrainRequest& r) {
        brain->input().feed(requestBytes(r));
        const std::vector<translagatr::BrainReply> replies = cycle();
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? translagatr::BrainReply{} : replies.front();
    }

    void open() {
        translagatr::BrainRequest r = request(translagatr::kOpHello, 0, rid++);
        r.nonce               = 0x5EED;
        const translagatr::BrainReply reply = one(r);
        EXPECT_EQ(reply.result, translagatr::kResultOk);
        session = reply.session;
    }

    translagatr::BrainState state() {
        const translagatr::BrainReply reply = one(request(translagatr::kOpGetState, session, rid++));
        EXPECT_EQ(reply.result, translagatr::kResultOk);
        return reply.state;
    }

    Assembled read(uint8_t kind, uint32_t doc_id = 0) {
        return readWhole([this](const translagatr::BrainRequest& r) { return one(r); }, session, rid,
                         kind, doc_id);
    }
};

const char* kField = R"(<Field resource_id="field" estimate_period_ms="200"/>)";

} // namespace

// ---- map document ------------------------------------------------------------

TEST(FieldMapDocument, OverrideFieldEncodesValidatesAndNamesItselfByCrc) {
    const FieldMapDocument doc = overrideDocument();
    const uint16_t         len = static_cast<uint16_t>(doc.bytes.size());
    EXPECT_EQ(len, translagatr::fieldMapLen(17));   // 9 goals, 4 loaders, 4 toggles
    EXPECT_EQ(translagatr::validateFieldMap(doc.bytes.data(), len), translagatr::DocError::kNone);
    EXPECT_EQ(doc.map_id, translagatr::crc32(doc.bytes.data(), len));
    EXPECT_NE(doc.map_id, 0u);
    EXPECT_EQ(doc.revision, 1u);

    translagatr::FieldMapHeader header;
    ASSERT_TRUE(translagatr::decodeFieldMapHeader(doc.bytes.data(), len, header));
    EXPECT_EQ(header.revision, 1u);
    EXPECT_EQ(header.object_count, 17u);
    EXPECT_EQ(header.min_x_mm, 0);
    EXPECT_EQ(header.min_y_mm, 0);
    EXPECT_EQ(header.max_x_mm, 3566);   // 3566.4 rounds inward
    EXPECT_EQ(header.max_y_mm, 3566);

    FieldMap    map;
    std::string err;
    ASSERT_TRUE(loadFieldMapFile(kOverride, map, err)) << err;
    ASSERT_EQ(doc.objects.size(), 17u);
    uint16_t previous = 0;
    for (uint16_t i = 0; i < header.object_count; ++i) {
        translagatr::FieldObjectRecord r;
        ASSERT_TRUE(translagatr::decodeFieldObjectRecord(doc.bytes.data(), len, i, r));
        const FieldMapDocument::Object& o = doc.objects[i];
        EXPECT_GT(r.object_id, previous);
        previous = r.object_id;
        EXPECT_EQ(r.object_id, o.record.object_id);
        if (o.landmark) {
            const LandmarkDecl* l = map.find(FieldObjectId{o.name});
            ASSERT_NE(l, nullptr) << o.name;
            EXPECT_EQ(r.object_id, l->wire_id);
            EXPECT_EQ(r.kind, translagatr::kObjectLandmark);
            EXPECT_EQ(r.flags,
                      translagatr::kObjectEstimated | translagatr::kObjectReference | translagatr::kObjectObstacle);
            EXPECT_LE(std::fabs(r.x_mm - l->nominal.x_m * 1000.0), 0.5) << o.name;
            EXPECT_LE(std::fabs(r.y_mm - l->nominal.y_m * 1000.0), 0.5) << o.name;
            EXPECT_EQ(r.heading_cdeg, 0);
            EXPECT_EQ(r.box_length_mm, 155u);   // 154.3 mm rounded up
            EXPECT_EQ(r.box_width_mm, 155u);
            EXPECT_EQ(r.box_x_mm, 0);
            EXPECT_EQ(r.box_y_mm, 0);
        } else {
            const ObstacleDecl* ob = map.findObstacle(o.name);
            ASSERT_NE(ob, nullptr) << o.name;
            EXPECT_EQ(r.object_id, ob->wire_id);
            EXPECT_EQ(r.kind, translagatr::kObjectFixed);
            EXPECT_EQ(r.flags, translagatr::kObjectObstacle);
            EXPECT_LE(std::fabs(r.x_mm - ob->pose.x_m * 1000.0), 0.5) << o.name;
            EXPECT_LE(std::fabs(r.y_mm - ob->pose.y_m * 1000.0), 0.5) << o.name;
            // never smaller than declared, at most 1 mm larger
            EXPECT_GE(r.box_length_mm, ob->box.size_x_m * 1000.0 - 1e-6) << o.name;
            EXPECT_LT(r.box_length_mm, ob->box.size_x_m * 1000.0 + 1.0) << o.name;
            EXPECT_GE(r.box_width_mm, ob->box.size_y_m * 1000.0 - 1e-6) << o.name;
            EXPECT_LT(r.box_width_mm, ob->box.size_y_m * 1000.0 + 1.0) << o.name;
        }
    }
    // loader 95 x 102.1 mm, toggle 660.2 x 52 mm
    EXPECT_EQ(doc.objects[9].record.object_id, 101u);
    EXPECT_EQ(doc.objects[9].record.box_length_mm, 95u);
    EXPECT_EQ(doc.objects[9].record.box_width_mm, 103u);
    EXPECT_EQ(doc.objects[13].record.object_id, 111u);
    EXPECT_EQ(doc.objects[13].record.box_length_mm, 661u);
    EXPECT_EQ(doc.objects[13].record.box_width_mm, 52u);
}

TEST(FieldMapDocument, AnyPlanningEditChangesTheMapId) {
    FieldMap         base;
    FieldMapDocument a;
    std::string      err;
    ASSERT_TRUE(parseInline(fieldXml(3, 1), base, err)) << err;
    ASSERT_TRUE(buildFieldMapDocument(base, a, err)) << err;

    FieldMap moved = base;
    moved.landmarks[1].nominal.x_m += 0.002;
    FieldMap bumped = base;
    bumped.revision = 4;
    FieldMap grown = base;
    grown.obstacles[0].box.size_x_m += 0.002;
    for (const FieldMap& edited : {moved, bumped, grown}) {
        FieldMapDocument b;
        ASSERT_TRUE(buildFieldMapDocument(edited, b, err)) << err;
        EXPECT_NE(a.map_id, b.map_id);
    }
    // display data is not in the document
    FieldMap renamed = base;
    renamed.name     = "another name";
    FieldMapDocument c;
    ASSERT_TRUE(buildFieldMapDocument(renamed, c, err)) << err;
    EXPECT_EQ(a.map_id, c.map_id);
}

TEST(FieldMapDocument, RotatedOffsetBoxesAndObjectsWithoutBoxes) {
    const char* xml = R"(
<Resource id="f" type="field_map" revision="7">
  <Boundary min_x_m="-0.5004" min_y_m="0.0006" max_x_m="2.0006" max_y_m="1.5"/>
  <Obstacle id="post" wire_id="40" x_m="1.2" y_m="0.8" heading_deg="-30">
    <CollisionBox x_m="0.0104" y_m="-0.0206" size_x_m="0.3" size_y_m="0.12" yaw_deg="195"/>
  </Obstacle>
  <Obstacle id="marker" wire_id="41" x_m="0.1" y_m="0.1" heading_deg="0"/>
  <Landmark id="beacon" wire_id="50">
    <NominalPose x_m="0.5" y_m="0.25" heading_deg="90"/>
  </Landmark>
</Resource>)";
    FieldMap         map;
    FieldMapDocument doc;
    std::string      err;
    ASSERT_TRUE(parseInline(xml, map, err)) << err;
    ASSERT_TRUE(buildFieldMapDocument(map, doc, err)) << err;
    ASSERT_EQ(doc.objects.size(), 3u);
    EXPECT_EQ(translagatr::validateFieldMap(doc.bytes.data(), static_cast<uint16_t>(doc.bytes.size())),
              translagatr::DocError::kNone);

    translagatr::FieldMapHeader header;
    ASSERT_TRUE(translagatr::decodeFieldMapHeader(doc.bytes.data(),
                                            static_cast<uint16_t>(doc.bytes.size()), header));
    EXPECT_EQ(header.min_x_mm, -500);   // inward: -500.4 -> -500
    EXPECT_EQ(header.min_y_mm, 1);      // 0.6 -> 1
    EXPECT_EQ(header.max_x_mm, 2000);   // 2000.6 -> 2000
    EXPECT_EQ(header.max_y_mm, 1500);

    // map order is by wire id, not declaration order
    EXPECT_EQ(doc.objects[0].name, "post");
    EXPECT_EQ(doc.objects[1].name, "marker");
    EXPECT_EQ(doc.objects[2].name, "beacon");

    // a landmark without a box is a reference, not an obstacle
    const translagatr::FieldObjectRecord& beacon = doc.objects[2].record;
    EXPECT_EQ(beacon.object_id, 50u);
    EXPECT_EQ(beacon.flags, translagatr::kObjectEstimated | translagatr::kObjectReference);
    EXPECT_EQ(beacon.heading_cdeg, 9000);
    EXPECT_EQ(beacon.box_length_mm, 0u);

    const translagatr::FieldObjectRecord& post = doc.objects[0].record;
    EXPECT_EQ(post.flags, translagatr::kObjectObstacle);
    EXPECT_EQ(post.heading_cdeg, -3000);
    EXPECT_EQ(post.box_x_mm, 10);
    EXPECT_EQ(post.box_y_mm, -21);
    EXPECT_EQ(post.box_heading_cdeg, -16500);   // 195 wraps to -165
    EXPECT_EQ(post.box_length_mm, 300u);
    EXPECT_EQ(post.box_width_mm, 120u);

    // a fixed element without a box is not an obstacle
    const translagatr::FieldObjectRecord& marker = doc.objects[1].record;
    EXPECT_EQ(marker.kind, translagatr::kObjectFixed);
    EXPECT_EQ(marker.flags, 0u);
    EXPECT_EQ(marker.box_width_mm, 0u);
}

TEST(FieldMapDocument, PublishingNeedsRevisionBoundaryWireIdsAndRoom) {
    const auto fails = [](const std::string& xml, const char* expect) {
        FieldMap         map;
        FieldMapDocument doc;
        std::string      err;
        ASSERT_TRUE(parseInline(xml, map, err)) << err;
        EXPECT_FALSE(buildFieldMapDocument(map, doc, err)) << expect;
        EXPECT_NE(err.find(expect), std::string::npos) << err;
        EXPECT_TRUE(doc.bytes.empty());
    };
    fails(R"(<Resource id="f" type="field_map">
               <Boundary min_x_m="0" min_y_m="0" max_x_m="1" max_y_m="1"/></Resource>)",
          "revision");
    fails(R"(<Resource id="f" type="field_map" revision="1"/>)", "Boundary");
    fails(R"(<Resource id="f" type="field_map" revision="1">
               <Boundary min_x_m="0" min_y_m="0" max_x_m="1" max_y_m="1"/>
               <Landmark id="g"><NominalPose x_m="0.5" y_m="0.5" heading_deg="0"/></Landmark>
             </Resource>)",
          "Landmark g needs a wire_id");
    fails(R"(<Resource id="f" type="field_map" revision="1">
               <Boundary min_x_m="0" min_y_m="0" max_x_m="1" max_y_m="1"/>
               <Obstacle id="far" wire_id="3" x_m="0" y_m="0" heading_deg="0">
                 <CollisionBox x_m="40" y_m="0" size_x_m="0.1" size_y_m="0.1"/></Obstacle>
             </Resource>)",
          "Obstacle far");
    fails(R"(<Resource id="f" type="field_map" revision="1">
               <Boundary min_x_m="0" min_y_m="0" max_x_m="0.0004" max_y_m="1"/></Resource>)",
          "Boundary");
    fails(fieldXml(120, 9), "129 objects");

    FieldMap         map;
    FieldMapDocument doc;
    std::string      err;
    ASSERT_TRUE(parseInline(fieldXml(120, 8), map, err)) << err;
    ASSERT_TRUE(buildFieldMapDocument(map, doc, err)) << err;   // exactly the cap
    EXPECT_EQ(doc.bytes.size(), translagatr::kFieldMapMaxLen);
}

TEST(FieldMapDocument, ReferenceHeaderNamesEveryLandmarkOnce) {
    FieldMap         map;
    FieldMapDocument doc;
    std::string      err;
    ASSERT_TRUE(parseInline(fieldXml(2, 1), map, err)) << err;
    ASSERT_TRUE(buildFieldMapDocument(map, doc, err)) << err;
    std::string header;
    ASSERT_TRUE(fieldReferencesHeader(doc, "some/field.xml", header, err)) << err;
    char id[16];
    std::snprintf(id, sizeof(id), "0x%08X", static_cast<unsigned>(doc.map_id));
    const std::string expected =
        "// Generated by navigatr_field_refs from some/field.xml. Do not edit.\n"
        "#pragma once\n"
        "#include \"investigatr/reference.h\"\n"
        "struct Field : investigatr::FieldReferences {\n"
        "    static constexpr investigatr::MapId kMapId    = " +
        std::string(id) +
        ";\n"
        "    static constexpr uint16_t           kRevision = 3;\n"
        "    static constexpr investigatr::Reference Goal0 = "
        "investigatr::Reference::object(1, kMapId);\n"
        "    static constexpr investigatr::Reference Goal1 = "
        "investigatr::Reference::object(2, kMapId);\n"
        "};\n";
    EXPECT_EQ(header, expected);   // obstacles are not references

    const auto refused = [&](const char* first, const char* second) {
        FieldMap         m;
        FieldMapDocument d;
        std::string      xml = R"(<Resource id="f" type="field_map" revision="1">
            <Boundary min_x_m="0" min_y_m="0" max_x_m="1" max_y_m="1"/>
            <Landmark id=")" + std::string(first) +
                          R"(" wire_id="1"><NominalPose x_m="0" y_m="0" heading_deg="0"/></Landmark>)";
        if (second != nullptr) {
            xml += R"(<Landmark id=")" + std::string(second) +
                   R"(" wire_id="2"><NominalPose x_m="0" y_m="0" heading_deg="0"/></Landmark>)";
        }
        xml += "</Resource>";
        std::string e;
        ASSERT_TRUE(parseInline(xml, m, e)) << e;
        ASSERT_TRUE(buildFieldMapDocument(m, d, e)) << e;
        std::string text;
        EXPECT_FALSE(fieldReferencesHeader(d, "x", text, e)) << first;
        EXPECT_NE(e.find("C++ name"), std::string::npos) << e;
    };
    refused("3d_goal", nullptr);      // starts with a digit
    refused("origin", nullptr);       // the base's Origin
    refused("goal_a", "goal__a");     // both GoalA
}

// ---- estimate records --------------------------------------------------------

TEST(FieldEstimate, NoopWorldEstimationGivesEveryObjectNominal) {
    const FieldMapDocument doc = overrideDocument();
    const std::vector<translagatr::FieldEstimateRecord> records =
        fieldEstimateRecords(doc, FieldState{}, RobotState{}, hostTime(5000));
    ASSERT_EQ(records.size(), doc.objects.size());
    for (std::size_t i = 0; i < records.size(); ++i) {
        const translagatr::FieldObjectRecord& o = doc.objects[i].record;
        EXPECT_EQ(records[i].object_id, o.object_id);
        EXPECT_EQ(records[i].source, translagatr::kEstimateSourceNominal);
        EXPECT_EQ(records[i].flags, translagatr::kEstimateValid);
        EXPECT_EQ(records[i].x_mm, o.x_mm);
        EXPECT_EQ(records[i].y_mm, o.y_mm);
        EXPECT_EQ(records[i].heading_cdeg, o.heading_cdeg);
        EXPECT_EQ(records[i].age_ms, 0u);
    }

    FieldDocuments docs(doc, 200);
    docs.update(FieldState{}, RobotState{}, hostTime(5000));
    ASSERT_EQ(docs.estimateId(), 1u);
    const std::vector<uint8_t> bytes = wholeEstimate(docs);
    EXPECT_EQ(bytes.size(), translagatr::fieldEstimateLen(17));
    EXPECT_EQ(translagatr::validateFieldEstimate(bytes.data(), static_cast<uint16_t>(bytes.size()),
                                           doc.bytes.data(),
                                           static_cast<uint16_t>(doc.bytes.size()), doc.map_id),
              translagatr::DocError::kNone);
}

TEST(FieldEstimate, ObservedOnlyInTheRobotsEpochComposedWithTheCurrentAnchor) {
    const FieldMapDocument doc = overrideDocument();
    RobotState             robot;
    robot.odometry_epoch  = 4;
    robot.anchor_revision = 2;
    robot.field_from_odom = Pose2D{0.5, -0.25, kPi / 2.0};

    FieldState field;
    const Pose2D T_odom{1.0, 2.0, 0.3};
    field.objects[FieldObjectId{"neutral_goal_0_center"}] = observedAt(T_odom, 4, 900);   // 5
    FieldObjectState seeded;   // map seeded: always the map nominal
    seeded.valid    = true;
    seeded.source   = EstimateSource::kFieldMap;
    seeded.pose     = FramedPose2D{};
    field.objects[FieldObjectId{"blue_goal_3_north"}] = seeded;   // 2
    FieldObjectState stale = observedAt(T_odom, 3, 900);           // other epoch
    field.objects[FieldObjectId{"red_goal_2_west"}] = stale;       // 6
    FieldObjectState invalid = observedAt(T_odom, 4, 900);
    invalid.valid            = false;
    field.objects[FieldObjectId{"red_goal_3_south"}] = invalid;   // 8
    FieldObjectState no_time = observedAt(T_odom, 4, 900);
    no_time.lastObservedAt   = MonotonicTime{};
    field.objects[FieldObjectId{"neutral_goal_4_south"}] = no_time;   // 9

    const std::vector<translagatr::FieldEstimateRecord> records =
        fieldEstimateRecords(doc, field, robot, hostTime(1000));

    // T_field_odom * T_odom_object: (0.5 - 2.0, -0.25 + 1.0), heading 0.3 + 90 deg
    const translagatr::FieldEstimateRecord center = recordFor(records, 5);
    EXPECT_EQ(center.source, translagatr::kEstimateSourceObserved);
    EXPECT_EQ(center.flags, translagatr::kEstimateValid);
    EXPECT_EQ(center.x_mm, -1500);
    EXPECT_EQ(center.y_mm, 750);
    EXPECT_EQ(center.heading_cdeg, radToCdeg(0.3 + kPi / 2.0));
    EXPECT_EQ(center.age_ms, 100u);

    for (const uint16_t id : {2, 6, 8, 9, 101}) {
        const translagatr::FieldEstimateRecord r = recordFor(records, id);
        const auto nominal = std::find_if(doc.objects.begin(), doc.objects.end(),
                                          [&](const FieldMapDocument::Object& o) {
                                              return o.record.object_id == id;
                                          });
        ASSERT_NE(nominal, doc.objects.end());
        EXPECT_EQ(r.source, translagatr::kEstimateSourceNominal) << id;
        EXPECT_EQ(r.x_mm, nominal->record.x_mm) << id;
        EXPECT_EQ(r.y_mm, nominal->record.y_mm) << id;
        EXPECT_EQ(r.age_ms, 0u) << id;
    }

    // the same estimate after the odometry epoch moved on is nominal again
    robot.odometry_epoch = 5;
    EXPECT_EQ(recordFor(fieldEstimateRecords(doc, field, robot, hostTime(1000)), 5).source,
              translagatr::kEstimateSourceNominal);

    // the document carries the robot frame it was composed under and validates
    robot.odometry_epoch = 4;
    FieldDocuments docs(doc, 200);
    docs.update(field, robot, hostTime(1000));
    const std::vector<uint8_t> bytes = wholeEstimate(docs);
    translagatr::FieldEstimateHeader header;
    ASSERT_TRUE(translagatr::decodeFieldEstimateHeader(bytes.data(),
                                                 static_cast<uint16_t>(bytes.size()), header));
    EXPECT_EQ(header.map_id, doc.map_id);
    EXPECT_EQ(header.estimate_id, 1u);
    EXPECT_EQ(header.odometry_epoch, 4u);
    EXPECT_EQ(header.anchor_revision, 2u);
    EXPECT_EQ(translagatr::validateFieldEstimate(bytes.data(), static_cast<uint16_t>(bytes.size()),
                                           doc.bytes.data(),
                                           static_cast<uint16_t>(doc.bytes.size()), doc.map_id),
              translagatr::DocError::kNone);
}

TEST(FieldEstimate, AgeAloneIsNotAChange) {
    FieldDocuments docs(overrideDocument(), 200);
    RobotState     robot;
    FieldState     field;
    field.objects[FieldObjectId{"neutral_goal_0_center"}] =
        observedAt(Pose2D{1.0, 1.0, 0.0}, 0, 1000);
    docs.update(field, robot, hostTime(1000));
    EXPECT_EQ(docs.estimateId(), 1u);   // the first is taken at once
    docs.update(field, robot, hostTime(5000));
    EXPECT_EQ(docs.estimateId(), 1u);   // the observation only aged

    // the snapshot keeps the age it was taken with
    const std::vector<uint8_t> bytes = wholeEstimate(docs);
    translagatr::FieldEstimateRecord r;
    ASSERT_TRUE(translagatr::decodeFieldEstimateRecord(bytes.data(),
                                                 static_cast<uint16_t>(bytes.size()), 4, r));
    EXPECT_EQ(r.object_id, 5u);
    EXPECT_EQ(r.source, translagatr::kEstimateSourceObserved);
    EXPECT_EQ(r.age_ms, 0u);
}

TEST(FieldEstimate, HeadingChangesCountAtCentidegreeResolution) {
    FieldDocuments      docs(overrideDocument(), 200);
    RobotState          robot;
    FieldState          field;
    const FieldObjectId center{"neutral_goal_0_center"};
    field.objects[center] = observedAt(Pose2D{1.0, 1.0, 0.0}, 0, 1000);
    docs.update(field, robot, hostTime(1000));
    field.objects[center] = observedAt(Pose2D{1.0, 1.0, degToRad(0.004)}, 0, 1100);
    docs.update(field, robot, hostTime(1300));
    EXPECT_EQ(docs.estimateId(), 1u);   // 0.4 cdeg rounds away
    field.objects[center] = observedAt(Pose2D{1.0, 1.0, degToRad(0.02)}, 0, 1400);
    docs.update(field, robot, hostTime(1500));
    EXPECT_EQ(docs.estimateId(), 2u);
}

TEST(FieldEstimate, RetainsThreeAndNeverReusesAnId) {
    const FieldMapDocument map = overrideDocument();
    FieldDocuments         docs(map, 200);
    RobotState             robot;
    FieldState             field;
    int64_t                t = 1000;
    docs.update(field, robot, hostTime(t));
    ASSERT_EQ(docs.estimateId(), 1u);

    // a change inside the period waits, then is taken
    robot.anchor_revision = 1;
    docs.update(field, robot, hostTime(t + 100));
    EXPECT_EQ(docs.estimateId(), 1u);
    docs.update(field, robot, hostTime(t + 200));
    EXPECT_EQ(docs.estimateId(), 2u);

    // sub-millimeter motion is not a change at wire resolution
    field.objects[FieldObjectId{"neutral_goal_0_center"}] =
        observedAt(Pose2D{1.0, 1.0, 0.0}, 0, t + 250);
    docs.update(field, robot, hostTime(t + 450));
    EXPECT_EQ(docs.estimateId(), 3u);   // nominal -> observed
    field.objects[FieldObjectId{"neutral_goal_0_center"}] =
        observedAt(Pose2D{1.0002, 1.0, 0.0}, 0, t + 600);
    docs.update(field, robot, hostTime(t + 700));
    EXPECT_EQ(docs.estimateId(), 3u);
    field.objects[FieldObjectId{"neutral_goal_0_center"}] =
        observedAt(Pose2D{1.002, 1.0, 0.0}, 0, t + 800);
    docs.update(field, robot, hostTime(t + 900));
    EXPECT_EQ(docs.estimateId(), 4u);

    // an epoch change is a change
    robot.odometry_epoch = 1;
    docs.update(field, robot, hostTime(t + 1100));
    EXPECT_EQ(docs.estimateId(), 5u);

    translagatr::BrainReply reply;
    EXPECT_EQ(docs.read(translagatr::kDocFieldEstimate, 2, 0, 96, reply), translagatr::kResultStale);
    for (const uint32_t id : {3u, 4u, 5u}) {
        EXPECT_EQ(docs.read(translagatr::kDocFieldEstimate, id, 0, 96, reply), translagatr::kResultOk);
        EXPECT_EQ(reply.doc_id, id);
    }
    EXPECT_EQ(docs.read(translagatr::kDocFieldEstimate, 0, 0, 96, reply), translagatr::kResultOk);
    EXPECT_EQ(reply.doc_id, 5u);

    docs.reset();
    EXPECT_EQ(docs.estimateId(), 0u);
    EXPECT_EQ(docs.read(translagatr::kDocFieldEstimate, 0, 0, 96, reply), translagatr::kResultUnavailable);
    docs.update(field, robot, hostTime(t + 1110));
    EXPECT_EQ(docs.estimateId(), 6u);
}

TEST(FieldEstimate, ReadRefusesUnknownDocumentsAndOffsetsPastTheEnd) {
    const FieldMapDocument map = overrideDocument();
    FieldDocuments         docs(map, 200);
    translagatr::BrainReply      reply;
    EXPECT_EQ(docs.read(translagatr::kDocFieldEstimate, 0, 0, 96, reply), translagatr::kResultUnavailable);
    docs.update(FieldState{}, RobotState{}, hostTime(10));

    const uint16_t total = static_cast<uint16_t>(map.bytes.size());
    ASSERT_EQ(docs.read(translagatr::kDocFieldMap, 0, 0, 200, reply), translagatr::kResultOk);
    EXPECT_EQ(reply.doc_id, map.map_id);
    EXPECT_EQ(reply.doc_crc32, map.map_id);
    EXPECT_EQ(reply.doc_total_len, total);
    EXPECT_EQ(reply.data_len, translagatr::kDocChunkMax);   // capped by the frame
    EXPECT_EQ(0, std::memcmp(reply.data, map.bytes.data(), translagatr::kDocChunkMax));

    ASSERT_EQ(docs.read(translagatr::kDocFieldMap, map.map_id, total - 20, 96, reply),
              translagatr::kResultOk);
    EXPECT_EQ(reply.data_len, 20u);   // the tail
    ASSERT_EQ(docs.read(translagatr::kDocFieldMap, 0, 8, 10, reply), translagatr::kResultOk);
    EXPECT_EQ(reply.data_len, 10u);   // max_len
    EXPECT_EQ(reply.doc_offset, 8u);

    EXPECT_EQ(docs.read(translagatr::kDocFieldMap, 0, total, 96, reply),
              translagatr::kResultInvalidArgument);
    EXPECT_EQ(docs.read(translagatr::kDocFieldMap, 0, 0, 0, reply), translagatr::kResultInvalidArgument);
    EXPECT_EQ(docs.read(3, 0, 0, 96, reply), translagatr::kResultInvalidArgument);
    EXPECT_EQ(docs.read(translagatr::kDocFieldMap, map.map_id ^ 1u, 0, 96, reply),
              translagatr::kResultStale);
    EXPECT_EQ(docs.read(translagatr::kDocFieldEstimate, 77, 0, 96, reply), translagatr::kResultStale);
    EXPECT_EQ(docs.read(translagatr::kDocFieldEstimate, 1, translagatr::fieldEstimateLen(17), 96, reply),
              translagatr::kResultInvalidArgument);
}

// ---- READ_DOC over the link ----------------------------------------------------

TEST(FieldReadDoc, OverrideDocumentsAssembleFromChunks) {
    DocHarness f(kOverride, R"(<Field resource_id="override_field"/>)", true);
    ASSERT_NE(f.publisher, nullptr) << f.build_error;
    f.open();
    const FieldMapDocument expected = overrideDocument();

    const translagatr::BrainState s = f.state();
    EXPECT_EQ(s.map_id, expected.map_id);
    EXPECT_EQ(s.estimate_id, 1u);

    const Assembled map = f.read(translagatr::kDocFieldMap);
    ASSERT_EQ(map.result, translagatr::kResultOk);
    EXPECT_EQ(map.chunks, 6);   // 500 bytes
    EXPECT_EQ(map.id, expected.map_id);
    EXPECT_EQ(map.bytes, expected.bytes);

    const Assembled estimate = f.read(translagatr::kDocFieldEstimate);
    ASSERT_EQ(estimate.result, translagatr::kResultOk);
    EXPECT_EQ(estimate.chunks, 4);   // 364 bytes
    EXPECT_EQ(estimate.id, s.estimate_id);
    EXPECT_EQ(translagatr::validateFieldEstimate(estimate.bytes.data(),
                                           static_cast<uint16_t>(estimate.bytes.size()),
                                           map.bytes.data(),
                                           static_cast<uint16_t>(map.bytes.size()), map.id),
              translagatr::DocError::kNone);
}

TEST(FieldReadDoc, AnotherObjectCountFromInlineXml) {
    for (const auto& counts : {std::pair<int, int>{40, 6}, std::pair<int, int>{1, 0}}) {
        DocHarness f(fieldXml(counts.first, counts.second), kField);
        ASSERT_NE(f.publisher, nullptr) << f.build_error;
        f.open();
        const uint16_t  n   = static_cast<uint16_t>(counts.first + counts.second);
        const Assembled map = f.read(translagatr::kDocFieldMap);
        ASSERT_EQ(map.result, translagatr::kResultOk);
        EXPECT_EQ(map.bytes.size(), translagatr::fieldMapLen(n));
        EXPECT_EQ(map.chunks, (translagatr::fieldMapLen(n) + 95) / 96);
        EXPECT_EQ(translagatr::validateFieldMap(map.bytes.data(), static_cast<uint16_t>(map.bytes.size())),
                  translagatr::DocError::kNone);
        EXPECT_EQ(f.state().map_id, map.id);

        const Assembled estimate = f.read(translagatr::kDocFieldEstimate);
        ASSERT_EQ(estimate.result, translagatr::kResultOk);
        EXPECT_EQ(estimate.bytes.size(), translagatr::fieldEstimateLen(n));
        EXPECT_EQ(translagatr::validateFieldEstimate(
                      estimate.bytes.data(), static_cast<uint16_t>(estimate.bytes.size()),
                      map.bytes.data(), static_cast<uint16_t>(map.bytes.size()), map.id),
                  translagatr::DocError::kNone);
    }
}

TEST(FieldReadDoc, ReplacedEstimateGoesStaleAndTheReaderRestarts) {
    DocHarness f(fieldXml(12, 2), R"(<Field resource_id="field" estimate_period_ms="10"/>)");
    ASSERT_NE(f.publisher, nullptr) << f.build_error;
    f.open();

    // first chunk of the current estimate, then the anchor moves twice
    const translagatr::BrainReply first =
        f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldEstimate, 0, 0));
    ASSERT_EQ(first.result, translagatr::kResultOk);
    const uint32_t id = first.doc_id;
    ASSERT_GT(first.doc_total_len, translagatr::kDocChunkMax);
    const auto move = [&] {
        ++f.robot.anchor_revision;
        f.cycle();
        f.cycle();
    };
    move();
    move();
    EXPECT_EQ(f.state().estimate_id, id + 2);
    EXPECT_EQ(f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldEstimate, id, 96)).result,
              translagatr::kResultOk);   // still one of the last three

    move();
    EXPECT_EQ(f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldEstimate, id, 192)).result,
              translagatr::kResultStale);

    // restart from the newest: complete and consistent with its header
    const Assembled again = f.read(translagatr::kDocFieldEstimate);
    ASSERT_EQ(again.result, translagatr::kResultOk);
    EXPECT_EQ(again.id, id + 3);
    translagatr::FieldEstimateHeader header;
    ASSERT_TRUE(translagatr::decodeFieldEstimateHeader(
        again.bytes.data(), static_cast<uint16_t>(again.bytes.size()), header));
    EXPECT_EQ(header.estimate_id, id + 3);
    EXPECT_EQ(header.anchor_revision, 3u);
}

TEST(FieldReadDoc, PublisherResetDropsEstimatesAndTakesANewOneAtOnce) {
    DocHarness f(fieldXml(3, 1), kField);   // 200 ms period
    ASSERT_NE(f.publisher, nullptr) << f.build_error;
    f.open();
    const translagatr::BrainState before = f.state();
    ASSERT_EQ(before.estimate_id, 1u);
    f.publisher->reset();
    f.cycle();   // well inside the period
    const translagatr::BrainState after = f.state();
    EXPECT_EQ(after.estimate_id, 2u);   // ids are never reused
    EXPECT_EQ(after.map_id, before.map_id);
    EXPECT_EQ(f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldEstimate, 1, 0)).result,
              translagatr::kResultStale);
}

TEST(FieldReadDoc, RefusalsOverTheLink) {
    DocHarness f(fieldXml(3, 1), kField);
    ASSERT_NE(f.publisher, nullptr) << f.build_error;
    f.open();
    const uint32_t map_id = f.state().map_id;
    const uint16_t total  = translagatr::fieldMapLen(4);
    EXPECT_EQ(f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldMap, 0, total)).result,
              translagatr::kResultInvalidArgument);
    EXPECT_EQ(f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldMap, map_id + 1, 0)).result,
              translagatr::kResultStale);
    EXPECT_EQ(f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldMap, 0, 0, 0)).result,
              translagatr::kResultInvalidArgument);
    const translagatr::BrainReply tail =
        f.one(readRequest(f.session, f.rid++, translagatr::kDocFieldMap, map_id, total - 1));
    ASSERT_EQ(tail.result, translagatr::kResultOk);
    EXPECT_EQ(tail.data_len, 1u);
}

TEST(FieldReadDoc, PublisherFieldConfiguration) {
    const auto refused = [](const std::string& field, const std::string& element,
                            const char* expect) {
        DocHarness f(field, element);
        EXPECT_EQ(f.publisher, nullptr) << expect;
        EXPECT_NE(f.build_error.find(expect), std::string::npos) << f.build_error;
    };
    const std::string good = fieldXml(2, 0);
    refused(good, R"(<Field resource_id="ghost"/>)", "unknown resource id ghost");
    refused(good, R"(<Field resource_id="brain_uart"/>)", "brain_uart");
    refused(good, R"(<Field resource_id="field" estimate_period_ms="0"/>)", "estimate_period_ms");
    refused(good, R"(<Field resource_id="field" period_ms="10"/>)", "unknown attribute period_ms");
    refused(good, R"(<Field/>)", "resource_id");
    refused(good, std::string(kField) + kField, "more than one Field");
    refused(R"(<Resource id="field" type="field_map">
                 <Boundary min_x_m="0" min_y_m="0" max_x_m="1" max_y_m="1"/></Resource>)",
            kField, "revision");
    refused(R"(<Resource id="field" type="field_map" revision="2">
                 <Landmark id="g" wire_id="1"><NominalPose x_m="0" y_m="0" heading_deg="0"/></Landmark>
               </Resource>)",
            kField, "Boundary");

    // without a Field the documents are absent
    DocHarness none(good, "");
    ASSERT_NE(none.publisher, nullptr) << none.build_error;
    none.open();
    const translagatr::BrainState s = none.state();
    EXPECT_EQ(s.map_id, 0u);
    EXPECT_EQ(s.estimate_id, 0u);
    EXPECT_EQ(none.one(readRequest(none.session, none.rid++, translagatr::kDocFieldMap, 0, 0)).result,
              translagatr::kResultUnavailable);
}

// ---- whole Pi ------------------------------------------------------------------

TEST(FieldReadDoc, WholePiServesTheFieldAndFollowsItsAnchor) {
    const std::string xml = R"(
<System>
    <Loop rate_hz="200"/>
    <Resources>
        <Resource id="pico_uart" type="memory_link"/>
        <Resource id="pico_telemetry" type="pico_telemetry">
            <Serial resource_id="pico_uart"/>
            <Output id="encoder_a" channel="0"/>
            <Output id="encoder_b" channel="1"/>
            <Output id="encoder_c" channel="2"/>
        </Resource>
        <Resource id="brain_uart" type="memory_link"/>
        )" + fieldXml(5, 2) + R"(
    </Resources>
    <Sensors>
        <Sensor id="enc_a" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_a"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_b" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_b"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
        <Sensor id="enc_c" type="pico_encoder_channel">
            <Source resource_id="pico_telemetry" output_id="encoder_c"/>
            <Calibration counts_per_revolution="4000"/>
        </Sensor>
    </Sensors>
    <Pipeline>
        <CommandCollection type="brain_link"><Serial resource_id="brain_uart"/></CommandCollection>
        <Localization>
            <Observation id="tracking_motion" type="tracking_wheel_motion">
                <TrackingWheel sensor_id="enc_a" label="left" radius_m="0.0254"
                               position_x_m="0" position_y_m="0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_b" label="right" radius_m="0.0254"
                               position_x_m="0" position_y_m="-0.13"
                               measurement_angle_deg="0" direction="positive"/>
                <TrackingWheel sensor_id="enc_c" label="rear" radius_m="0.0254"
                               position_x_m="-0.12" position_y_m="0"
                               measurement_angle_deg="90" direction="positive"/>
                <Output observation_id="tracking_motion"/>
            </Observation>
            <Estimator type="planar_motion_integrator">
                <Motion observation_id="tracking_motion"/>
            </Estimator>
        </Localization>
        <WorldEstimation><Estimator id="none" type="noop"/></WorldEstimation>
        <TargetResolution type="noop"/>
        <Publishing type="brain_link">
            <Serial resource_id="brain_uart"/>
            <Field resource_id="field" estimate_period_ms="20"/>
        </Publishing>
    </Pipeline>
</System>)";
    FunctionRegistry functions;
    registerAll(functions);
    std::string err;
    auto        system = System::buildFromString(xml.c_str(), functions, err);
    ASSERT_NE(system, nullptr) << err;
    auto link = system->resources().require<SerialLink>(ResourceId{"brain_uart"}, err);
    auto* brain = dynamic_cast<MemoryLink*>(link.get());
    ASSERT_NE(brain, nullptr);
    int64_t clock_us = 1000000, now_ms = 0;
    brain->setClock([&] { return clock_us; });
    const auto step = [&] {
        clock_us += 5000;
        now_ms += 5;
        system->step(hostTime(now_ms));
        return takeReplies(*brain);
    };
    const Exchange one = [&](const translagatr::BrainRequest& r) {
        brain->input().feed(requestBytes(r));
        const std::vector<translagatr::BrainReply> replies = step();
        EXPECT_EQ(replies.size(), 1u);
        return replies.empty() ? translagatr::BrainReply{} : replies.front();
    };
    step();

    uint16_t            rid   = 1;
    translagatr::BrainRequest hello = request(translagatr::kOpHello, 0, rid++);
    hello.nonce               = 7;
    const uint32_t session    = one(hello).session;

    FieldMap         map;
    FieldMapDocument expected;
    ASSERT_TRUE(parseInline(fieldXml(5, 2), map, err)) << err;
    ASSERT_TRUE(buildFieldMapDocument(map, expected, err)) << err;
    translagatr::BrainState s = one(request(translagatr::kOpGetState, session, rid++)).state;
    EXPECT_EQ(s.map_id, expected.map_id);
    EXPECT_EQ(s.estimate_id, 1u);
    EXPECT_EQ(readWhole(one, session, rid, translagatr::kDocFieldMap).bytes, expected.bytes);

    // a placement moves the anchor; the next snapshot names it
    translagatr::BrainRequest place = request(translagatr::kOpSetPose, session, rid++);
    place.x_mm                = 610;
    place.y_mm                = 457;
    place.heading_cdeg        = 9000;
    ASSERT_EQ(one(place).result, translagatr::kResultOk);
    for (int i = 0; i < 5; ++i) {
        step();
    }
    s = one(request(translagatr::kOpGetState, session, rid++)).state;
    EXPECT_EQ(s.anchor_revision, 1u);
    EXPECT_EQ(s.estimate_id, 2u);
    const Assembled estimate = readWhole(one, session, rid, translagatr::kDocFieldEstimate);
    ASSERT_EQ(estimate.result, translagatr::kResultOk);
    EXPECT_EQ(estimate.id, s.estimate_id);
    translagatr::FieldEstimateHeader header;
    ASSERT_TRUE(translagatr::decodeFieldEstimateHeader(
        estimate.bytes.data(), static_cast<uint16_t>(estimate.bytes.size()), header));
    EXPECT_EQ(header.anchor_revision, s.anchor_revision);
    EXPECT_EQ(header.odometry_epoch, s.odometry_epoch);
    EXPECT_EQ(translagatr::validateFieldEstimate(estimate.bytes.data(),
                                           static_cast<uint16_t>(estimate.bytes.size()),
                                           expected.bytes.data(),
                                           static_cast<uint16_t>(expected.bytes.size()),
                                           expected.map_id),
              translagatr::DocError::kNone);
    for (uint16_t i = 0; i < header.object_count; ++i) {
        translagatr::FieldEstimateRecord r;
        ASSERT_TRUE(translagatr::decodeFieldEstimateRecord(
            estimate.bytes.data(), static_cast<uint16_t>(estimate.bytes.size()), i, r));
        EXPECT_EQ(r.source, translagatr::kEstimateSourceNominal);   // noop world estimation
    }
}
