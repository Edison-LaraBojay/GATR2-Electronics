// field_refs.cpp
// navigatr_field_refs: reads a field definition with the runtime parser,
// builds its brain link map document and writes the Brain header of named
// field references (struct Field) with that map id.
//
//   navigatr_field_refs <field.xml> <output.h> [<source label>]
//
// The label names the field file in the header's first line and defaults to
// <field.xml> as given. The checked-in header is regenerated with the
// field_references build target; navigatr_tests fails when it is stale.

#include <cstdio>
#include <fstream>
#include <string>

#include "impl/publishing/field_documents.h"

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::fprintf(stderr, "usage: navigatr_field_refs <field.xml> <output.h> [<source label>]\n");
        return 2;
    }
    const std::string input  = argv[1];
    const std::string output = argv[2];
    std::string       label  = argc == 4 ? argv[3] : input;
    for (char& c : label) {
        if (c == '\\') {
            c = '/';
        }
    }

    navigatr::FieldMap         map;
    navigatr::FieldMapDocument doc;
    std::string                header;
    std::string                err;
    if (!navigatr::loadFieldMapFile(input, map, err) ||
        !navigatr::buildFieldMapDocument(map, doc, err) ||
        !navigatr::fieldReferencesHeader(doc, label, header, err)) {
        std::fprintf(stderr, "navigatr_field_refs: %s\n", err.c_str());
        return 1;
    }
    std::ofstream out(output, std::ios::binary | std::ios::trunc);
    out << header;
    out.close();
    if (!out) {
        std::fprintf(stderr, "navigatr_field_refs: cannot write %s\n", output.c_str());
        return 1;
    }
    std::printf("map_id 0x%08X revision %u objects %u -> %s\n",
                static_cast<unsigned>(doc.map_id), static_cast<unsigned>(doc.revision),
                static_cast<unsigned>(doc.objects.size()), output.c_str());
    return 0;
}
