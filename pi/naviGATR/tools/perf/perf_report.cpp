// perf_report.cpp
// Markdown tables from run_perf.sh output folders (server_rN.json,
// browser_rN.json). One column per folder, so a baseline and an after run
// sit side by side:
//
//   navigatr_perf_report <dir>[=label] [<dir>[=label] ...]
//
// A cell is p50 / p95 / max: the median over repeats of each run's p50 and
// p95, and the largest max seen in any run. Counts and rates are the median
// over repeats.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

#include "perf_json.h"

using namespace navigatr::perf;

namespace
{

struct Set {
    std::string       label;
    std::vector<Json> server;
    std::vector<Json> browser;
};

bool readJson(const std::string& path, Json& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parseJson(ss.str(), out);
}

double median(std::vector<double> v) {
    v.erase(std::remove_if(v.begin(), v.end(), [](double x) { return !std::isfinite(x); }),
            v.end());
    if (v.empty()) {
        return NAN;
    }
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double maxOf(const std::vector<double>& v) {
    double m = NAN;
    for (double x : v) {
        if (std::isfinite(x)) {
            m = std::isnan(m) ? x : std::max(m, x);
        }
    }
    return m;
}

std::string fmt(double v, int digits = 1) {
    if (!std::isfinite(v)) {
        return "-";
    }
    char buf[48];
    if (std::fabs(v) >= 100000.0) {
        std::snprintf(buf, sizeof(buf), "%.0f", v);
    } else {
        std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
    }
    return buf;
}

// a summary object reached from each run by getter
using Getter = std::function<const Json*(const Json&)>;

std::string summaryCell(const std::vector<Json>& runs, const Getter& get, int digits = 1) {
    std::vector<double> p50, p95, mx;
    for (const Json& r : runs) {
        const Json* s = get(r);
        if (s == nullptr || s->get("n") == nullptr || s->get("n")->number == 0) {
            continue;
        }
        p50.push_back(s->get("p50")->number);
        p95.push_back(s->get("p95")->number);
        mx.push_back(s->get("max")->number);
    }
    if (p50.empty()) {
        return "-";
    }
    return fmt(median(p50), digits) + " / " + fmt(median(p95), digits) + " / " +
           fmt(maxOf(mx), digits);
}

std::string valueCell(const std::vector<Json>& runs, const Getter& get, int digits = 1) {
    std::vector<double> v;
    for (const Json& r : runs) {
        const Json* x = get(r);
        if (x != nullptr) {
            if (x->type == Json::Type::kNumber) {
                v.push_back(x->number);
            } else if (x->type == Json::Type::kBool) {
                v.push_back(x->b ? 1.0 : 0.0);
            }
        }
    }
    if (v.empty()) {
        return "-";
    }
    std::string cell = fmt(median(v), digits);
    if (v.size() > 1) {
        cell += " (" + fmt(*std::min_element(v.begin(), v.end()), digits) + "-" +
                fmt(*std::max_element(v.begin(), v.end()), digits) + ")";
    }
    return cell;
}

// per-bucket median over repeats of an array of numbers
std::string timelineCell(const std::vector<Json>& runs, const Getter& get) {
    std::vector<std::vector<double>> cols;
    for (const Json& r : runs) {
        const Json* a = get(r);
        if (a == nullptr || a->type != Json::Type::kArray) {
            continue;
        }
        for (std::size_t i = 0; i < a->items.size(); ++i) {
            if (cols.size() <= i) {
                cols.resize(i + 1);
            }
            if (a->items[i].type == Json::Type::kNumber) {
                cols[i].push_back(a->items[i].number);
            }
        }
    }
    if (cols.empty()) {
        return "-";
    }
    std::string out;
    for (std::size_t i = 0; i < cols.size(); ++i) {
        const double m = median(cols[i]);
        out += (i ? ", " : "") + fmt(m, std::isfinite(m) && m < 10.0 ? 1 : 0);
    }
    return out;
}

const Json* path(const Json* v, std::initializer_list<const char*> keys) {
    return v != nullptr ? v->at(keys) : nullptr;
}

// client by name within a server scenario
const Json* client(const Json& run, const char* scenario, const char* name) {
    const Json* clients = run.at({"scenarios", scenario, "clients"});
    if (clients == nullptr) {
        return nullptr;
    }
    for (const Json& c : clients->items) {
        const Json* n = c.get("name");
        if (n != nullptr && n->str == name) {
            return &c;
        }
    }
    return nullptr;
}

double serverDelta(const Json* scenario, const char* key) {
    if (scenario == nullptr) {
        return NAN;
    }
    const Json* a = scenario->at({"server_workers_after", "inspection", key});
    const Json* b = scenario->at({"server_workers_before", "inspection", key});
    return a != nullptr && b != nullptr ? a->number - b->number : NAN;
}

const Json* browserScenario(const Json& run, const std::string& name) {
    const Json* list = run.get("scenarios");
    if (list == nullptr) {
        return nullptr;
    }
    for (const Json& s : list->items) {
        const Json* n = s.get("scenario");
        if (n != nullptr && n->str == name) {
            return &s;
        }
    }
    return nullptr;
}

struct Table {
    std::vector<std::string>              header;
    std::vector<std::vector<std::string>> rows;

    void print() const {
        std::string line = "|";
        std::string sep  = "|";
        for (const std::string& h : header) {
            line += " " + h + " |";
            sep += "---|";
        }
        std::printf("%s\n%s\n", line.c_str(), sep.c_str());
        for (const auto& r : rows) {
            std::string l = "|";
            for (const std::string& c : r) {
                l += " " + c + " |";
            }
            std::printf("%s\n", l.c_str());
        }
        std::printf("\n");
    }
};

Table newTable(const std::vector<Set>& sets, const char* first) {
    Table t;
    t.header.push_back(first);
    for (const Set& s : sets) {
        t.header.push_back(s.label);
    }
    return t;
}

void addSummaryRow(Table& t, const std::vector<Set>& sets, const std::string& name,
                   bool browser, const Getter& get, int digits = 1) {
    std::vector<std::string> row{name};
    for (const Set& s : sets) {
        row.push_back(summaryCell(browser ? s.browser : s.server, get, digits));
    }
    t.rows.push_back(row);
}

void addValueRow(Table& t, const std::vector<Set>& sets, const std::string& name, bool browser,
                 const Getter& get, int digits = 1) {
    std::vector<std::string> row{name};
    for (const Set& s : sets) {
        row.push_back(valueCell(browser ? s.browser : s.server, get, digits));
    }
    t.rows.push_back(row);
}

// several values per cell, "a / b / c", each the median over repeats
void addMultiRow(Table& t, const std::vector<Set>& sets, const std::string& name, bool browser,
                 const std::vector<Getter>& gets, int digits = 0) {
    std::vector<std::string> row{name};
    for (const Set& s : sets) {
        const std::vector<Json>& runs = browser ? s.browser : s.server;
        std::string              cell;
        bool                     any  = false;
        for (std::size_t i = 0; i < gets.size(); ++i) {
            std::vector<double> v;
            for (const Json& r : runs) {
                const Json* x = gets[i](r);
                if (x != nullptr && x->type == Json::Type::kNumber) {
                    v.push_back(x->number);
                }
            }
            any = any || !v.empty();
            cell += (i ? " / " : "") + fmt(median(v), digits);
        }
        row.push_back(any ? cell : "-");
    }
    t.rows.push_back(row);
}

// a number computed from one run, for the value rows (read at once)
const Json* computed(double v) {
    static Json tmp;
    if (!std::isfinite(v)) {
        return nullptr;
    }
    tmp        = Json{};
    tmp.type   = Json::Type::kNumber;
    tmp.number = v;
    return &tmp;
}

double numberAt(const Json* v, std::initializer_list<const char*> keys) {
    const Json* x = v != nullptr ? v->at(keys) : nullptr;
    return x != nullptr && x->type == Json::Type::kNumber ? x->number : NAN;
}

// the page's WebSocket receive rate in one browser scenario, all kinds
double wsBytesPerS(const Json* scenario) {
    if (scenario == nullptr) {
        return NAN;
    }
    const Json* v = scenario->get("ws_received_bytes_s");
    if (v != nullptr && v->type == Json::Type::kNumber) {
        return v->number;
    }
    const Json* msgs = scenario->at({"probe", "messages"});   // older results
    if (msgs == nullptr) {
        return NAN;
    }
    double sum = 0.0;
    for (const auto& kv : msgs->fields) {
        const Json* b = kv.second.get("bytes_s");
        sum += b != nullptr ? b->numberOr(0.0) : 0.0;
    }
    return sum;
}

// K of a _netK scenario name, 0 when none
double requestedKbps(const std::string& name) {
    std::size_t at = 0;
    while ((at = name.find("_net", at)) != std::string::npos) {
        at += 4;
        const double k = std::atof(name.c_str() + at);
        if (k > 0.0) {
            return k;
        }
    }
    return 0.0;
}

void serverTables(const std::vector<Set>& sets) {
    std::printf("### Server: snapshot/state build (in-process, direct calls)\n\n");
    Table b = newTable(sets, "metric (p50 / p95 / max)");
    addSummaryRow(b, sets, "full snapshot build, us", false,
                  [](const Json& r) { return r.at({"scenarios", "build", "snapshot_build_us"}); },
                  0);
    addSummaryRow(b, sets, "full snapshot size, bytes", false,
                  [](const Json& r) { return r.at({"scenarios", "build", "snapshot_bytes"}); }, 0);
    addSummaryRow(
        b, sets, "snapshot without trail, build us", false,
        [](const Json& r) { return r.at({"scenarios", "build", "snapshot_no_trail_build_us"}); },
        0);
    addSummaryRow(
        b, sets, "snapshot without trail, bytes", false,
        [](const Json& r) { return r.at({"scenarios", "build", "snapshot_no_trail_bytes"}); }, 0);
    addSummaryRow(b, sets, "inspect/2 state build, us", false,
                  [](const Json& r) { return r.at({"scenarios", "build", "state_build_us"}); },
                  0);
    addSummaryRow(b, sets, "inspect/2 state size, bytes", false,
                  [](const Json& r) { return r.at({"scenarios", "build", "state_bytes"}); }, 0);
    addSummaryRow(b, sets, "inspect/2 history build, us", false,
                  [](const Json& r) { return r.at({"scenarios", "build", "history_build_us"}); },
                  0);
    addSummaryRow(b, sets, "inspect/2 history size, bytes", false,
                  [](const Json& r) { return r.at({"scenarios", "build", "history_bytes"}); }, 0);
    addValueRow(b, sets, "hello size, bytes", false,
                [](const Json& r) { return r.at({"scenarios", "build", "hello_bytes"}); }, 0);
    addValueRow(b, sets, "trail entries in snapshot", false,
                [](const Json& r) {
                    return r.at({"scenarios", "build", "last_snapshot_section_bytes",
                                 "trail_entries"});
                },
                0);
    for (const char* section : {"trail", "robot", "localization", "field_objects",
                                "detection_frames", "brain_link", "pico", "events", "sources",
                                "workers", "diagnostics"}) {
        const std::string key = section;
        addValueRow(b, sets, "  section `" + key + "`, bytes", false,
                    [key](const Json& r) {
                        const Json* s =
                            r.at({"scenarios", "build", "last_snapshot_section_bytes"});
                        return s != nullptr ? s->get(key) : nullptr;
                    },
                    0);
    }
    b.print();

    struct ClientRow {
        const char* scenario;
        const char* client;
        const char* title;
    };
    const ClientRow crs[] = {
        {"fast", "fast", "fast client alone"},
        {"slow", "fast", "fast client, next to a slow one"},
        {"slow", "slow", "slow client (throttled reads)"},
        {"stall", "observer", "observer, next to a stalling client"},
        {"stall", "stall", "stalling client"},
        {"burst", "observer", "observer, next to a preview burst"},
        {"burst", "burst", "preview burst client"},
    };
    for (const ClientRow& cr : crs) {
        std::printf("### Server feed: %s (`%s` scenario)\n\n", cr.title, cr.scenario);
        Table       t  = newTable(sets, "metric");
        const char* sc = cr.scenario;
        const char* cn = cr.client;
        const auto  c  = [sc, cn](const Json& r) { return client(r, sc, cn); };
        addValueRow(t, sets, "state/snapshot messages per s", false,
                    [c](const Json& r) { return path(c(r), {"state", "per_s"}); });
        addValueRow(t, sets, "bytes read per s", false,
                    [c](const Json& r) { return path(c(r), {"read_bytes_s"}); }, 0);
        addValueRow(t, sets, "distinct publications received", false,
                    [c](const Json& r) { return path(c(r), {"state", "distinct_publications"}); },
                    0);
        addValueRow(t, sets, "localization publication rate, Hz", false,
                    [c](const Json& r) { return path(c(r), {"state", "publication_rate_hz"}); });
        addSummaryRow(t, sets, "message size, bytes (p50 / p95 / max)", false,
                      [c](const Json& r) { return path(c(r), {"state", "size_bytes"}); }, 0);
        addSummaryRow(t, sets, "receive interval, ms", false,
                      [c](const Json& r) { return path(c(r), {"state", "interval_ms"}); });
        addSummaryRow(
            t, sets, "source age at publish, ms", false,
            [c](const Json& r) { return path(c(r), {"state", "source_age_at_publish_ms"}); });
        // host_ms/host_us are stamped before the document is built, so this
        // includes the build (inspect/1 host_ms also truncates up to 1 ms)
        addSummaryRow(
            t, sets, "publish to receive: build + queue + socket + read, ms (exact clock)", false,
            [c](const Json& r) { return path(c(r), {"state", "publish_to_receive_ms"}); }, 2);
        addValueRow(
            t, sets, "  p50 minus p50 build time of that document, ms (queue + socket + read, estimate)",
            false,
            [c](const Json& r) {
                const Json* ct = r.at({"conditions", "contract"});
                const bool  v2 = ct != nullptr && ct->str == "navigatr.inspect/2";
                const Json* b  = r.at({"scenarios", "build",
                                       v2 ? "state_build_us" : "snapshot_build_us"});
                if (numberAt(b, {"n"}) > 0.0) {
                    return computed(numberAt(c(r), {"state", "publish_to_receive_ms", "p50"}) -
                                    numberAt(b, {"p50"}) / 1000.0);
                }
                return computed(NAN);
            },
            2);
        addSummaryRow(t, sets, "pose age at receipt, ms (exact)", false,
                      [c](const Json& r) { return path(c(r), {"state", "age_at_receipt_ms"}); });
        addValueRow(t, sets, "messages older than fresh_ms at receipt", false,
                    [c](const Json& r) { return path(c(r), {"state", "stale_over_fresh_ms"}); },
                    0);
        addSummaryRow(t, sets, "RTT, ms", false,
                      [c](const Json& r) { return path(c(r), {"rtt_ms"}); }, 2);
        // spec 3.4 estimate (inspect/2 pongs); in-process it sits next to the exact value
        addSummaryRow(t, sets, "ping-offset estimate of publish to receive, ms (+- below)", false,
                      [c](const Json& r) {
                          return path(c(r), {"ping_offset_estimate", "publish_to_receive_ms"});
                      },
                      2);
        addSummaryRow(t, sets, "  its +- half-width (RTT/2 of the pong used), ms", false,
                      [c](const Json& r) {
                          return path(c(r), {"ping_offset_estimate", "half_width_ms"});
                      },
                      2);
        // "-" when there were no estimates at all (inspect/1: unstamped pongs)
        const auto est_n = [c](const Json& r, const char* key) -> const Json* {
            if (!(numberAt(c(r), {"ping_offset_estimate", "estimates"}) > 0.0)) {
                return nullptr;
            }
            return path(c(r), {"ping_offset_estimate", key});
        };
        addMultiRow(t, sets, "  estimates too wide to use / all", false,
                    {[est_n](const Json& r) { return est_n(r, "too_wide"); },
                     [est_n](const Json& r) { return est_n(r, "estimates"); }});
        addValueRow(t, sets, "closed by server (1 = yes)", false,
                    [c](const Json& r) { return path(c(r), {"closed_by_server"}); }, 0);
        // the server's own per-client counters (inspect/2 diag.inspection.clients)
        for (const char* chan : {"state", "diag"}) {
            const std::string ch = chan;
            addMultiRow(t, sets, "server view: `" + ch + "` sent / replaced unsent / refused",
                        false,
                        {[c, ch](const Json& r) {
                             return path(c(r), {"server_view", "channels", ch.c_str(), "sent"});
                         },
                         [c, ch](const Json& r) {
                             return path(c(r), {"server_view", "channels", ch.c_str(), "replaced"});
                         },
                         [c, ch](const Json& r) {
                             return path(c(r), {"server_view", "channels", ch.c_str(), "refused"});
                         }});
        }
        addMultiRow(t, sets, "server view: previews sent / replaced unsent / refused", false,
                    {[c](const Json& r) {
                         return path(c(r), {"server_view", "preview_total", "sent"});
                     },
                     [c](const Json& r) {
                         return path(c(r), {"server_view", "preview_total", "replaced"});
                     },
                     [c](const Json& r) {
                         return path(c(r), {"server_view", "preview_total", "refused"});
                     }});
        addValueRow(t, sets, "server view: reliable messages sent", false,
                    [c](const Json& r) {
                        return path(c(r), {"server_view", "reliable_total", "sent"});
                    },
                    0);
        addSummaryRow(t, sets, "server view: queued bytes, per diag", false,
                      [c](const Json& r) { return path(c(r), {"server_view", "queued_bytes"}); },
                      0);
        if (std::string(cn) == "stall") {
            addValueRow(t, sets, "first message after resume, lag ms", false,
                        [c](const Json& r) {
                            return path(c(r), {"stall", "first_after_resume_lag_ms"});
                        });
            addValueRow(t, sets, "stale messages delivered after resume", false,
                        [c](const Json& r) {
                            return path(c(r), {"stall", "stale_messages_after_resume"});
                        },
                        0);
            addValueRow(t, sets, "time to first fresh message after resume, ms", false,
                        [c](const Json& r) {
                            return path(c(r), {"stall", "time_to_fresh_after_resume_ms"});
                        });
        }
        if (std::string(cn) == "burst") {
            addSummaryRow(t, sets, "publish to receive before burst, ms", false,
                          [c](const Json& r) {
                              return path(c(r), {"burst", "before_publish_to_receive_ms"});
                          },
                          2);
            addSummaryRow(t, sets, "publish to receive during burst, ms", false,
                          [c](const Json& r) {
                              return path(c(r), {"burst", "during_publish_to_receive_ms"});
                          },
                          2);
            addValueRow(t, sets, "preview messages per s", false, [c](const Json& r) {
                return path(c(r), {"by_type", "binary_preview", "per_s"});
            });
        }
        {
            std::vector<std::string> row{"publish to receive, max per 1 s of the run, ms"};
            for (const Set& st : sets) {
                row.push_back(timelineCell(st.server, [c](const Json& r) {
                    return path(c(r), {"timeline", "max_lag_ms"});
                }));
            }
            t.rows.push_back(row);
        }
        if (std::string(cn) == "fast" || std::string(cn) == "observer") {
            const std::string scen = sc;
            addValueRow(t, sets, "host CPU busy during scenario, % of all cores", false,
                        [scen](const Json& r) {
                            return r.at({"scenarios", scen.c_str(), "host_cpu",
                                         "machine_busy_pct"});
                        },
                        0);
            addValueRow(
                t, sets, "  of which other processes, %", false,
                [scen](const Json& r) {
                    return r.at({"scenarios", scen.c_str(), "host_cpu", "other_pct"});
                },
                0);
            addValueRow(t, sets, "estimation worker rate after scenario, Hz", false,
                        [scen](const Json& r) {
                            return r.at({"scenarios", scen.c_str(), "server_workers_after",
                                         "estimation", "rate_hz"});
                        });
            addValueRow(t, sets, "estimation worker max cycle since start, ms", false,
                        [scen](const Json& r) {
                            return r.at({"scenarios", scen.c_str(), "server_workers_after",
                                         "estimation", "max_cycle_ms"});
                        });
            addValueRow(t, sets, "estimation worker overruns since start", false,
                        [scen](const Json& r) {
                            return r.at({"scenarios", scen.c_str(), "server_workers_after",
                                         "estimation", "overruns"});
                        },
                        0);
        }
        if (std::string(cn) != "fast" || std::string(sc) != "slow") {
            // whole-server counters, once per scenario. snapshots_skipped
            // changed meaning: inspect/1 refused a full FIFO; inspect/2
            // counts states replaced unsent (latest-wins, by design) or refused.
            const std::string scen = sc;
            const auto d = [scen](const char* key) {
                return [scen, key](const Json& r) {
                    return computed(serverDelta(r.at({"scenarios", scen.c_str()}), key));
                };
            };
            addValueRow(t, sets,
                        "server: state messages not delivered, whole scenario (inspect/1: "
                        "refused; inspect/2: replaced unsent or refused)",
                        false, d("snapshots_skipped"), 0);
            addValueRow(t, sets,
                        "server (inspect/2): messages replaced unsent, all clients and channels",
                        false, d("messages_replaced"), 0);
            addValueRow(t, sets, "server (inspect/2): messages refused, all clients and channels",
                        false, d("messages_refused"), 0);
            addMultiRow(t, sets, "server (inspect/2): clients closed stalled / reliable overflow",
                        false, {d("closed_stalled"), d("closed_reliable_overflow")});
        }
        t.print();
    }
}

void browserTables(const std::vector<Set>& sets) {
    std::vector<std::string> names;
    for (const Set& s : sets) {
        for (const Json& run : s.browser) {
            const Json* list = run.get("scenarios");
            if (list == nullptr) {
                continue;
            }
            for (const Json& sc : list->items) {
                const std::string n = sc.get("scenario") != nullptr ? sc.get("scenario")->str : "";
                if (!n.empty() && std::find(names.begin(), names.end(), n) == names.end()) {
                    names.push_back(n);
                }
            }
        }
    }
    if (!names.empty()) {
        std::printf("### Browser: page load\n\n");
        Table t = newTable(sets, "metric");
        addValueRow(t, sets, "page loads until the viewer was live (1 = first try)", true,
                    [](const Json& r) { return r.at({"conditions", "load_attempts"}); }, 0);
        addValueRow(t, sets, "viewer live (1 = yes)", true,
                    [](const Json& r) { return r.at({"conditions", "viewer_live"}); }, 0);
        addValueRow(t, sets, "server session changed during the run (1 = yes, invalid)", true,
                    [](const Json& r) { return r.at({"conditions", "server_changed"}); }, 0);
        t.print();
    }
    for (const std::string& name : names) {
        std::printf("### Browser: `%s`\n\n", name.c_str());
        const auto s = [name](const Json& r) { return browserScenario(r, name); };
        const double want_kbps = requestedKbps(name);
        if (want_kbps > 0.0) {
            // CDP network emulation has not slowed the viewer's WebSocket
            // in the Chrome tested; say so rather than print a table that
            // reads like a slow-network result
            for (const Set& st : sets) {
                std::vector<double> got;
                for (const Json& r : st.browser) {
                    got.push_back(wsBytesPerS(s(r)));
                }
                const double m = median(got);
                std::printf("Network emulation (%s): %.0f kB/s requested; the page received %s "
                            "kB/s over its WebSockets (median).%s\n",
                            st.label.c_str(), want_kbps, fmt(m / 1000.0, 0).c_str(),
                            std::isfinite(m) && m > want_kbps * 1000.0 * 1.1
                                ? " The throttle was NOT applied to the WebSocket: this is not a "
                                  "slow-network result."
                                : "");
            }
            std::printf("\n");
        }
        Table t = newTable(sets, "metric");
        addValueRow(t, sets, "host CPU busy during scenario, % of all cores", true,
                    [s](const Json& r) { return path(s(r), {"host_cpu", "machine_busy_pct"}); },
                    0);
        addValueRow(t, sets, "frames per s (rAF)", true,
                    [s](const Json& r) { return path(s(r), {"probe", "frames", "fps"}); });
        addSummaryRow(t, sets, "frame interval, ms (p50 / p95 / max)", true, [s](const Json& r) {
            return path(s(r), {"probe", "frames", "interval_ms"});
        });
        addValueRow(t, sets, "frames over 33 ms", true,
                    [s](const Json& r) { return path(s(r), {"probe", "frames", "over_33ms"}); },
                    0);
        addSummaryRow(t, sets, "page rAF callback time per frame, ms", true,
                      [s](const Json& r) { return path(s(r), {"probe", "raf_callback_ms"}); });
        addValueRow(t, sets, "long tasks (> 50 ms)", true,
                    [s](const Json& r) { return path(s(r), {"probe", "long_tasks", "count"}); },
                    0);
        addSummaryRow(t, sets, "long task duration, ms", true, [s](const Json& r) {
            return path(s(r), {"probe", "long_tasks", "ms"});
        });
        addValueRow(t, sets, "main thread busy, % (CDP TaskDuration)", true,
                    [s](const Json& r) -> const Json* {
                        static Json tmp;
                        const Json* x = path(s(r), {"cdp_metrics", "task_s_per_s"});
                        if (x == nullptr) {
                            return nullptr;
                        }
                        tmp        = *x;
                        tmp.number = x->number * 100.0;
                        return &tmp;
                    },
                    0);
        addValueRow(t, sets, "WebSocket onmessage time, ms per s", true, [s](const Json& r) {
            return path(s(r), {"probe", "message_handler_total_ms_per_s"});
        });
        for (const char* kind : {"snapshot", "state", "diag", "binary"}) {
            const std::string k = kind;
            addValueRow(t, sets, "`" + k + "` messages per s", true, [s, k](const Json& r) {
                return path(s(r), {"probe", "messages", k.c_str(), "per_s"});
            });
            addSummaryRow(t, sets, "`" + k + "` onmessage handler, ms", true,
                          [s, k](const Json& r) {
                              return path(s(r), {"probe", "messages", k.c_str(), "handler_ms"});
                          },
                          2);
        }
        addSummaryRow(t, sets, "receive to next frame, ms", true, [s](const Json& r) {
            return path(s(r), {"probe", "receive_to_next_frame_ms"});
        });
        addSummaryRow(t, sets, "arrival lag vs host_ms, relative to best, ms (estimate)", true,
                      [s](const Json& r) { return path(s(r), {"probe", "relative_lag_ms"}); });
        addSummaryRow(t, sets, "input event ack, ms (orbit only)", true,
                      [s](const Json& r) { return path(s(r), {"input_ack_ms"}); });
        addValueRow(t, sets, "layouts per s", true,
                    [s](const Json& r) { return path(s(r), {"cdp_metrics", "layouts_per_s"}); });
        addValueRow(t, sets, "DOM mutation records per s", true, [s](const Json& r) {
            return path(s(r), {"probe", "dom_mutation_records_per_s"});
        });
        addValueRow(t, sets, "DOM nodes incl. detached (CDP Nodes, end)", true,
                    [s](const Json& r) { return path(s(r), {"cdp_metrics", "dom_nodes_end"}); },
                    0);
        addSummaryRow(t, sets, "JS heap used, bytes (sampled 2 Hz)", true,
                      [s](const Json& r) { return path(s(r), {"probe", "heap_bytes"}); }, 0);
        addValueRow(t, sets, "page WebSocket receive rate, kB/s", true,
                    [s](const Json& r) { return computed(wsBytesPerS(s(r)) / 1000.0); });
        const auto d = [s](const char* key) {
            return [s, key](const Json& r) -> const Json* {
                const Json* sc = s(r);
                if (sc == nullptr) {
                    return nullptr;
                }
                return computed(
                    numberAt(sc, {"server_after", "workers", "inspection", key}) -
                    numberAt(sc, {"server_before", "workers", "inspection", key}));
            };
        };
        addValueRow(t, sets,
                    "server: state messages not delivered during scenario (inspect/1: refused; "
                    "inspect/2: replaced unsent or refused)",
                    true, d("snapshots_skipped"), 0);
        addValueRow(t, sets, "server (inspect/2): messages refused during scenario", true,
                    d("messages_refused"), 0);
        t.print();
    }
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: navigatr_perf_report <dir>[=label] ...\n");
        return 2;
    }
    std::vector<Set> sets;
    for (int i = 1; i < argc; ++i) {
        std::string       arg = argv[i];
        Set               s;
        const std::size_t eq = arg.find('=');
        const std::string dir = eq == std::string::npos ? arg : arg.substr(0, eq);
        s.label               = eq == std::string::npos ? dir : arg.substr(eq + 1);
        for (int r = 1; r <= 50; ++r) {
            Json j;
            if (readJson(dir + "/server_r" + std::to_string(r) + ".json", j)) {
                s.server.push_back(j);
            }
            Json b;
            if (readJson(dir + "/browser_r" + std::to_string(r) + ".json", b)) {
                s.browser.push_back(b);
            }
        }
        std::printf("<!-- %s: %zu server runs, %zu browser runs -->\n", s.label.c_str(),
                    s.server.size(), s.browser.size());
        // which contract each part measured, so a mislabeled set shows
        std::string sc, bc;
        for (const Json& r : s.server) {
            const Json* c = r.at({"conditions", "contract"});
            const std::string v = c != nullptr ? c->str : "?";
            if (sc.find(v) == std::string::npos) {
                sc += (sc.empty() ? "" : ", ") + v;
            }
        }
        for (const Json& r : s.browser) {
            const Json* c = r.at({"conditions", "server_hello_start", "contract"});
            const std::string v = c != nullptr ? c->str : "not recorded";
            if (bc.find(v) == std::string::npos) {
                bc += (bc.empty() ? "" : ", ") + v;
            }
        }
        std::printf("- `%s`: server runs %zu (contract %s); browser runs %zu (server contract "
                    "%s)\n",
                    s.label.c_str(), s.server.size(), sc.empty() ? "-" : sc.c_str(),
                    s.browser.size(), bc.empty() ? "-" : bc.c_str());
        sets.push_back(std::move(s));
    }
    std::printf("\nCells: p50 / p95 / max, where p50 and p95 are medians over repeats and max "
                "is the largest seen in any repeat. Single values: median (min-max) over "
                "repeats.\n\n");
    serverTables(sets);
    browserTables(sets);
    return 0;
}
