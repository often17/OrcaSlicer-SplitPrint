#include "Scheduler.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>

#include "libslic3r/Exception.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/format.hpp"

namespace Slic3r::SplitPrint {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// Extra room a park leaves beyond the strict gap, so rounding never makes it touch.
constexpr double kParkMargin = 0.5;
// The sync the exporter writes before the end G-code.
const std::string kEndSync = "end";

// ---- One G-code line ---------------------------------------------------------------------

struct Line
{
    enum Kind { Other, Sync, Motion, Absolute, Relative, AbsoluteE, RelativeE, SetPosition, Home };
    Kind        kind = Other;
    std::string sync_id;
    bool        has[4] = {false, false, false, false}; // X Y Z E
    double      val[4] = {0., 0., 0., 0.};
    bool        arc    = false;
    double      i = 0., j = 0.;
};

Line parse_line(const std::string &raw)
{
    Line   out;
    size_t p = raw.find_first_not_of(" \t");
    if (p == std::string::npos)
        return out;
    if (raw[p] == ';') {
        const size_t s = raw.find("SPLITPRINT_SYNC", p);
        if (s != std::string::npos) {
            size_t b = raw.find_first_not_of(" \t", s + 15);
            size_t e = raw.find_last_not_of(" \t\r\n");
            if (b != std::string::npos && e != std::string::npos && e >= b) {
                out.kind    = Line::Sync;
                out.sync_id = raw.substr(b, e - b + 1);
            }
        }
        return out;
    }
    const std::string code = raw.substr(p, raw.find(';', p) == std::string::npos ? std::string::npos : raw.find(';', p) - p);
    size_t            q    = 0;
    while (q < code.size() && !std::isspace((unsigned char) code[q]))
        ++q;
    std::string cmd = code.substr(0, q);
    for (char &c : cmd)
        c = char(std::toupper((unsigned char) c));
    if (cmd == "G90") { out.kind = Line::Absolute; return out; }
    if (cmd == "G91") { out.kind = Line::Relative; return out; }
    if (cmd == "M82") { out.kind = Line::AbsoluteE; return out; }
    if (cmd == "M83") { out.kind = Line::RelativeE; return out; }
    if (cmd == "G28") { out.kind = Line::Home; return out; }
    const bool motion = cmd == "G0" || cmd == "G1" || cmd == "G2" || cmd == "G3";
    if (!motion && cmd != "G92")
        return out;
    out.kind = motion ? Line::Motion : Line::SetPosition;
    out.arc  = cmd == "G2" || cmd == "G3";
    // Words: a letter followed by a number.
    for (size_t k = q; k < code.size();) {
        const char letter = char(std::toupper((unsigned char) code[k]));
        if (!std::isalpha((unsigned char) letter)) {
            ++k;
            continue;
        }
        // G-code numbers always use a decimal point, whatever the user's locale says.
        const size_t sign = k + 1 < code.size() && code[k + 1] == '+' ? 1 : 0;
        size_t       len  = 0;
        const double v    = string_to_double_decimal_point(std::string_view(code).substr(k + 1 + sign), &len);
        if (len > 0)
            len += sign;
        if (len > 0) {
            const int idx = letter == 'X' ? 0 : letter == 'Y' ? 1 : letter == 'Z' ? 2 : letter == 'E' ? 3 : -1;
            if (idx >= 0) {
                out.has[idx] = true;
                out.val[idx] = v;
            } else if (letter == 'I')
                out.i = v;
            else if (letter == 'J')
                out.j = v;
        }
        k += 1 + len;
    }
    return out;
}

bool is_travel(const Line &l) { return l.kind == Line::Motion && (l.has[0] || l.has[1]) && !l.has[3]; }

// ---- Position tracking -------------------------------------------------------------------

struct Step
{
    double u_lo = kInf, u_hi = -kInf; // swept along the split axis by this line (start..end, arcs whole circle)
    double u_end = 0.;
    bool   moved     = false;
    bool   z_changed = false;
    bool   extruding = false;
    double z_end     = 0.;
};

struct Tracker
{
    int    ax       = 0; // 0 = X, 1 = Y
    double pos[4]   = {0., 0., 0., 0.};
    bool   absolute = true, absolute_e = true;

    Step apply(const Line &l)
    {
        Step s;
        switch (l.kind) {
        case Line::Absolute: absolute = absolute_e = true; break;
        case Line::Relative: absolute = absolute_e = false; break;
        case Line::AbsoluteE: absolute_e = true; break;
        case Line::RelativeE: absolute_e = false; break;
        case Line::SetPosition:
            for (int k = 0; k < 4; ++k)
                if (l.has[k])
                    pos[k] = l.val[k];
            break;
        case Line::Motion: {
            const double start_u = pos[ax], start_v = pos[1 - ax], start_z = pos[2];
            for (int k = 0; k < 3; ++k)
                if (l.has[k])
                    pos[k] = absolute ? l.val[k] : pos[k] + l.val[k];
            if (l.has[3]) {
                const double e = absolute_e ? l.val[3] : pos[3] + l.val[3];
                s.extruding    = e > pos[3] + 1e-9 && (l.has[0] || l.has[1]);
                pos[3]         = e;
            }
            s.moved = true;
            s.u_lo  = std::min(start_u, pos[ax]);
            s.u_hi  = std::max(start_u, pos[ax]);
            if (l.arc) {
                // Conservative: an arc stays inside its full circle.
                const double c = start_u + (ax == 0 ? l.i : l.j);
                const double r = std::hypot(l.i, l.j);
                s.u_lo         = std::min(s.u_lo, c - r);
                s.u_hi         = std::max(s.u_hi, c + r);
            }
            (void) start_v;
            s.z_changed = std::abs(pos[2] - start_z) > 1e-9;
            break;
        }
        default: break;
        }
        s.u_end = pos[ax];
        s.z_end = pos[2];
        return s;
    }
};

int axis_index(char axis) { return std::toupper((unsigned char) axis) == 'Y' ? 1 : 0; }

std::string fmt3(double v)
{
    return float_to_string_decimal_point(v, 3);
}

// ---- Per-stream analysis -----------------------------------------------------------------

// A stretch of one head's layer that starts right before a travel (or at the layer start):
// the smallest unit the scheduler moves between intervals.
struct Chunk
{
    size_t insert_at = 0;                  // syncs and parks for this chunk go before this line
    double time      = 0.;
    double u_lo = kInf, u_hi = -kInf;      // what the chunk's own moves sweep (start position excluded)
    double u_end     = 0.;
};

struct Layer
{
    size_t             sync_line = 0;      // the "L<n>" sync
    std::string        sync_id;
    std::optional<double> z;               // Z the layer prints at, if it changes Z
    std::vector<Chunk> chunks;
    size_t             end_line  = 0;      // one past the layer's last line
};

struct Analysis
{
    std::vector<Layer> layers;
    double             u_before_first = 0.; // where the head is when the first layer starts
    double             prologue_time  = 0.;
};

// `park_u`: where the head is sent right before the first layer (see schedule_streams).
Analysis analyse(const HeadStream &s, int ax, double park_u)
{
    Analysis a;
    Tracker  t;
    t.ax = ax;
    std::vector<size_t> syncs;
    std::vector<Line>   parsed(s.lines.size());
    for (size_t i = 0; i < s.lines.size(); ++i) {
        parsed[i] = parse_line(s.lines[i]);
        if (parsed[i].kind == Line::Sync)
            syncs.push_back(i);
    }
    auto time_of = [&s](size_t i) { return i < s.line_time.size() ? s.line_time[i] : 0.; };

    size_t i = 0;
    for (; i < (syncs.empty() ? s.lines.size() : syncs.front()); ++i) {
        t.apply(parsed[i]);
        a.prologue_time += time_of(i);
    }
    a.u_before_first = t.pos[ax];
    t.pos[ax]        = park_u;
    for (size_t n = 0; n < syncs.size(); ++n) {
        // After the "end" sync each head runs its own end G-code; nothing there is scheduled.
        if (parsed[syncs[n]].sync_id == kEndSync)
            break;
        Layer layer;
        layer.sync_line = syncs[n];
        layer.sync_id   = parsed[syncs[n]].sync_id;
        layer.end_line  = n + 1 < syncs.size() ? syncs[n + 1] : s.lines.size();
        Chunk cur;
        cur.insert_at = layer.sync_line + 1;
        bool first    = true;
        for (i = layer.sync_line + 1; i < layer.end_line; ++i) {
            const Line &l = parsed[i];
            if (is_travel(l) && !first) {
                layer.chunks.push_back(cur);
                cur           = Chunk();
                cur.insert_at = i;
            }
            const Step st = t.apply(l);
            if (st.moved) {
                first   = false;
                cur.u_lo = std::min(cur.u_lo, st.u_lo);
                cur.u_hi = std::max(cur.u_hi, st.u_hi);
            }
            if (st.z_changed && !layer.z)
                layer.z = st.z_end;
            cur.u_end = st.u_end;
            cur.time += time_of(i);
        }
        if (cur.u_lo == kInf)
            cur.u_end = t.pos[ax];
        layer.chunks.push_back(cur);
        a.layers.push_back(std::move(layer));
    }
    return a;
}

} // namespace

std::vector<size_t> sync_lines(const std::vector<std::string> &lines)
{
    std::vector<size_t> out;
    for (size_t i = 0; i < lines.size(); ++i)
        if (parse_line(lines[i]).kind == Line::Sync)
            out.push_back(i);
    return out;
}

std::string head_gcode_path(const std::string &path, int head)
{
    const std::string tag = ".T" + std::to_string(head);
    const size_t      dot = path.find_last_of('.');
    const size_t      sep = path.find_last_of("/\\");
    if (dot == std::string::npos || (sep != std::string::npos && dot < sep))
        return path + tag;
    return path.substr(0, dot) + tag + path.substr(dot);
}

ScheduleResult schedule_streams(const std::array<HeadStream, 2> &streams, const ScheduleParams &params)
{
    const int      ax = axis_index(params.axis);
    const Analysis a[2] = {analyse(streams[0], ax, params.bed_min), analyse(streams[1], ax, params.bed_max)};
    if (a[0].layers.size() != a[1].layers.size())
        throw Slic3r::SlicingError("Two heads, one object: the two heads' G-code has a different number of layers.");
    for (size_t n = 0; n < a[0].layers.size(); ++n)
        if (a[0].layers[n].sync_id != a[1].layers[n].sync_id)
            throw Slic3r::SlicingError("Two heads, one object: the two heads' G-code has different layer syncs.");

    // Lines to insert before line i of each stream, in order.
    std::array<std::map<size_t, std::vector<std::string>>, 2> inserts;
    ScheduleResult                                            out;
    const char                                                axis = char(std::toupper((unsigned char) params.axis));
    const std::string                                         park_feed = "F" + std::to_string(int(std::lround(params.travel_speed * 60.)));

    // Before the first layer each head goes to its own end of the bed, so its position is
    // known to the scheduler and to the checker.
    double pos[2]   = {params.bed_min, params.bed_max};
    double clock[2] = {a[0].prologue_time, a[1].prologue_time};
    for (int h = 0; h < 2; ++h) {
        if (a[h].layers.empty())
            continue;
        inserts[h][a[h].layers.front().sync_line].push_back(std::string("G1 ") + axis + fmt3(pos[h]) + " " + park_feed +
                                                            " ; SPLITPRINT park before the first layer");
        clock[h] += std::abs(pos[h] - a[h].u_before_first) / params.travel_speed;
    }

    for (size_t n = 0; n < a[0].layers.size(); ++n) {
        const Layer *L[2] = {&a[0].layers[n], &a[1].layers[n]};
        const std::string &id = L[0]->sync_id;
        // Layer sync: both heads meet.
        double t = std::max(clock[0], clock[1]);
        // The shared Z moves in its own interval, before either head extrudes on this layer.
        const std::optional<double> z = L[0]->z ? L[0]->z : L[1]->z;
        if (z) {
            for (int h = 0; h < 2; ++h) {
                auto &v = inserts[h][L[h]->sync_line + 1];
                v.push_back("G1 Z" + fmt3(*z) + " F" + std::to_string(int(std::lround(params.z_speed * 60.))) +
                            " ; SPLITPRINT shared Z");
                v.push_back("; SPLITPRINT_SYNC " + id + "z");
            }
            t += 0.2;
            ++out.syncs_added;
        }
        clock[0] = clock[1] = t;

        size_t idx[2] = {0, 0};
        const size_t cnt[2] = {L[0]->chunks.size(), L[1]->chunks.size()};
        double hi0 = pos[0], lo1 = pos[1];
        bool   fresh = true;
        int    sub   = 0;
        auto has = [&](int h) { return idx[h] < cnt[h]; };
        auto fits = [&](int h) {
            const Chunk &c = L[h]->chunks[idx[h]];
            return h == 0 ? std::max(hi0, c.u_hi) + params.gap <= lo1 : hi0 + params.gap <= std::min(lo1, c.u_lo);
        };
        // Whether the chunk would fit right after a sync, with both heads where they stand.
        auto fits_fresh = [&](int h) {
            const Chunk &c = L[h]->chunks[idx[h]];
            return h == 0 ? std::max(pos[0], c.u_hi) + params.gap <= pos[1] : pos[0] + params.gap <= std::min(pos[1], c.u_lo);
        };
        auto accept = [&](int h) {
            const Chunk &c = L[h]->chunks[idx[h]];
            if (h == 0)
                hi0 = std::max(hi0, c.u_hi);
            else
                lo1 = std::min(lo1, c.u_lo);
            if (c.u_lo != kInf)
                pos[h] = c.u_end;
            clock[h] += c.time;
            ++idx[h];
            fresh = false;
        };
        auto insert_line = [&](int h) { return has(h) ? L[h]->chunks[idx[h]].insert_at : L[h]->end_line; };
        auto sync = [&]() {
            const std::string marker = "; SPLITPRINT_SYNC " + id + "." + std::to_string(++sub);
            for (int h = 0; h < 2; ++h)
                inserts[h][insert_line(h)].push_back(marker);
            clock[0] = clock[1] = std::max(clock[0], clock[1]);
            hi0                 = pos[0];
            lo1                 = pos[1];
            fresh               = true;
            ++out.syncs_added;
        };
        // Where head y must go so the other head's next chunk fits.
        auto park_target = [&](int y) -> std::optional<double> {
            const int x = 1 - y;
            if (!has(x))
                return std::nullopt;
            const Chunk &c = L[x]->chunks[idx[x]];
            const double target = y == 1 ? std::max(pos[0], c.u_hi) + params.gap + kParkMargin
                                         : std::min(pos[1], c.u_lo) - params.gap - kParkMargin;
            if (target < params.bed_min - 1e-6 || target > params.bed_max + 1e-6)
                return std::nullopt;
            return target;
        };

        bool parked_last = false;
        while (has(0) || has(1)) {
            const int h = has(0) && (!has(1) || clock[0] <= clock[1]) ? 0 : 1;
            const int o = 1 - h;
            if (has(h) && fits(h)) {
                accept(h);
                parked_last = false;
                continue;
            }
            // The head that is behind is only blocked by where the other head went earlier in
            // this interval: sync now, or the other head races on and the two take turns.
            if (has(h) && !fresh && fits_fresh(h)) {
                sync();
                continue;
            }
            if (has(o) && fits(o)) {
                accept(o);
                parked_last = false;
                continue;
            }
            if (!fresh) {
                sync();
                continue;
            }
            // Neither head can go on from where both stand: one of them makes room.
            if (parked_last)
                throw Slic3r::SlicingError(
                    Slic3r::format("Two heads, one object: cannot keep the heads %1% mm apart on layer %2%.", params.gap, id));
            int y = !has(0) ? 0 : !has(1) ? 1 : (clock[1] >= clock[0] ? 1 : 0);
            std::optional<double> target = park_target(y);
            if (!target) {
                y      = 1 - y;
                target = park_target(y);
            }
            if (!target)
                throw Slic3r::SlicingError(Slic3r::format(
                    "Two heads, one object: on layer %1% the heads cannot get %2% mm apart without leaving the bed.", id, params.gap));
            inserts[y][insert_line(y)].push_back(std::string("G1 ") + axis + fmt3(*target) + " " + park_feed + " ; SPLITPRINT park");
            clock[y] += std::abs(*target - pos[y]) / params.travel_speed;
            pos[y] = *target;
            ++out.parks;
            fresh = false; // the park itself fills an interval
            sync();
            parked_last = true;
        }
    }
    out.print_time = std::max(clock[0], clock[1]);

    for (int h = 0; h < 2; ++h) {
        const std::vector<std::string> &src = streams[h].lines;
        std::vector<std::string>       &dst = out.lines[h];
        dst.reserve(src.size() + inserts[h].size() * 2);
        auto it = inserts[h].begin();
        for (size_t i = 0; i <= src.size(); ++i) {
            if (it != inserts[h].end() && it->first == i) {
                dst.insert(dst.end(), it->second.begin(), it->second.end());
                ++it;
            }
            if (i < src.size())
                dst.push_back(src[i]);
        }
    }
    return out;
}

std::vector<std::string> check_streams(const std::array<std::vector<std::string>, 2> &lines, const ScheduleParams &params)
{
    struct Interval
    {
        std::string id;
        double      lo = kInf, hi = -kInf;
        size_t      lo_line = 0, hi_line = 0;
        bool        extruded   = false;
        size_t      bad_z_line = 0;
        std::optional<double> z_after;
    };
    const int ax = axis_index(params.axis);
    std::vector<std::string> errors;
    std::array<std::vector<Interval>, 2> iv;
    for (int h = 0; h < 2; ++h) {
        Tracker t;
        t.ax = ax;
        Interval *cur = nullptr;
        for (size_t n = 0; n < lines[h].size(); ++n) {
            const Line l = parse_line(lines[h][n]);
            if (l.kind == Line::Sync) {
                iv[h].push_back(Interval{l.sync_id});
                if (l.sync_id == kEndSync)
                    break; // the end G-code, like the start G-code, is the machine's own business
                cur     = &iv[h].back();
                cur->lo = cur->hi = t.pos[ax];
                cur->lo_line = cur->hi_line = n + 1;
                continue;
            }
            if (l.kind == Line::Home && cur != nullptr) {
                errors.push_back(Slic3r::format("T%1% line %2%: G28 after the first sync cannot be checked", h, n + 1));
                continue;
            }
            const Step s = t.apply(l);
            if (cur == nullptr || !s.moved)
                continue;
            if (s.u_lo < cur->lo) { cur->lo = s.u_lo; cur->lo_line = n + 1; }
            if (s.u_hi > cur->hi) { cur->hi = s.u_hi; cur->hi_line = n + 1; }
            if (s.z_changed) {
                if (cur->extruded && cur->bad_z_line == 0)
                    cur->bad_z_line = n + 1;
                cur->z_after = s.z_end;
            }
            if (s.extruding)
                cur->extruded = true;
        }
    }
    if (iv[0].size() != iv[1].size()) {
        errors.push_back(Slic3r::format("the heads have different numbers of syncs (%1% and %2%)", iv[0].size(), iv[1].size()));
        return errors;
    }
    if (iv[0].empty()) {
        errors.push_back("no SPLITPRINT_SYNC markers: nothing can be checked");
        return errors;
    }
    for (size_t k = 0; k < iv[0].size(); ++k) {
        const Interval &a = iv[0][k], &b = iv[1][k];
        if (a.id != b.id) {
            errors.push_back(Slic3r::format("sync #%1% differs: T0 has %2%, T1 has %3%", k, a.id, b.id));
            return errors;
        }
        if (a.hi + params.gap > b.lo + 1e-6)
            errors.push_back(Slic3r::format("after sync %1%: gantries too close: T0 reaches %2% (line %3%), T1 reaches %4% (line %5%), need %6%",
                                            a.id, a.hi, a.hi_line, b.lo, b.lo_line, params.gap));
        for (const Interval *x : {&a, &b})
            if (x->bad_z_line)
                errors.push_back(Slic3r::format("after sync %1%: Z moves after extruding without a sync (T%2% line %3%)", a.id,
                                                x == &a ? 0 : 1, x->bad_z_line));
        if (a.z_after.has_value() != b.z_after.has_value() || (a.z_after && std::abs(*a.z_after - *b.z_after) > 1e-6))
            errors.push_back(Slic3r::format("after sync %1%: the heads disagree on Z", a.id));
    }
    return errors;
}

} // namespace Slic3r::SplitPrint
