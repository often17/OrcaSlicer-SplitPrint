#include "Scheduler.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include <string_view>

#include <tbb/parallel_for.h>
#include <tbb/parallel_invoke.h>

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
    enum Kind { Other, Sync, Timed, Motion, Absolute, Relative, AbsoluteE, RelativeE, SetPosition, Home };
    Kind        kind = Other;
    std::string sync_id;
    bool        has[4] = {false, false, false, false}; // X Y Z E
    double      val[4] = {0., 0., 0., 0.};
    bool        arc    = false;
    double      i = 0., j = 0.;
};

// True when `word` is `cmd` (upper case), ignoring the case of `word`.
bool is_command(std::string_view word, std::string_view cmd)
{
    if (word.size() != cmd.size())
        return false;
    for (size_t k = 0; k < word.size(); ++k)
        if (std::toupper((unsigned char) word[k]) != cmd[k])
            return false;
    return true;
}

Line parse_line(std::string_view raw)
{
    Line   out;
    size_t p = raw.find_first_not_of(" \t");
    if (p == std::string_view::npos)
        return out;
    if (raw[p] == ';') {
        if (raw.find("SPLITPRINT_TIMED", p) != std::string_view::npos) {
            out.kind = Line::Timed;
            return out;
        }
        const size_t s = raw.find("SPLITPRINT_SYNC", p);
        if (s != std::string_view::npos) {
            size_t b = raw.find_first_not_of(" \t", s + 15);
            size_t e = raw.find_last_not_of(" \t\r\n");
            if (b != std::string_view::npos && e != std::string_view::npos && e >= b) {
                out.kind    = Line::Sync;
                out.sync_id = std::string(raw.substr(b, e - b + 1));
            }
        }
        return out;
    }
    const std::string_view code = raw.substr(p, raw.find(';', p) == std::string_view::npos ? std::string_view::npos : raw.find(';', p) - p);
    size_t                 q    = 0;
    while (q < code.size() && !std::isspace((unsigned char) code[q]))
        ++q;
    const std::string_view cmd = code.substr(0, q);
    if (is_command(cmd, "G90")) { out.kind = Line::Absolute; return out; }
    if (is_command(cmd, "G91")) { out.kind = Line::Relative; return out; }
    if (is_command(cmd, "M82")) { out.kind = Line::AbsoluteE; return out; }
    if (is_command(cmd, "M83")) { out.kind = Line::RelativeE; return out; }
    if (is_command(cmd, "G28")) { out.kind = Line::Home; return out; }
    const bool arc    = is_command(cmd, "G2") || is_command(cmd, "G3");
    const bool motion = arc || is_command(cmd, "G0") || is_command(cmd, "G1");
    if (!motion && !is_command(cmd, "G92"))
        return out;
    out.kind = motion ? Line::Motion : Line::SetPosition;
    out.arc  = arc;
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
        const double v    = string_to_double_decimal_point(code.substr(k + 1 + sign), &len);
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
    double e_delta   = 0.;
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
                s.e_delta      = e - pos[3];
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

// One line of a layer, for timed planning.
struct LineSweep
{
    double dt    = 0.;
    double lo    = 0., hi = 0.; // swept along the split axis (where the head is, if it does not move)
    bool   moved = false;
};

struct Layer
{
    size_t             sync_line = 0;      // the "L<n>" sync
    std::string        sync_id;
    std::optional<double> z;               // Z the layer prints at, if it changes Z
    std::vector<Chunk> chunks;
    std::vector<LineSweep> body;           // every line after the sync, in order
    double             u_end     = 0.;     // where the head is at the end of the layer
    size_t             first_e_line = std::numeric_limits<size_t>::max(); // first line that moves the extruder
    bool               retracted_at_end = false; // the stream leaves the nozzle retracted
    bool               relative_e_at_end = false;
    size_t             end_line  = 0;      // one past the layer's last line
};

// ---- Reading a head's stream a layer at a time ------------------------------------------

// One head's stream while it is scheduled: what is read of it, and what its analysis carries
// from one layer to the next.
struct HeadPass
{
    const ReadLine            *read      = nullptr;
    const std::vector<double> *line_time = nullptr;
    size_t                     index     = 0; // input lines read
    Tracker                    t;
    bool                       retracted = false;
    // The sync that starts the next layer (or the end), already read.
    std::optional<std::string> sync;
    double                     sync_time = 0.;
    std::string                sync_id;
    // The layer being scheduled: its sync, then its body.
    std::vector<std::string> lines;
    std::vector<double>      times;

    bool next(std::string &line, double &seconds)
    {
        if (!(*read)(line))
            return false;
        seconds = line_time != nullptr && index < line_time->size() ? (*line_time)[index] : 0.;
        ++index;
        return true;
    }
    bool has_layer() const { return sync.has_value() && sync_id != kEndSync; }
};

// Reads the lines before the first sync, which go out unchanged. At the first sync the head is
// sent to `park_u` (see schedule_and_check()).
void read_prologue(HeadPass &p, int ax, double park_u, double &prologue_time, double &u_before_first, const WriteLine &write)
{
    std::string line;
    double      dt = 0.;
    while (p.next(line, dt)) {
        Line l = parse_line(line);
        if (l.kind == Line::Sync) {
            u_before_first = p.t.pos[ax];
            p.t.pos[ax]    = park_u;
            p.sync         = std::move(line);
            p.sync_time    = dt;
            p.sync_id      = std::move(l.sync_id);
            return;
        }
        p.t.apply(l);
        prologue_time += dt;
        write(line, dt);
    }
    u_before_first = p.t.pos[ax];
}

// Reads the layer that p.sync starts, up to the next sync, and analyses it. Line indices are
// into p.lines, where the layer's sync is line 0.
Layer read_layer(HeadPass &p, int ax)
{
    p.lines.clear();
    p.times.clear();
    p.lines.push_back(std::move(*p.sync));
    p.times.push_back(p.sync_time);
    p.sync.reset();
    Layer layer;
    layer.sync_line = 0;
    layer.sync_id   = std::move(p.sync_id);
    Tracker &t      = p.t;
    Chunk    cur;
    cur.insert_at = 1;
    bool        first = true;
    std::string line;
    double      dt = 0.;
    while (p.next(line, dt)) {
        Line l = parse_line(line);
        if (l.kind == Line::Sync) {
            p.sync      = std::move(line);
            p.sync_time = dt;
            p.sync_id   = std::move(l.sync_id);
            break;
        }
        const size_t i = p.lines.size();
        p.lines.push_back(std::move(line));
        p.times.push_back(dt);
        if (is_travel(l) && !first) {
            layer.chunks.push_back(cur);
            cur           = Chunk();
            cur.insert_at = i;
        }
        const Step st = t.apply(l);
        if (l.kind == Line::Motion && l.has[3]) {
            layer.first_e_line = std::min(layer.first_e_line, i);
            if (st.e_delta < -1e-9)
                p.retracted = true;
            else if (st.e_delta > 1e-9)
                p.retracted = false;
        }
        if (st.moved) {
            first    = false;
            cur.u_lo = std::min(cur.u_lo, st.u_lo);
            cur.u_hi = std::max(cur.u_hi, st.u_hi);
            layer.body.push_back({dt, st.u_lo, st.u_hi, true});
        } else
            layer.body.push_back({dt, t.pos[ax], t.pos[ax], false});
        if (st.z_changed && !layer.z)
            layer.z = st.z_end;
        cur.u_end = st.u_end;
        cur.time += dt;
    }
    layer.end_line = p.lines.size();
    if (cur.u_lo == kInf)
        cur.u_end = t.pos[ax];
    layer.chunks.push_back(cur);
    layer.u_end             = t.pos[ax];
    layer.retracted_at_end  = p.retracted;
    layer.relative_e_at_end = !t.absolute_e;
    return layer;
}

// ---- Timed intervals ---------------------------------------------------------------------

// From t0 to t1 (seconds since the interval started) a head sweeps [lo, hi] along the split axis.
struct TimedSweep
{
    double t0 = 0., t1 = 0., lo = 0., hi = 0.;
};

constexpr double kTimeBin = 0.05; // s, time resolution of the timed check
// Extra slack the planner keeps over what the checks demand. The plan leaves out the few
// hundredths of a second the retractions it adds take, which check_streams counts; and
// tools/splitprint/check.py times the moves with its own model of Klipper, which comes out a
// few percent apart from the G-code processor's.
constexpr double kPlanSlack    = 0.1;  // s
constexpr double kPlanSlackRel = 0.05; // share of the time since the interval started

// First time at which the heads could come closer than `gap` if either runs up to
// margin_abs + margin_rel * t ahead of the other; nullopt if they never do. After its last
// sweep each head rests at rest[h] until the interval ends.
std::optional<double> timed_conflict(const std::array<std::vector<TimedSweep>, 2> &tl, const std::array<double, 2> &rest,
                                     double gap, double margin_abs, double margin_rel)
{
    double end_h[2] = {0., 0.};
    for (int h = 0; h < 2; ++h)
        for (const TimedSweep &s : tl[h])
            end_h[h] = std::max(end_h[h], s.t1);
    const double end   = std::max(end_h[0], end_h[1]);
    const size_t n     = size_t(std::ceil((end + margin_abs + margin_rel * end) / kTimeBin)) + 2;
    auto         bin   = [n](double t) { return std::min(n - 1, size_t(std::max(0., t) / kTimeBin)); };
    std::vector<double> hi0(n, -kInf), lo1(n, kInf);
    for (int h = 0; h < 2; ++h) {
        auto put = [&](size_t k, double lo, double hi) {
            if (h == 0)
                hi0[k] = std::max(hi0[k], hi);
            else
                lo1[k] = std::min(lo1[k], lo);
        };
        for (const TimedSweep &s : tl[h])
            for (size_t k = bin(s.t0), b = bin(std::max(s.t0, s.t1 - 1e-9)); k <= b; ++k)
                put(k, s.lo, s.hi);
        for (size_t k = bin(end_h[h]); k < n; ++k)
            put(k, rest[h], rest[h]);
    }
    // The furthest T0 reaches in the bins [k - r, k + r]: a sliding-window maximum, as both
    // ends of the window only move forward while k grows (r grows by margin_rel <= 1 per bin).
    std::vector<size_t> window(n); // bins by falling hi0, a deque in [head, tail)
    size_t              head = 0, tail = 0, right = 0;
    for (size_t k = 0; k < n; ++k) {
        const size_t r  = size_t(std::ceil((margin_abs + margin_rel * double(k + 1) * kTimeBin) / kTimeBin));
        const size_t lo = k > r ? k - r : 0, hi = std::min(n - 1, k + r);
        for (; right <= hi; ++right) {
            while (tail > head && hi0[window[tail - 1]] <= hi0[right])
                --tail;
            window[tail++] = right;
        }
        while (window[head] < lo)
            ++head;
        if (hi0[window[head]] + gap > lo1[k] + 1e-6)
            return double(k) * kTimeBin;
    }
    return std::nullopt;
}

// How a head starts a timed layer: first a travel to `park`, if set, then a wait. And whether
// it travels to `park_end` once its work on the layer is done, to make room for the other head.
struct TimedStart
{
    double                delay = 0.;
    std::optional<double> park;
    std::optional<double> park_end;
};

// A head's timeline over a timed layer that it starts at `start`; `busy` is when it is done and
// `rest` where it then stays.
std::vector<TimedSweep> timed_layer(const Layer &layer, double start, const TimedStart &how, double travel_speed, double &busy,
                                    double &rest)
{
    std::vector<TimedSweep> out;
    out.reserve(layer.body.size() + 2);
    double t = 0., cur = start;
    if (how.park) {
        const double dt = std::abs(*how.park - start) / travel_speed;
        out.push_back({t, t + dt, std::min(start, *how.park), std::max(start, *how.park)});
        t += dt;
        cur = *how.park;
    }
    if (how.delay > 0.) {
        out.push_back({t, t + how.delay, cur, cur});
        t += how.delay;
    }
    // Until its first move the head is where it actually is, which the stream alone cannot know.
    bool moved = false;
    for (const LineSweep &s : layer.body) {
        double lo = s.lo, hi = s.hi;
        if (!moved) {
            if (s.moved) {
                lo    = std::min(lo, cur);
                hi    = std::max(hi, cur);
                moved = true;
            } else
                lo = hi = cur;
        }
        out.push_back({t, t + s.dt, lo, hi});
        t += s.dt;
    }
    rest = moved ? layer.u_end : cur;
    if (how.park_end) {
        const double dt = std::abs(*how.park_end - rest) / travel_speed;
        out.push_back({t, t + dt, std::min(rest, *how.park_end), std::max(rest, *how.park_end)});
        t += dt;
        rest = *how.park_end;
    }
    busy = t;
    return out;
}

// Tries to print layer L as one timed interval, both heads starting at pos[] at clock[] (equal).
// The head that yields may first travel out of the way and wait; the other may travel out of
// the way when it is done. Emits the lines through emit(head, at_end, line, seconds), at the
// start of the interval or at its end, and moves pos[] and clock[] to the layer end.
template<class Emit, class ParkLine>
bool plan_timed_layer(const Layer *const *L, double *pos, const ScheduleParams &params, Emit &&emit, ParkLine &&park_line, double *clock)
{
    // Where a head is clear of everything the other head does on this layer.
    double hi0 = pos[0], lo1 = pos[1];
    for (const LineSweep &s : L[0]->body)
        hi0 = std::max(hi0, s.hi);
    for (const LineSweep &s : L[1]->body)
        lo1 = std::min(lo1, s.lo);
    std::optional<double> clear[2];
    if (const double p = lo1 - params.gap - kParkMargin; p >= params.bed_min - 1e-6)
        clear[0] = p;
    if (const double p = hi0 + params.gap + kParkMargin; p <= params.bed_max + 1e-6)
        clear[1] = p;
    // A park at the start only helps when it takes the head outwards.
    std::optional<double> park_at[2];
    if (clear[0] && *clear[0] < pos[0])
        park_at[0] = clear[0];
    if (clear[1] && *clear[1] > pos[1])
        park_at[1] = clear[1];

    std::array<std::vector<TimedSweep>, 2> tl;
    std::array<double, 2>                  busy{}, rest{};
    std::array<TimedStart, 2>              best;
    std::array<double, 2>                  best_busy{}, best_rest{};
    double                                 best_end = kInf;
    auto try_plan = [&](const std::array<TimedStart, 2> &how) {
        for (int h = 0; h < 2; ++h)
            tl[h] = timed_layer(*L[h], pos[h], how[h], params.travel_speed, busy[h], rest[h]);
        if (timed_conflict(tl, rest, params.gap, params.margin_abs + kPlanSlack, params.margin_rel + kPlanSlackRel))
            return false;
        if (const double end = std::max(busy[0], busy[1]); end < best_end) {
            best_end  = end;
            best      = how;
            best_busy = busy;
            best_rest = rest;
        }
        return true;
    };

    // Waits to try: fine steps first, then coarser, up to one head after the other.
    double nominal = 0.;
    for (int h = 0; h < 2; ++h) {
        timed_layer(*L[h], pos[h], {}, params.travel_speed, busy[h], rest[h]);
        nominal += busy[h];
    }
    const double limit = nominal * (1. + params.margin_rel) + params.margin_abs + 2.;
    for (double d = 0.; d <= limit; d = d < 10. ? d + 0.25 : d * 1.15) {
        const double delay = std::round(d * 1000.) / 1000.;
        bool         found = false;
        for (int y = 0; y < 2; ++y)
            for (const bool park : {false, true})
                for (const bool park_end : {false, true}) {
                    // y yields: it may park first and waits; the other head may make room when done.
                    if ((delay == 0. && !park && !park_end && y == 1) || (park && !park_at[y]) || (park_end && !clear[1 - y]))
                        continue;
                    std::array<TimedStart, 2> how;
                    how[y].delay = delay;
                    if (park)
                        how[y].park = park_at[y];
                    if (park_end)
                        how[1 - y].park_end = clear[1 - y];
                    found |= try_plan(how);
                }
        if (found)
            break;
    }
    if (best_end == kInf)
        return false;

    for (int h = 0; h < 2; ++h) {
        emit(h, false, "; SPLITPRINT_TIMED " + fmt3(best_busy[h]), 0.);
        if (best[h].park)
            emit(h, false, park_line(*best[h].park, " ; SPLITPRINT park"), std::abs(*best[h].park - pos[h]) / params.travel_speed);
        if (best[h].delay > 0.)
            emit(h, false, "G4 P" + std::to_string(std::lround(best[h].delay * 1000.)) + " ; SPLITPRINT wait at the layer start", best[h].delay);
        if (best[h].park_end) {
            double busy_h = 0., rest_h = 0.;
            TimedStart plain = best[h];
            plain.park_end.reset();
            timed_layer(*L[h], pos[h], plain, params.travel_speed, busy_h, rest_h);
            emit(h, true, park_line(*best[h].park_end, " ; SPLITPRINT make room"), std::abs(*best[h].park_end - rest_h) / params.travel_speed);
        }
        pos[h] = best_rest[h];
        clock[h] += best_busy[h];
    }
    return true;
}

// ---- The check, a line at a time ---------------------------------------------------------

// What check_streams() checks, fed one line at a time per head. The two heads may be fed from
// different threads; compare() and finish() run when neither is.
class StreamChecker
{
public:
    StreamChecker(const ScheduleParams &params, bool have_times) : m_params(params), m_have_times(have_times)
    {
        for (Head &head : m_heads)
            head.t.ax = axis_index(params.axis);
    }

    void feed(int h, const std::string &line, double dt)
    {
        Head        &s = m_heads[h];
        const size_t n = s.lines++;
        if (s.ended)
            return; // the end G-code, like the start G-code, is the machine's own business
        const int  ax = s.t.ax;
        const Line l  = parse_line(line);
        if (l.kind == Line::Sync) {
            if (s.cur) {
                s.done.push_back(std::move(*s.cur));
                s.cur.reset();
            }
            ++s.intervals;
            if (l.sync_id == kEndSync) {
                s.done.push_back(Interval{l.sync_id});
                s.ended = true;
                return;
            }
            Interval iv{l.sync_id};
            iv.lo = iv.hi = iv.rest = s.t.pos[ax];
            iv.lo_line = iv.hi_line = n + 1;
            s.cur = std::move(iv);
            return;
        }
        if (l.kind == Line::Timed && s.cur) {
            s.cur->timed = true;
            return;
        }
        if (l.kind == Line::Home && s.cur) {
            s.errors.push_back(Slic3r::format("T%1% line %2%: G28 after the first sync cannot be checked", h, n + 1));
            return;
        }
        const double start = s.t.pos[ax];
        const Step   st    = s.t.apply(l);
        if (!s.cur)
            return;
        Interval *cur = &*s.cur;
        cur->sweeps.push_back({cur->clock, cur->clock + dt, st.moved ? st.u_lo : start, st.moved ? st.u_hi : start});
        cur->clock += dt;
        cur->rest = s.t.pos[ax];
        if (!st.moved)
            return;
        if (st.u_lo < cur->lo) { cur->lo = st.u_lo; cur->lo_line = n + 1; }
        if (st.u_hi > cur->hi) { cur->hi = st.u_hi; cur->hi_line = n + 1; }
        if (st.z_changed) {
            if (cur->extruded && cur->bad_z_line == 0)
                cur->bad_z_line = n + 1;
            cur->z_after = st.z_end;
        }
        if (st.extruding)
            cur->extruded = true;
    }

    // Compares the intervals both heads have finished.
    void compare()
    {
        while (!m_stopped && !m_heads[0].done.empty() && !m_heads[1].done.empty()) {
            compare_pair(m_heads[0].done.front(), m_heads[1].done.front(), m_compared++);
            m_heads[0].done.pop_front();
            m_heads[1].done.pop_front();
        }
    }

    // All violations: each head's own, then those between the heads.
    std::vector<std::string> finish()
    {
        for (Head &s : m_heads)
            if (s.cur) {
                s.done.push_back(std::move(*s.cur));
                s.cur.reset();
            }
        std::vector<std::string> errors = std::move(m_heads[0].errors);
        errors.insert(errors.end(), std::make_move_iterator(m_heads[1].errors.begin()), std::make_move_iterator(m_heads[1].errors.end()));
        if (m_heads[0].intervals != m_heads[1].intervals) {
            errors.insert(errors.end(), m_errors.begin(), m_errors.end());
            errors.push_back(Slic3r::format("the heads have different numbers of syncs (%1% and %2%)", m_heads[0].intervals, m_heads[1].intervals));
            return errors;
        }
        if (m_heads[0].intervals == 0) {
            errors.push_back("no SPLITPRINT_SYNC markers: nothing can be checked");
            return errors;
        }
        compare();
        errors.insert(errors.end(), m_errors.begin(), m_errors.end());
        return errors;
    }

private:
    struct Interval
    {
        std::string id;
        double      lo = kInf, hi = -kInf;
        size_t      lo_line = 0, hi_line = 0;
        bool        extruded   = false;
        size_t      bad_z_line = 0;
        std::optional<double> z_after;
        bool        timed = false;
        std::vector<TimedSweep> sweeps; // with times, for a timed interval
        double      clock = 0.;
        double      rest  = 0.;         // where the head is at the end
    };
    struct Head
    {
        Tracker                  t;
        std::optional<Interval>  cur;
        std::deque<Interval>     done;      // finished, not compared yet
        size_t                   lines     = 0;
        size_t                   intervals = 0;
        bool                     ended     = false;
        std::vector<std::string> errors;
    };

    void compare_pair(const Interval &a, const Interval &b, size_t k)
    {
        const ScheduleParams &params = m_params;
        if (a.id != b.id) {
            m_errors.push_back(Slic3r::format("sync #%1% differs: T0 has %2%, T1 has %3%", k, a.id, b.id));
            m_stopped = true;
            return;
        }
        if (a.timed != b.timed)
            m_errors.push_back(Slic3r::format("after sync %1%: only one head's interval is timed", a.id));
        else if (a.timed) {
            // Both heads start together; they must stay apart even when one runs ahead.
            if (!m_have_times)
                m_errors.push_back(Slic3r::format("after sync %1%: a timed interval cannot be checked without move times", a.id));
            else if (const std::optional<double> t = timed_conflict({a.sweeps, b.sweeps}, {a.rest, b.rest}, params.gap, params.margin_abs,
                                                                    params.margin_rel))
                m_errors.push_back(Slic3r::format("after sync %1%: gantries may come closer than %2% mm about %3% s into the interval",
                                                  a.id, params.gap, *t));
        } else if (a.hi + params.gap > b.lo + 1e-6)
            m_errors.push_back(Slic3r::format("after sync %1%: gantries too close: T0 reaches %2% (line %3%), T1 reaches %4% (line %5%), need %6%",
                                              a.id, a.hi, a.hi_line, b.lo, b.lo_line, params.gap));
        for (const Interval *x : {&a, &b})
            if (x->bad_z_line)
                m_errors.push_back(Slic3r::format("after sync %1%: Z moves after extruding without a sync (T%2% line %3%)", a.id,
                                                  x == &a ? 0 : 1, x->bad_z_line));
        if (a.z_after.has_value() != b.z_after.has_value() || (a.z_after && std::abs(*a.z_after - *b.z_after) > 1e-6))
            m_errors.push_back(Slic3r::format("after sync %1%: the heads disagree on Z", a.id));
    }

    const ScheduleParams    &m_params;
    bool                     m_have_times;
    std::array<Head, 2>      m_heads;
    size_t                   m_compared = 0;
    bool                     m_stopped  = false;
    std::vector<std::string> m_errors; // between the heads, in interval order
};

// Schedules both streams a layer at a time; with `check`, the written lines go through the
// checker too.
StreamResult run_schedule(const std::array<ReadLine, 2> &read, const std::array<const std::vector<double> *, 2> &line_time,
                          const std::array<WriteLine, 2> &write, const ScheduleParams &params, bool check,
                          const std::function<void()> &each_layer)
{
    const int                    ax = axis_index(params.axis);
    StreamResult                 out;
    std::optional<StreamChecker> checker;
    if (check)
        checker.emplace(params, true);
    auto put = [&](int h, std::string &line, double dt) {
        if (checker)
            checker->feed(h, line, dt);
        write[h](line, dt);
    };
    const std::array<WriteLine, 2> put_head{[&](std::string &line, double dt) { put(0, line, dt); },
                                            [&](std::string &line, double dt) { put(1, line, dt); }};

    HeadPass p[2];
    double   prologue_time[2] = {0., 0.}, u_before_first[2] = {0., 0.};
    for (int h = 0; h < 2; ++h) {
        p[h].read      = &read[h];
        p[h].line_time = line_time[h];
        p[h].t.ax      = ax;
    }
    // Before the first layer each head goes to its own end of the bed, so its position is
    // known to the scheduler and to the checker.
    tbb::parallel_invoke([&] { read_prologue(p[0], ax, params.bed_min, prologue_time[0], u_before_first[0], put_head[0]); },
                         [&] { read_prologue(p[1], ax, params.bed_max, prologue_time[1], u_before_first[1], put_head[1]); });
    auto same_layer_count = [&] {
        if (p[0].has_layer() != p[1].has_layer())
            throw Slic3r::SlicingError("Two heads, one object: the two heads' G-code has a different number of layers.");
    };
    same_layer_count();

    const char        axis      = char(std::toupper((unsigned char) params.axis));
    const std::string park_feed = "F" + std::to_string(int(std::lround(params.travel_speed * 60.)));
    auto park_line = [&](double target, const char *what) { return std::string("G1 ") + axis + fmt3(target) + " " + park_feed + what; };

    double pos[2]   = {params.bed_min, params.bed_max};
    double clock[2] = {prologue_time[0], prologue_time[1]};
    bool   unretract_pending[2] = {false, false};
    if (p[0].has_layer())
        for (int h = 0; h < 2; ++h) {
            const double dt = std::abs(pos[h] - u_before_first[h]) / params.travel_speed;
            if (params.max_velocity > 0. && params.square_corner_velocity > 0.) {
                std::string limits = "SET_VELOCITY_LIMIT VELOCITY=" + fmt3(params.max_velocity) + " SQUARE_CORNER_VELOCITY=" +
                                     fmt3(params.square_corner_velocity) + " ; SPLITPRINT the limits the move times were estimated with";
                put(h, limits, 0.);
            }
            std::string park = park_line(pos[h], " ; SPLITPRINT park before the first layer");
            put(h, park, dt);
            clock[h] += dt;
        }

    // Lines to insert before line i of each head's layer, in order, with their estimated time.
    std::array<std::map<size_t, std::vector<std::pair<std::string, double>>>, 2> inserts;
    while (p[0].has_layer()) {
        if (each_layer)
            each_layer();
        if (p[0].sync_id != p[1].sync_id)
            throw Slic3r::SlicingError("Two heads, one object: the two heads' G-code has different layer syncs.");
        Layer layers[2];
        tbb::parallel_invoke([&] { layers[0] = read_layer(p[0], ax); }, [&] { layers[1] = read_layer(p[1], ax); });
        const Layer       *L[2] = {&layers[0], &layers[1]};
        const std::string &id   = L[0]->sync_id;
        inserts[0].clear();
        inserts[1].clear();
        // Layer sync: both heads meet.
        double t = std::max(clock[0], clock[1]);
        // The shared Z moves in its own interval, before either head extrudes on this layer.
        const std::optional<double> z = L[0]->z ? L[0]->z : L[1]->z;
        if (z) {
            for (int h = 0; h < 2; ++h) {
                auto &v = inserts[h][L[h]->sync_line + 1];
                v.push_back({"G1 Z" + fmt3(*z) + " F" + std::to_string(int(std::lround(params.z_speed * 60.))) + " ; SPLITPRINT shared Z", 0.2});
                v.push_back({"; SPLITPRINT_SYNC " + id + "z", 0.});
            }
            t += 0.2;
            ++out.syncs_added;
        }
        clock[0] = clock[1] = t;

        // Heads wait at layer changes: retract at the end of each layer, and undo it right before
        // the next layer first moves the extruder, whatever the stream does there.
        for (int h = 0; h < 2; ++h) {
            const double r = params.retract_length[h];
            if (unretract_pending[h] && L[h]->first_e_line != std::numeric_limits<size_t>::max()) {
                inserts[h][L[h]->first_e_line].push_back({"G1 E" + fmt3(r) + " F" + std::to_string(std::lround(params.deretract_speed[h] * 60.)) +
                                                          " ; SPLITPRINT unretract", r / params.deretract_speed[h]});
                unretract_pending[h] = false;
            }
            if (r > 0. && !unretract_pending[h] && !L[h]->retracted_at_end && L[h]->relative_e_at_end) {
                inserts[h][L[h]->end_line].push_back({"G1 E" + fmt3(-r) + " F" + std::to_string(std::lround(params.retract_speed[h] * 60.)) +
                                                      " ; SPLITPRINT retract for the layer change", r / params.retract_speed[h]});
                unretract_pending[h] = true;
            }
        }

        // Without a timed plan the heads take turns between syncs, parking when both want the middle.
        auto plan_synced_layer = [&] {
            const size_t stops_before = out.syncs_added + out.parks;
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
                    inserts[h][insert_line(h)].push_back({marker, 0.});
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
                const double dt = std::abs(*target - pos[y]) / params.travel_speed;
                inserts[y][insert_line(y)].push_back({park_line(*target, " ; SPLITPRINT park"), dt});
                clock[y] += dt;
                pos[y] = *target;
                ++out.parks;
                fresh = false; // the park itself fills an interval
                sync();
                parked_last = true;
            }
            if (out.syncs_added + out.parks > stops_before)
                ++out.stopped_layers;
        };
        if (params.timed && plan_timed_layer(L, pos, params, [&](int h, bool at_end, std::string line, double dt) {
                inserts[h][at_end ? L[h]->end_line : L[h]->sync_line + 1].push_back({std::move(line), dt});
            }, park_line, clock))
            ++out.timed_layers;
        else
            plan_synced_layer();

        // Each head's layer goes out with what the plan inserts.
        auto emit = [&](int h) {
            auto it = inserts[h].begin();
            for (size_t i = 0; i <= p[h].lines.size(); ++i) {
                if (it != inserts[h].end() && it->first == i) {
                    for (auto &[line, dt] : it->second)
                        put(h, line, dt);
                    ++it;
                }
                if (i < p[h].lines.size())
                    put(h, p[h].lines[i], p[h].times[i]);
            }
        };
        tbb::parallel_invoke([&] { emit(0); }, [&] { emit(1); });
        if (checker)
            checker->compare();
        same_layer_count();
    }
    out.print_time = std::max(clock[0], clock[1]);

    // The end sync and the end G-code go out unchanged.
    auto rest = [&](int h) {
        if (p[h].sync)
            put(h, *p[h].sync, p[h].sync_time);
        std::string line;
        double      dt = 0.;
        while (p[h].next(line, dt))
            put(h, line, dt);
    };
    tbb::parallel_invoke([&] { rest(0); }, [&] { rest(1); });
    if (checker)
        out.errors = checker->finish();
    return out;
}

} // namespace

bool is_sync_line(std::string_view line)
{
    const size_t p = line.find_first_not_of(" \t");
    return p != std::string_view::npos && line[p] == ';' && parse_line(line).kind == Line::Sync;
}

std::vector<size_t> sync_lines(const std::vector<std::string> &lines)
{
    std::vector<size_t> out;
    for (size_t i = 0; i < lines.size(); ++i)
        if (is_sync_line(lines[i]))
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

ScheduleResult schedule_streams(std::array<HeadStream, 2> streams, const ScheduleParams &params)
{
    ScheduleResult           out;
    std::array<size_t, 2>    next{0, 0};
    std::array<ReadLine, 2>  read;
    std::array<WriteLine, 2> write;
    for (int h = 0; h < 2; ++h) {
        out.lines[h].reserve(streams[h].lines.size());
        out.line_time[h].reserve(streams[h].lines.size());
        read[h] = [&streams, &next, h](std::string &line) {
            if (next[h] >= streams[h].lines.size())
                return false;
            line = std::move(streams[h].lines[next[h]++]);
            return true;
        };
        write[h] = [&out, h](std::string &line, double dt) {
            out.lines[h].push_back(std::move(line));
            out.line_time[h].push_back(dt);
        };
    }
    const StreamResult r = run_schedule(read, {&streams[0].line_time, &streams[1].line_time}, write, params, false, {});
    out.syncs_added    = r.syncs_added;
    out.parks          = r.parks;
    out.timed_layers   = r.timed_layers;
    out.stopped_layers = r.stopped_layers;
    out.print_time     = r.print_time;
    return out;
}

StreamResult schedule_and_check(const std::array<ReadLine, 2> &read, const std::array<const std::vector<double> *, 2> &line_time,
                                const std::array<WriteLine, 2> &write, const ScheduleParams &params,
                                const std::function<void()> &each_layer)
{
    return run_schedule(read, line_time, write, params, true, each_layer);
}

std::vector<std::string> check_streams(const std::array<std::vector<std::string>, 2> &lines, const ScheduleParams &params,
                                       const std::array<std::vector<double>, 2> *line_time)
{
    StreamChecker checker(params, line_time != nullptr);
    for (int h = 0; h < 2; ++h)
        for (size_t n = 0; n < lines[h].size(); ++n)
            checker.feed(h, lines[h][n], line_time != nullptr && n < (*line_time)[h].size() ? (*line_time)[h][n] : 0.);
    return checker.finish();
}

} // namespace Slic3r::SplitPrint
