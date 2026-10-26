#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

// Turns the two per-head G-code streams of a split print into streams that are safe to run
// at the same time on two gantries that can never pass each other.
//
// A stream is split into intervals by "; SPLITPRINT_SYNC <id>" lines. A sync is a barrier:
// the host coordinator lets neither head past it until both have reached it. Inside an
// interval the heads run with no timing guarantee, so safety is a property of what each head
// sweeps in that interval: max(T0) + gap <= min(T1) along the split axis.
//
// The exporter writes one sync per layer ("L<n>"). The scheduler adds:
//  - an explicit Z move and a sync after every layer sync, so the shared Z moves while
//    nobody extrudes ("L<n>z");
//  - syncs where a head's next stretch of work would come within `gap` of what the other
//    head sweeps ("L<n>.<k>");
//  - park moves when both heads want the middle: the head that yields travels away along
//    the split axis before the other goes on.
// Syncs and parks go only right before travel moves, where the head is retracted.
//
// Timed layers (ScheduleParams::timed): every sync stops both heads, and a head standing still
// over the part leaves a mark. So the scheduler first tries to print each layer as one interval
// with no sync inside it, keeping the heads apart by time instead: both heads start the layer
// together, and with the estimated time of every line, the plan must hold even when one head
// runs up to `margin_abs + margin_rel * t` seconds ahead of the other, t being the time since
// the layer started. Such an interval starts with "; SPLITPRINT_TIMED <seconds>", the time
// this head is busy in it. When the heads would meet, the head that yields waits at the start
// of the layer (G4), after first travelling out of the way if needed: that wait is part of the
// layer change, never in the middle of the layer. Only when no wait makes the plan hold does
// the layer fall back to syncs.
namespace Slic3r::SplitPrint {

struct ScheduleParams
{
    char   axis         = 'X';   // split axis letter in the G-code
    double gap          = 60.;   // minimum nozzle distance along the axis, mm
    double bed_min      = 0.;    // reachable range along the axis, mm
    double bed_max      = 0.;
    double travel_speed = 200.;  // mm/s, for parks
    double z_speed      = 10.;   // mm/s, for the layer Z moves
    bool   timed        = false; // try timed layers first
    double margin_abs   = 0.5;   // s, timing slack at the start of a timed interval
    double margin_rel   = 0.1;   // share of the time since the interval started
    // Each head retracts at the end of a layer, for the wait at the layer change (0 = never).
    std::array<double, 2> retract_length{0., 0.};   // mm
    std::array<double, 2> retract_speed{30., 30.};  // mm/s
    std::array<double, 2> deretract_speed{30., 30.};
    // Limits the move times were estimated with; written to each head so the printer uses them
    // too (0 = not written).
    double max_velocity           = 0.; // mm/s
    double square_corner_velocity = 0.; // mm/s
};

struct HeadStream
{
    std::vector<std::string> lines;
    std::vector<double>      line_time; // seconds each line takes, same size as `lines` (may be shorter: missing = 0)
};

struct ScheduleResult
{
    std::array<std::vector<std::string>, 2> lines;
    std::array<std::vector<double>, 2>      line_time;  // estimated seconds of each output line
    size_t                                  syncs_added = 0;
    size_t                                  parks       = 0;
    size_t                                  timed_layers   = 0; // layers printed without a stop
    size_t                                  stopped_layers = 0; // layers that needed syncs inside them
    double                                  print_time  = 0.; // estimated, both heads running
};

// Throws Slic3r::SlicingError when the heads cannot be kept apart (a part that needs a head
// beyond the bed to make room) or when the streams don't share the same layer syncs.
// The streams are taken by value: their lines move into the result.
ScheduleResult schedule_streams(std::array<HeadStream, 2> streams, const ScheduleParams &params);

// Independent check of finished streams, the C++ twin of tools/splitprint/check.py.
// Timed intervals are checked against `line_time` (seconds per line); without it they fail.
// Returns one message per violation; empty = safe.
std::vector<std::string> check_streams(const std::array<std::vector<std::string>, 2> &lines, const ScheduleParams &params,
                                       const std::array<std::vector<double>, 2> *line_time = nullptr);

// The same scheduling and check, a layer at a time, so a print of any size needs little memory.
// Reads each head's next line into `line`; false at the end.
using ReadLine = std::function<bool(std::string &line)>;
// Takes each head's next scheduled line and the seconds it takes; it may move the text away.
using WriteLine = std::function<void(std::string &line, double seconds)>;
struct StreamResult
{
    size_t                   syncs_added    = 0;
    size_t                   parks          = 0;
    size_t                   timed_layers   = 0;
    size_t                   stopped_layers = 0;
    double                   print_time     = 0.;
    std::vector<std::string> errors; // from check_streams() on the written streams
};
// `line_time` holds the seconds each input line takes (missing = 0); `each_layer` runs before
// every layer, to cancel. Throws as schedule_streams() does.
StreamResult schedule_and_check(const std::array<ReadLine, 2> &read, const std::array<const std::vector<double> *, 2> &line_time,
                                const std::array<WriteLine, 2> &write, const ScheduleParams &params,
                                const std::function<void()> &each_layer = {});

// "part.gcode" -> "part.T0.gcode".
std::string head_gcode_path(const std::string &path, int head);

// Whether a line is a "; SPLITPRINT_SYNC <id>" barrier.
bool is_sync_line(std::string_view line);

// Index of the line after which each interval starts, per stream: element k is the line
// index of the k-th sync. Used to place each move of a stream on the shared timeline.
std::vector<size_t> sync_lines(const std::vector<std::string> &lines);

} // namespace Slic3r::SplitPrint
