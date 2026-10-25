#pragma once

#include <array>
#include <cstddef>
#include <string>
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
namespace Slic3r::SplitPrint {

struct ScheduleParams
{
    char   axis         = 'X';   // split axis letter in the G-code
    double gap          = 60.;   // minimum nozzle distance along the axis, mm
    double bed_min      = 0.;    // reachable range along the axis, mm
    double bed_max      = 0.;
    double travel_speed = 200.;  // mm/s, for parks
    double z_speed      = 10.;   // mm/s, for the layer Z moves
};

struct HeadStream
{
    std::vector<std::string> lines;
    std::vector<double>      line_time; // seconds each line takes, same size as `lines` (may be shorter: missing = 0)
};

struct ScheduleResult
{
    std::array<std::vector<std::string>, 2> lines;
    size_t                                  syncs_added = 0;
    size_t                                  parks       = 0;
    double                                  print_time  = 0.; // estimated, both heads running
};

// Throws Slic3r::SlicingError when the heads cannot be kept apart (a part that needs a head
// beyond the bed to make room) or when the streams don't share the same layer syncs.
ScheduleResult schedule_streams(const std::array<HeadStream, 2> &streams, const ScheduleParams &params);

// Independent check of finished streams, the C++ twin of tools/splitprint/check.py.
// Returns one message per violation; empty = safe.
std::vector<std::string> check_streams(const std::array<std::vector<std::string>, 2> &lines, const ScheduleParams &params);

// "part.gcode" -> "part.T0.gcode".
std::string head_gcode_path(const std::string &path, int head);

// Index of the line after which each interval starts, per stream: element k is the line
// index of the k-th sync. Used to place each move of a stream on the shared timeline.
std::vector<size_t> sync_lines(const std::vector<std::string> &lines);

} // namespace Slic3r::SplitPrint
