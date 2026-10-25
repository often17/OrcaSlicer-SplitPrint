#pragma once

#include <array>
#include <string>

namespace Slic3r {
class Print;
struct GCodeProcessorResult;
} // namespace Slic3r

// Export side of a split print: one G-code file per head next to the combined one, made safe
// by the scheduler and checked, plus a preview that shows both heads on one timeline.
namespace Slic3r::SplitPrint {

// True when any object of the print is split between the two heads.
bool print_is_split(const Print &print);

// Schedules the two head streams already written to head_gcode_path(path, 0/1) by an export
// with GCode::set_split_head(), checks them and rewrites them in place. `head_results` are the
// processor results of those two exports (their per-move times feed the scheduler).
// When `preview` is set, its moves are replaced by both heads' moves merged on the shared
// timeline, its print time becomes the two-head time and split_one_head_time keeps the time
// the combined (one head) G-code takes.
// Throws Slic3r::SlicingError if the heads cannot be kept apart or the check fails.
void finish_head_streams(const Print &print, const std::string &path, const std::array<GCodeProcessorResult *, 2> &head_results,
                         GCodeProcessorResult *preview);

} // namespace Slic3r::SplitPrint
