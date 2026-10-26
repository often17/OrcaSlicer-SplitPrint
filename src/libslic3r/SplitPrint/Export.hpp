#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r {
class GCode;
class Print;
struct GCodeProcessorResult;
} // namespace Slic3r

// Export side of a split print: one G-code file per head next to the combined one, made safe
// by the scheduler and checked, plus a preview that shows both heads on one timeline.
// A large print has far more moves than fit in memory, so the files are scheduled a layer at a
// time and the G-code processor's moves are dropped as soon as they are used.
namespace Slic3r::SplitPrint {

// True when any object of the print is split between the two heads.
bool print_is_split(const Print &print);

// Exports head `head`'s own stream (GCode::set_split_head()) to `path` with `gcode`, and
// returns the seconds each line of the written file takes.
std::vector<double> export_head_stream(Print &print, GCode &gcode, const std::string &path, int head);

// Makes `gcode`'s export drop the processor's moves: the preview replaces the combined file's
// moves with the heads' own (merge_head_preview()).
void drop_export_moves(GCode &gcode);

// Both heads' scheduled streams, processed for the preview.
struct HeadsPreview;

// Schedules the two head streams that export_head_stream() wrote to head_gcode_path(path, 0/1),
// checks them and replaces the files with the scheduled ones. `line_time` holds the seconds per
// line export_head_stream() returned. With `for_preview`, returns what merge_head_preview()
// needs. Only reads the print, so it may run while the combined file is exported. Throws
// Slic3r::SlicingError if the heads cannot be kept apart or the check fails.
std::shared_ptr<HeadsPreview> schedule_head_streams(const Print &print, const std::string &path, std::array<std::vector<double>, 2> line_time,
                                                    bool for_preview);

// Replaces the moves of `preview`, the combined file's result, by both heads' moves merged on
// the shared timeline. Its print time becomes the two-head time and split_one_head_time keeps
// the time the combined (one head) G-code takes. Keeps at most `preview_moves` moves per head
// (0: as many as a share of the computer's memory holds); a larger print's preview shows every
// second, fourth, ... layer. Does nothing when `heads` or `preview` is null.
void merge_head_preview(const Print &print, const std::shared_ptr<HeadsPreview> &heads, GCodeProcessorResult *preview,
                        size_t preview_moves = 0);

} // namespace Slic3r::SplitPrint
