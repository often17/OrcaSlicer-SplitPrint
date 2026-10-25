#include "Export.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <boost/nowide/fstream.hpp>

#include "Scheduler.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/format.hpp"

namespace Slic3r::SplitPrint {

namespace {

constexpr size_t kNormal = static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal);

std::vector<std::string> read_lines(const std::string &path)
{
    boost::nowide::ifstream in(path, std::ios::binary);
    if (!in)
        throw Slic3r::RuntimeError(Slic3r::format("Two heads, one object: cannot read %1%", path));
    std::vector<std::string> lines;
    std::string              line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        lines.push_back(std::move(line));
    }
    return lines;
}

void write_lines(const std::string &path, const std::vector<std::string> &lines)
{
    boost::nowide::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out)
        throw Slic3r::RuntimeError(Slic3r::format("Two heads, one object: cannot write %1%", path));
    for (const std::string &l : lines)
        out << l << '\n';
    if (!out)
        throw Slic3r::RuntimeError(Slic3r::format("Two heads, one object: cannot write %1%", path));
}

// Seconds each line of a stream takes, from the processor's moves (gcode_id is the 1-based line).
std::vector<double> line_times(const GCodeProcessorResult &result, size_t lines)
{
    std::vector<double> t(lines, 0.);
    for (const GCodeProcessorResult::MoveVertex &m : result.moves)
        if (m.gcode_id >= 1 && m.gcode_id <= lines)
            t[m.gcode_id - 1] += double(m.time[kNormal]);
    return t;
}

void process_file(const Print &print, const std::string &path, GCodeProcessorResult &out)
{
    GCodeProcessor processor;
    GCodeProcessor::s_IsBBLPrinter = print.is_BBL_printer();
    const Vec3d origin             = print.get_plate_origin();
    processor.set_xy_offset(origin(0), origin(1));
    processor.process_file(path);
    out = processor.extract_result();
}

// Replaces preview.moves by both heads' moves on the shared timeline. A sync is a barrier, so
// an interval starts for both heads when the slower one finished the previous interval.
void merge_on_timeline(const std::array<GCodeProcessorResult, 2> &heads, const std::array<std::vector<size_t>, 2> &syncs,
                       GCodeProcessorResult &preview)
{
    using Move = GCodeProcessorResult::MoveVertex;
    const size_t intervals = syncs[0].size() + 1;
    struct Timed
    {
        double      start;
        double      end;
        const Move *move;
        int         head;
    };
    // Interval of each move: the number of syncs above its line.
    std::array<std::vector<std::vector<const Move *>>, 2> by_interval;
    for (int h = 0; h < 2; ++h) {
        by_interval[h].assign(intervals, {});
        for (size_t i = 1; i < heads[h].moves.size(); ++i) {
            const Move  &m    = heads[h].moves[i];
            const size_t line = m.gcode_id >= 1 ? size_t(m.gcode_id - 1) : 0;
            const size_t k    = size_t(std::upper_bound(syncs[h].begin(), syncs[h].end(), line) - syncs[h].begin());
            by_interval[h][std::min(k, intervals - 1)].push_back(&m);
        }
    }

    std::vector<Move> merged;
    merged.reserve(heads[0].moves.size() + heads[1].moves.size() + 1024);
    if (!heads[0].moves.empty())
        merged.push_back(heads[0].moves.front());
    const Move *last_of[2] = {nullptr, nullptr};
    int         last_head  = 0;
    unsigned    layer      = 0;
    double      now        = 0.; // end of the last move put on the timeline
    double      t0         = 0.; // start of the current interval
    for (size_t k = 0; k < intervals; ++k) {
        std::vector<Timed> timed;
        double             end = t0;
        for (int h = 0; h < 2; ++h) {
            double t = t0;
            for (const Move *m : by_interval[h][k]) {
                const double d = double(m->time[kNormal]);
                timed.push_back({t, t + d, m, h});
                t += d;
            }
            end = std::max(end, t);
        }
        std::stable_sort(timed.begin(), timed.end(), [](const Timed &a, const Timed &b) { return a.start < b.start; });
        for (const Timed &e : timed) {
            Move m = *e.move;
            // Each head's file has only its own tool and no T command, so give the preview the head.
            m.extruder_id = static_cast<unsigned char>(e.head);
            // libvgcode wants layers in order; the heads cross a layer tag at slightly different times.
            layer      = std::max(layer, m.layer_id);
            m.layer_id = layer;
            if (e.head != last_head && last_of[e.head] != nullptr) {
                // Start the head's path from where that head is, not from the other head:
                // a no-op move is never drawn but sets where the next segment begins.
                Move jump      = *last_of[e.head];
                jump.type      = EMoveType::Noop;
                jump.time      = {0.f, 0.f};
                jump.layer_id  = layer;
                jump.internal_only = true;
                jump.extruder_id   = static_cast<unsigned char>(e.head);
                merged.push_back(jump);
            }
            const double new_now = std::max(now, e.end);
            m.time[kNormal]      = float(new_now - now);
            now                  = new_now;
            merged.push_back(m);
            last_of[e.head] = e.move;
            last_head       = e.head;
        }
        t0 = end;
    }
    preview.moves = std::move(merged);
    preview.print_statistics.modes[kNormal].time = float(std::max(now, t0));
}

} // namespace

bool print_is_split(const Print &print)
{
    // Same rule as PrintApply, which only splits objects when there are two filaments to give them.
    if (print.config().filament_diameter.size() < 2)
        return false;
    for (const PrintObject *object : print.objects())
        if (object->config().split_print_enable.value)
            return true;
    return false;
}

void finish_head_streams(const Print &print, const std::string &path, const std::array<GCodeProcessorResult *, 2> &head_results,
                         GCodeProcessorResult *preview)
{
    // The GUI runs with the user's number format; G-code needs a decimal point.
    CNumericLocalesSetter locales_setter;
    const PrintConfig    &cfg = print.config();
    ScheduleParams        params;
    params.axis = 'X';
    for (const PrintObject *object : print.objects())
        if (object->config().split_print_enable.value) {
            params.axis = object->config().split_axis.value == SplitPrintAxis::Y ? 'Y' : 'X';
            break;
        }
    params.gap = cfg.split_gantry_min_gap.value;
    BoundingBoxf bed;
    for (const Vec2d &p : cfg.printable_area.values)
        bed.merge(p);
    const int ax   = params.axis == 'Y' ? 1 : 0;
    params.bed_min = bed.min(ax);
    params.bed_max = bed.max(ax);
    if (!cfg.travel_speed.values.empty() && !cfg.travel_speed.is_nil(0) && cfg.travel_speed.get_at(0) > 0.)
        params.travel_speed = cfg.travel_speed.get_at(0);
    if (!cfg.machine_max_speed_z.values.empty() && cfg.machine_max_speed_z.values.front() > 0.)
        params.z_speed = std::min(params.z_speed, cfg.machine_max_speed_z.values.front());

    std::array<HeadStream, 2>  streams;
    std::array<std::string, 2> paths;
    for (int h = 0; h < 2; ++h) {
        paths[h]             = head_gcode_path(path, h);
        streams[h].lines     = read_lines(paths[h]);
        streams[h].line_time = line_times(*head_results[h], streams[h].lines.size());
    }
    const ScheduleResult scheduled = schedule_streams(streams, params);
    const std::vector<std::string> errors = check_streams(scheduled.lines, params);
    if (!errors.empty())
        throw Slic3r::SlicingError(Slic3r::format("Two heads, one object: the collision check failed: %1%", errors.front()));
    for (int h = 0; h < 2; ++h)
        write_lines(paths[h], scheduled.lines[h]);

    if (preview == nullptr)
        return;
    std::array<GCodeProcessorResult, 2> heads;
    std::array<std::vector<size_t>, 2>  syncs;
    for (int h = 0; h < 2; ++h) {
        process_file(print, paths[h], heads[h]);
        syncs[h] = sync_lines(scheduled.lines[h]);
    }
    preview->split_one_head_time = preview->print_statistics.modes[kNormal].time;
    merge_on_timeline(heads, syncs, *preview);
    preview->split_print = true;
}

} // namespace Slic3r::SplitPrint
