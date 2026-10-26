#include "Export.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "Scheduler.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/format.hpp"

namespace Slic3r::SplitPrint {

namespace {

// PrintBase::throw_if_canceled() is protected.
void throw_if_canceled(const Print &print)
{
    if (print.canceled())
        throw CanceledException();
}

constexpr size_t kNormal = static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal);
constexpr size_t kBlock  = size_t(1) << 24; // bytes read or written at once

using Move = GCodeProcessorResult::MoveVertex;

template<class T> void release(std::vector<T> &v) { std::vector<T>().swap(v); }

// Reads a file a line at a time, in large blocks: much faster than getline on big files. A '\r'
// before the line end is dropped.
class LineReader
{
public:
    explicit LineReader(const std::string &path) : m_in(path, std::ios::binary), m_block(kBlock, '\0')
    {
        if (!m_in)
            throw Slic3r::RuntimeError(Slic3r::format("Two heads, one object: cannot read %1%", path));
    }

    bool next(std::string &line)
    {
        for (;;) {
            const size_t e = std::string_view(m_block.data(), m_size).find('\n', m_pos);
            if (e != std::string_view::npos) {
                const std::string_view l(m_block.data() + m_pos, e - m_pos);
                m_pos = e + 1;
                if (m_partial.empty())
                    line.assign(l);
                else {
                    m_partial.append(l);
                    line.swap(m_partial);
                    m_partial.clear();
                }
                if (!line.empty() && line.back() == '\r')
                    line.pop_back();
                return true;
            }
            m_partial.append(m_block.data() + m_pos, m_size - m_pos);
            m_pos = m_size = 0;
            if (m_eof)
                break;
            m_in.read(m_block.data(), std::streamsize(m_block.size()));
            m_size = size_t(m_in.gcount());
            m_eof  = !m_in;
        }
        if (m_partial.empty())
            return false;
        line.swap(m_partial);
        m_partial.clear();
        if (line.back() == '\r')
            line.pop_back();
        return true;
    }

private:
    boost::nowide::ifstream m_in;
    std::string             m_block;
    size_t                  m_pos = 0, m_size = 0;
    bool                    m_eof = false;
    std::string             m_partial; // the start of a line that runs into the next block
};

// Writes a file a line at a time, in large blocks.
class LineWriter
{
public:
    explicit LineWriter(const std::string &path) : m_path(path), m_out(path, std::ios::binary | std::ios::trunc)
    {
        if (!m_out)
            throw Slic3r::RuntimeError(Slic3r::format("Two heads, one object: cannot write %1%", path));
        m_block.reserve(kBlock);
    }

    void put(std::string_view line)
    {
        m_block.append(line);
        m_block.push_back('\n');
        if (m_block.size() >= kBlock - 4096)
            flush();
    }

    void close()
    {
        flush();
        m_out.close();
        if (!m_out)
            throw Slic3r::RuntimeError(Slic3r::format("Two heads, one object: cannot write %1%", m_path));
    }

private:
    void flush()
    {
        m_out.write(m_block.data(), std::streamsize(m_block.size()));
        m_block.clear();
    }

    std::string             m_path;
    boost::nowide::ofstream m_out;
    std::string             m_block;
};

// Adds a move's time to the line it belongs to (gcode_id is the 1-based line).
void add_line_time(std::vector<double> &t, const Move &m)
{
    if (m.gcode_id < 1)
        return;
    if (m.gcode_id > t.size())
        t.resize(m.gcode_id, 0.);
    t[m.gcode_id - 1] += double(m.time[kNormal]);
}

// The seconds per line as the processor first numbered the lines, moved to the lines of the
// file it wrote: post-processing inserts lines, and `lines_map` holds original -> final ids. An
// id it does not list keeps its number, as GCodeProcessor's synchronize_moves() does.
std::vector<double> renumber_lines(const std::vector<double> &original, const std::vector<std::pair<size_t, size_t>> &lines_map)
{
    std::vector<double> out;
    // Sized once: a file can have hundreds of millions of lines.
    out.reserve(std::max(original.size(), lines_map.empty() ? size_t(0) : lines_map.back().second));
    auto                it = lines_map.begin();
    for (size_t id = 1; id <= original.size(); ++id) {
        if (original[id - 1] == 0.)
            continue;
        while (it != lines_map.end() && it->first < id)
            ++it;
        const size_t final_id = it != lines_map.end() && it->first == id ? it->second : id;
        if (final_id > out.size())
            out.resize(final_id, 0.);
        out[final_id - 1] += original[id - 1];
    }
    return out;
}

// Keeps the moves of every k-th layer of a head's file, from the first layer on, and doubles k
// whenever it would keep more than `budget` moves. Moves before the first layer stay.
class ThinnedMoves
{
public:
    explicit ThinnedMoves(size_t budget) : m_budget(budget) {}

    void take(const Move *begin, const Move *end)
    {
        for (const Move *m = begin; m != end; ++m)
            if (m_moves.empty() || kept(m->layer_id, m_step)) {
                // Never grow past the budget by doubling.
                if (m_moves.size() == m_moves.capacity() && m_moves.capacity() <= m_budget && m_moves.capacity() * 2 > m_budget + 1)
                    m_moves.reserve(m_budget + 1);
                m_moves.push_back(*m);
                if (m_moves.size() > m_budget && m_step < kMaxStep)
                    thin(m_step * 2);
            }
    }

    // Keeps only every `step`-th layer; `step` is a power of two.
    void thin(unsigned step)
    {
        if (step <= m_step)
            return;
        m_step = step;
        // The first move is where the head starts.
        m_moves.erase(std::remove_if(m_moves.begin() + 1, m_moves.end(), [step](const Move &m) { return !kept(m.layer_id, step); }),
                      m_moves.end());
    }

    unsigned           step() const { return m_step; }
    std::vector<Move> &moves() { return m_moves; }

private:
    static constexpr unsigned kMaxStep = 1u << 20;
    static bool               kept(unsigned layer_id, unsigned step) { return layer_id == 0 || (layer_id - 1) % step == 0; }

    size_t            m_budget;
    unsigned          m_step = 1;
    std::vector<Move> m_moves;
};

// How many moves the preview may keep of each head.
size_t preview_budget()
{
    // Both heads' moves, with the jumps the merge adds between them, then take about an eighth of
    // the memory, and the viewer's copy about as much again.
    return std::max<size_t>(size_t(1) << 20, total_physical_memory() / 8 / sizeof(Move) / 3);
}

// Processes a head's scheduled file for the preview; `keep` takes the moves.
void process_file(const Print &print, const std::string &path, GCodeProcessorResult &out, ThinnedMoves &keep)
{
    // Runs on worker threads, which have their own number format.
    CNumericLocalesSetter locales_setter;
    GCodeProcessor        processor;
    // The combined export may be running and has set the same value.
    if (GCodeProcessor::s_IsBBLPrinter != print.is_BBL_printer())
        GCodeProcessor::s_IsBBLPrinter = print.is_BBL_printer();
    const Vec3d origin = print.get_plate_origin();
    processor.set_xy_offset(origin(0), origin(1));
    processor.set_move_sink({[&keep](const Move *begin, const Move *end) { keep.take(begin, end); }, {}});
    processor.process_file(path);
    out = processor.extract_result();
    keep.take(out.moves.data(), out.moves.data() + out.moves.size());
    release(out.moves);
    // The preview shows the combined file's lines.
    release(out.lines_ends);
}

// Replaces preview.moves by both heads' moves on the shared timeline. A sync is a barrier, so
// an interval starts for both heads when the slower one finished the previous interval.
// `planned` holds the scheduler's seconds per line: they include waits (G4), which the G-code
// processor books on whichever move it plans next, so they set the moves' times. In a `thinned`
// preview most lines have no moves left; the time they take still passes.
void merge_on_timeline(const std::array<GCodeProcessorResult, 2> &heads, const std::array<std::vector<size_t>, 2> &syncs,
                       const std::array<std::vector<double>, 2> &planned, bool thinned, GCodeProcessorResult &preview)
{
    const size_t intervals = syncs[0].size() + 1;
    struct Timed
    {
        double      start;
        double      end;
        const Move *move;
        int         head;
    };
    // What each head does, in order: a move, or a wait (no move) of some seconds; and where each
    // interval's steps start.
    struct Step
    {
        double      time;
        const Move *move;
    };
    std::array<std::vector<Step>, 2>   steps;
    std::array<std::vector<size_t>, 2> interval_start;
    tbb::parallel_for(0, 2, [&](int h) {
        const std::vector<Move> &moves = heads[h].moves;
        const size_t             lines = planned[h].size();
        // The moves of each line, in move order. The first move is where the head starts, not a
        // move of any line.
        std::vector<uint32_t> by_line;
        by_line.reserve(moves.size());
        for (size_t i = 1; i < moves.size(); ++i)
            if (const unsigned id = moves[i].gcode_id; id >= 1 && id <= lines)
                by_line.push_back(uint32_t(i));
        std::stable_sort(by_line.begin(), by_line.end(), [&moves](uint32_t a, uint32_t b) { return moves[a].gcode_id < moves[b].gcode_id; });
        steps[h].reserve(by_line.size() + syncs[h].size());
        interval_start[h].assign(intervals + 1, 0);
        size_t k = 0, next = 0;
        for (size_t line = 0; line < lines; ++line) {
            // Interval of a line: the number of syncs at or above it.
            while (k < syncs[h].size() && syncs[h][k] <= line)
                interval_start[h][++k] = steps[h].size();
            const size_t first = next;
            while (next < by_line.size() && moves[by_line[next]].gcode_id == line + 1)
                ++next;
            if (first == next) {
                if (planned[h][line] > 0.) {
                    // One wait stands for a run of lines without moves.
                    if (thinned && steps[h].size() > interval_start[h][k] && steps[h].back().move == nullptr)
                        steps[h].back().time += planned[h][line];
                    else
                        steps[h].push_back({planned[h][line], nullptr});
                }
                continue;
            }
            double processed = 0.;
            for (size_t i = first; i < next; ++i)
                processed += double(moves[by_line[i]].time[kNormal]);
            const double n = double(next - first);
            for (size_t i = first; i < next; ++i) {
                const Move &m = moves[by_line[i]];
                steps[h].push_back({planned[h][line] * (processed > 0. ? double(m.time[kNormal]) / processed : 1. / n), &m});
            }
        }
        while (k < intervals)
            interval_start[h][++k] = steps[h].size();
    });

    // Interval k starts at t0[k], when the slower head finished interval k - 1.
    std::vector<double> t0(intervals + 1, 0.);
    for (size_t k = 0; k < intervals; ++k) {
        double end = t0[k];
        for (int h = 0; h < 2; ++h) {
            double t = t0[k];
            for (size_t i = interval_start[h][k]; i < interval_start[h][k + 1]; ++i)
                t += steps[h][i].time;
            end = std::max(end, t);
        }
        t0[k + 1] = end;
    }
    // Both heads' moves of interval k, in the order they start.
    auto interval_moves = [&](size_t k, std::vector<Timed> &timed) {
        timed.clear();
        for (int h = 0; h < 2; ++h) {
            double t = t0[k];
            for (size_t i = interval_start[h][k]; i < interval_start[h][k + 1]; ++i) {
                const Step &step = steps[h][i];
                if (step.move != nullptr)
                    timed.push_back({t, t + step.time, step.move, h});
                t += step.time;
            }
        }
        std::stable_sort(timed.begin(), timed.end(), [](const Timed &a, const Timed &b) { return a.start < b.start; });
    };

    // The intervals are merged in parallel, each into its own part of the result. A first pass
    // finds what each interval needs from the ones before it, and how many moves it adds.
    struct Summary
    {
        size_t      moves = 0;
        int         first_head = -1, last_head = -1;
        size_t      switches_to[2] = {0, 0}; // later moves of head h right after one of the other head
        const Move *last_of[2] = {nullptr, nullptr};
        double      latest_end = 0.;
        unsigned    top_layer  = 0;
    };
    std::vector<Summary> summary(intervals);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, intervals), [&](const tbb::blocked_range<size_t> &range) {
        std::vector<Timed> timed;
        for (size_t k = range.begin(); k < range.end(); ++k) {
            interval_moves(k, timed);
            Summary &sum = summary[k];
            sum.moves    = timed.size();
            for (size_t i = 0; i < timed.size(); ++i) {
                const Timed &e = timed[i];
                if (i > 0 && e.head != timed[i - 1].head)
                    ++sum.switches_to[e.head];
                sum.last_of[e.head] = e.move;
                sum.latest_end      = std::max(sum.latest_end, e.end);
                sum.top_layer       = std::max(sum.top_layer, e.move->layer_id);
            }
            if (!timed.empty()) {
                sum.first_head = timed.front().head;
                sum.last_head  = timed.back().head;
            }
        }
    });
    // Where each interval starts: its place in the result and the state the moves before it leave.
    struct Start
    {
        size_t      offset    = 0;
        int         last_head = 0;
        const Move *last_of[2] = {nullptr, nullptr};
        double      now       = 0.; // end of the last move put on the timeline
        unsigned    layer     = 0;
    };
    std::vector<Start> start(intervals + 1);
    start[0].offset = heads[0].moves.empty() ? 0 : 1;
    for (size_t k = 0; k < intervals; ++k) {
        const Summary &sum  = summary[k];
        Start          next = start[k];
        if (sum.moves > 0) {
            // A move of the other head than the last one starts with a jump to where its head is,
            // unless its head has not moved yet.
            size_t jumps = 0;
            if (sum.first_head != next.last_head && next.last_of[sum.first_head] != nullptr)
                ++jumps;
            for (int h = 0; h < 2; ++h)
                if (sum.switches_to[h] > 0)
                    jumps += sum.switches_to[h] - (next.last_of[h] == nullptr && h != sum.first_head ? 1 : 0);
            next.offset += sum.moves + jumps;
            for (int h = 0; h < 2; ++h)
                if (sum.last_of[h] != nullptr)
                    next.last_of[h] = sum.last_of[h];
            next.last_head = sum.last_head;
            next.now       = std::max(next.now, sum.latest_end);
            next.layer     = std::max(next.layer, sum.top_layer);
        }
        start[k + 1] = next;
    }

    std::vector<Move> merged(start[intervals].offset);
    if (!heads[0].moves.empty())
        merged.front() = heads[0].moves.front();
    tbb::parallel_for(tbb::blocked_range<size_t>(0, intervals), [&](const tbb::blocked_range<size_t> &range) {
        std::vector<Timed> timed;
        for (size_t k = range.begin(); k < range.end(); ++k) {
            interval_moves(k, timed);
            const Start &st         = start[k];
            size_t       out        = st.offset;
            const Move  *last_of[2] = {st.last_of[0], st.last_of[1]};
            int          last_head  = st.last_head;
            unsigned     layer      = st.layer;
            double       now        = st.now;
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
                    Move &jump         = merged[out++];
                    jump               = *last_of[e.head];
                    jump.type          = EMoveType::Noop;
                    jump.time          = {0.f, 0.f};
                    jump.layer_id      = layer;
                    jump.internal_only = true;
                    jump.extruder_id   = static_cast<unsigned char>(e.head);
                }
                const double new_now = std::max(now, e.end);
                m.time[kNormal]      = float(new_now - now);
                now                  = new_now;
                merged[out++]        = m;
                last_of[e.head]      = e.move;
                last_head            = e.head;
            }
            assert(out == start[k + 1].offset);
        }
    });
    preview.moves = std::move(merged);
    preview.print_statistics.modes[kNormal].time = float(std::max(start[intervals].now, t0[intervals]));
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

std::vector<double> export_head_stream(Print &print, GCode &gcode, const std::string &path, int head)
{
    // The seconds of each line, as the processor numbers the lines while it reads them, until
    // post-processing moves them to the lines of the written file.
    std::vector<double> times;
    GCodeProcessor::MoveSink sink;
    sink.take = [&times](const Move *begin, const Move *end) {
        for (const Move *m = begin; m != end; ++m)
            add_line_time(times, *m);
    };
    sink.renumbered = [&times](const std::vector<std::pair<size_t, size_t>> &lines_map) { times = renumber_lines(times, lines_map); };
    gcode.set_split_head(head);
    gcode.set_move_sink(std::move(sink));
    GCodeProcessorResult result;
    gcode.do_export(&print, path.c_str(), &result, nullptr);
    // The moves the processor still had, already renumbered.
    for (const Move &m : result.moves)
        add_line_time(times, m);
    return times;
}

void drop_export_moves(GCode &gcode)
{
    gcode.set_move_sink({[](const Move *, const Move *) {}, {}});
}

struct HeadsPreview
{
    std::array<std::string, 2>         paths; // the scheduled head files
    std::array<std::vector<size_t>, 2> syncs;
    std::array<std::vector<double>, 2> line_time;
    size_t                             stopped_layers = 0;
};

std::shared_ptr<HeadsPreview> schedule_head_streams(const Print &print, const std::string &path, std::array<std::vector<double>, 2> line_time,
                                                    bool for_preview)
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
    params.gap        = cfg.split_gantry_min_gap.value;
    params.timed      = cfg.split_timed_layers.value;
    params.margin_rel = cfg.split_timing_margin.value / 100.;
    // The G-code processor estimates Klipper's times with these (the jerk standing in for the
    // square corner velocity); the printer must run with the same.
    if (cfg.gcode_flavor.value == gcfKlipper && !cfg.machine_max_speed_x.values.empty() && !cfg.machine_max_jerk_x.values.empty()) {
        params.max_velocity           = cfg.machine_max_speed_x.values.front();
        params.square_corner_velocity = cfg.machine_max_jerk_x.values.front();
    }
    for (int h = 0; h < 2; ++h) {
        params.retract_length[h] = cfg.retraction_length.get_at(h);
        if (const double v = cfg.retraction_speed.get_at(h); v > 0.)
            params.retract_speed[h] = v;
        const double v = cfg.deretraction_speed.get_at(h);
        params.deretract_speed[h] = v > 0. ? v : params.retract_speed[h];
    }
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

    // The scheduled files replace the exported ones only once they pass the check.
    std::array<std::string, 2> paths, temp;
    for (int h = 0; h < 2; ++h) {
        paths[h] = head_gcode_path(path, h);
        temp[h]  = paths[h] + ".tmp";
    }
    struct RemoveTemp
    {
        const std::array<std::string, 2> &files;
        ~RemoveTemp()
        {
            boost::system::error_code ec;
            for (const std::string &f : files)
                boost::filesystem::remove(f, ec);
        }
    } remove_temp{temp};

    auto         out = for_preview ? std::make_shared<HeadsPreview>() : nullptr;
    if (out)
        // The scheduler adds a few lines per layer.
        for (int h = 0; h < 2; ++h)
            out->line_time[h].reserve(line_time[h].size() + line_time[h].size() / 32 + 4096);
    StreamResult scheduled;
    {
        std::array<std::optional<LineReader>, 2> in;
        std::array<std::optional<LineWriter>, 2> written;
        for (int h = 0; h < 2; ++h) {
            in[h].emplace(paths[h]);
            written[h].emplace(temp[h]);
        }
        const std::array<ReadLine, 2> read{[&in](std::string &line) { return in[0]->next(line); },
                                           [&in](std::string &line) { return in[1]->next(line); }};
        std::array<size_t, 2> count{0, 0};
        auto write = [&](int h) -> WriteLine {
            return [&, h](std::string &line, double dt) {
                if (out) {
                    if (is_sync_line(line))
                        out->syncs[h].push_back(count[h]);
                    out->line_time[h].push_back(dt);
                }
                ++count[h];
                written[h]->put(line);
            };
        };
        scheduled = schedule_and_check(read, {&line_time[0], &line_time[1]}, {write(0), write(1)}, params, [&print] { throw_if_canceled(print); });
        for (int h = 0; h < 2; ++h)
            written[h]->close();
    }
    release(line_time[0]);
    release(line_time[1]);
    if (!scheduled.errors.empty())
        throw Slic3r::SlicingError(Slic3r::format("Two heads, one object: the collision check failed: %1%", scheduled.errors.front()));
    for (int h = 0; h < 2; ++h)
        boost::filesystem::rename(temp[h], paths[h]);
    if (out) {
        out->paths          = paths;
        out->stopped_layers = scheduled.stopped_layers;
    }
    return out;
}

void merge_head_preview(const Print &print, const std::shared_ptr<HeadsPreview> &heads, GCodeProcessorResult *preview, size_t preview_moves)
{
    if (!heads || preview == nullptr)
        return;
    throw_if_canceled(print);
    preview->split_one_head_time  = preview->print_statistics.modes[kNormal].time;
    preview->split_stopped_layers = int(heads->stopped_layers);
    // The combined file's moves are replaced by both heads' moves: free them first. The moves
    // point at lines of the heads' files, so the combined file's line table would show the wrong
    // text in the G-code window (and the viewer copies it); without it the window stays closed.
    release(preview->moves);
    release(preview->lines_ends);

    // The preview plays both heads' files as written, with the scheduler's times.
    const size_t                        budget = preview_moves > 0 ? preview_moves : preview_budget();
    std::array<ThinnedMoves, 2>         keep{ThinnedMoves(budget), ThinnedMoves(budget)};
    std::array<GCodeProcessorResult, 2> processed;
    tbb::parallel_for(0, 2, [&](int h) { process_file(print, heads->paths[h], processed[h], keep[h]); });
    const unsigned step = std::max(keep[0].step(), keep[1].step());
    for (int h = 0; h < 2; ++h) {
        keep[h].thin(step);
        processed[h].moves = std::move(keep[h].moves());
    }
    merge_on_timeline(processed, heads->syncs, heads->line_time, step > 1, *preview);
    preview->split_preview_layer_step = int(step);
    preview->split_print              = true;
}

} // namespace Slic3r::SplitPrint
