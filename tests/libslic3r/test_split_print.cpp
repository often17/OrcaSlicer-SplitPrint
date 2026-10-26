#include <catch2/catch_all.hpp>

#include <algorithm>
#include <array>
#include <clocale>
#include <limits>
#include <memory>
#include <cstdint>
#include <string>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/generators/catch_generators.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/SplitPrint/Phases.hpp"
#include "libslic3r/SplitPrint/Splitter.hpp"
#include "libslic3r/SplitPrint/Scheduler.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using namespace Slic3r::SplitPrint;
using Catch::Matchers::WithinRel;

namespace {

coord_t mm(double v) { return coord_t(scale_(v)); }

ExPolygon rect_mm(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon({Point(mm(x0), mm(y0)), Point(mm(x1), mm(y0)), Point(mm(x1), mm(y1)), Point(mm(x0), mm(y1))}));
}

// Split at 50 mm with a 10 mm band, 0.5 mm rows and fingers reaching 1..4 mm past the split.
SplitParams params(SplitAxis axis, InterfacePattern pattern)
{
    SplitParams p;
    p.axis       = axis;
    p.split_pos  = mm(50.);
    p.nogo_width = mm(10.);
    p.row_pitch  = mm(0.5);
    p.length_min = mm(1.);
    p.length_max = mm(4.);
    p.pattern    = pattern;
    p.period     = 6;
    p.seed       = 7;
    return p;
}

} // namespace

TEST_CASE("Split halves cover the layer without overlapping", "[SplitPrint]")
{
    const SplitAxis        axis    = GENERATE(SplitAxis::X, SplitAxis::Y);
    const InterfacePattern pattern = GENERATE(InterfacePattern::Alternating, InterfacePattern::Random, InterfacePattern::Sine);
    const int              layer   = GENERATE(0, 1, 17);
    // A 40 x 40 mm square centred on the split line, with a hole across the band.
    ExPolygon part = rect_mm(30., 30., 70., 70.);
    part.holes.push_back(rect_mm(45., 45., 55., 55.).contour);
    part.holes.back().reverse();

    const SplitResult r = split_layer({part}, params(axis, pattern), layer);

    REQUIRE_FALSE(r.tool[0].empty());
    REQUIRE_FALSE(r.tool[1].empty());
    const double total = area(r.tool[0]) + area(r.tool[1]);
    CHECK_THAT(total, WithinRel(part.area(), 1e-6));
    CHECK_THAT(area(intersection_ex(r.tool[0], r.tool[1])), Catch::Matchers::WithinAbs(0., double(mm(0.01)) * mm(0.01)));
}

TEST_CASE("Each tool reaches past the split line into the other side", "[SplitPrint]")
{
    const SplitAxis axis = GENERATE(SplitAxis::X, SplitAxis::Y);
    const SplitParams p  = params(axis, InterfacePattern::Alternating);
    const SplitResult r  = split_layer({rect_mm(20., 20., 80., 80.)}, p, 0);

    const BoundingBox b0 = get_extents(r.tool[0]);
    const BoundingBox b1 = get_extents(r.tool[1]);
    const coord_t     t0_max = axis == SplitAxis::X ? b0.max.x() : b0.max.y();
    const coord_t     t1_min = axis == SplitAxis::X ? b1.min.x() : b1.min.y();
    // The longest fingers set the reach: split +/- length_max.
    CHECK(t0_max == p.split_pos + p.length_max);
    CHECK(t1_min == p.split_pos - p.length_max);
}

TEST_CASE("Finger lengths stay within the configured range and the band", "[SplitPrint]")
{
    const InterfacePattern pattern = GENERATE(InterfacePattern::Alternating, InterfacePattern::Random, InterfacePattern::Sine);
    SplitParams            p       = params(SplitAxis::X, pattern);
    for (int layer = 0; layer < 5; ++layer)
        for (int64_t row = -50; row < 50; ++row) {
            const coord_t len = finger_length(p, layer, row);
            CHECK(len >= p.length_min);
            CHECK(len <= p.length_max);
        }
    // A maximum longer than half the band is cut back to the band edge.
    p.length_max = mm(20.);
    for (int64_t row = 0; row < 20; ++row)
        CHECK(finger_length(p, 0, row) <= p.nogo_width / 2);
}

TEST_CASE("Alternating pattern gives each tool both short and long fingers", "[SplitPrint]")
{
    const SplitParams p = params(SplitAxis::X, InterfacePattern::Alternating);
    // Rows 0 and 2 are both T0 fingers on layer 0; they must differ.
    REQUIRE(row_owner(p, 0, 0) == row_owner(p, 0, 2));
    CHECK(finger_length(p, 0, 0) != finger_length(p, 0, 2));
}

TEST_CASE("Both interlock alternates rows and swaps them every layer", "[SplitPrint]")
{
    const SplitParams p = params(SplitAxis::X, InterfacePattern::Alternating);
    REQUIRE(p.interlock == Interlock::Both);
    REQUIRE(p.layers_per_block == 1);
    for (int64_t row = -3; row < 3; ++row) {
        CHECK(row_owner(p, 0, row) != row_owner(p, 0, row + 1));
        CHECK(row_owner(p, 0, row) != row_owner(p, 1, row));
    }
}

TEST_CASE("In-plane interlock keeps the same rows on every layer", "[SplitPrint]")
{
    SplitParams p = params(SplitAxis::X, InterfacePattern::Alternating);
    p.interlock   = Interlock::InPlane;
    p.layers_per_block = 3;
    for (int64_t row = -3; row < 3; ++row) {
        CHECK(row_owner(p, 0, row) != row_owner(p, 0, row + 1));
        for (int layer = 1; layer < 8; ++layer)
            CHECK(row_owner(p, 0, row) == row_owner(p, layer, row));
    }
}

TEST_CASE("Between-layers interlock gives the whole band to one head per block", "[SplitPrint]")
{
    SplitParams p      = params(SplitAxis::X, InterfacePattern::Alternating);
    p.interlock        = Interlock::Layers;
    p.layers_per_block = 3;
    // Layers 0-2 are one block, 3-5 the next.
    for (int layer = 0; layer < 9; ++layer) {
        const int expected = (layer / 3) % 2;
        for (int64_t row = -3; row < 3; ++row)
            CHECK(row_owner(p, layer, row) == expected);
    }
    // In a T0 block every row reaches past the split; in a T1 block none does.
    const SplitResult t0_block = split_layer({rect_mm(20., 20., 80., 80.)}, p, 0);
    const SplitResult t1_block = split_layer({rect_mm(20., 20., 80., 80.)}, p, 3);
    CHECK(get_extents(t0_block.tool[0]).max.x() > p.split_pos);
    CHECK(get_extents(t1_block.tool[1]).min.x() < p.split_pos);
    CHECK(get_extents(t1_block.tool[0]).max.x() < p.split_pos);
}

TEST_CASE("Layers per finger sets how long rows keep their owner", "[SplitPrint]")
{
    SplitParams p      = params(SplitAxis::X, InterfacePattern::Alternating);
    p.layers_per_block = GENERATE(1, 2, 4);
    for (int layer = 0; layer < 12; ++layer) {
        const bool same_block = (layer + 1) / p.layers_per_block == layer / p.layers_per_block;
        CHECK((row_owner(p, layer, 0) == row_owner(p, layer + 1, 0)) == same_block);
    }
}

TEST_CASE("Random pattern is the same every time for the same seed", "[SplitPrint]")
{
    SplitParams p = params(SplitAxis::X, InterfacePattern::Random);
    for (int64_t row = 0; row < 20; ++row)
        CHECK(finger_length(p, 3, row) == finger_length(p, 3, row));
    bool differs = false;
    for (int64_t row = 0; row < 20; ++row)
        differs |= finger_length(p, 3, row) != finger_length(p, 4, row);
    CHECK(differs);
}

namespace {

// A head stream with `layers` layers. On each layer the head travels to each x in `xs` and
// prints a 20 mm line along Y there. Every motion line takes 1 s.
HeadStream stream_mm(int layers, const std::vector<double> &xs)
{
    HeadStream s;
    auto add = [&s](const std::string &line, double t) {
        s.lines.push_back(line);
        s.line_time.push_back(t);
    };
    add("G90", 0.);
    add("M83", 0.);
    for (int n = 0; n < layers; ++n) {
        add("; SPLITPRINT_SYNC L" + std::to_string(n), 0.);
        add("G1 Z" + std::to_string(0.2 * (n + 1)) + " F600", 1.);
        for (double x : xs) {
            add("G1 X" + std::to_string(x) + " Y10 F12000", 1.);
            add("G1 X" + std::to_string(x) + " Y30 E1 F1800", 1.);
        }
    }
    return s;
}

ScheduleParams lane_params()
{
    ScheduleParams p;
    p.axis         = 'X';
    p.gap          = 60.;
    p.bed_min      = 0.;
    p.bed_max      = 800.;
    p.travel_speed = 200.;
    return p;
}

} // namespace

TEST_CASE("Heads working far apart run together without extra syncs", "[SplitPrint]")
{
    const std::array<HeadStream, 2> in{stream_mm(3, {50., 100., 150.}), stream_mm(3, {600., 650., 700.})};
    const ScheduleResult            r = schedule_streams(in, lane_params());
    CHECK(r.parks == 0);
    // Only the shared-Z sync of each layer.
    CHECK(r.syncs_added == 3);
    CHECK(check_streams(r.lines, lane_params()).empty());
    // Each head takes 7 s per layer (Z + 3 travels + 3 lines); running together, about 7 s per layer.
    CHECK(r.print_time < 3 * 7. + 10.);
}

TEST_CASE("Heads that both need the middle take turns and stay apart", "[SplitPrint]")
{
    // T0 prints right up to the split line, T1 starts just past it: far closer than the 60 mm gap.
    const std::array<HeadStream, 2> in{stream_mm(2, {300., 395.}), stream_mm(2, {405., 500.})};
    const ScheduleResult            r = schedule_streams(in, lane_params());
    CHECK(r.syncs_added > 2);
    const std::vector<std::string> errors = check_streams(r.lines, lane_params());
    INFO((errors.empty() ? std::string() : errors.front()));
    CHECK(errors.empty());
    // Same work, same number of extrusions as before scheduling.
    for (int h = 0; h < 2; ++h)
        CHECK(std::count_if(r.lines[h].begin(), r.lines[h].end(), [](const std::string &l) { return l.find(" E1 ") != std::string::npos; }) ==
              std::count_if(in[h].lines.begin(), in[h].lines.end(), [](const std::string &l) { return l.find(" E1 ") != std::string::npos; }));
}

TEST_CASE("The collision check catches heads that come too close", "[SplitPrint]")
{
    // Unscheduled streams: nothing keeps T0 at 395 away from T1 at 405.
    const std::array<std::vector<std::string>, 2> lines{stream_mm(1, {395.}).lines, stream_mm(1, {405.}).lines};
    const std::vector<std::string>                errors = check_streams(lines, lane_params());
    REQUIRE_FALSE(errors.empty());
    CHECK(errors.front().find("too close") != std::string::npos);
}

TEST_CASE("Streams with different layers are refused", "[SplitPrint]")
{
    const std::array<HeadStream, 2> in{stream_mm(2, {50.}), stream_mm(3, {700.})};
    CHECK_THROWS(schedule_streams(in, lane_params()));
}

TEST_CASE("A part that needs a head beyond the bed is refused", "[SplitPrint]")
{
    // On a 100 mm bed the heads can never be 60 mm apart around the middle.
    ScheduleParams                  p = lane_params();
    p.bed_max                         = 100.;
    const std::array<HeadStream, 2> in{stream_mm(1, {45.}), stream_mm(1, {55.})};
    CHECK_THROWS(schedule_streams(in, p));
}

TEST_CASE("Head streams are named after the combined G-code", "[SplitPrint]")
{
    CHECK(head_gcode_path("C:/out/part.gcode", 0) == "C:/out/part.T0.gcode");
    CHECK(head_gcode_path("/tmp/.123.4.gcode", 1) == "/tmp/.123.4.T1.gcode");
    CHECK(head_gcode_path("/tmp/dir.d/part", 1) == "/tmp/dir.d/part.T1");
}

TEST_CASE("Islands away from the band go whole to the nearer tool", "[SplitPrint]")
{
    const SplitParams p = params(SplitAxis::X, InterfacePattern::Alternating);
    const ExPolygon   left   = rect_mm(10., 10., 30., 30.);
    const ExPolygon   right  = rect_mm(70., 10., 90., 30.);
    // Narrower than the band and centred right of the split.
    const ExPolygon   inside = rect_mm(51., 40., 54., 60.);

    const SplitResult r = split_layer({left, right, inside}, p, 0);

    REQUIRE(r.tool[0].size() == 1);
    REQUIRE(r.tool[1].size() == 2);
    CHECK_THAT(area(r.tool[0]), WithinRel(left.area(), 1e-9));
    CHECK_THAT(area(r.tool[1]), WithinRel(right.area() + inside.area(), 1e-9));
}

namespace {

// Lengths (mm) of the paths in an entity, and the lowest and highest x they reach.
void walk_paths(const ExtrusionEntity &e, double &length, coord_t &x_lo, coord_t &x_hi)
{
    auto points = [&](const ExtrusionPath &p) {
        const Points3 &pts = p.polyline.points;
        for (size_t i = 0; i < pts.size(); ++i) {
            x_lo = std::min(x_lo, pts[i].x());
            x_hi = std::max(x_hi, pts[i].x());
            if (i > 0)
                length += unscale<double>((pts[i] - pts[i - 1]).cast<double>().head<2>().norm());
        }
    };
    if (auto *path = dynamic_cast<const ExtrusionPath *>(&e))
        points(*path);
    else if (auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&e))
        for (const ExtrusionPath &p : multi->paths)
            points(p);
    else if (auto *loop = dynamic_cast<const ExtrusionLoop *>(&e))
        for (const ExtrusionPath &p : loop->paths)
            points(p);
    else if (auto *coll = dynamic_cast<const ExtrusionEntityCollection *>(&e))
        for (const ExtrusionEntity *c : coll->entities)
            walk_paths(*c, length, x_lo, x_hi);
}

ExtrusionLoop square_loop(double size_mm)
{
    ExtrusionPath path(erExternalPerimeter, 0.05, 0.4f, 0.2f);
    const coord_t s = scaled<coord_t>(size_mm);
    path.polyline.points = Points3{Point3(coord_t(0), coord_t(0)), Point3(s, coord_t(0)), Point3(s, s), Point3(coord_t(0), s),
                                   Point3(coord_t(0), coord_t(0))};
    return ExtrusionLoop(path);
}

} // namespace

TEST_CASE("Clipping a loop at the phase cut keeps each side once", "[SplitPrint]")
{
    // A 100 mm square cut at x = 30 mm: below the cut lie the left wall and 30 mm of the top and
    // bottom walls (160 mm), above it the rest (240 mm).
    const ExtrusionLoop loop = square_loop(100.);
    const coord_t       cut  = scaled<coord_t>(30.);
    std::unique_ptr<ExtrusionEntity> high = clip_entity(loop, SplitAxis::X, cut, true);
    std::unique_ptr<ExtrusionEntity> low  = clip_entity(loop, SplitAxis::X, cut, false);
    REQUIRE(high);
    REQUIRE(low);
    double  len_hi = 0., len_lo = 0.;
    coord_t lo_hi = std::numeric_limits<coord_t>::max(), hi_hi = std::numeric_limits<coord_t>::lowest();
    coord_t lo_lo = lo_hi, hi_lo = hi_hi;
    walk_paths(*high, len_hi, lo_hi, hi_hi);
    walk_paths(*low, len_lo, lo_lo, hi_lo);
    CHECK_THAT(len_hi, WithinRel(240., 1e-3));
    CHECK_THAT(len_lo, WithinRel(160., 1e-3));
    CHECK(lo_hi >= cut);
    CHECK(hi_lo <= cut);
    // The pieces below the cut meet at the loop start (0, 0), so they are one open path.
    CHECK(dynamic_cast<const ExtrusionEntityCollection *>(low.get()) == nullptr);
}

TEST_CASE("Entities wholly on one side of the cut are kept or dropped as they are", "[SplitPrint]")
{
    const ExtrusionLoop                           loop = square_loop(20.);
    const std::vector<const ExtrusionEntity *>    in{&loop};
    std::vector<std::unique_ptr<ExtrusionEntity>> owned;
    const auto kept = clip_entities(in, SplitAxis::X, scaled<coord_t>(-5.), true, owned);
    REQUIRE(kept.size() == 1);
    CHECK(kept.front() == &loop);
    CHECK(owned.empty());
    CHECK(clip_entities(in, SplitAxis::X, scaled<coord_t>(-5.), false, owned).empty());
}

TEST_CASE("Phase cuts leave the gap between what the two heads print at once", "[SplitPrint]")
{
    SplitParams p;
    p.split_pos  = scaled<coord_t>(200.);
    p.nogo_width = scaled<coord_t>(20.);
    p.length_max = scaled<coord_t>(8.);
    const double gap = 60.;
    // Phase 0: T0 keeps x >= cut0 but reaches at most the band's far edge; T1 keeps x >= cut1.
    CHECK(unscale<double>(phase_cut(p, gap, 1) - p.band_hi()) >= gap);
    // Phase 1: T1 keeps x < cut1 but reaches down to the band's near edge; T0 keeps x < cut0.
    CHECK(unscale<double>(p.band_lo() - phase_cut(p, gap, 0)) >= gap);
}

TEST_CASE("Scheduling reads and writes G-code numbers with a point in any locale", "[SplitPrint]")
{
    // Orca writes short numbers like Z.2; the GUI runs with the user's number format, which may use a comma.
    std::array<HeadStream, 2> in;
    for (int h = 0; h < 2; ++h) {
        const std::string x = h == 0 ? "50.5" : "650.5";
        for (const std::string &line : std::vector<std::string>{"G90", "M83", "; SPLITPRINT_SYNC L0", "G1 Z.2", "G1 X" + x + " Y10 F12000", "G1 Y30.25 E1.5 F1800"}) {
            in[h].lines.push_back(line);
            in[h].line_time.push_back(1.);
        }
    }
    const std::string old_locale = std::setlocale(LC_NUMERIC, nullptr);
    const char       *comma      = nullptr;
    for (const char *name : {"de-DE", "pt-PT", "de_DE.UTF-8", "pt_PT.UTF-8", "fr_FR.UTF-8"})
        if ((comma = std::setlocale(LC_NUMERIC, name)) != nullptr)
            break;
    if (comma == nullptr)
        SKIP("no locale with a decimal comma is installed");
    struct Restore
    {
        std::string name;
        ~Restore() { std::setlocale(LC_NUMERIC, name.c_str()); }
    } restore{old_locale};

    const ScheduleResult r = schedule_streams(in, lane_params());
    for (int h = 0; h < 2; ++h) {
        // The layer's Z move was read, so the shared Z move is in its own interval.
        CHECK(std::count(r.lines[h].begin(), r.lines[h].end(), "; SPLITPRINT_SYNC L0z") == 1);
        for (const std::string &line : r.lines[h])
            CHECK(line.find(',') == std::string::npos);
    }
    CHECK(std::find(r.lines[0].begin(), r.lines[0].end(), "G1 X0.000 F12000 ; SPLITPRINT park before the first layer") != r.lines[0].end());
    CHECK(check_streams(r.lines, lane_params()).empty());
}

namespace {

ScheduleParams timed_params()
{
    ScheduleParams p = lane_params();
    p.timed          = true;
    p.margin_abs     = 0.5;
    p.margin_rel     = 0.1;
    return p;
}

size_t count_with(const std::vector<std::string> &lines, const char *text)
{
    return size_t(std::count_if(lines.begin(), lines.end(), [text](const std::string &l) { return l.find(text) != std::string::npos; }));
}

} // namespace

TEST_CASE("Timed layers hand over the middle without a stop", "[SplitPrint]")
{
    // T0 prints at the split line first, then far from it; T1 far first, then at the split line.
    // Every line takes 1 s, so T0 has left the middle long before T1 gets there.
    const std::array<HeadStream, 2> in{stream_mm(3, {395., 300., 250., 200.}), stream_mm(3, {700., 650., 600., 405.})};
    const ScheduleResult            r = schedule_streams(in, timed_params());
    CHECK(r.timed_layers == 3);
    CHECK(r.stopped_layers == 0);
    for (int h = 0; h < 2; ++h) {
        CHECK(count_with(r.lines[h], "; SPLITPRINT_TIMED ") == 3);
        // Only the layer and shared-Z syncs: none inside a layer.
        CHECK(count_with(r.lines[h], "; SPLITPRINT_SYNC ") == 6);
    }
    const std::vector<std::string> errors = check_streams(r.lines, timed_params(), &r.line_time);
    INFO((errors.empty() ? std::string() : errors.front()));
    CHECK(errors.empty());
    // Without the move times nothing proves a timed layer safe.
    CHECK_FALSE(check_streams(r.lines, timed_params()).empty());
}

TEST_CASE("A head that would meet the other waits at the start of the layer", "[SplitPrint]")
{
    // T0 starts right at the split line, and T1's only work is right past it: one of them goes
    // first and the other waits, before doing anything on the layer.
    const std::array<HeadStream, 2> in{stream_mm(1, {395., 200.}), stream_mm(1, {405.})};
    const ScheduleResult            r = schedule_streams(in, timed_params());
    REQUIRE(r.timed_layers == 1);
    CHECK(count_with(r.lines[0], "G4 P") + count_with(r.lines[1], "G4 P") == 1);
    const int waiting = count_with(r.lines[0], "G4 P") == 1 ? 0 : 1;
    const std::vector<std::string> &w = r.lines[waiting];
    const auto timed = std::find_if(w.begin(), w.end(), [](const std::string &l) { return l.rfind("; SPLITPRINT_TIMED", 0) == 0; });
    REQUIRE(timed != w.end());
    // Right after the line that opens the interval, or after a travel out of the way.
    const auto wait = std::find_if(timed, w.end(), [](const std::string &l) { return l.rfind("G4 P", 0) == 0; });
    CHECK(std::none_of(timed, wait, [](const std::string &l) { return l.find(" E") != std::string::npos; }));
    CHECK(check_streams(r.lines, timed_params(), &r.line_time).empty());
}

TEST_CASE("A head running later than the timing margin allows fails the check", "[SplitPrint]")
{
    const std::array<HeadStream, 2> in{stream_mm(1, {395., 200.}), stream_mm(1, {405.})};
    const ScheduleResult            r = schedule_streams(in, timed_params());
    REQUIRE(check_streams(r.lines, timed_params(), &r.line_time).empty());
    // The head that goes first takes twice as long: seconds late, more than the margin, while
    // the other comes in on time.
    const int first = count_with(r.lines[0], "G4 P") == 0 ? 0 : 1;
    std::array<std::vector<double>, 2> slow = r.line_time;
    for (double &t : slow[first])
        t *= 2.;
    const std::vector<std::string> errors = check_streams(r.lines, timed_params(), &slow);
    REQUIRE_FALSE(errors.empty());
    CHECK(errors.front().find("closer than") != std::string::npos);
}

TEST_CASE("Retraction for the layer change is undone before the next layer extrudes", "[SplitPrint]")
{
    ScheduleParams p = timed_params();
    p.retract_length = {0.8, 0.8};
    const std::array<HeadStream, 2> in{stream_mm(3, {100.}), stream_mm(3, {700.})};
    const ScheduleResult            r = schedule_streams(in, p);
    for (int h = 0; h < 2; ++h) {
        // Three layer ends, two following layers that extrude.
        CHECK(count_with(r.lines[h], "SPLITPRINT retract for the layer change") == 3);
        CHECK(count_with(r.lines[h], "SPLITPRINT unretract") == 2);
    }
}
