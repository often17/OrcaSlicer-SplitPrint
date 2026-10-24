#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cstdint>

#include "catch2/catch_test_macros.hpp"
#include "catch2/generators/catch_generators.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/SplitPrint/Splitter.hpp"
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

TEST_CASE("Row ownership flips every layer only when layer shift is on", "[SplitPrint]")
{
    SplitParams p = params(SplitAxis::X, InterfacePattern::Alternating);
    for (int64_t row = -3; row < 3; ++row) {
        CHECK(row_owner(p, 0, row) != row_owner(p, 0, row + 1));
        CHECK(row_owner(p, 0, row) != row_owner(p, 1, row));
    }
    p.layer_shift = false;
    for (int64_t row = -3; row < 3; ++row)
        CHECK(row_owner(p, 0, row) == row_owner(p, 1, row));
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
