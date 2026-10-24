#include "Splitter.hpp"

#include <algorithm>
#include <cmath>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"

namespace Slic3r::SplitPrint {

namespace {

int64_t floor_div(int64_t a, int64_t b) { return a / b - ((a % b != 0) && ((a < 0) != (b < 0))); }

int64_t mod2(int64_t a) { return ((a % 2) + 2) % 2; }

// splitmix64: a portable hash, so the Random pattern slices the same on every platform.
uint64_t mix(uint64_t x)
{
    x += 0x9e3779b97f4a7c15ull;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
    return x ^ (x >> 31);
}

// (u, v) = (coordinate along the split axis, coordinate across it).
coord_t along(const SplitParams &p, const Point &pt) { return p.axis == SplitAxis::X ? pt.x() : pt.y(); }
coord_t across(const SplitParams &p, const Point &pt) { return p.axis == SplitAxis::X ? pt.y() : pt.x(); }
Point   to_xy(const SplitParams &p, coord_t u, coord_t v) { return p.axis == SplitAxis::X ? Point(u, v) : Point(v, u); }

// The region T0 owns around one island: everything below the per-row boundary.
// Its left edge sits well outside the island, so only the stepped edge cuts it.
Polygon t0_staircase(const SplitParams &p, int layer_id, const BoundingBox &bbox)
{
    const coord_t margin = p.row_pitch;
    const coord_t u_far  = std::min(along(p, bbox.min), along(p, bbox.max)) - margin;
    const coord_t v_lo   = std::min(across(p, bbox.min), across(p, bbox.max)) - margin;
    const coord_t v_hi   = std::max(across(p, bbox.min), across(p, bbox.max)) + margin;
    const int64_t k0     = floor_div(v_lo, p.row_pitch);
    const int64_t k1     = floor_div(v_hi, p.row_pitch);

    Polygon poly;
    poly.points.reserve(size_t(2 * (k1 - k0 + 1) + 2));
    poly.points.push_back(to_xy(p, u_far, coord_t(k0 * p.row_pitch)));
    for (int64_t k = k0; k <= k1; ++k) {
        const coord_t b = row_boundary(p, layer_id, k);
        poly.points.push_back(to_xy(p, b, coord_t(k * p.row_pitch)));
        poly.points.push_back(to_xy(p, b, coord_t((k + 1) * p.row_pitch)));
    }
    poly.points.push_back(to_xy(p, u_far, coord_t((k1 + 1) * p.row_pitch)));
    // Swapping axes for SplitAxis::Y mirrors the winding.
    poly.make_counter_clockwise();
    return poly;
}

} // namespace

int row_owner(const SplitParams &params, int layer_id, int64_t row)
{
    return int(mod2(row + (params.layer_shift ? layer_id : 0)));
}

coord_t finger_length(const SplitParams &params, int layer_id, int64_t row)
{
    const coord_t lo = std::max<coord_t>(0, std::min(params.length_min, params.length_max));
    const coord_t hi = std::max(lo, std::max(params.length_min, params.length_max));
    double        t  = 1.;
    switch (params.pattern) {
    case InterfacePattern::Alternating:
        // Consecutive fingers of the same tool are two rows apart.
        t = mod2(floor_div(row, 2)) == 0 ? 1. : 0.;
        break;
    case InterfacePattern::Random: {
        const uint64_t h = mix(mix(mix(params.seed) ^ uint64_t(uint32_t(layer_id))) ^ uint64_t(row));
        t                = double(h >> 11) / double(1ull << 53);
        break;
    }
    case InterfacePattern::Sine: {
        const double period = std::max(2, params.period);
        t                   = 0.5 + 0.5 * std::sin(2. * PI *double(row) / period);
        break;
    }
    }
    const coord_t len = lo + coord_t(std::llround(t * double(hi - lo)));
    return std::clamp<coord_t>(len, 0, std::max<coord_t>(0, params.nogo_width / 2));
}

coord_t row_boundary(const SplitParams &params, int layer_id, int64_t row)
{
    const coord_t len = finger_length(params, layer_id, row);
    return row_owner(params, layer_id, row) == 0 ? params.split_pos + len : params.split_pos - len;
}

SplitResult split_layer(const ExPolygons &slice, const SplitParams &params, int layer_id)
{
    SplitResult out;
    if (params.row_pitch <= 0) {
        out.tool[0] = slice;
        return out;
    }
    for (const ExPolygon &island : slice) {
        const BoundingBox bbox = get_extents(island);
        const coord_t     u_lo = std::min(along(params, bbox.min), along(params, bbox.max));
        const coord_t     u_hi = std::max(along(params, bbox.min), along(params, bbox.max));
        if (u_hi <= params.band_lo()) {
            out.tool[0].push_back(island);
        } else if (u_lo >= params.band_hi()) {
            out.tool[1].push_back(island);
        } else if (u_lo >= params.band_lo() && u_hi <= params.band_hi()) {
            // Too small to share: fingers would only make it weaker.
            out.tool[(u_lo + u_hi) / 2 < params.split_pos ? 0 : 1].push_back(island);
        } else {
            const Polygons stair{t0_staircase(params, layer_id, bbox)};
            append(out.tool[0], intersection_ex(island, stair));
            append(out.tool[1], diff_ex(island, stair));
        }
    }
    return out;
}

} // namespace Slic3r::SplitPrint
