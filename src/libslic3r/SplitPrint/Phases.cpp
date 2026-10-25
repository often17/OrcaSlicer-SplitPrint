#include "Phases.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace Slic3r::SplitPrint {

namespace {

// Extra room on top of the gap, for line width and rounding.
constexpr double kPhaseMarginMm = 1.;

struct Side
{
    int     ax;
    coord_t cut;
    bool    keep_high;
    bool    inside(const Point3 &p) const { return keep_high ? p[ax] >= cut : p[ax] < cut; }
};

void u_range(const ExtrusionEntity &entity, int ax, coord_t &lo, coord_t &hi, bool &known)
{
    auto points = [&](const Points3 &pts) {
        for (const Point3 &p : pts) {
            lo = std::min(lo, p[ax]);
            hi = std::max(hi, p[ax]);
        }
    };
    if (auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        points(path->polyline.points);
    else if (auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        for (const ExtrusionPath &p : multi->paths)
            points(p.polyline.points);
    else if (auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        for (const ExtrusionPath &p : loop->paths)
            points(p.polyline.points);
    else if (auto *coll = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const ExtrusionEntity *e : coll->entities)
            u_range(*e, ax, lo, hi, known);
    else
        known = false;
}

// Where segment a-b crosses u = cut.
Point3 crossing_at(const Point3 &a, const Point3 &b, const Side &s)
{
    const double t = double(s.cut - a[s.ax]) / double(b[s.ax] - a[s.ax]);
    return Point3(double(a.x()) + t * double(b.x() - a.x()), double(a.y()) + t * double(b.y() - a.y()),
                  double(a.z()) + t * double(b.z() - a.z()));
}

struct Runs
{
    std::vector<Points3> runs;
    bool                 ends_inside = false;
};

// The parts of a polyline on the kept side.
Runs clip_points(const Points3 &pts, const Side &s)
{
    Runs    out;
    Points3 run;
    auto    close = [&]() {
        if (run.size() >= 2)
            out.runs.push_back(std::move(run));
        run.clear();
    };
    for (size_t i = 0; i < pts.size(); ++i) {
        const bool in = s.inside(pts[i]);
        if (in) {
            if (run.empty() && i > 0)
                run.push_back(crossing_at(pts[i - 1], pts[i], s));
            if (run.empty() || run.back() != pts[i])
                run.push_back(pts[i]);
        } else if (!run.empty()) {
            const Point3 c = crossing_at(pts[i - 1], pts[i], s);
            if (run.back() != c)
                run.push_back(c);
            close();
        }
    }
    out.ends_inside = !run.empty();
    close();
    return out;
}

ExtrusionPath with_points(const ExtrusionPath &src, Points3 &&pts)
{
    ExtrusionPath piece(src);
    piece.polyline.points = std::move(pts);
    piece.polyline.fitting_result.clear();
    return piece;
}

// Clips consecutive paths. Pieces that stay connected across a path boundary share a group.
std::vector<ExtrusionPaths> clip_paths(const ExtrusionPaths &paths, const Side &s, bool &first_starts_inside, bool &last_ends_inside)
{
    std::vector<ExtrusionPaths> groups;
    bool                        open = false;
    first_starts_inside              = !paths.empty() && !paths.front().polyline.points.empty() && s.inside(paths.front().polyline.points.front());
    for (const ExtrusionPath &path : paths) {
        const Points3 &pts       = path.polyline.points;
        const bool     starts_in = !pts.empty() && s.inside(pts.front());
        Runs           r         = clip_points(pts, s);
        for (size_t k = 0; k < r.runs.size(); ++k) {
            if (!(k == 0 && starts_in && open))
                groups.emplace_back();
            groups.back().push_back(with_points(path, std::move(r.runs[k])));
        }
        open = r.ends_inside && !r.runs.empty();
    }
    last_ends_inside = open;
    return groups;
}

std::unique_ptr<ExtrusionEntity> from_groups(std::vector<ExtrusionPaths> &&groups)
{
    std::vector<std::unique_ptr<ExtrusionEntity>> pieces;
    for (ExtrusionPaths &g : groups) {
        if (g.empty())
            continue;
        if (g.size() == 1)
            pieces.emplace_back(std::make_unique<ExtrusionPath>(std::move(g.front())));
        else
            pieces.emplace_back(std::make_unique<ExtrusionMultiPath>(g));
    }
    if (pieces.empty())
        return nullptr;
    if (pieces.size() == 1)
        return std::move(pieces.front());
    auto coll = std::make_unique<ExtrusionEntityCollection>();
    for (std::unique_ptr<ExtrusionEntity> &p : pieces)
        coll->entities.push_back(p.release());
    return coll;
}

// Whether clipping turned a single path or loop into a collection of pieces. The G-code writer
// does not take nested collections, so the pieces replace the entity in its parent list.
bool cut_into_pieces(const ExtrusionEntity &src, const ExtrusionEntity &clipped)
{
    return dynamic_cast<const ExtrusionEntityCollection *>(&src) == nullptr &&
           dynamic_cast<const ExtrusionEntityCollection *>(&clipped) != nullptr;
}

// Moves the pieces out of a collection that owns them.
template<typename Dst> void take_pieces(ExtrusionEntityCollection &pieces, Dst &dst)
{
    ExtrusionEntitiesPtr taken;
    taken.swap(pieces.entities);
    for (ExtrusionEntity *e : taken)
        dst.push_back(e);
}

std::unique_ptr<ExtrusionEntity> clip(const ExtrusionEntity &entity, const Side &s)
{
    coord_t lo = std::numeric_limits<coord_t>::max(), hi = std::numeric_limits<coord_t>::lowest();
    bool    known = true;
    u_range(entity, s.ax, lo, hi, known);
    if (!known || lo > hi) // nothing to clip by: print it whole in the first phase; the scheduler still checks it
        return s.keep_high ? std::unique_ptr<ExtrusionEntity>(entity.clone()) : nullptr;
    if (s.keep_high ? lo >= s.cut : hi < s.cut)
        return std::unique_ptr<ExtrusionEntity>(entity.clone());
    if (s.keep_high ? hi < s.cut : lo >= s.cut)
        return nullptr;

    if (auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        Runs                        r = clip_points(path->polyline.points, s);
        std::vector<ExtrusionPaths> groups;
        for (Points3 &run : r.runs)
            groups.push_back({with_points(*path, std::move(run))});
        return from_groups(std::move(groups));
    }
    if (auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        bool first_in, last_in;
        return from_groups(clip_paths(multi->paths, s, first_in, last_in));
    }
    if (auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        bool                        first_in, last_in;
        std::vector<ExtrusionPaths> groups = clip_paths(loop->paths, s, first_in, last_in);
        // The loop is closed: a piece running through its start continues at its end.
        if (groups.size() > 1 && first_in && last_in) {
            ExtrusionPaths &tail = groups.back();
            tail.insert(tail.end(), std::make_move_iterator(groups.front().begin()), std::make_move_iterator(groups.front().end()));
            groups.front() = std::move(tail);
            groups.pop_back();
        }
        return from_groups(std::move(groups));
    }
    auto *coll = static_cast<const ExtrusionEntityCollection *>(&entity); // u_range knew the type
    auto  out  = std::make_unique<ExtrusionEntityCollection>();
    out->no_sort = coll->no_sort;
    for (const ExtrusionEntity *e : coll->entities)
        if (std::unique_ptr<ExtrusionEntity> c = clip(*e, s)) {
            if (cut_into_pieces(*e, *c))
                take_pieces(static_cast<ExtrusionEntityCollection &>(*c), out->entities);
            else
                out->entities.push_back(c.release());
        }
    if (out->entities.empty())
        return nullptr;
    return out;
}

} // namespace

SplitParams make_split_params(const PrintObjectConfig &cfg, const BoundingBox &bbox)
{
    SplitParams p;
    p.axis       = cfg.split_axis.value == SplitPrintAxis::Y ? SplitAxis::Y : SplitAxis::X;
    p.split_pos  = (p.axis == SplitAxis::X ? bbox.center().x() : bbox.center().y()) + coord_t(scale_(cfg.split_position.value));
    p.nogo_width = coord_t(scale_(cfg.split_nogo_width.value));
    p.length_min = coord_t(scale_(cfg.split_interface_length_min.value));
    p.length_max = coord_t(scale_(cfg.split_interface_length_max.value));
    switch (cfg.split_interface_pattern.value) {
    case SplitInterfacePattern::Random: p.pattern = InterfacePattern::Random; break;
    case SplitInterfacePattern::Sine: p.pattern = InterfacePattern::Sine; break;
    default: p.pattern = InterfacePattern::Alternating; break;
    }
    p.period = cfg.split_interface_period.value;
    switch (cfg.split_interface_interlock.value) {
    case SplitInterlockMode::InPlane: p.interlock = Interlock::InPlane; break;
    case SplitInterlockMode::Layers: p.interlock = Interlock::Layers; break;
    default: p.interlock = Interlock::Both; break;
    }
    p.layers_per_block = std::max(1, cfg.split_interface_layers_per_block.value);
    return p;
}

SplitParams object_split_params(const PrintObject &object)
{
    BoundingBox bbox;
    for (const Layer *layer : object.layers())
        for (const BoundingBox &island : layer->lslices_bboxes)
            bbox.merge(island);
    return make_split_params(object.config(), bbox);
}

coord_t phase_cut(const SplitParams &params, double gap_mm, int head)
{
    // Nothing of T0 lies beyond the band's far edge, and nothing of T1 before its near edge.
    const coord_t reach = std::max(params.nogo_width / 2, params.length_max);
    const coord_t away  = reach + coord_t(scale_(gap_mm + kPhaseMarginMm));
    return head == 0 ? params.split_pos - away : params.split_pos + away;
}

coord_t plate_split_pos(const SplitParams &params, const Point &shift)
{
    return params.split_pos + (params.axis == SplitAxis::Y ? shift.y() : shift.x());
}

std::unique_ptr<ExtrusionEntity> clip_entity(const ExtrusionEntity &entity, SplitAxis axis, coord_t cut, bool keep_high)
{
    return clip(entity, Side{axis == SplitAxis::Y ? 1 : 0, cut, keep_high});
}

std::vector<const ExtrusionEntity *> clip_entities(const std::vector<const ExtrusionEntity *> &entities, SplitAxis axis, coord_t cut,
                                                   bool keep_high, std::vector<std::unique_ptr<ExtrusionEntity>> &owned)
{
    const Side                           s{axis == SplitAxis::Y ? 1 : 0, cut, keep_high};
    std::vector<const ExtrusionEntity *> out;
    out.reserve(entities.size());
    for (const ExtrusionEntity *e : entities) {
        coord_t lo = std::numeric_limits<coord_t>::max(), hi = std::numeric_limits<coord_t>::lowest();
        bool    known = true;
        u_range(*e, s.ax, lo, hi, known);
        if (!known || lo > hi) { // not clippable: whole, in the first phase only
            if (keep_high)
                out.push_back(e);
            continue;
        }
        if (keep_high ? lo >= cut : hi < cut) { // wholly on the kept side
            out.push_back(e);
            continue;
        }
        std::unique_ptr<ExtrusionEntity> c = clip(*e, s);
        if (!c)
            continue;
        if (cut_into_pieces(*e, *c)) {
            std::vector<ExtrusionEntity *> pieces;
            take_pieces(static_cast<ExtrusionEntityCollection &>(*c), pieces);
            for (ExtrusionEntity *piece : pieces) {
                out.push_back(piece);
                owned.emplace_back(piece);
            }
        } else {
            out.push_back(c.get());
            owned.push_back(std::move(c));
        }
    }
    return out;
}

} // namespace Slic3r::SplitPrint
