#pragma once

#include <memory>
#include <vector>

#include "Splitter.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r {
class ExtrusionEntity;
class PrintObject;
class PrintObjectConfig;
} // namespace Slic3r

// Each head prints its layer in two phases, so that the two heads are not near the middle at
// the same time and can work in parallel most of the layer:
//   phase 0: T0 prints what lies near the middle, T1 what lies far from it;
//   phase 1: T0 prints its far part, T1 its near part.
// In both phases each head keeps the side above its cut (u >= cut) in phase 0 and the side
// below it in phase 1. The cuts are a gap plus the no-go half width away from the split line,
// so in phase 0 T1 stays at least a gap beyond anything T0 can reach, and the same in phase 1.
// The scheduler still checks every move; the phases only make conflicts rare.
namespace Slic3r::SplitPrint {

// Split line and band of an object, from its config and the extent of its slices.
// row_pitch is left at 0; the caller sets it when it builds fingers.
SplitParams make_split_params(const PrintObjectConfig &config, const BoundingBox &slices_bbox);

// The same for a sliced object, from the extent of all its layers.
SplitParams object_split_params(const PrintObject &object);

// Where a head's near and far parts meet along the split axis.
coord_t phase_cut(const SplitParams &params, double gap_mm, int head);

// Keeps the part of `entity` on one side of the line u = cut (u >= cut when keep_high).
// A loop or path that crosses the line is cut into open paths. Returns nullptr when nothing
// is left on that side.
std::unique_ptr<ExtrusionEntity> clip_entity(const ExtrusionEntity &entity, SplitAxis axis, coord_t cut, bool keep_high);

// clip_entity over a list of borrowed entities. Entities wholly on the kept side are returned
// as they are; clipped copies are stored in `owned`.
std::vector<const ExtrusionEntity *> clip_entities(const std::vector<const ExtrusionEntity *> &entities, SplitAxis axis, coord_t cut,
                                                   bool keep_high, std::vector<std::unique_ptr<ExtrusionEntity>> &owned);

} // namespace Slic3r::SplitPrint
