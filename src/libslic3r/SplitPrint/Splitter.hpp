#pragma once

#include <array>
#include <cstdint>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/libslic3r.h"

// Splits one layer of a part between two toolheads that print it at the same time.
// T0 owns the low side of the split axis, T1 the high side. Inside the no-go band the
// two sides interlock with fingers: rows one line pitch tall, owned alternately, each
// reaching a variable length past the split line into the other side.
namespace Slic3r::SplitPrint {

enum class SplitAxis { X, Y };

enum class InterfacePattern {
    Alternating, // fingers of one tool alternate between the min and max length
    Random,      // uniform in [min, max], deterministic for a given seed/layer/row
    Sine,        // smooth wave between min and max with `period` rows per cycle
};

struct SplitParams
{
    SplitAxis        axis        = SplitAxis::X;
    coord_t          split_pos   = 0; // position of the split line along `axis`
    coord_t          nogo_width  = 0; // full width of the no-go band
    coord_t          row_pitch   = 0; // finger row height, usually one line width
    coord_t          length_min  = 0; // how far a finger reaches past the split line
    coord_t          length_max  = 0;
    InterfacePattern pattern     = InterfacePattern::Alternating;
    int              period      = 4; // rows per cycle for Sine
    bool             layer_shift = true; // flip row ownership every layer
    uint32_t         seed        = 0;

    coord_t band_lo() const { return split_pos - nogo_width / 2; }
    coord_t band_hi() const { return split_pos + nogo_width / 2; }
};

struct SplitResult
{
    std::array<ExPolygons, 2> tool; // tool[0] = T0, tool[1] = T1
};

// Which tool owns finger row `row` on layer `layer_id` (0 or 1). Rows sit on a global
// grid: row k spans [k * row_pitch, (k + 1) * row_pitch) across the split axis.
int row_owner(const SplitParams &params, int layer_id, int64_t row);

// Finger length of a row, clamped to [length_min, length_max] and to half the band.
coord_t finger_length(const SplitParams &params, int layer_id, int64_t row);

// Position along the split axis where a row changes from T0 to T1:
// split_pos + L for a T0 finger, split_pos - L for a T1 finger.
coord_t row_boundary(const SplitParams &params, int layer_id, int64_t row);

// Splits a layer's islands. An island fully on one side of the band goes to that
// side's tool. An island lying entirely inside the band goes whole to the tool on
// whose side its centre sits. Every other island is cut along the finger staircase.
// The two results never overlap, and together they cover the input.
SplitResult split_layer(const ExPolygons &slice, const SplitParams &params, int layer_id);

} // namespace Slic3r::SplitPrint
