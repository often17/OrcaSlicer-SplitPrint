# Split print: two heads, one object

## What it does
On a printer with two independent gantries (`split_print_enable`), one object is
split between two heads that print it at the same time. Tool 1 prints the side below
the split line along `split_axis`, tool 2 the side above it. Around the line is a
no-go zone of width `split_nogo_width`. Only one head may be inside it at a time,
and the two sides interlock there with fingers.

## Geometry (`src/libslic3r/SplitPrint/Splitter`)
- The zone is cut into rows `split_interface_finger_width` perimeter spacings tall
  (default 2), on a grid anchored at 0, so rows line up between layers and between
  islands. A finger one line wide becomes a separate short line with a retraction
  on each side; on a long joint that costs more time than the second head saves.
  From two lines up, the wall runs in and out of the fingers as one loop.
- Each row has one owner. `split_interface_interlock` sets how owners change:
  - `both`: owners alternate row by row, and every row swaps owner after each block
    of `split_interface_layers_per_block` layers. The joint locks within a layer and
    between layers.
  - `in_plane`: owners alternate row by row, the same on every layer. The joint locks
    within a layer only.
  - `layers`: every row of a layer has the same owner, which swaps after each block of
    layers. The joint locks between layers only.
- A row's boundary sits at `split + L` when tool 1 owns it and at `split - L` when
  tool 2 does. `L` lies within `split_interface_length_min/max`, is capped at half
  the zone, and its variation is set by `split_interface_pattern`.
- Tool 1's area is the layer intersected with the staircase polygon these boundaries
  form, and tool 2's area is the rest. The two areas never overlap and together cover
  the layer.
- An island entirely on one side of the zone goes whole to that side's head. An
  island entirely inside the zone goes whole to the head on whose side its centre lies.

## Pipeline
The split reuses the color-painting path, because that path already turns
"per-layer area per extruder" into per-extruder regions.
- `Print::apply` registers extruders 1 and 2 as painting extruders for the object.
- `PrintObject::slice` feeds `apply_mm_segmentation` the split shapes in place of the
  painted ones.

Painting on a split object is ignored, with a warning.

## Two streams (`src/libslic3r/SplitPrint/Export`, `GCode.cpp`)
Each gantry has its own Klipper board, so each head gets its own G-code file next to
the exported one (`part.T0.gcode`, `part.T1.gcode`). `Print::export_gcode` writes
both head files first and the combined file last; the GUI export copies all three.
- A head's file holds only its own extrusions, temperatures and tool. It has no `T`
  commands, and the `split_head` placeholder (0 or 1, -1 in the combined file) lets
  start G-code tell the heads apart.
- `; SPLITPRINT_SYNC <id>` markers are barriers: neither head goes past one until
  both have reached it. `L<n>` opens layer n, `L<n>z` follows the shared Z move,
  `L<n>.<k>` are handoffs the scheduler adds inside a layer, and `end` comes before
  each head's own end G-code, which nothing checks.
- Before the first layer each head parks at its own end of the bed along the split
  axis, so both positions are known when checking starts.

## Printing at the same time (`SplitPrint/Phases`)
Orca prints a layer as whole walls then whole infill, each spanning the head's side
from far to near. Two heads printing that way would both reach the middle in every
interval and take turns. So each head prints its layer in two phases, cut along the
split axis at a gap plus half the no-go zone away from the split line:
- phase 0: T0 prints its part near the middle, T1 its part far from it;
- phase 1: T0 prints its far part, T1 its near part.
Walls and infill that cross the cut are clipped into open paths. The order flips
every layer, so a head starts a layer where it ended the last one. The phases only
make conflicts rare; the scheduler still decides when each head may move.

## Scheduler and check (`SplitPrint/Scheduler`)
The scheduler reads both head files with the G-code processor's per-line times and
cuts each layer into chunks that start at travel moves. Within one sync interval the
rule is `max(T0) + split_gantry_min_gap <= min(T1)` along the split axis, over every
position either head reaches in that interval. It advances the head that is behind
while its chunk fits. When that head is blocked only by where the other head went
earlier in the interval, it adds a sync. When neither head can move from where both
stand, it parks one of them out of the way (`; SPLITPRINT park`). If even that cannot
make room within the bed, slicing fails. Every sync stops both motion queues, so the
scheduler adds as few as it can.

`check_streams` checks the result again before anything is written: the gap rule,
Z changing only after a sync and before any extrusion, both heads agreeing on Z, and
no homing after the first sync. `tools/splitprint/check.py` is an independent copy of
the same check, which the Klipper coordinator runs before it starts a print.

## Preview and time
The combined file has both heads with tool changes, one after the other, so it is
only a preview source and never printable. After the head files are written, both
are processed again and their moves are merged on one timeline: an interval starts
for both heads when the slower one finished the previous interval. The preview then
plays both heads at once, with a marker on each, and its print time is the two-head
time. The combined file's time is kept as the one-head time, and the plater shows a
notification with both times and the faster choice.

## Constraints
- Both gantries are at the same height, so they can never pass each other. The split
  axis is the axis they would collide on.
- With the option off, slicing and G-code are unchanged.
- A split print needs layer-by-layer printing, no prime tower, no spiral vase, no
  belt printer, and no Z hop: Z is shared, and only the layer syncs move it.
