# Split print: two heads, one object

## What it does
On a printer with two independent gantries (`split_print_enable`), one object is
split between two heads that print it at the same time. Tool 1 prints the side below
the split line along `split_axis`, tool 2 the side above it. Around the line is a
no-go zone of width `split_nogo_width`. Only one head may be inside it at a time,
and the two sides interlock there with fingers.

## Geometry (`src/libslic3r/SplitPrint/Splitter`)
- The zone is cut into rows one perimeter spacing tall, on a grid anchored at 0, so
  rows line up between layers and between islands.
- Each row has one owner, alternating between the heads. With
  `split_interface_layer_shift` the owner also flips every layer.
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

## Constraints
- Both gantries are at the same height, so they can never pass each other. The split
  axis is the axis they would collide on.
- With the option off, slicing is unchanged.
- The current G-code is a single stream with tool changes. It shows the split in the
  preview but is not printable on a two-gantry machine. Printing needs the two-stream
  export and the collision check in `tools/splitprint/check.py`.
