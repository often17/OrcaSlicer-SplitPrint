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
Scheduling the head files only reads the print, so it runs on its own thread while the
combined file is written. A tall part reaches hundreds of millions of moves and lines,
far more than fit in memory, so nothing holds a whole file's moves or lines:
- the G-code processor hands its finished moves to a sink (`GCodeProcessor::MoveSink`)
  in large batches instead of keeping them. A head export keeps only the seconds per
  line, moved to the final line numbers once post-processing has inserted its lines;
  the combined export keeps none. The few checks that read every move after processing
  (`check_multi_extruder_gcode_valid`) get what they need from a summary of the moves
  taken;
- the scheduler and its check run a layer at a time: they read both head files a layer
  at a time, write the scheduled lines to temporary files, and replace the head files
  only once the check has passed;
- the line tables that only the G-code window and the line renumbering need are not
  kept, the three exports share one seam placer (`GCode::take_seam_placer`), and the
  preview is built only once the combined export has been freed.
- A head's file holds only its own extrusions, temperatures and tool. It has no `T`
  commands, and the `split_head` placeholder (0 or 1, -1 in the combined file) lets
  start G-code tell the heads apart.
- `; SPLITPRINT_SYNC <id>` markers are barriers: neither head goes past one until
  both have reached it. `L<n>` opens layer n, `L<n>z` follows the shared Z move,
  `L<n>.<k>` are handoffs the scheduler adds inside a layer, and `end` comes before
  each head's own end G-code, which nothing checks.
- Before the first layer each head parks at its own end of the bed along the split
  axis, so both positions are known when checking starts.
- Brim and skirt are cut at the split line, and each head prints the part on its own
  side. A whole skirt loop would take one head around the other.

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

### Layers without stops (`split_timed_layers`)
A sync in the middle of a layer stops both heads over the part, and a nozzle standing
still on a print leaves a mark. A whole layer cannot be kept apart by position alone:
a gap-wide strip at the joint is out of reach for one head while the other works next
to it. So by default the scheduler first tries to print each layer as one interval with
no sync inside it, and keeps the heads apart by time:
- both heads start the layer together, right after the Z move (`L<n>z`), and the
  interval is marked `; SPLITPRINT_TIMED <seconds>` with the time that head is busy;
- with the estimated time of every line, the plan must hold even when one head runs up
  to `0.5 s + split_timing_margin × t` ahead of the other, t being the time since the
  layer started. The planner adds a little more slack of its own (0.1 s and 5 %), so its
  plans pass the check with room to spare;
- when the heads would meet, one head yields: it may travel out of the way first and
  then waits (`G4`, `; SPLITPRINT wait at the layer start`). The other head may travel
  out of the way at the end of its layer (`; SPLITPRINT make room`). Of all the delays
  tried, the plan that ends the layer earliest wins. Waits and travels belong to the
  layer change, never to the middle of a layer;
- each head retracts at the end of the layer (`; SPLITPRINT retract for the layer
  change`) and undoes it before its first extrusion in the next one, so the wait does
  not ooze;
- for Klipper, the velocity limit and square corner velocity the times were estimated
  with are written to each head (`SET_VELOCITY_LIMIT`), so the printer moves the way the
  estimate assumes.

Only a layer no wait can save falls back to syncs; the plater says how many layers did.
Timing errors pile up within a layer, never across layers: every layer starts at a sync.

A part narrow enough that the heads can never work at the same time prints each layer
one head after the other. That costs a little more than one head alone; the times in
the preview legend show it.

`check_streams` checks the result again before anything is written: the gap rule,
Z changing only after a sync and before any extrusion, both heads agreeing on Z, and
no homing after the first sync. For a timed interval it applies the time rule to the
scheduler's planned line times (the G-code processor books a `G4` on the wrong move,
so its own times can't be used there). `tools/splitprint/check.py` is an independent
copy of the same check, which the Klipper coordinator runs before it starts a print. It
has its own time model, built like Klipper's look-ahead: junction speeds from the square
corner velocity, trapezoids with the current acceleration, and short moves held back by
`minimum_cruise_ratio`.

## Klipper coordinator (`tools/splitprint/run_dual.py`)
Each gantry runs its own Klipper and Moonraker instance, and Klipper cannot wait for
another printer, so a host script keeps the heads in step. It:
- refuses to start unless both files carry the same syncs and pass `check.py`, and
  both instances report Klippy `ready`;
- runs T1's start G-code before T0's, so T1 homes towards its end of the bed first;
- sends each sync interval to both heads at once with Moonraker
  `POST /printer/gcode/script`, ending each with `M400`. That call returns once the
  commands have run, so when both calls are back both heads are at the barrier, and
  only then does the next interval go out;
- calls `/printer/emergency_stop` on both instances when either fails or on Ctrl+C;
- after each timed layer, compares how long each head took with its
  `SPLITPRINT_TIMED` time. If one head ran ahead of the other by more than half the
  timing margin, the slicer's times don't match this printer: it stops at that layer
  change with exit code 3. Both heads are idle at the barrier then, so no emergency
  stop is needed.

Each barrier drains both motion queues, which is why the scheduler adds as few syncs
as it can. `tools/splitprint/klipper/` holds example configs for two BTT Octopus Pro
boards with EBB CAN toolheads. The second board has no Z motor: Klipper's corexy
kinematics still needs a `stepper_z`, so it gets an unconnected one whose position
`PRINT_START` sets with `SET_KINEMATIC_POSITION`. Klipper's soft limits only know
their own head, so the limits are set as tight as the gap allows but the slicer,
`check.py` and the coordinator are what keep the gantries apart.

## Preview and time
The combined file has both heads with tool changes, one after the other, so it is
only a preview source and never printable. After the head files are written, both
are processed again and their moves are merged on one timeline: an interval starts
for both heads when the slower one finished the previous interval. Intervals are
merged in parallel: a first pass counts each interval's moves, so each one writes into
its own slice of a result sized exactly. The preview then
plays both heads at once, with a marker on each, and its print time is the two-head
time. In a timed interval each line takes the time the scheduler planned for it. The
combined file's time is kept as the one-head time. The preview legend shows both times
and what two heads save, and the plater shows a notification with both times and the
faster choice.

The preview keeps at most a share of the computer's memory worth of moves per head. A
part with more keeps every second, fourth, ... layer (the first layer always), and the
notification says so; the head files keep every layer. The time of the layers left out
still passes on the timeline. The moves point at lines of the head files, so the G-code
window, which shows the combined file, stays closed.

## Constraints
- Both gantries are at the same height, so they can never pass each other. The split
  axis is the axis they would collide on.
- With the option off, slicing and G-code are unchanged.
- A split print needs layer-by-layer printing, no prime tower, no spiral vase, no
  belt printer, and no Z hop: Z is shared, and only the layer syncs move it.
