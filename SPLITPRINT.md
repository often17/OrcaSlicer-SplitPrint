# SPLITPRINT.md — the goal of this fork (read first, every session)

This fork of OrcaSlicer slices **one part** so that **two independent CoreXY
toolheads print it at the same time**. Each head builds its own side. The sides
join in the middle through **interleaved fingers of variable length**. A
**no-go zone** in the middle may hold only one head at a time.

## The machine
- One printer, **two full CoreXY gantries at the same height**, both able to reach
  the whole bed. **One bed, shared Z.** Two control boards (too many steppers for one).
- Both X-beams sit at the same height, so **the gantries can never pass each other**.
  Gantry T0 always stays on the low side of T1 along the split axis. The split line
  is perpendicular to that axis (`split_axis`, default X = left/right like the sketch).
- **Electronics: Klipper on two BTT Octopus Pro v1.1, one per gantry, each with a BTT
  EBB CAN board on its toolhead.**
  - Each Octopus runs its **own Klipper + Moonraker instance**; both instances run on
    one host. One Klipper instance can't drive two independent toolheads (`dual_carriage`
    only does copy/mirror), so two instances are required.
  - Each instance has its own CAN bus (Octopus as USB-CAN bridge) to its own EBB.
  - Shared Z: the real Z steppers live on board A (T0). Klipper's corexy kinematics
    still need a `stepper_z` on board B, so board B gets a dummy one (unused driver
    slot, no motor, position set with `SET_KINEMATIC_POSITION` instead of homing).
    Both streams then carry the same Z moves; only board A's are real.
- **Sync between the two boards: the host coordinator** `tools/splitprint/run_dual.py`.
  It sends each head the G-code of one sync interval through Moonraker
  `POST /printer/gcode/script`, which returns only after the commands have finished
  (verified in the Moonraker docs). Each chunk ends with `M400`. When both calls have
  returned, the barrier is passed and it sends the next pair. If either side errors,
  it calls `/printer/emergency_stop` on **both**.
  - Every sync drains the motion queue (a short stop), so the scheduler must use as
    few syncs as possible: layer ends and no-go-zone handoffs only.
- Klipper has no command that waits for another printer. Don't try to build the
  barrier inside Klipper macros.

```
   T0 (left / red)          NO-GO ZONE           T1 (right / green)
 ===================|<----- width W ----->|=====================
 ---------------------------->|                     row k   (T0 finger, length L_k)
                    |       |<------------------------------ row k+1 (T1 finger)
 ------------------------------------>|              row k+2
                    |    |<--------------------------------- row k+3
                  split-W/2   split   split+W/2
```
- Rows are `split_interface_finger_width` line pitches tall (default 2) on a global grid
  (anchored at 0), so all layers line up. One-line fingers each need a retraction and
  made a 510 mm cylinder slower with two heads than with one.
- `split_interface_interlock`: `both` = rows alternate T0/T1 and swap every
  `split_interface_layers_per_block` layers; `in_plane` = rows alternate, same every
  layer; `layers` = the whole band swaps owner every block of layers.
- A T0 finger reaches `split + L`, a T1 finger reaches `split - L`, with `L` in
  [min, max] from the pattern (alternating / random / sine). `L <= W/2`.

## Hard rules
1. **Collision safety first.** No G-code leaves the slicer unless the checker passes:
   gantry order held, gap ≥ `split_gantry_min_gap`, head footprints never overlap,
   and never both heads in the no-go zone at once. `tools/splitprint/check.py` is the
   independent second check.
2. **Both heads finish layer N before either starts N+1** (shared Z).
3. **Feature off = upstream behaviour.** With `split_print_enable = false`, the G-code
   must match upstream exactly, except the settings comment block at the end (it lists
   the new `split_*` keys). Gate every change.
4. **Keep the upstream diff small.** New logic goes in `src/libslic3r/SplitPrint/`.
   Hooks in upstream files stay small and are marked `// SPLITPRINT:`.
5. Follow `AGENTS.md`:
   - Scaled `coord_t` geometry and the `ClipperUtils.hpp` helpers.
   - Tests in the existing suites (`tests/libslic3r/test_split_print.cpp` for pure
     geometry, `tests/fff_print/` for pipeline and G-code).
   - Translatable strings.
   - Design doc in `docs/HLSD/split_print.md`.
6. Reuse before writing: `IMEXHelpers`/`IMEXZones` (gantries, clearance),
   `apply_mm_segmentation` (per-extruder regions), and `GCodeProcessor` (move timing).
7. Don't guess firmware syntax. Check the Klipper/Moonraker docs before writing
   coordinator or config code.
8. Klipper soft limits can't see the other gantry. Each instance only knows its own
   head, so collision safety rests on the slicer, `check.py` and the coordinator.
   Set each board's `position_min`/`position_max` as tight as the machine allows anyway.

## Out of scope (unless the user asks)
More than 2 heads, curved or angled split lines, multi-material within one side,
supports that cross the split (each support goes to the side it stands on).

## Code map
| Piece | Where |
|---|---|
| Split + fingers geometry (pure) | `src/libslic3r/SplitPrint/Splitter.{hpp,cpp}` |
| One head vs two time | merged preview timeline in `SplitPrint/Export.cpp`, notification in `Plater.cpp` |
| Pipeline hook | `apply_split_print()` next to `apply_mm_segmentation` in `src/libslic3r/PrintObjectSlice.cpp` |
| Options | `PrintConfig.{hpp,cpp}`, `Preset.cpp` lists, `PrintObject::invalidate_state_by_config_options`, `Tab.cpp` |
| Two streams + export | `src/libslic3r/SplitPrint/Export.{hpp,cpp}`, hooks in `Print::export_gcode`, `GCode.cpp`, `BackgroundSlicingProcess.cpp` |
| Near / far phases (parallel printing) | `src/libslic3r/SplitPrint/Phases.{hpp,cpp}`, island loop in `GCode::process_layer` |
| Scheduler + in-slicer check | `src/libslic3r/SplitPrint/Scheduler.{hpp,cpp}` |
| Preview: second head marker | `GCodeViewer::render_markers` |
| Independent checker | `tools/splitprint/check.py` |
| Klipper coordinator (host) | `tools/splitprint/run_dual.py` (runs `check.py` first, refuses to start on failure) |
| Klipper config examples | `tools/splitprint/klipper/printer_t0.cfg`, `printer_t1.cfg` |

## How the split reaches the pipeline
- `PrintApply.cpp`: when `split_print_enable` is on, extruders 1 and 2 are added to
  `painting_extruders`, so each object gets one region per head. `last_split_print`
  forces the regions to rebuild when the setting is toggled.
- `PrintObjectSlice.cpp`: `split_print_segmentation()` builds per-layer shapes per head
  with `Splitter`. `apply_mm_segmentation()` (which now takes the shapes as an
  argument) moves them into those regions. Perimeters, infill and tool ordering then
  run unchanged.
- Export writes `part.T0.gcode` and `part.T1.gcode` (the files the printer runs) and
  the combined `part.gcode`, which has tool changes and is **only for the preview, never
  for printing**. Design: `docs/HLSD/split_print.md`.

## Status
- [x] Clone, branch `splitprint`
- [x] Build tools installed
- [x] Deps build, slicer build (`build_win.bat -s --tests`; VS 2026 Build Tools)
- [x] Splitter geometry + tests (`libslic3r_tests "[SplitPrint]"`, passing)
- [x] Options + GUI
- [x] Pipeline hook + tests (`fff_print_tests "[SplitPrint]"`, passing)
- [x] Milestone M1: fingers interlock (checked on a rendered layer, and in
      `fff_print_tests` "Printed lines fill the no-go zone"). In the GUI, hide **Seams**
      in the legend: both halves' seams line up at the split and look like a gap.
- [x] Test printer **LS01** (`resources/profiles/LS*`): 800×800×800, 2× 0.4, Klipper.
      `0.20mm Standard @LS01` = split off (default, keeps the profile validator's
      filament-change test working); `0.20mm Two-Head Split @LS01` = split on.
- [x] Interlock direction (`both` / `in_plane` / `layers`) + layers per finger + finger width.
- [x] One head vs two time, shown as a notification after slicing (from the scheduled
      streams, so it includes no-go-zone waits).
- Known unrelated failures: 2 Timeutils tests fail by 1 h on UK-timezone machines around DST change.
- [x] Two streams + scheduler + sync markers; heads print near/far phases in parallel
      (300 mm test part: about 42% less time than one head). Preview plays both heads.
- [x] Python checker (neutral format)
- [x] Fork: `origin` = https://github.com/often17/OrcaSlicer-SplitPrint, `upstream` = SoftFever
- [x] Brim and skirt cut at the split line, each head prints its side.
- [x] Klipper coordinator `run_dual.py` + example configs (tested against fake
      Moonraker servers: `python -I tools/splitprint/test_run_dual.py`; the configs'
      pins come from Klipper's Octopus Pro v1.1 and EBB v1.2 samples, not yet run on
      the boards). Run: `run_dual.py part.T0.gcode part.T1.gcode --t0 URL --t1 URL`,
      or `--dry-run` to check only.
- [ ] Real-printer dry run (no filament, heads far apart first)
