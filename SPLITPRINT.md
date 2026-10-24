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
- Rows are one line pitch tall on a global grid (anchored at 0), so all layers line up.
- A row's owner alternates T0/T1. With `split_interface_layer_shift`, the parity also
  flips every layer, so the joint locks in Z.
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
| Pipeline hook | `apply_split_print()` next to `apply_mm_segmentation` in `src/libslic3r/PrintObjectSlice.cpp` |
| Options | `PrintConfig.{hpp,cpp}`, `Preset.cpp` lists, `PrintObject::invalidate_state_by_config_options`, `Tab.cpp` |
| Two streams + scheduler | `src/libslic3r/SplitPrint/Scheduler.{hpp,cpp}`, hook in `GCode.cpp` |
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
- Until Phase 4 the output is one stream with tool changes. That is only good for the
  preview, **never for printing**.

## Status
- [x] Clone, branch `splitprint`
- [x] Build tools installed
- [ ] Deps build, slicer build
- [x] Splitter geometry + tests written (not yet compiled)
- [x] Options + GUI written (not yet compiled)
- [x] Pipeline hook written (not yet compiled). Milestone M1: preview shows split + fingers
- [ ] Two streams + scheduler + sync markers
- [x] Python checker (neutral format)
- [x] Fork: `origin` = https://github.com/often17/OrcaSlicer-SplitPrint, `upstream` = SoftFever
- [ ] Klipper coordinator `run_dual.py` + example configs
- [ ] Real-printer dry run (no filament, heads far apart first)
