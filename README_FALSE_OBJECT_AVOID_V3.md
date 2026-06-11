# FALSE OBJECT / UNKNOWN COLOR AVOID V3

This version keeps the stable line speed from the line-only firmware and adds flexible handling for false/unknown objects.

## Main changes

1. `COLOR_READ_TIMEOUT_MS` is now `10000U`.
2. If the robot stops for the first object but cannot confirm a valid color within 10 seconds, it no longer waits forever.
3. The robot runs the same avoid routine: backup -> turn right -> go forward -> turn left -> search line.
4. After line is found again:
   - false/unknown first object returns to `STATE_LINE_TO_PICK`
   - wrong/unknown target object returns to `STATE_LINE_CARRYING`
5. Added `avoid_return_state` and debug variable `dbg_avoid_return_state`.
6. `STATE_WRONG_COLOR_SEARCH_LINE` now returns to `avoid_return_state`, not always to carrying mode.

## Debug variables

Watch these in Keil:

- `dbg_robot_state`
- `dbg_avoid_return_state`
- `dbg_line5_mask`
- `dbg_line5_search_phase`
- `dbg_line5_lost_count`
- `dbg_distance_cm`
- `dbg_tcs_status`
- `dbg_tcs_c`, `dbg_tcs_r`, `dbg_tcs_g`, `dbg_tcs_b`
- `dbg_color_sample_count`

## Behavior summary

- Valid first object color -> pick object.
- First object unknown/false for 10s -> avoid and continue looking for pickup object.
- Target color matches -> drop object and run to finish marker.
- Target color wrong or unknown for 10s -> avoid and continue carrying/searching.
