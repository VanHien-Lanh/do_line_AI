# Stage 2 Full Robot - Line Recovery V2

Changes in this build:

1. Restored stable line speed close to the original line-only firmware:
   - LINE5_BASE_SPEED_255 = 85
   - LINE5_CARRY_SPEED_255 = 75
   - LINE5_MAX_SPEED_255 = 180

2. Fixed the lost-line behavior.
   The previous integrated code stopped when all 5 sensors lost the line and `line5_remember == 0`, or after the lost-line counter reached the stop threshold. This caused the robot to stand still when it left the track.

3. Added multi-direction line search.
   When all 5 sensors read no line, the robot now continues searching:
   - first uses the remembered direction if available;
   - otherwise moves slowly forward;
   - then sweeps right;
   - then sweeps left;
   - then backs up slightly;
   - repeats until the line is detected again.

4. Removed forced stop in wrong-color line recovery.
   After avoiding a wrong color, the robot keeps searching for the line instead of stopping after a timeout.

5. UART debug is OFF by default for smoother line tracking:
   - ROBOT_UART_DEBUG = 0
   Set it to 1 when you need UART logs.

Watch variables:
- dbg_line5_mask
- dbg_line5_lost_count
- dbg_line5_search_phase
- dbg_line5_remember
- dbg_line5_error
- dbg_line5_correction

Search phase values:
- 0: line found / normal PID
- 1: using remembered direction
- 2: slow forward search
- 3: sweep primary direction
- 4: sweep opposite direction
- 5: slow backup
