# Requirements Document

## Introduction

This feature implements "Change Requirement 1: Minimum-Bit Data Representation" for the IS-02 robot car firmware in `app_program/`. The firmware runs on a Raspberry Pi Pico W (RP2040, built single-core with `SMP=0`) under micro T-Kernel 3, with Pico SDK drivers (`hal_*.c`) and an lwIP MQTT client (`app_net.c`) that runs in the CYW43 radio-owner task.

Today the firmware represents most application data in full-width types: `robot_status_t g_status` (app_types.h) holds every snapshot field as a 32-bit `volatile` word or float. `motion_cmd_t`, `nav_event_t`, `obstacle_profile_t` and `tlm_event_t` (a 20-byte name plus a 72-byte text detail) share one 40-block message pool (os_glue.c), and every block is sized for the largest body. The ISR in `hal_isr.c` shares separate `bool` and `uint32_t` variables with the tasks. Every MQTT payload is text: `tsk_telemetry.c` builds JSON with `jfmt.c` (a telemetry frame is about 300 to 450 bytes, stored in a 64-slot × 512-byte offline ring in `app_net.c`), and commands arrive as text such as `SET:NAME=VALUE`.

The change makes each application data field use only the bits its valid range needs, packs boolean and fault conditions into shared bitmasks, makes access to shared packed data race-free, and replaces the text MQTT payloads with compact binary frames. The existing module and task architecture stays the same, and so do the robot's behaviour and the MQTT topic names. The laptop tools in `MQTT/` (`dashboard.html`, `telemetry_monitor.py`, `is02_tool.py`, `robot_params.py`, `robot_map.py`) are updated so they can decode and encode the binary payloads. The Week 13 evidence must also be produced: a data representation table, a RAM usage comparison, a message size comparison, and correctness and performance evidence.

Out of scope: USB console log lines (`log_line`, `console_status`). These are human-readable diagnostics on the USB serial port. They are not application data exchanged between tasks or over MQTT, so they stay as text.

## Glossary

- **Firmware**: The IS-02 robot application in `app_program/`, built with `build_make/robot.mk` (`APP_ROBOT=1`).
- **Baseline_Build**: The Firmware at the git commit recorded before any change for this feature, used as the reference for every before/after comparison.
- **Revised_Build**: The Firmware after this feature is implemented.
- **Application_Field**: One item of application information held by the Firmware, for example a system state, a wheel speed, a sensor reading, a counter, a flag or a barcode letter.
- **Valid_Range**: The set of values an Application_Field can legitimately take, including any sentinel value such as "no echo" or "none".
- **Resolution**: The smallest step a continuous Application_Field is represented with, for example 0.1 mm/s.
- **Required_Bits**: For an Application_Field, the value ceil(log2(N)), where N is the number of distinct values in the Valid_Range at the stated Resolution (N = 1 needs 0 bits).
- **Packed_Field**: An Application_Field stored in exactly the Required_Bits inside a Storage_Unit, using a C bit-field or explicit masks and shifts.
- **Storage_Unit**: An aligned 8, 16 or 32-bit memory word that holds one or more Packed_Fields.
- **Private_Control_Variable**: A continuous quantity (for example a PI integrator, a filter state or an intermediate geometric result) that only one task reads and writes and that never crosses a task, ISR or network boundary.
- **Fault_Mask**: The single shared bitmask in which each fault condition and each link or sensor loss condition has its own bit.
- **Status_Snapshot**: The shared robot status record that replaces the field layout of `robot_status_t g_status`, which tasks write and TSK_TELEMETRY reads.
- **Task_Shared_Data**: Any Storage_Unit that two or more tasks read or write, including the Status_Snapshot, odometry, heartbeats, the buzzer pattern, the stop reason, and the motion done id and result.
- **ISR_Shared_Data**: Any Storage_Unit that an interrupt handler (`hal_gpio_isr`, `hal_timer_isr`, the IMU INT1 callback or the 10 ms cyclic handler) and a task both access, including encoder counts, odometer, echo timing, barcode edge and raw sample rings, ADC range values, the barcode configuration and the VBAT reading.
- **Inter_Task_Message**: The body of a message sent through `g_mbx_motion`, `g_mbx_nav`, `g_mbx_obstacle` or `g_mbx_tlm`.
- **Message_Pool**: The fixed-size memory pool (`g_mpf_msg`) that provides Inter_Task_Message blocks.
- **Event_Code**: A numeric identifier that replaces the text name of an event (for example `BARCODE`, `STATE`, `EMERGENCY_STOP`).
- **Reason_Code**: A numeric identifier that replaces a fixed text reason, for example the E-stop or ERROR reason stored in `g_stop_reason`, the avoidance abort reason, or an ack result.
- **Wire_Codec**: The Firmware module that encodes and decodes binary MQTT payloads using explicit masks and shifts.
- **Telemetry_Frame**: The binary payload published on `is02/telemetry` at 2 Hz.
- **Heartbeat_Frame**: The binary payload published on `is02/status` at 1 Hz.
- **Event_Frame**: The binary payload published on `is02/event`.
- **Ack_Frame**: The binary payload published on `is02/ack` in reply to a command or configuration message.
- **Command_Frame**: The binary payload received on `is02/cmd`.
- **Map_Frame**: The binary barcode map payload, retained on `is02/map` and also carried inside a MAP Command_Frame.
- **Config_Frame**: The binary calibration payload retained on `is02/config`.
- **Format_Version**: A version number carried in every binary MQTT payload that identifies the layout used.
- **Protocol_Specification**: The repository document that defines the bit layout, byte order, Valid_Range, Resolution and Required_Bits of every binary MQTT payload, and the meaning of every Event_Code, Reason_Code and Parameter_ID.
- **Parameter_ID**: A numeric identifier for each run-time parameter in the `PARAMS[]` table in `app_params.c` (55 entries in the Baseline_Build).
- **Parameter_Store**: The run-time parameter storage that replaces the field layout of `app_params_t g_params`.
- **Frame_Ring**: The offline telemetry buffer in `app_net.c` (`g_frames`, 64 slots in the Baseline_Build).
- **Laptop_Codec**: The shared decoding and encoding code that the laptop tools use for binary payloads, with one implementation in Python and one in JavaScript.
- **Dashboard**: The browser dashboard `MQTT/dashboard.html`, which connects to Mosquitto over WebSockets on port 9001.
- **Telemetry_Monitor**: The console monitor `MQTT/telemetry_monitor.py`, which runs checks L1 to L6 and writes the CSV and log files.
- **Laptop_Tools**: `MQTT/dashboard.html`, `MQTT/telemetry_monitor.py`, `MQTT/is02_tool.py`, `MQTT/robot_params.py` and `MQTT/robot_map.py`.
- **Host_Test_Suite**: Tests that build and run on the laptop (PC) with no robot hardware, covering the Wire_Codec, the Laptop_Codec and the packed field accessors.
- **Data_Representation_Table**: The repository document that lists every Application_Field with its Valid_Range, Resolution, Required_Bits and implementation representation.
- **Evidence_Report**: The repository documents that hold the RAM usage comparison, the message size comparison and the correctness and performance evidence for the Week 13 submission.
- **Critical_Section**: A code region in which preemption by the competing context is prevented. The competing context is the interrupt handler when interrupts are masked (T-Kernel `DI`/`EI` or SDK `save_and_disable_interrupts`), or another task when dispatch is disabled (`tk_dis_dsp`/`tk_ena_dsp`).
- **Saturation_Counter**: The single firmware-wide count of stores that were clamped to a Valid_Range limit or replaced because the value was NaN (the "saturation counter" of Requirement 1 and the saturation count of the Heartbeat_Frame).

## Requirements

### Requirement 1: Field sizing rule

**User Story:** As a student developer, I want one rule that decides how many bits each Application_Field gets, so that every representation choice is consistent and can be checked.

#### Acceptance Criteria

1. THE Firmware SHALL store each Application_Field that is not a Private_Control_Variable as a Packed_Field whose bit width equals the Required_Bits recorded for that Application_Field in the Data_Representation_Table.
2. THE Firmware SHALL use, for each continuous Application_Field that the Baseline_Build publishes in the JSON telemetry frame with d decimal places, a Resolution no larger than 10^−d of the published unit (for example 0.1 mm/s or finer for a speed published in mm/s with 1 decimal place, and 1 unit or finer for a field published with 0 decimal places).
3. THE Firmware SHALL represent each sentinel value (for example "no echo", "no barcode", "not calibrated") as one reserved code inside the Packed_Field of the Application_Field, where the reserved code is counted as one of the N values used to compute the Required_Bits, differs from the code of every non-sentinel value, and is recorded with its meaning in the Data_Representation_Table.
4. IF a value to be stored lies outside the Valid_Range of the Application_Field, THEN THE Firmware SHALL store the nearest non-sentinel limit of the Valid_Range and increment the saturation counter by 1.
5. THE Firmware SHALL keep each Private_Control_Variable in its computation type (float32 or int32).
6. THE Data_Representation_Table SHALL list each Private_Control_Variable with its name, the single task that reads and writes it, its computation type, and the reason the Private_Control_Variable keeps its computation type.
7. THE Firmware SHALL contain, for each packed type, a compile-time assertion that the size in bytes of the type equals the size recorded in the Data_Representation_Table, so that the Firmware build fails when the two sizes differ.
8. WHEN the Firmware stores a continuous value that lies inside the Valid_Range into a Packed_Field, THE Firmware SHALL store the code of the nearest multiple of the Resolution, rounding exact halfway values away from zero, so that the decoded value differs from the original value by no more than half of the Resolution.
9. IF a value to be stored in a continuous Packed_Field is NaN, THEN THE Firmware SHALL store the sentinel code of the Application_Field when the Application_Field has one, store the lower limit of the Valid_Range otherwise, and increment the saturation counter by 1.
10. THE Firmware SHALL keep one firmware-wide saturation counter that is 0 at boot, increases by 1 on each saturated or NaN store, and holds at 65,535 instead of wrapping to 0.

### Requirement 2: Vehicle and controller states

**User Story:** As a student developer, I want every state machine and enumeration stored in the fewest bits, so that state data matches the examples in the change requirement.

#### Acceptance Criteria

1. THE Firmware SHALL store the system state (`sys_state_t`, the 8 values `SYS_INIT` to `SYS_ERROR`, with `SYS_STATE_COUNT` excluded from the Valid_Range) as a 3-bit Packed_Field.
2. THE Firmware SHALL store the line follower state (`line_state_t`, the 7 values `LINE_CENTERED` to `LINE_LOST`) as a 3-bit Packed_Field.
3. THE Firmware SHALL store the hump profiler state (`hump_state_t`, the 4 values `HUMP_FLAT` to `HUMP_DESCENT`) as a 2-bit Packed_Field.
4. THE Firmware SHALL store the motion event (`motion_event_t`, the 8 values `MEV_STATIONARY` to `MEV_IMPACT`, with `MEV_COUNT` excluded from the Valid_Range) as a 3-bit Packed_Field.
5. THE Firmware SHALL store the buzzer pattern (the 7 values `BUZZ_NONE` = 0 to `BUZZ_WARN` = 6) as a 3-bit Packed_Field.
6. THE Firmware SHALL store the navigation action (`nav_action_t`, the 5 values `ACT_NONE` to `ACT_RIGHT`) as a 3-bit Packed_Field and the motion result (`motion_result_t`, the 3 values `MOTION_OK`, `MOTION_STOPPED` and `MOTION_TIMEOUT`) as a 2-bit Packed_Field.
7. THE Firmware SHALL store each signed direction value whose Valid_Range is −1, 0 and +1 (`line_ctx_t.side`, `search_ctx_t.dir`, `bypass_plan_t.side`) as a 2-bit signed Packed_Field, and reading the field SHALL return the same signed value −1, 0 or +1 that was last written.
8. THE Firmware SHALL store every `bool` member of a controller context (including `line_ctx_t.b_last_curve`, `line_ctx_t.b_both_prev` and `search_ctx_t.b_second`) and every file-scope `bool` state variable of a task (including `g_searching` and `g_det_on` in `tsk_line.c`) as a 1-bit Packed_Field, with all 1-bit Packed_Fields of one context or one task file grouped into a single Storage_Unit of the smallest width (8, 16 or 32 bits) that holds them.
9. THE Firmware SHALL store the E-stop and ERROR reason held in `g_stop_reason` as a 4-bit Reason_Code Packed_Field whose Valid_Range is the 10 values: none, remote E-stop (MQTT), BTN2 stop, collision (IMU), obstacle closer than 8 cm, watchdog (control task stopped), calibration missing, IMU fault, battery critical, and fault reported.
10. WHEN the Firmware prints the stop reason on the USB console status line, THE Firmware SHALL print the same reason text that the Baseline_Build prints for the same E-stop or ERROR cause.
11. WHEN any Packed_Field listed in criteria 1 to 9 is written with a value inside its Valid_Range, THE Firmware SHALL return that exact value on the next read of the field and SHALL leave every other Packed_Field in the same Storage_Unit unchanged, as verified by the Host_Test_Suite for every value in each Valid_Range.
12. IF a write to a Packed_Field listed in criteria 1 to 9 supplies a value outside the field's Valid_Range, THEN THE Firmware SHALL reject the write, keep the field's previous value and leave every other Packed_Field in the same Storage_Unit unchanged.
13. IF an enumeration listed in criteria 1 to 9 is given more values than its Packed_Field width can hold, THEN THE Firmware build SHALL fail with a compile-time error that names the affected field.

### Requirement 3: Shared fault bitmask

**User Story:** As a student developer, I want all fault and loss conditions in one bitmask, so that faults are not held in separate integer or boolean variables.

#### Acceptance Criteria

1. THE Fault_Mask SHALL hold one bit for each of the 13 fault conditions defined by the `FAULT_*` constants of the Baseline_Build (`FAULT_IMU_I2C` through `FAULT_TELEMETRY` in `app_types.h`), each at the same bit position (0 to 12) as its Baseline_Build constant, including `FAULT_ULTRASONIC` and `FAULT_TELEMETRY`, which the Baseline_Build defines but never sets.
2. THE Fault_Mask SHALL hold one bit each for Wi-Fi link loss, MQTT session loss and ultrasonic no-echo at bit positions 13, 14 and 15, for 16 defined bits in total, and at start-up, before the first `app_net_poll` call, the Wi-Fi link loss and MQTT session loss bits SHALL be 1 and the ultrasonic no-echo bit SHALL be 0.
3. WHEN the existing encoder loss check in TSK_MOTION finds a wheel with duty above `ENC_LOSS_DUTY` (0.30) and zero encoder ticks for at least `ENC_LOSS_MS` (200 ms), THE Firmware SHALL set the `FAULT_ENCODER_LOSS` bit as the only motor stall and encoder fault bit, with no separate stall bit, and SHALL request the ERROR state as the Baseline_Build does.
4. THE Firmware SHALL store the Fault_Mask in one aligned 16-bit Storage_Unit, which is the smallest of 8, 16 or 32 bits that holds all 16 defined bits, and every reader SHALL get the whole Fault_Mask from a single 16-bit read.
5. WHEN any task sets or clears one or more Fault_Mask bits, THE Firmware SHALL perform the read-modify-write of the Fault_Mask inside a Critical_Section that excludes every other context that writes the Fault_Mask, so that after two contexts set or clear different bits at the same time, every requested change is present and no other bit has changed value.
6. THE Firmware SHALL replace each separate boolean or integer variable that duplicates a Fault_Mask condition with a read of the corresponding Fault_Mask bit, including `g_wifi_up` and `g_mqtt_up` in `app_net.c` (where `net_wifi_up` and `net_mqtt_up` return the inverse of the Wi-Fi link loss and MQTT session loss bits) and the `b_dead` copy of `FAULT_IMU_DEAD` in `tsk_imu.c`.
7. WHEN `app_net_poll` detects that the Wi-Fi link condition (netif up, link up and an IPv4 address assigned) or the MQTT session condition has changed, THE Firmware SHALL set the matching loss bit when the condition becomes false and clear it when the condition becomes true, before that `app_net_poll` call returns.
8. WHEN TSK_OBSTACLE gets an ultrasonic reading from `hal_us_read_cm`, THE Firmware SHALL set the ultrasonic no-echo bit if the result is below 0, and clear the bit if the result is 0 or above, before TSK_OBSTACLE uses that reading.
9. IF the Wi-Fi link loss, MQTT session loss or ultrasonic no-echo bit is set, THEN THE Firmware SHALL make the same state decisions as the Baseline_Build makes under the same conditions. This includes the TSK_SAFETY check that leaves ERROR only when `FAULT_CONFIG` is the only fault, which SHALL compare only the 13 Baseline_Build fault bits (positions 0 to 12).

### Requirement 4: Motion control and encoder data

**User Story:** As a student developer, I want motion commands, odometry and encoder data packed to their ranges, so that the motion path uses the minimum bits without changing how the car drives.

#### Acceptance Criteria

1. THE Firmware SHALL encode each motion command with a type field of 3 bits (Valid_Range `MC_LINE` = 1 to `MC_STOP` = 5), a source field of 2 bits (Valid_Range `SRC_LINE` = 0 to `SRC_SAFETY` = 3), and a 16-bit command id field (Valid_Range 0 to 65535, where 0 means "not posted").
2. THE Firmware SHALL carry in each motion command only the parameter fields used by the command type of that message: left and right wheel speeds for `MC_LINE`; turn angle, speed and timeout for `MC_TURN`; distance, speed and timeout for `MC_BYPASS`; the speed limit scale for `MC_SPEED_LIMIT`; and no parameter fields for `MC_STOP`.
3. THE Firmware SHALL represent motion command wheel speeds and primitive speeds at a Resolution of 0.1 mm/s over the Valid_Range -250.0 to +250.0 mm/s (`SPEED_MAX_MM_S`), turn angles at 0.1 degree over -360.0 to +360.0 degrees, distances at 0.1 mm, timeouts at 1 ms, and the speed limit scale at 0.01 over 0.00 to 1.00, with the distance and timeout Valid_Ranges taken from the Data_Representation_Table.
4. WHEN TSK_MOTION_CTRL decodes a motion command, THE Firmware SHALL reproduce each parameter value within half of the Resolution of the parameter, as shown by a Host_Test_Suite round-trip test that covers the minimum, maximum and zero value of every parameter of every command type.
5. THE Firmware SHALL store the motion done id (16 bits) and the motion result (2 bits, `MOTION_OK` to `MOTION_TIMEOUT`) in one 32-bit Storage_Unit that TSK_MOTION_CTRL writes with a single aligned store and `motion_wait` reads with a single aligned load, so that `motion_wait` never pairs a done id with the result of a different primitive.
6. THE Firmware SHALL store the shared odometry record (`odometry_t`) with each field packed to the Valid_Range and Resolution recorded in the Data_Representation_Table, and `odometry_get` SHALL return all fields from the same `odometry_put` call by accessing the record inside a Critical_Section.
7. THE Firmware SHALL store the two encoder direction flags (`g_enc_rev`, right and left wheel) as 1-bit Packed_Fields in one Storage_Unit, written only by `hal_enc_set_reversed` with a single aligned store.
8. THE Firmware SHALL keep the speed controller internals (`wheel_ctl_t` target, integrator and duty) as Private_Control_Variables at their Baseline_Build width, accessed only by TSK_MOTION_CTRL.
9. IF a caller of the motion API passes a parameter value outside the Valid_Range of that parameter, THEN THE Firmware SHALL encode the nearest Valid_Range bound in the motion command and post the command.
10. WHEN the Revised_Build and the Baseline_Build execute the same motion API call with the same inputs, THE Firmware SHALL compute the same `MC_TURN` or `MC_BYPASS` target encoder count, within ±1 count, and the same timeout, within ±1 ms.

### Requirement 5: Line and barcode sensing data

**User Story:** As a student developer, I want line sensor, ADC and barcode data packed to their ranges, so that sensing data uses the minimum bits.

#### Acceptance Criteria

1. THE Firmware SHALL represent each of the left and right digital line sensor readings as a 2-bit Packed_Field with exactly three used codes: 1 = black, 0 = white, and −1 = polarity not set, where "polarity not set" means the LINE_DO_BLACK_HIGH parameter is below 0. The fourth 2-bit code SHALL never be stored. Reading the field SHALL return the value −1, 0 or 1.
2. THE Firmware SHALL represent each barcode sensor ADC value, ADC minimum, ADC maximum and barcode switch threshold (black and white) as a 12-bit Packed_Field with Valid_Range 0 to 4095. For every value from 0 to 4095, reading the field back SHALL return the value that was written.
3. THE Firmware SHALL represent the last decoded barcode character as a 6-bit index. Codes 0 to 42 SHALL map one-to-one to the 43 Code 39 data characters (0 to 9, A to Z, '-', '.', space, '$', '/', '+', '%') in the order listed in the Data_Representation_Table. Code 63 SHALL mean "none". Codes 43 to 62 SHALL never be stored. When the index is converted back to a character, the result SHALL be the same character the Baseline_Build stores for the same decode, or 0 for "none".
4. THE Firmware SHALL store each barcode edge ring entry (`hal_bc_edge_t`) with the black/white level as a 1-bit Packed_Field and the odometer value as a 31-bit Packed_Field in the same 32-bit Storage_Unit. The odometer field SHALL hold hal_odometer() modulo 2^31 at the moment of the edge. The edge ring SHALL keep its Baseline_Build capacity of 64 entries.
5. THE Firmware SHALL store each raw barcode sample ADC value in the raw sample ring as a 12-bit Packed_Field. The ring SHALL keep its Baseline_Build capacity of 256 entries. Each popped sample SHALL return the same ADC value that the 1 kHz timer interrupt read for that sample.
6. THE Firmware SHALL store the barcode sampling flags (`g_bc_enabled`, `g_bc_black_high`, `g_bc_black`, `g_raw_enabled`) as 1-bit Packed_Fields in a single shared Storage_Unit.
7. WHEN a task changes one of the barcode sampling flags (through `hal_barcode_config` or `hal_barcode_raw`), THE Firmware SHALL perform the read-modify-write of the flags' Storage_Unit inside a Critical_Section that masks interrupts, so that the other three flags keep the values they held immediately before the change.
8. IF no barcode ADC sample was taken between two consecutive calls to `hal_ir_adc_range`, THEN THE Firmware SHALL return the last barcode ADC value as both the minimum and the maximum, the same result as the Baseline_Build, without storing any value above 4095 as a "no sample" marker.
9. WHEN TSK_LINE_BARCODE processes a recorded barcode edge sequence (BARCODE_DETECT = 0) or raw sample sequence (BARCODE_DETECT = 1), THE Firmware SHALL produce the same decode result (decoded, failed or none), decoded character and read direction (forward or reversed) as the Baseline_Build for that sequence. The test set SHALL include each of the 43 Code 39 data characters read both forward and reversed.

### Requirement 6: Obstacle detection data

**User Story:** As a student developer, I want ultrasonic readings and the obstacle profile packed to their ranges, so that obstacle data uses the minimum bits.

#### Acceptance Criteria

1. THE Firmware SHALL represent each ultrasonic distance held in Task_Shared_Data or in an Inter_Task_Message (the last forward reading and the 5 coarse scan readings in the Status_Snapshot, and the closest-point distance and 5 coarse scan readings in the obstacle profile message) as a 13-bit Packed_Field that holds 0 to 431.0 cm (the limit set by the 25 ms echo timeout) at a Resolution of 0.1 cm, rounded to the nearest 0.1 cm, plus one reserved code for "no echo".
2. IF an ultrasonic reading is "no echo" (no falling Echo edge was timed, or the timed echo exceeds 25,000 µs), THEN THE Firmware SHALL store the reserved "no echo" code in that Packed_Field, never a distance value.
3. THE Firmware SHALL represent the chosen bypass side (`'L'`, `'R'` or none) as a 2-bit Packed_Field in which each of the 3 values has a distinct code listed in the Data_Representation_Table, and SHALL never write the fourth code.
4. THE Firmware SHALL encode the obstacle profile message posted to `g_mbx_obstacle` with the validity flag as a 1-bit Packed_Field, the chosen bypass side as the 2-bit Packed_Field of criterion 3, the left and right edge angles over a Valid_Range of -60.0 to +60.0 degrees at a Resolution of 0.1 degree, and the lateral positions, width, offset, clearances and coarse scan readings at a Resolution of 0.1 cm. Each value SHALL be a Packed_Field of the Required_Bits for the Valid_Range recorded for it in the Data_Representation_Table.
5. IF an obstacle profile value lies outside the Valid_Range recorded for it in the Data_Representation_Table, THEN THE Firmware SHALL store the nearest bound of that Valid_Range in the obstacle profile message and leave the working obstacle profile `g_prof` in TSK_OBSTACLE_SCAN unchanged.
6. WHEN TSK_NAV_PLANNER receives an obstacle profile message, THE Firmware SHALL decode each value that lies within its Valid_Range to within half of the Resolution of that value. It SHALL decode the validity flag, the chosen bypass side and every "no echo" code exactly, so that a "no echo" reading is never decoded as a distance.
7. THE Firmware SHALL keep the working obstacle profile and scans used by the bypass planner inside TSK_OBSTACLE_SCAN (`g_prof`, `g_fine`, `g_view`) as Private_Control_Variables in their Baseline_Build representation. Given the same fine scan and coarse sweep inputs, the computed profile, the chosen bypass side and the bypass plan SHALL then be identical to the Baseline_Build values.
8. THE Firmware SHALL store the obstacle counter, the barcode counter and the hump counter in the Status_Snapshot, each as a Packed_Field of the Required_Bits for the per-run maximum recorded in the Data_Representation_Table.
9. IF an obstacle, barcode or hump event occurs while the matching counter already holds its per-run maximum, THEN THE Firmware SHALL keep that counter at its per-run maximum, never wrapping to a lower value, and SHALL leave the other Packed_Fields in the same Storage_Unit unchanged.

### Requirement 7: Hump estimation data

**User Story:** As a student developer, I want pitch and hump height data packed to their ranges, so that hump estimation data uses the minimum bits.

#### Acceptance Criteria

1. THE Firmware SHALL store the pitch angle in the Status_Snapshot as an 11-bit signed Packed_Field in units of 0.1 degree, with a Valid_Range of −90.0 to +90.0 degrees (−900 to +900 units, 1801 values).
2. THE Firmware SHALL store the current hump height in the Status_Snapshot as a 15-bit signed Packed_Field in units of 0.1 mm with a Valid_Range of −1500.0 to +1500.0 mm (30001 values), and SHALL store the hump peak and the run maximum hump height each as a 14-bit unsigned Packed_Field in units of 0.1 mm with a Valid_Range of 0.0 to 1500.0 mm (15001 values), where the 1500 mm bound is the hump length at which the hump profiler abandons a hump, and the Data_Representation_Table SHALL list each of these three fields with its Valid_Range, Resolution and Required_Bits.
3. THE Firmware SHALL store each hump profiler sample counter (`n_up`, `n_down`, `n_level`) as an unsigned Packed_Field with a Valid_Range of 0 to the larger of `HUMP_START_SAMPLES` and `HUMP_END_SAMPLES` (0 to 5, 3 bits, for the Baseline_Build values 3 and 5), and WHEN a counter that already holds its maximum value is incremented, THE Firmware SHALL keep the counter at its maximum value instead of wrapping to 0.
4. THE Firmware SHALL store the turn rate in the Status_Snapshot as a 14-bit signed Packed_Field in units of 0.1 degree per second, with a Valid_Range of −720.0 to +720.0 degrees per second (14401 values).
5. WHEN TSK_TERRAIN_IMU writes the pitch angle, the current hump height, the hump peak, the run maximum hump height or the turn rate to the Status_Snapshot, THE Firmware SHALL store the value rounded to the nearest 0.1-unit step, with exact halves rounded away from zero.
6. IF a pitch angle, hump height, hump peak, run maximum hump height or turn rate value to be stored lies outside the Valid_Range of its Packed_Field, THEN THE Firmware SHALL store the nearest Valid_Range bound and SHALL leave the other Packed_Fields in the same Storage_Unit unchanged.
7. WHEN the same sequence of pitch samples and distance steps is fed to the hump profiler of the Baseline_Build and of the Revised_Build, THE Revised_Build SHALL produce HUMP_START, HUMP_PEAK and HUMP_END at the same sample indices as the Baseline_Build, the valid or false-slope result SHALL match for every HUMP_END, and every reported hump height SHALL differ from the Baseline_Build value by no more than 0.1 mm.

### Requirement 8: Inter-task queue messages

**User Story:** As a student developer, I want mailbox messages packed, so that the message pool and each message use the minimum bytes.

#### Acceptance Criteria

1. THE Firmware SHALL encode each navigation event message in one byte as a 3-bit type holding the `nav_event_type_t` values 1 (`NAV_BARCODE`) to 7 (`NAV_STOP`) and a 5-bit barcode letter holding 26 codes for A to Z plus one reserved code for "none".
2. THE Firmware SHALL encode each telemetry event message as an Event_Code Packed_Field followed by the typed binary arguments that the Protocol_Specification defines for that Event_Code, with the Event_Code width equal to the Required_Bits for the number of distinct event names that the Baseline_Build passes to `post_event`.
3. THE Firmware SHALL replace the detail text of each Baseline_Build `post_event` call with a Reason_Code, a state code, a barcode letter code or typed binary arguments, so that the Laptop_Codec can rebuild the Baseline_Build detail text from the Protocol_Specification, with each numeric value at a Resolution equal to or finer than the decimal places the Baseline_Build printed for that value.
4. THE Firmware SHALL set the Message_Pool block size to the size of the kernel message header plus the size of the largest packed Inter_Task_Message body, including array arguments at their maximum element count, rounded up to the next multiple of 4 bytes.
5. THE Firmware SHALL keep the four mailboxes (`g_mbx_motion`, `g_mbx_nav`, `g_mbx_obstacle`, `g_mbx_tlm`), the 40-block count of the Message_Pool, and the Baseline_Build rule that the receiving task releases each block to the Message_Pool after copying the message body.
6. FOR ALL Inter_Task_Message values with every field inside its Valid_Range, THE Host_Test_Suite SHALL show that decoding an encoded message gives each discrete field (type, source, Event_Code, Reason_Code, letter, flag, side) equal to the original and each continuous field within half of its Resolution of the original (round-trip property).
7. IF the Message_Pool has no free block when a task posts an Inter_Task_Message, THEN THE Firmware SHALL return false from the post function without waiting, send no message, and leave the sending task's state unchanged, as the Baseline_Build does.
8. IF a navigation event is posted with a barcode letter that is neither A to Z nor "none", THEN THE Firmware SHALL return false from `post_nav` and post no message to `g_mbx_nav`.
9. IF an array argument of a telemetry event (for example the `BC_DBG` widths or the `FINE_SCAN` readings) has more elements than the maximum count that the Protocol_Specification defines for its Event_Code, THEN THE Firmware SHALL post the elements in consecutive messages of that Event_Code, each holding no more than the maximum count, with the same per-event limits as the Baseline_Build (at most 6 `BC_DBG` width parts per window and 11 points per `FINE_SCAN` event).

### Requirement 9: Safe access to packed data shared between tasks

**User Story:** As a student developer, I want packed data shared between tasks to be race-free, so that packing never corrupts a neighbouring field.

#### Acceptance Criteria

1. THE Firmware SHALL assign each Storage_Unit of Task_Shared_Data either exactly one writing task or a Critical_Section with dispatch disabled around every write, and THE Data_Representation_Table SHALL record, for each such Storage_Unit, its writing task or tasks, its reading tasks and which of the two protection methods applies.
2. WHEN a task updates one Packed_Field inside a Storage_Unit of Task_Shared_Data that another task also writes, THE Firmware SHALL perform the read, modify and write inside one Critical_Section with dispatch disabled (`tk_dis_dsp`/`tk_ena_dsp`), and SHALL re-enable dispatch on every exit path of that Critical_Section.
3. WHEN a single writing task updates a Storage_Unit of Task_Shared_Data, THE Firmware SHALL compose the complete new value of the Storage_Unit in a local variable and write it to the Storage_Unit with one naturally aligned store of the Storage_Unit's width (8, 16 or 32 bits, at an address that is a multiple of that width in bytes).
4. WHEN a task reads a Storage_Unit of Task_Shared_Data, THE Firmware SHALL read the Storage_Unit with one naturally aligned load of the Storage_Unit's width and extract every needed Packed_Field from that one loaded value, with no second load of the same Storage_Unit for the same use.
5. THE Firmware SHALL provide access to packed Task_Shared_Data only through accessor functions or macros declared in a plain C header that includes only C standard headers and other Firmware headers that include no micro T-Kernel or Pico SDK header. The header SHALL compile unchanged in the micro T-Kernel task code, the Pico SDK driver code and the Host_Test_Suite.
6. WHILE a task holds a Critical_Section with dispatch disabled for Task_Shared_Data, THE Firmware SHALL make no kernel call that can move the calling task into the WAITING state, and SHALL keep the Critical_Section no longer than 10 µs as measured with the RP2040 microsecond timer.
7. IF a set of Packed_Fields that must be read as one consistent set spans more than one Storage_Unit (each such set listed in the Data_Representation_Table, including the odometry fields and the motion done id with its result), THEN THE Firmware SHALL perform every write and every read of that set inside one Critical_Section with dispatch disabled, so that a reader never gets fields from two different updates.
8. WHEN the on-target concurrency test runs two tasks at different priorities, with the higher-priority task woken every 1 ms, and each task updates a different Packed_Field of one shared Storage_Unit 10,000 times, THE Firmware SHALL end the test with each updated Packed_Field equal to the last value its task wrote and every other Packed_Field in that Storage_Unit equal to its value before the test.

### Requirement 10: Safe access to packed data shared between an ISR and a task

**User Story:** As a student developer, I want packed data shared with interrupt handlers to be race-free, so that encoder edges, echo timing and barcode samples are never lost or corrupted.

#### Acceptance Criteria

1. WHEN a task performs a read-modify-write on a Storage_Unit of ISR_Shared_Data, or reads a value of ISR_Shared_Data that spans more than one Storage_Unit, THE Firmware SHALL perform that access inside an interrupt-masked Critical_Section that masks every interrupt handler that accesses those Storage_Units, for the whole duration of the access.
2. THE Firmware SHALL keep each interrupt-masked Critical_Section used for ISR_Shared_Data at or below 10 µs. The duration is measured with the RP2040 microsecond timer from just before interrupts are masked to just after they are unmasked, over at least 1,000 entries of each Critical_Section during a run that includes driving at `SPEED_MAX_MM_S`, and the maximum observed duration of each Critical_Section is recorded in the Evidence_Report.
3. THE Firmware SHALL place Packed_Fields written by an interrupt handler and Packed_Fields written by a task in the same Storage_Unit only where every task write to that Storage_Unit is made inside a Critical_Section as stated in criterion 1.
4. THE Firmware SHALL keep each single-producer, single-consumer ring index (barcode edge ring, raw sample ring, command slots, Frame_Ring) in a Storage_Unit written only by the producer or only by the consumer, with the producer writing every field of an entry before it advances the head index and the consumer reading every field of an entry before it advances the tail index.
5. IF a field of ISR_Shared_Data would need a read-modify-write that shares a Storage_Unit with a field written from a different context and cannot be protected within the 10 µs bound of criterion 2, THEN THE Firmware SHALL give that field its own aligned Storage_Unit that holds no other Application_Field.
6. THE Data_Representation_Table SHALL record, for each field given its own Storage_Unit under criterion 5, the context that writes the field and the reason it could not share a Storage_Unit.
7. WHILE the robot drives at `SPEED_MAX_MM_S` over the same marked straight 1,000 mm course, repeated 5 times with each build, THE Firmware SHALL report a mean per-wheel encoder transition count within 1 % of the Baseline_Build mean for that course, with the per-run increase of `hal_enc_errors` no greater than the largest per-run increase seen with the Baseline_Build.
8. IF the barcode edge ring or the raw sample ring is full when the interrupt handler has a new entry, THEN THE Firmware SHALL discard the new entry, leave every unread entry and both ring indexes unchanged, and increase the `hal_barcode_overruns` count by 1.
9. WHEN the robot passes over the same Code 39 barcode 5 times at the same speed used for the Baseline_Build test, THE Firmware SHALL decode the same characters as the Baseline_Build on every pass, and the total increase of `hal_barcode_overruns` over the 5 passes SHALL be no greater than the Baseline_Build total.
10. WHEN the robot is stationary with the ultrasonic sensor facing a flat target at 20 cm and then at 100 cm, THE Firmware SHALL report a median of 20 consecutive distance readings within ±1 cm of the Baseline_Build median for the same set-up.

### Requirement 11: Binary telemetry frame

**User Story:** As a student developer, I want telemetry sent as a compact binary frame, so that MQTT carries binary fields instead of text values.

#### Acceptance Criteria

1. THE Wire_Codec SHALL encode each Telemetry_Frame as the Format_Version header (Requirement 16) followed by binary Packed_Fields, written with explicit masks and shifts, in the field order, byte order and bit order defined by the Protocol_Specification.
2. THE Telemetry_Frame SHALL carry every field of the Baseline_Build JSON telemetry frame: sequence number, robot time, state, left and right wheel speeds, left and right duties, left and right encoder counts, distance, left and right line sensors, barcode ADC with its minimum and maximum over the last 0.5 s and its black and white thresholds, last barcode, pitch, hump height, peak and maximum, ultrasonic distance, the five coarse scan readings, bypass side, motion event, turn rate, Fault_Mask, battery voltage, RSSI and Frame_Ring fill level.
3. THE Wire_Codec SHALL encode each Telemetry_Frame field at a Resolution no coarser than the Baseline_Build JSON value: 0.1 unit for wheel speed (mm/s), duty (%), distance (mm), pitch (degrees), hump height, peak and maximum (mm), ultrasonic distance and coarse scan readings (cm) and turn rate (degrees per second), and 1 unit for every other numeric field.
4. THE Wire_Codec SHALL encode the sequence number as a 15-bit Packed_Field that increases by 1 modulo 32,768 for each Telemetry_Frame produced (wrap period 16,384 s, about 4.55 hours, at 2 Hz), and the robot time as a 24-bit Packed_Field in milliseconds since boot, modulo 16,777,216 (wrap period about 4.66 hours).
5. THE Wire_Codec SHALL produce every Telemetry_Frame with one fixed length in bytes. That length SHALL be recorded in the Protocol_Specification and SHALL equal the header bits plus the Required_Bits of every field, rounded up to the next whole byte, with every padding bit set to 0.
6. THE Firmware SHALL make each Frame_Ring slot's data area equal to the fixed Telemetry_Frame length and SHALL keep the Baseline_Build's capacity of 64 Telemetry_Frames.
7. WHILE the MQTT session is up, THE Firmware SHALL publish one Telemetry_Frame on `is02/telemetry` every 500 ms (2 Hz). IF TSK_TELEMETRY falls more than one 500 ms period behind, THEN THE Firmware SHALL skip the missed periods and not send a burst of catch-up frames, as the Baseline_Build does.
8. IF a Telemetry_Frame is posted while the Frame_Ring already holds 64 frames, THEN THE Firmware SHALL discard the oldest buffered frame, store the new frame, and increase the dropped-frame count by exactly 1.
9. WHEN the MQTT session comes up again after an offline period, THE Firmware SHALL publish the buffered Telemetry_Frames oldest first, each byte-for-byte identical to the frame as encoded.
10. IF a Telemetry_Frame field has no current value (no barcode decoded, no bypass side chosen, no ultrasonic echo, or barcode sampling off for the thresholds), THEN THE Wire_Codec SHALL encode that field as the sentinel code the Protocol_Specification defines for it.

### Requirement 12: Binary heartbeat, event and ack frames

**User Story:** As a student developer, I want status, event and ack messages sent in binary, so that every robot-to-laptop payload uses compact binary fields.

#### Acceptance Criteria

1. WHEN TSK_TELEMETRY publishes the 1 Hz heartbeat, THE Wire_Codec SHALL encode the Heartbeat_Frame with these fields: the sequence number and the robot time as wrapping counters with the Required_Bits for a wrap period of at least 4 hours at 1 Hz, with the robot time at the same Resolution as in the Telemetry_Frame; the state as a 3-bit Packed_Field; the RSSI from −128 to 0 dBm at a Resolution of 1 dB; the Frame_Ring fill level from 0 to 64; the uptime in seconds from 0 to at least 86,400; the reconnect count, the dropped-frame count and the saturation count (Requirement 1 criterion 10), each from 0 to at least 65,535 and saturating at its upper limit; the Fault_Mask (Requirement 3); the parameter table version (Requirement 14 criterion 5); and the broker IPv4 address as a 4-byte value holding the four octets in dotted-decimal order.
2. WHEN TSK_TELEMETRY receives a telemetry event message, THE Wire_Codec SHALL encode one Event_Frame holding the robot time, the Event_Code and the typed binary arguments defined for that Event_Code in the Protocol_Specification. There SHALL be one Event_Code for each event name the Baseline_Build publishes (for example `STATE`, `BARCODE`, `EMERGENCY_STOP`, `FINE_SCAN`, `BC_DBG`), and the arguments SHALL carry the same numeric values and fixed reasons as the Baseline_Build detail text, at the number of decimal places that text uses.
3. THE Wire_Codec SHALL encode each variable-length numeric event detail of the Baseline_Build (for example the `BC_DBG` widths and the `FINE_SCAN` readings) as a count Packed_Field followed by an array of that many Packed_Fields. The count Packed_Field SHALL have the Required_Bits for 0 up to the maximum element count per Event_Frame recorded in the Protocol_Specification, and the Event_Frame length SHALL equal the fixed part length plus the count times the element size.
4. WHEN TSK_TELEMETRY answers a Command_Frame or a Config_Frame, THE Wire_Codec SHALL encode one Ack_Frame holding the robot time, the command code of the answered Command_Frame (or the config code for a Config_Frame), a result Reason_Code and the typed binary arguments of that result. There SHALL be one Reason_Code for each ack result text of the Baseline_Build (for example "ok", "queued", "pong", "nack: unknown parameter"). The arguments SHALL be the map letters and letter count for map acks, the ping number for PING, and the applied count and rejected Parameter_IDs for a Config_Frame (Requirement 14 criterion 4).
5. WHEN a GET:PARAMS Command_Frame arrives, THE Wire_Codec SHALL encode the reply as one or more Ack_Frames. Each part SHALL carry a part number counting from 1 in sending order and a list of Parameter_ID and value pairs, with each value in its Parameter_Store representation (Requirement 15), and each part SHALL be no longer than one `app_net.c` message slot. Across all parts, every Parameter_ID SHALL appear exactly once.
6. WHEN TSK_SAFETY_MON has turned the motor PWM off in response to a remote ESTOP Command_Frame, THE Wire_Codec SHALL encode one E-stop Ack_Frame carrying the time from the frame's arrival to PWM off as a 16-bit Packed_Field in microseconds, with a Valid_Range of 0 to 65,535 µs and a Resolution of 1 µs.
7. THE Wire_Codec SHALL produce every Heartbeat_Frame with the same fixed length in bytes.
8. IF a variable-length event detail has more elements than the maximum element count per Event_Frame, THEN THE Wire_Codec SHALL split the detail across consecutive Event_Frames, each with a part number counting from 1, with no element lost or repeated, and with no more parts than the Baseline_Build publishes for that detail (6 for the `BC_DBG` widths of one barcode window).

### Requirement 13: Binary command frames

**User Story:** As a student developer, I want commands sent to the robot in binary, so that command payloads also use compact binary fields.

#### Acceptance Criteria

1. THE Wire_Codec SHALL decode each Command_Frame as the Format_Version, then a command code Packed_Field, then the typed arguments of that command: a 5-bit letter (values 0 to 25 for A to Z) for BARCODE, a Map_Frame for MAP, a 16-bit ping number (0 to 65535) for PING, a Parameter_ID followed by a value in the encoding the Protocol_Specification defines for that Parameter_ID for SET, and no arguments for ESTOP, RESUME, START, STOP, MAP:CLEAR, GET:MAP and GET:PARAMS.
2. THE Wire_Codec SHALL accept exactly one distinct command code for each of the 11 Baseline_Build commands (ESTOP, RESUME, START, STOP, BARCODE, MAP, MAP:CLEAR, GET:MAP, PING, SET and GET:PARAMS), with each code and its frame length recorded in the Protocol_Specification.
3. WHEN a Command_Frame with the ESTOP command code passes the checks in criterion 5, THE Firmware SHALL set `EVF_REMOTE_ESTOP` and wake the safety task directly from the network context (the radio-owner task that runs `app_net.c`), without placing the command in the TSK_TELEMETRY command queue, as the Baseline_Build fast path does.
4. WHILE the motors are driven with a non-zero PWM duty, WHEN an ESTOP Command_Frame arrives, THE Firmware SHALL set the motor PWM to zero within 10 ms, measured from the receive time recorded by `app_net.c` to the PWM-off time reported in the E-stop Ack_Frame (NFR21).
5. IF a Command_Frame has a Format_Version other than the one the Revised_Build implements, a command code not assigned in the Protocol_Specification, a byte length different from the length the Protocol_Specification defines for its command code, or a BARCODE letter value from 26 to 31, THEN THE Firmware SHALL reply with one Ack_Frame carrying a rejection Reason_Code that is distinct for each of these 4 causes, and SHALL leave the system state, the Fault_Mask, the Parameter_Store and the barcode map unchanged.
6. IF a SET Command_Frame carries a Parameter_ID not present in the `PARAMS[]` table, a value that is NaN or infinite, or a value below the parameter's minimum or above its maximum in the `PARAMS[]` table, THEN THE Firmware SHALL reply with an Ack_Frame carrying the Reason_Code for the Baseline_Build's matching rejection (unknown parameter, bad number or out of range) and leave the Parameter_Store unchanged.
7. WHERE the legacy text command build option is enabled, THE Firmware SHALL accept both Command_Frames and the Baseline_Build text commands on `is02/cmd`, and SHALL perform the same action and give the same ack result for each text command as the Baseline_Build.
8. WHEN a Command_Frame other than ESTOP passes the checks in criteria 5 and 6, THE Firmware SHALL perform the action the Baseline_Build performs for the matching text command in the same system state, and SHALL reply with one Ack_Frame (one per numbered part for GET:PARAMS) whose result Reason_Code corresponds to the Baseline_Build ack result for that command and state, for example "starting" or "only from IDLE" for START, "only while LINE_FOLLOWING" for BARCODE, and "pong" with the received ping number for PING.
9. WHERE the legacy text command build option is disabled, IF a Baseline_Build text command arrives on `is02/cmd`, THEN THE Firmware SHALL reject it with the unknown Format_Version Reason_Code from criterion 5 and leave the robot state unchanged.
10. FOR ALL valid Command_Frames, THE Host_Test_Suite SHALL verify that encoding with the Laptop_Codec and then decoding with the Wire_Codec yields the same command code and argument values (round-trip property).

### Requirement 14: Binary barcode map and configuration

**User Story:** As a student developer, I want the retained barcode map and calibration sent in binary, so that the retained command data is compact too.

#### Acceptance Criteria

1. WHEN a Map_Frame arrives on `is02/map` or inside a MAP Command_Frame, THE Wire_Codec SHALL decode it as a Format_Version followed by one presence bit and one 5-bit letter code (0 = A to 25 = Z) for each movement in the order forward, left, right and U-turn, with the bit layout defined in the Protocol_Specification.
2. IF a Map_Frame carries a Format_Version other than the one the Firmware implements, has a byte length different from the Map_Frame length in the Protocol_Specification, has a letter code from 26 to 31 for a movement whose presence bit is 1, or names the same letter for two movements whose presence bits are 1, THEN THE Firmware SHALL reject the whole Map_Frame, reply with an Ack_Frame carrying a Reason_Code that identifies the cause, and keep all 26 letter mappings unchanged, as the Baseline_Build does.
3. THE Wire_Codec SHALL decode each Config_Frame as a Format_Version, then a count from 0 to the number of Parameter_IDs (55 in the Baseline_Build), then that many Parameter_ID and value pairs, with each value encoded at the Valid_Range and Resolution that the Data_Representation_Table gives for that parameter.
4. WHEN a Config_Frame arrives on `is02/config`, THE Firmware SHALL apply each valid pair in frame order, where a pair is valid if its Parameter_ID exists in `PARAMS[]` and its decoded value lies within that parameter's minimum and maximum in `PARAMS[]`, SHALL leave the parameter of each invalid pair unchanged, and SHALL reply with one Ack_Frame carrying the applied count and the Parameter_IDs of all rejected pairs (0 to 55 entries).
5. WHEN THE Firmware publishes a Heartbeat_Frame, THE Firmware SHALL include the parameter table version, made of the number of Parameter_IDs and a checksum of the parameter names in Parameter_ID order computed as defined in the Protocol_Specification, so that adding, removing or reordering a `PARAMS[]` entry changes the version.
6. WHEN THE Firmware accepts a Map_Frame, THE Firmware SHALL replace the whole map: each letter named by a movement whose presence bit is 1 is mapped to that movement, and every other letter becomes unmapped. If all four presence bits are 0, the Firmware SHALL clear the map, as `MAP:CLEAR` does in the Baseline_Build.
7. IF a Config_Frame carries a Format_Version other than the one the Firmware implements, has a count greater than the number of Parameter_IDs, or has a byte length different from the length its count implies in the Protocol_Specification, THEN THE Firmware SHALL reject the whole Config_Frame without changing any parameter and SHALL reply with an Ack_Frame carrying a Reason_Code that identifies the cause and an applied count of 0.
8. IF the parameter table version in the most recently received Heartbeat_Frame differs from the parameter table version held by the Laptop_Codec, THEN THE Laptop_Tools SHALL not publish a Config_Frame and SHALL show a warning that the robot's parameter table does not match the tool's.

### Requirement 15: Run-time parameter representation

**User Story:** As a student developer, I want discrete parameters packed, so that the Parameter_Store also follows the minimum-bit rule.

#### Acceptance Criteria

1. THE Parameter_Store SHALL store each of the 9 two-value parameters, `ENC_R_REVERSED`, `ENC_L_REVERSED`, `SERVO_LOW_ANGLE_IS_RIGHT`, `TURN_SNAP`, `BARCODE_AT_JUNCTION`, `BARCODE_DETECT`, `BARCODE_DEBUG`, `OBST_LEGACY` and `OBST_TRUST_MISS` (Valid_Range {0, 1}), as a 1-bit Packed_Field.
2. THE Parameter_Store SHALL store each of the 5 tri-state parameters, `MOTOR_MAPPING_CONFIRMED`, `ENC_DIRECTION_CONFIRMED`, `LINE_DO_BLACK_HIGH`, `SERVO_CALIBRATED` and `IMU_FORWARD_SIGN` (Valid_Range {−1, 0, 1}, where −1 is the CAL_UNSET sentinel or, for `IMU_FORWARD_SIGN`, a sign), as a 2-bit Packed_Field that represents all three values.
3. THE Parameter_Store SHALL store `IMU_FORWARD_AXIS` (Valid_Range −1 to 2, 4 values) as a 2-bit Packed_Field and `BARCODE_LEVELS` (Valid_Range 1 to 6, 6 values) as a 3-bit Packed_Field.
4. THE Parameter_Store SHALL keep each of the 39 parameters not listed in criteria 1 to 3 as float32, and the Data_Representation_Table SHALL list every one of the 55 Parameter_IDs with its Valid_Range, its representation (Packed_Field width or float32) and, for each float32 parameter, the reason it stays float32.
5. WHEN a task reads a parameter after start-up or after an accepted SET command or Config_Frame, THE Parameter_Store SHALL return a value numerically equal to the value the Baseline_Build holds for the same start-up defaults and the same sequence of accepted inputs, including the value −1 for a tri-state parameter left at CAL_UNSET.
6. IF a SET command or Config_Frame gives a parameter from criteria 1 to 3 a value that is outside its Valid_Range or is not a whole number, THEN THE Firmware SHALL reject the value in the same way the Baseline_Build rejects an out-of-range value, report that parameter as rejected in the Ack_Frame, and leave the stored value unchanged.
7. WHEN a SET command or Config_Frame updates one Packed_Field, THE Parameter_Store SHALL leave every other Packed_Field in the same Storage_Unit unchanged, and a task reading that Packed_Field during the update SHALL obtain either the complete previous value or the complete new value.

### Requirement 16: Format versioning and protocol specification

**User Story:** As a student developer, I want the binary layout written down and versioned, so that the robot and every laptop tool agree on the format.

#### Acceptance Criteria

1. THE Wire_Codec SHALL begin every Telemetry_Frame, Heartbeat_Frame, Event_Frame, Ack_Frame, Command_Frame, Map_Frame and Config_Frame with a header in which the Format_Version field has the same byte offset (byte 0), bit position and bit width in all seven payload types, so that a decoder can read the Format_Version before it knows the rest of the layout.
2. THE Protocol_Specification SHALL define, for each of the seven binary payload types and for each command code of the Command_Frame, the total payload length in bytes, the byte order, the bit order, and the field order, and for each field its bit offset, bit width, Valid_Range, Resolution, Required_Bits and sentinel codes.
3. THE Protocol_Specification SHALL list every Event_Code, Reason_Code, command code and Parameter_ID with its numeric value and the text the Baseline_Build used for it, covering all 55 `PARAMS[]` entries by their Baseline_Build NAME, with no numeric value used twice within the same code category.
4. WHEN a layout change to any binary payload is committed, where a layout change is any change to field order, bit offset, bit width, Resolution, byte order, payload length, or the numeric value or meaning of a code, or the addition or removal of a field or code, THE Wire_Codec SHALL encode a Format_Version exactly 1 greater than the Format_Version of the previous committed layout.
5. WHEN the Format_Version is incremented, THE Protocol_Specification SHALL state the new Format_Version as the current Format_Version and record the list of layout changes made since the previous Format_Version.
6. IF the Format_Version used by the Wire_Codec, the Python Laptop_Codec or the JavaScript Laptop_Codec differs from the current Format_Version stated in the Protocol_Specification, THEN THE Host_Test_Suite SHALL report a test failure that names the codec with the differing value.

### Requirement 17: Laptop tools decode and encode binary payloads

**User Story:** As a student developer, I want the dashboard and the laptop scripts to work with the binary payloads, so that monitoring, commands and the demo still work.

#### Acceptance Criteria

1. WHEN the Laptop_Codec receives a Telemetry_Frame, Heartbeat_Frame, Event_Frame or Ack_Frame whose Format_Version and length match the Protocol_Specification, THE Laptop_Codec SHALL decode every field defined for that frame, return each numeric field in the unit the Baseline_Build JSON payload used for it, and map each Event_Code, Reason_Code and command code to the text the Protocol_Specification lists for it.
2. THE Laptop_Codec SHALL encode every Command_Frame, Map_Frame and Config_Frame defined by the Protocol_Specification, producing the Format_Version header, field order, byte order, bit order and total length that the Protocol_Specification defines for that frame.
3. WHEN the Dashboard receives a binary payload on an `is02/` topic, THE Dashboard SHALL display the same fields, events and acks, with the same labels and units, that the Baseline_Build JSON payloads produced, with each displayed numeric value within half of the field's Resolution of the value the Baseline_Build JSON payload carried for the same robot state.
4. WHEN the Telemetry_Monitor runs against the Revised_Build, THE Telemetry_Monitor SHALL evaluate checks L1 to L6 with the same pass and fail thresholds as for the Baseline_Build, and write the CSV with the same column names in the same order as for the Baseline_Build.
5. WHEN the user triggers an action that the Laptop_Tools provide in the Baseline_Build (the dashboard buttons, the monitor keys, `is02_tool.py cmd`, `map`, `set` and `config`, `robot_map.py` and `robot_params.py`), THE Laptop_Tools SHALL publish the corresponding binary Command_Frame, Map_Frame or Config_Frame on the same topic and with the same retain flag as the Baseline_Build, and display the decoded Ack_Frame, or the Baseline_Build's no-ack message if no Ack_Frame arrives within the Baseline_Build ack wait time (4 s, or 3 s for `set`).
6. WHEN a Laptop_Tool reads the `robot_params.txt` NAME=VALUE text file, THE Laptop_Tool SHALL ignore the blank lines and comment lines the Baseline_Build ignored, and convert each NAME to the Parameter_ID the Protocol_Specification lists for it before sending.
7. IF the Laptop_Codec receives a payload on an `is02/` topic with a Format_Version not defined in the Protocol_Specification, or a length in bytes that differs from the length the Protocol_Specification defines for that frame, THEN THE Laptop_Codec SHALL write the topic, the Format_Version and the length to the tool's log output, discard the payload, leave the previously displayed values unchanged and continue processing later payloads.
8. IF the parameter table version in a Heartbeat_Frame (the Parameter_ID count or the Parameter_ID order checksum) differs from the Laptop_Codec's table, THEN THE Laptop_Tools SHALL show a parameter table mismatch warning that names the received and expected count and checksum, and keep the warning visible until a Heartbeat_Frame with a matching parameter table version arrives.
9. IF a NAME in the user's input (the `robot_params.txt` file or a `set` argument) is not in the Laptop_Codec's Parameter_ID table, or its VALUE is not a number or lies outside the parameter's Valid_Range, THEN THE Laptop_Tool SHALL report the line number or argument and the NAME, publish no Command_Frame or Config_Frame for that input, and leave any retained `is02/config` payload unchanged.
10. IF a decoded Event_Frame or Ack_Frame carries an Event_Code or Reason_Code that the Protocol_Specification does not list, THEN THE Laptop_Tools SHALL display the frame with the numeric code in place of the text name and write the unknown code to the tool's log output.

### Requirement 18: Codec correctness

**User Story:** As a student developer, I want the encoders and decoders tested against each other, so that I can show the binary format loses no required information.

#### Acceptance Criteria

1. THE Host_Test_Suite SHALL verify, for at least 1,000 generated value sets for each of the Telemetry_Frame, Heartbeat_Frame and Ack_Frame, and at least 100 generated value sets for each Event_Code of the Event_Frame, with every field value within its Valid_Range, that decoding with the Python Laptop_Codec and with the JavaScript Laptop_Codec after encoding with the Wire_Codec returns each continuous field within half of its Resolution of the original value, and returns each integer, enumerated, bitmask and sentinel field equal to the original value (round-trip property).
2. THE Host_Test_Suite SHALL verify, for at least 1,000 generated value sets for each command code of the Command_Frame and for each of the Map_Frame and Config_Frame, with every field value within its Valid_Range and every continuous field value a whole multiple of its Resolution, that decoding with the Wire_Codec after encoding with the Python Laptop_Codec and with the JavaScript Laptop_Codec returns every field equal to the original value (round-trip property).
3. THE Host_Test_Suite SHALL verify, for at least 1,000 generated valid binary payloads of each payload type, that the Wire_Codec, the Python Laptop_Codec and the JavaScript Laptop_Codec each re-encode a decoded payload to a byte sequence identical to the input, where a valid binary payload has a known Format_Version, the length the Protocol_Specification defines for its payload type and command code, every field code within its Valid_Range or equal to a defined sentinel code, and every reserved bit set to zero (decode then encode is the identity on bytes).
4. THE Host_Test_Suite SHALL verify, for each field the Wire_Codec encodes, that an input one Resolution step below the lower Valid_Range limit, one Resolution step above the upper Valid_Range limit, the minimum and maximum value of the field's input type, and, for float inputs, negative and positive infinity each encode as the nearest Valid_Range limit (saturation property).
5. THE Host_Test_Suite SHALL verify, for each Storage_Unit that holds two or more Packed_Fields, at least 1,000 generated sequences of 1 to 16 packed accessor writes, with values that include 0 and 2^Required_Bits − 1 for each Packed_Field, that each Packed_Field reads back the last value written to it, each Packed_Field not written in the sequence keeps its initial value, and the Storage_Unit bits outside all Packed_Fields keep their initial values (field independence property).
6. THE Host_Test_Suite SHALL run the Wire_Codec C sources compiled for the PC, the Python Laptop_Codec and the JavaScript Laptop_Codec against one shared set of test vectors, and SHALL verify that all three implementations produce identical byte sequences when encoding each vector and identical field values when decoding each vector.
7. THE Host_Test_Suite SHALL verify, for each payload type, that the decoder for that payload type (the Wire_Codec for Command_Frame, Map_Frame and Config_Frame; the Python and JavaScript Laptop_Codecs for all other payload types) rejects without returning decoded field values a payload with an unknown Format_Version, a payload 1 byte shorter than the defined length, a payload 1 byte longer than the defined length, and a zero-length payload.
8. WHEN a Host_Test_Suite run completes, THE Host_Test_Suite SHALL print, for each property in criteria 1 to 7, the payload type or Storage_Unit tested, the number of cases run, the number of cases failed and the random seed used to generate the cases.
9. IF any Host_Test_Suite case fails, THEN THE Host_Test_Suite SHALL print the property, the payload type or Storage_Unit, the input values and the random seed of the failing case, and SHALL exit with a non-zero status.

### Requirement 19: Preserve architecture and behaviour

**User Story:** As a student developer, I want the change to leave the code structure and the car's behaviour intact, so that everything that worked before still works.

#### Acceptance Criteria

1. THE Revised_Build SHALL create the same seven tasks as the Baseline_Build with the same priorities: TSK_SAFETY_MON at 2, TSK_MOTION_CTRL at 4, TSK_LINE_BARCODE at 5, TSK_OBSTACLE_SCAN at 6, TSK_TERRAIN_IMU at 7, TSK_NAV_PLANNER at 8 and TSK_TELEMETRY at 10 (smaller number = higher priority), and SHALL create no additional application task.
2. THE Revised_Build SHALL keep the kernel objects of the Baseline_Build: the `g_flg_sys` event flag group with the 18 `EVF_*` bits at the same bit positions (bit 0 `EVF_COLLISION` to bit 17 `EVF_LOCAL_STOP`), the four mailboxes `g_mbx_motion`, `g_mbx_nav`, `g_mbx_obstacle` and `g_mbx_tlm`, the priority-inheritance I2C mutex `MTX_I2C_BUS`, the 40-block Message_Pool and the 10 ms cyclic handler.
3. THE Revised_Build SHALL keep every module of the Baseline_Build `app_program/` directory, adding new modules only for the Wire_Codec and the packed field accessors, and SHALL keep the pure-logic modules (`line_logic`, `imu_logic`, `obstacle_logic`, `barcode_logic`, `bc_detect`, `bc_multi`, `code39`, `nav_map`, `speed_ctl`) free of hardware access, so that none of their sources includes `hal.h`, `hal_internal.h`, `pins.h`, `tk/tkernel.h` or any Pico SDK header, and each compiles with a PC host compiler.
4. THE Revised_Build SHALL keep the MQTT topic names of the Baseline_Build (`is02/telemetry`, `is02/status`, `is02/event`, `is02/ack`, `is02/cmd`, `is02/map`, `is02/config`), the broker discovery beacon on UDP port 47002 (followed only after 2 consecutive failed connects), and the reconnect back-off that starts at 1000 ms, doubles after each failed attempt up to 16000 ms, and returns to 1000 ms after a successful connect or a beacon-driven broker change.
5. WHEN the Revised_Build is built from clean with the existing `build_make/robot.mk` (`APP_ROBOT=1`) and the unchanged application warning flags (`-Wall -Wextra -Wshadow -Wdouble-promotion`), THE build SHALL complete with exit status 0, report no more compiler warnings than a clean build of the Baseline_Build, and report no warning in any source file added or changed by this feature.
6. THE Revised_Build SHALL give every Storage_Unit, Packed_Field container and binary payload buffer added by this feature static storage duration, and SHALL add no call to `malloc`, `calloc`, `realloc` or `free` and no new kernel memory pool.
7. WHEN the Revised_Build completes 3 runs on the same track layout, with the same barcode map and the same `robot_params.txt` values used for 3 Baseline_Build runs, THE Firmware SHALL produce in each run the same decoded barcode letter sequence, the same turn direction at each junction, the same obstacle avoidance result (bypass complete or abort), the same hump count, the same final system state and the same pass/fail result for Telemetry_Monitor checks L1 to L6 as the Baseline_Build, as recorded in the Telemetry_Monitor CSV and log files.
8. THE Revised_Build SHALL keep the task periods of the Baseline_Build: the cyclic handler wakes TSK_SAFETY_MON and TSK_LINE_BARCODE every 10 ms tick and wakes TSK_MOTION_CTRL and TSK_TERRAIN_IMU every 20 ms on alternate ticks, TSK_OBSTACLE_SCAN pings at 70 ms intervals while line following, and TSK_TELEMETRY wakes at least every 100 ms, publishes a Telemetry_Frame every 500 ms and publishes a Heartbeat_Frame every 1000 ms.
9. WHILE the MQTT session is down, THE Firmware SHALL store each new Telemetry_Frame in the Frame_Ring holding at least 64 frames, overwrite the oldest frame and increment the dropped-frame count when the Frame_Ring is full, and after reconnecting publish the stored frames oldest first, at no more than 4 frames per radio poll.
10. WHEN any E-stop trigger of the Baseline_Build occurs (BTN2 local stop, remote ESTOP command, proximity reading below 8 cm, IMU collision, battery critical or heartbeat watchdog timeout of 300 ms), THE Firmware SHALL turn the motors off, enter the EMERGENCY STOP state, publish the Event_Frame and Ack_Frame that carry the same information as the Baseline_Build event and ack, and leave EMERGENCY STOP only through the same user action as the Baseline_Build.

### Requirement 20: Preserve timing and performance

**User Story:** As a student developer, I want proof that packing and binary encoding do not slow the robot, so that the change causes no unacceptable timing degradation.

#### Acceptance Criteria

1. WHILE the system state is SYS_LINE_FOLLOWING, SYS_MANEUVER or SYS_OBSTACLE_AVOIDANCE, THE Firmware SHALL complete every TSK_SAFETY_MON and TSK_LINE_BARCODE cycle within 10 ms of its release by the 10 ms cyclic handler, and every TSK_MOTION_CTRL and TSK_TERRAIN_IMU cycle within 20 ms of its release, with zero cycles exceeding these limits across 3 consecutive full track runs, where a full track run starts when the robot enters SYS_LINE_FOLLOWING at the track start and ends when the robot stops at the track end.
2. WHILE the system state is SYS_LINE_FOLLOWING, SYS_MANEUVER or SYS_OBSTACLE_AVOIDANCE, THE Firmware SHALL raise zero watchdog E-stops (`HEARTBEAT_TIMEOUT_MS` = 300 ms) during each of 3 consecutive full track runs.
3. THE Revised_Build SHALL keep the worst-case observed execution time of `hal_gpio_isr` and of `hal_timer_isr` at or below the larger of 110 % of the Baseline_Build's worst-case observed execution time and the Baseline_Build's worst-case observed execution time plus 1 µs, where each build is measured with the RP2040 microsecond timer over at least 10,000 invocations of each handler recorded during 3 consecutive full track runs.
4. THE Wire_Codec SHALL encode one Telemetry_Frame with both a mean and a maximum execution time no greater than the Baseline_Build's mean and maximum time to build one JSON telemetry frame, where each time runs from reading the Status_Snapshot to having the complete payload in the output buffer, excludes MQTT publish time, and is measured with the RP2040 microsecond timer over 1,000 consecutive encodes of the same Status_Snapshot content on the robot.
5. WHILE the MQTT session is up, THE Firmware SHALL publish live Telemetry_Frames at a rate between 1.8 Hz and 2.2 Hz, measured with robot timestamps (monitor check L1) over a window of at least 60 s containing at least 120 frames, excluding frames replayed from the Frame_Ring.
6. THE Evidence_Report SHALL record, for each measurement in criteria 1 to 5, the Baseline_Build value where a comparison applies, the Revised_Build value, the sample count, the test conditions (track, number of runs, measurement window) and a pass or fail result against the stated limit.

### Requirement 21: Data representation table

**User Story:** As a student, I want a data representation table in the repository, so that I can submit the Week 13 documentation the change requirement asks for.

#### Acceptance Criteria

1. THE Data_Representation_Table SHALL list each Application_Field as exactly one row under one heading per category: motion control and encoder data, line and barcode sensing, obstacle detection, hump estimation, vehicle and controller states, Task_Shared_Data, ISR_Shared_Data, Inter_Task_Messages, and MQTT telemetry and command payloads. Each row SHALL name the Revised_Build C type and member that holds the Application_Field, or for an MQTT payload field the payload and its bit offset as defined in the Protocol_Specification, so that every Packed_Field declared in the Revised_Build and every field of every binary MQTT payload has a matching row.
2. THE Data_Representation_Table SHALL give for each Application_Field: the Valid_Range with units, including every sentinel value; the Resolution with units, or "1" for integer, enumerated and flag fields; the Required_Bits; the Baseline_Build C type and size in bits, or "none" where the Baseline_Build has no counterpart; and the Revised_Build representation, given as C bit-field or mask and shift, the name and width (8, 16 or 32 bits) of the Storage_Unit, and the bit offset and bit width of the field inside that Storage_Unit.
3. THE Data_Representation_Table SHALL name, for each Storage_Unit of Task_Shared_Data and ISR_Shared_Data, every writing context by task name or interrupt handler name. It SHALL also name exactly one protection from this set: single writer with one aligned store, Critical_Section with dispatch disabled, Critical_Section with interrupts masked, or single-producer single-consumer ring index.
4. THE Data_Representation_Table SHALL be stored as a Markdown file under `docs/` in the repository, and `README.md` SHALL contain a relative link that opens that file when followed in the repository.
5. THE Data_Representation_Table SHALL give, for each Application_Field, a Required_Bits value equal to ceil(log2(N)), where N is the number of distinct values in the listed Valid_Range at the listed Resolution, and a Revised_Build bit width equal to that Required_Bits value for every field that is not a Private_Control_Variable.
6. THE Data_Representation_Table SHALL give, for each packed type in the Revised_Build, the total size of the type in bytes, which is the size checked by the compile-time assertion of Requirement 1 criterion 7.
7. THE Data_Representation_Table SHALL match the Revised_Build at the commit submitted for Week 13, so that every Storage_Unit, bit offset, bit width and packed type size in the Data_Representation_Table equals the corresponding declaration in the Firmware source at that commit.

### Requirement 22: RAM usage comparison

**User Story:** As a student, I want a measured RAM comparison, so that I can show how much RAM the change saves.

#### Acceptance Criteria

1. THE Evidence_Report SHALL give, for the Baseline_Build and for the Revised_Build, the `.data` size, the `.bss` size and their sum in bytes, as reported by `arm-none-eabi-size` on the linked ELF of each build, and SHALL include the unedited `arm-none-eabi-size` output of each build.
2. THE Evidence_Report SHALL give the static RAM saving as the Baseline_Build `.data` plus `.bss` sum minus the Revised_Build `.data` plus `.bss` sum, in bytes and as a percentage of the Baseline_Build sum rounded to one decimal place.
3. THE Evidence_Report SHALL give for both builds the RAM size in bytes, taken from the linker map file, and the saving in bytes of each of these items: the Status_Snapshot (`g_status`), the Message_Pool buffer (`g_mpf_buf`), the Frame_Ring (`g_frames`), the message slots (`g_msgs`), the barcode edge ring (`g_bc_ring`), the raw sample ring (`g_raw_er`, `g_raw_el` and `g_raw_adc` summed), the command slots (`g_cmds`) and the Parameter_Store (`g_params`).
4. THE Evidence_Report SHALL record for each measurement the git commit hash of the build measured, the `make` command line with its build options (for example `APP_ROBOT=1`), and the `arm-none-eabi-gcc` version used.
5. THE Revised_Build SHALL have a `.data` plus `.bss` sum, measured as stated in criterion 1, less than or equal to the `.data` plus `.bss` sum of the Baseline_Build.
6. IF the Revised_Build renames, splits or merges a symbol listed in criterion 3, THEN THE Evidence_Report SHALL list the Revised_Build symbols that replace it and give the sum of their sizes as the Revised_Build size of that item.
7. THE Evidence_Report SHALL take the Baseline_Build and Revised_Build measurements from builds that use the same `make` command line, the same build options and the same `arm-none-eabi-gcc` version, each built after `make clean` from a working tree with no uncommitted changes at the recorded commit.

### Requirement 23: Message size comparison

**User Story:** As a student, I want measured message sizes before and after, so that I can show the size reduction of inter-task and MQTT messages.

#### Acceptance Criteria

1. THE Evidence_Report SHALL give, for the Baseline_Build and for the Revised_Build, the `sizeof` in bytes of each Inter_Task_Message body type (`motion_cmd_t`, `nav_event_t`, `obstacle_profile_t`, `tlm_event_t` or their Revised_Build replacements) and the configured block size in bytes of one Message_Pool block, with every value taken from the RP2040 target build compiled with `arm-none-eabi` (not a host build).
2. THE Evidence_Report SHALL give, for each MQTT topic (`is02/telemetry`, `is02/status`, `is02/event`, `is02/ack`, `is02/cmd`, `is02/map`, `is02/config`) in both builds, the MQTT application payload size in bytes (excluding the MQTT fixed header, topic name and packet identifier) as the minimum, the arithmetic mean to 0.1 byte and the maximum over one recorded track run, together with the number of payloads counted and the git commit of the build used.
3. THE Evidence_Report SHALL give, for the Telemetry_Frame, the sum of the Required_Bits of every Packed_Field in the frame, the frame length in bytes, and the number of unused padding bits, calculated as 8 × frame length in bytes − the sum of the Required_Bits.
4. THE Firmware SHALL produce every Telemetry_Frame with a length in bytes no greater than 0.25 × the arithmetic mean Baseline_Build `is02/telemetry` payload length reported under criterion 2.
5. THE Evidence_Report SHALL give, for each Inter_Task_Message body type, for the Message_Pool block and for each MQTT topic, the size reduction as a percentage, calculated as (Baseline_Build size − Revised_Build size) ÷ Baseline_Build size × 100, rounded to 0.1 %, using the mean payload size for MQTT topics.
6. IF an MQTT topic carries no payload during the recorded track run of a build, THEN THE Evidence_Report SHALL give that topic's minimum, mean and maximum payload size from a separate recorded session of the same build containing at least 1 payload of each message type sent on that topic, and SHALL state that the values come from that separate session.

### Requirement 24: Correctness and performance evidence

**User Story:** As a student, I want recorded evidence that nothing broke and timing is acceptable, so that I can show the demonstrator the change is safe.

#### Acceptance Criteria

1. THE Evidence_Report SHALL include, for each property in Requirement 18 criteria 1 to 5 and for the cross-codec run in Requirement 18 criterion 6, the pass or fail result of the Host_Test_Suite, the number of generated cases (at least 100 per property), and the codec pairs exercised (C Wire_Codec with Python Laptop_Codec, and C Wire_Codec with JavaScript Laptop_Codec).
2. THE Evidence_Report SHALL include the `telemetry_monitor.py` summary, with the pass or fail result and the detail text of each check L1 to L6, for one full track run with the Baseline_Build and one full track run with the Revised_Build on the same track, where each run includes at least one command answered by an Ack_Frame (check L5) and at least one E-stop command (check L6).
3. THE Evidence_Report SHALL include a functional checklist that states, for each feature in Requirement 19 criterion 7 (line following, barcode decoding, junction navigation, obstacle avoidance, hump detection, E-stop, recovery and telemetry), the expected outcome on the track and the observed outcome on each build, and marks the feature pass only if the observed outcome equals the expected outcome.
4. THE Evidence_Report SHALL include, for each build, the worst-case observed and average execution time in microseconds of TSK_SAFETY_MON, TSK_LINE_BARCODE, TSK_MOTION_CTRL and TSK_TERRAIN_IMU cycles, measured with the RP2040 microsecond timer over one full track run, together with the number of cycles measured per task.
5. IF any measurement in the Evidence_Report falls outside a limit in Requirement 20, or an E-stop PWM-off time exceeds 10 ms, THEN THE Evidence_Report SHALL state the measured value, the limit, the Requirement 20 criterion (or check L6) that sets the limit, and the identified cause.
6. THE Evidence_Report SHALL include, for each build, the worst-case observed and average execution time in microseconds of `hal_gpio_isr` and `hal_timer_isr`, measured with the RP2040 microsecond timer over one full track run, and the Revised_Build worst-case time of each ISR as a percentage of the Baseline_Build worst-case time.
7. THE Evidence_Report SHALL include, for each build, the worst-case observed and average time in microseconds to produce one telemetry frame (JSON build in the Baseline_Build, Wire_Codec encode in the Revised_Build) over at least 100 frames, each E-stop PWM-off time in microseconds reported by check L6, the number of watchdog E-stops raised during the track run, and the measured Telemetry_Frame rate in Hz.
8. IF a Host_Test_Suite property fails, a check L1 to L6 fails, or a functional checklist feature is marked fail on either build, THEN THE Evidence_Report SHALL state the failing item, the failing input (for a property) or the observed outcome (for a check or feature), and the identified cause.
