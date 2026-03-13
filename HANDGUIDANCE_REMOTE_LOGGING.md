# Hand-Guidance Remote Logging Changes

## Why This Document Exists

This document explains the full set of changes made in this workstream after the previous commit.

The order here is intentional:

1. Why each decision was made
2. What was changed
3. How the final system is intended to run

The goal is to make the reasoning explicit before discussing implementation details.

## Validation Status

What was validated locally:

- The Sunrise Java code compiles with Java 6 target settings:
  - `AbstractSunriseStreamBridge.java`
  - `HandGuidanceBridge.java`
  - `GPIOHandGuidanceLoggerBridge.java`
- The changed Python files and launch files pass `py_compile`.

What was not validated locally:

- Live robot execution on the KUKA controller
- Live ROS2 launch/runtime on the Ubuntu VM
- End-to-end network communication against the real controller

So this document describes the tested code structure and intended runtime flow, with honest limits on what could be validated from this machine.

## Read This First

This Sunrise workspace only contains the controller-side Java applications.

The full feature has two halves:

1. A Sunrise Java app running on the KUKA controller
2. A ROS-side client/node running on the Ubuntu machine that connects to the controller TCP port

If only one half is running, the system will look broken:

- if the Java app is not running, ROS has nothing to connect to
- if the ROS client/node is not running, the Java app just waits for a TCP client

So "how to use this" always means "which controller app do I run, and which ROS path do I pair it with?"

## Quick Start

### Recommended: Real GPIO Hand-Guidance Workflow

Use this when you want the robot to behave the way the physical setup is already designed to work.

1. On the KUKA controller, run `lbr_fri_ros2.GPIOHandGuidanceLoggerBridge`
2. In the ROS configuration, enable `gpio_handguidance_logger.enabled`
3. Launch the ROS stack
4. Use `S1` to enter or leave the real GPIO hand-guidance mode
5. Use `S4` to start or stop recording
6. Expect both of these outputs:
   - a ROS-side combined take from `multi_sensor_recorder`
   - an optional Sunrise controller-side `.log` file

The important behavioral point is this:

- `GPIOHandGuidanceLoggerBridge` does not call Sunrise `handGuiding()`
- it only streams robot state and starts/stops the Sunrise `DataRecorder`
- GPIO remains the thing that actually changes the robot into hand-guidance mode

### Experimental: Sunrise-Controlled Hand-Guidance Workflow

Use this only when you intentionally want to test the direct Sunrise `handGuiding()` path.

1. On the KUKA controller, run `lbr_fri_ros2.HandGuidanceBridge`
2. In the ROS configuration, enable `handguidance.enabled`
3. Launch the ROS stack
4. Start a session through the ROS service path or by sending the TCP `START` command
5. Stop the session through the ROS service path or by sending the TCP `STOP` command

In this path:

- Sunrise itself enters `handGuiding()`
- Sunrise starts/stops the `DataRecorder`
- the controller owns the hand-guidance motion lifecycle

## What Runs Where

Controller-side Java classes in this repo:

- `src/lbr_fri_ros2/HandGuidanceBridge.java`
- `src/lbr_fri_ros2/GPIOHandGuidanceLoggerBridge.java`

Controller-side shared base class:

- `src/lbr_fri_ros2/AbstractSunriseStreamBridge.java`

What to actually run on the controller:

- run `HandGuidanceBridge` if you want the experimental Sunrise-controlled path
- run `GPIOHandGuidanceLoggerBridge` if you want the real GPIO-compatible logging path
- do not run `AbstractSunriseStreamBridge` directly because it is only a shared base class

ROS-side components referenced in this document live outside this Sunrise workspace. They are the pieces that:

- open the TCP connection to the controller
- publish the streamed robot state into ROS topics
- expose ROS services used by the switchboard
- record those topics into the common take output

## TCP Command Protocol

Both Java bridges expose a simple newline-delimited TCP protocol.

Ports:

- `30010`: `HandGuidanceBridge`
- `30011`: `GPIOHandGuidanceLoggerBridge`

Commands accepted by both bridges:

- `PING`: health check
- `STATUS`: returns current state and the current log filename
- `START`: starts a session
- `STOP`: stops a session

Responses sent by the bridge:

- `OK,...`: command succeeded
- `ERR,...`: command failed or was unknown
- `SAMPLE,...`: streamed robot sample data

Only one TCP client is supported at a time. If a second client connects, the bridge drops the previous client and serves the new one.

What `START` means depends on the bridge:

- in `HandGuidanceBridge`, `START` means "start recording and enter Sunrise handGuiding()"
- in `GPIOHandGuidanceLoggerBridge`, `START` means "start recording only"

What `STOP` means depends on the bridge:

- in `HandGuidanceBridge`, `STOP` means "stop the Sunrise hand-guidance session and stop recording"
- in `GPIOHandGuidanceLoggerBridge`, `STOP` means "stop recording only"

That difference is the single most important thing to understand before using these two apps.

### Example Manual TCP Session

If you want to test the controller app without the full ROS stack, you can connect with any TCP client that sends newline-terminated text.

Example sequence:

1. Connect to port `30010` or `30011`
2. Send `PING`
3. Expect `OK,PING`
4. Send `STATUS`
5. Expect something like `OK,STATUS,idle,-`
6. Send `START`
7. Expect an `OK,START,...` response
8. While the session is active, expect repeated `SAMPLE,...` lines
9. Send `STOP`
10. Expect `OK,STOP,idle,<filename>`

This is useful for checking the controller-side bridge by itself before bringing ROS, switchboard routing, or recorder integration into the picture.

## The Central Why

The most important clarification from the user was this:

- GPIO-triggered hand-guidance mode is not the same thing as Sunrise `handGuiding()`
- The GPIO hand-guidance mode is the preferred user-facing behavior
- The original `S4` spirit must be preserved:
  - services are launched separately
  - switches do not bring topics/services up and down
  - `S4` only controls whether data is recorded
  - the main output should remain the combined multi-sensor take output

That changed the design significantly.

The final solution therefore preserves two independent paths:

1. An experimental Sunrise-controlled hand-guidance path
2. A corrected GPIO-compatible logging path

The corrected path is the one aligned with the user's workflow.

## Bird's-Eye View

There are now two separate paths in the codebase.

### Path 1: Experimental Sunrise Hand-Guidance Bridge

Why it exists:

- It was the first implementation attempt
- The user explicitly asked not to delete it
- It may still be useful for comparison/testing

What it does:

- Sunrise receives `START` / `STOP`
- Sunrise itself enters `handGuiding()`
- Sunrise starts/stops a `DataRecorder`
- ROS receives live streamed state

Operator intent:

- This is a separate test path
- It is not the preferred GPIO hand-guidance workflow

### Path 2: Corrected GPIO Hand-Guidance Logger

Why it exists:

- It matches the user's actual workflow
- GPIO remains responsible for the real hand-guidance mode
- Sunrise is used only as a logger/streamer
- `S4` remains the recording switch
- `multi_sensor_recorder` remains the canonical combined output

What it does:

- `S1` still activates hardware GPIO hand-guidance mode through Arduino
- a Sunrise logger bridge stays running separately
- that logger bridge does not call `handGuiding()`
- `S4` toggles recording
- the KUKA hand-guidance topics are folded into the common multi-sensor take output

## Why The Architecture Was Split This Way

### Why keep two paths

Because the user asked to keep the earlier attempt and the corrected design independent so both can be tested.

### Why keep GPIO hand-guidance separate from Sunrise `handGuiding()`

Because they are behaviorally different on this robot setup, and the user explicitly prefers the GPIO-triggered mode for actual use.

### Why keep `S4` as the recording switch

Because that matches the original switchboard design:

- launch services first
- keep them available
- use `S4` only to control recording state

### Why keep `multi_sensor_recorder` as the main output

Because the user's established workflow is based on synchronized topic subscriptions written into a common take directory and common CSV.

Creating only a controller-side KUKA `.log` file would not preserve that workflow.

### Why still keep the controller-side KUKA `.log`

Because the Sunrise `DataRecorder` is still useful as a secondary artifact and was already part of the original Sunrise logging concept.

So the final corrected path now produces:

- the common ROS-side combined take output
- optionally, the controller-side Sunrise `.log`

### Why refactor for modularity

Because the initial implementation duplicated a lot of transport and parsing logic. The user explicitly asked for code that minimizes total written lines, stays readable for humans, and remains maintainable.

The refactor therefore focused on reducing repeated logic in the highest-value places first:

- shared ROS TCP bridge behavior
- shared Sunrise Java TCP/sample/recorder behavior
- shared recorder topic mapping
- shared switchboard recording fan-out

## Final File-Level Changes

### Sunrise Workspace

#### `src/lbr_fri_ros2/AbstractSunriseStreamBridge.java`

Why:

- `HandGuidanceBridge.java` and `GPIOHandGuidanceLoggerBridge.java` shared a large amount of Java 6-safe Sunrise boilerplate
- duplicating controller/tool setup, TCP server setup, sample streaming, recorder creation, and text protocol handling would increase maintenance cost

What:

- added a shared Sunrise base class
- centralizes:
  - controller/tool/TCP setup
  - port resolution
  - `DataRecorder` creation
  - TCP command handling: `PING`, `STATUS`, `START`, `STOP`
  - sample packet generation
  - socket cleanup

How:

- subclasses only define what differs:
  - port names
  - whether samples stream only while active or always
  - what “start session” means
  - how session shutdown works
  - whether motion lifecycle needs monitoring

#### `src/lbr_fri_ros2/HandGuidanceBridge.java`

Why:

- preserve the original experimental path without deleting it

What:

- reduced to a small subclass over `AbstractSunriseStreamBridge`
- still uses Sunrise `handGuiding()`
- still starts/stops a `DataRecorder`
- still streams KUKA state

How:

- implements motion-specific behavior only:
  - `moveAsync(handGuiding())`
  - motion completion monitoring
  - motion cancellation and await during shutdown

#### `src/lbr_fri_ros2/GPIOHandGuidanceLoggerBridge.java`

Why:

- provide a Sunrise logger that does not alter the hand-guidance mode
- support the real GPIO-based workflow

What:

- reduced to a small subclass over `AbstractSunriseStreamBridge`
- does not call `handGuiding()`
- only starts/stops the Sunrise `DataRecorder`
- streams live KUKA state continuously

How:

- `START` means “start recording”
- `STOP` means “stop recording”
- sample streaming stays available while the bridge is connected

### ROS2 Workspace

#### `data_collection/data_collection/sunrise_stream_bridge_base.py`

Why:

- the two ROS bridge nodes were nearly identical
- transport code, parsing, publishers, and service command handling should not be duplicated

What:

- added a shared ROS-side base class for Sunrise TCP bridges

How:

- parameterizes only the differences:
  - node name
  - port
  - bridge label
  - service name

#### `data_collection/data_collection/sunrise_handguidance_bridge.py`

Why:

- preserve the experimental path

What:

- now a thin wrapper around the shared ROS base class

How:

- configures:
  - node name `sunrise_handguidance`
  - port `30010`
  - service `/sunrise_handguidance/set_enabled`

#### `data_collection/data_collection/sunrise_gpio_handguidance_logger.py`

Why:

- provide the ROS-facing side of the corrected GPIO-compatible logging path

What:

- now a thin wrapper around the shared ROS base class

How:

- configures:
  - node name `sunrise_gpio_handguidance_logger`
  - port `30011`
  - service `/sunrise_gpio_handguidance_logger/set_recording`

#### `data_collection/data_collection/switchboard_router.py`

Why:

- preserve `S4` as the recording fan-out point
- avoid creating separate switch behavior for the corrected GPIO logger path

What:

- `S4` now fans out to one more optional recording target:
  - `/sunrise_gpio_handguidance_logger/set_recording`
- the recording fan-out was refactored into a reusable list

How:

- `S1`, `S2`, `S3`, `S5` keep their existing meanings
- `S4` calls all configured recording services

#### `data_collection/data_collection/multi_sensor_recorder.py`

Why:

- the corrected path needed to preserve the existing combined-take workflow
- KUKA hand-guidance data needed to be recorded alongside the other modalities in the same output

What:

- added optional KUKA topic parameters:
  - `kuka_joint_topic`
  - `kuka_pose_topic`
  - `kuka_wrench_topic`
- added synchronized KUKA fields to the common CSV output when those topics are configured

How:

- the recorder now subscribes to the configured KUKA topics
- it stores them in buffers like the other modalities
- on each sample tick it writes the nearest synchronized KUKA values into the same row

This keeps the spirit of the original recorder intact:

- topic-based subscription
- synchronized sampling
- one common take directory
- one common CSV per segment

#### `data_collection/setup.py`

Why:

- the new ROS executables need to be installable through the package

What:

- added:
  - `sunrise_handguidance_bridge`
  - `sunrise_gpio_handguidance_logger`

#### `data_collection/package.xml`

Why:

- the new ROS bridge code needs only the existing geometry/sensor/service dependencies

What:

- no extra dependency was ultimately needed for the pruned bridge implementation

### Launch And Configuration

#### `service_launch/launch/switchboard_core.launch.py`

Why:

- switchboard launch should only wire the services it needs

What:

- added optional routing for the GPIO logger recording service

How:

- if `gpio_handguidance_logger.enabled` is true in YAML, the switchboard router receives:
  - `/sunrise_gpio_handguidance_logger/set_recording`

#### `service_launch/launch/record_take.launch.py`

Why:

- service nodes should be launched separately from switch behavior
- the recorder should subscribe to the correct KUKA topic namespace automatically

What:

- launches `sunrise_gpio_handguidance_logger` when enabled
- injects KUKA topic parameters into `multi_sensor_recorder`
- added a small helper for KUKA topic mapping

How:

- if `gpio_handguidance_logger.enabled` is true:
  - recorder subscribes to `/sunrise_gpio_handguidance_logger/*`
- else if the experimental `handguidance.enabled` path is true:
  - recorder subscribes to `/sunrise_handguidance/*`

#### `service_launch/config/sensors.yaml`

Why:

- both launch behavior and recorder wiring are driven from YAML in this stack

What:

- added `gpio_handguidance_logger` section:
  - `enabled`
  - `robot_host`
  - `robot_port`
  - `tcp_frame_id`
  - `reconnect_period_s`
  - `socket_timeout_s`

## Final Runtime Flows

### Experimental Sunrise Path

Use when testing the direct Sunrise `handGuiding()` approach.

1. Run `HandGuidanceBridge` on the KUKA controller
2. Enable `handguidance.enabled` in `sensors.yaml`
3. Launch the ROS stack
4. Use `S5` to start/stop the Sunrise hand-guidance session

Result:

- Sunrise enters `handGuiding()`
- Sunrise may record a controller-side `.log`
- ROS receives streamed KUKA state
- `multi_sensor_recorder` can subscribe to those topics if configured via launch

### Corrected GPIO-Compatible Path

Use when following the intended production workflow.

1. Run `GPIOHandGuidanceLoggerBridge` on the KUKA controller
2. Enable `gpio_handguidance_logger.enabled` in `sensors.yaml`
3. Launch the ROS stack
4. Use `S1` to enter GPIO hand-guidance mode
5. Use `S4` to start recording
6. Move the robot
7. Use `S4` to stop recording
8. Use `S1` to leave GPIO hand-guidance mode

Result:

- GPIO still controls the real hand-guidance mode
- Sunrise only logs/streams
- `multi_sensor_recorder` writes synchronized KUKA data into the same take output as the other modalities
- Sunrise may also produce the controller-side `.log` file as a secondary artifact

## Output Locations

### Common ROS Take Output

Driven by `record_take.launch.py` and `multi_sensor_recorder.py`.

This remains the main output for review and synchronization.

Typical contents:

- `<take>_1.csv`, `<take>_2.csv`, ...
- mocap YAML samples
- camera images
- ATI data
- optional KUKA hand-guidance fields in the same CSV rows

### KUKA Controller Log

Driven by Sunrise `DataRecorder`.

Typical location on the controller:

- `//192.168.10.118/KRC/Roboter/Log/DataRecorder/<timestamp>.log`

This is useful, but it is not meant to replace the combined take output.

## What Was Intentionally Not Changed

- `lbr_fri_ros2_stack` was not modified
- the earlier experimental Sunrise path was not deleted
- the GPIO hand-guidance trigger path through Arduino was not replaced

## Final Design Summary

The final design is based on one core principle:

- keep the user's established workflow intact unless there is a strong reason not to

That means:

- GPIO remains the real hand-guidance mode trigger
- `S4` remains the recording switch
- `multi_sensor_recorder` remains the main combined output
- Sunrise logging is added as a supporting capability, not a replacement for the existing take pipeline

The experimental Sunrise-driven path is still preserved, but the corrected GPIO-compatible path is the one aligned with the user's stated intent.
