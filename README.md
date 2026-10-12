# RDV
High speed rendezvous system

## Files:
RDV/  
├── main.py              # tiny entry point  
├── app.py               # main run sequence  
├── config.py            # tuning values  
├── camera.py            # camera model  
├── target_tracker.py    # target projection  
├── controller.py        # visual PD controller  
├── mavlink_client.py    # MAVLink wrapper  
├── math_utils.py        # rotation/math helpers  
└── logger.py            # console output  

## Parameter setup:
- Make sure the aircraft is properly tuned with a known hover value.
- Set the thrust value accordingly (climb and post takeoff).
- Ideally climb is enough to stabilize, then followed by higher throttle.

## Tuning values:
- camera resolution and FOV
- target position
- PD gains
- climb thrust
- max tilt
= loop rate

## SITL Setup:
1. Setup the SITL environment with port 14551 for companion  
python3 Tools/autotest/sim_vehicle.py -v ArduCopter  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551  

2. Run the program  
python3 main.py

## Live OV9281 camera

The live camera path is implemented in `live_camera.py` and `live_app.py`.
`main.py` selects it with `USE_REAL_CAMERA = True`. The synthetic target and
world-coordinate parameters are unused in this mode. Existing simulated code
remains available with `USE_REAL_CAMERA = False`.

Install NumPy, pymavlink, and an OpenCV **contrib build with GUI support** on the
Raspberry Pi. A headless OpenCV build cannot show the selection window. The
`rpicam-vid` command must be installed and the camera usable from your desktop
session. Run `python3 main.py`.

1. Leave `DRY_RUN = True` initially: this opens the camera without connecting to
   MAVLink or sending commands. Click the target; a 40x40 full-resolution box
   appears. Observe it follow the intended target, rather than the background.
2. Wait for ten consecutive accepted tracker updates (`LOCK READY`). Enter
   confirms the selected target. Dry run prints confirmation and keeps previewing.
   R clears selection; click again to change targets. Q/Esc/window close cancels.
3. Before flight, verify the lens FOV in `config.py` and camera mounting. The
   existing controller expects an **upward-looking camera**, image-right toward
   aircraft-right and image-down toward aircraft-forward. Rotation and axis signs
   are configurable; validate the resulting roll/pitch direction with props
   removed. A forward/downward camera needs a different controller mapping.
4. Set `DRY_RUN = False`, restart, click and verify the target, then press Enter.
   **Enter authorizes automatic GUIDED_NOGPS mode, arming, level climb, and tracking.**
   No MAVLink connection or flight command is made before this confirmation.
   The app checks fresh telemetry, a disarmed start, `GUID_OPTIONS` bit 8, and
   confirms GUIDED_NOGPS mode and arming from heartbeats. Parameters are never changed.
5. After confirmation, clicking cannot select a different target. Q/Esc/Ctrl+C,
   R, tracker failure/rejected jump, camera failure, or stale telemetry stops
   tracking. If the most recently received mode is still GUIDED_NOGPS, LAND
   is requested and checked against heartbeats. Switching out of GUIDED_NOGPS
   stops attitude commands and preserves the pilot's selected mode.

Camera/tracker target rate is 20 FPS; measured FPS depends on hardware and box
size. Preview defaults to 640x360 at 10 FPS. Change `PREVIEW_DIVISOR` to 4 for
320x180; selection size stays 40x40 camera pixels. `PREVIEW_FPS` sets rendering
cadence, capped by processing throughput. Compact stats report processed-frame
FPS, mean conversion/tracker/preview time, estimated computational headroom, and
skipped capture frames. Headroom excludes camera waits and is not a guaranteed
maximum camera rate. The newest complete frame is used; queues do not accumulate.

Pixel errors are `(u-cx)/fx` and `(v-cy)/fy`; the controller uses elapsed frame
time and suppresses the initial derivative spike. Live CSV logs use
`LIVE_CSV_LOG_PATH` with observed pixels, normalized errors, commands, and altitude
above the launch baseline. No target world position or range is inferred from a single camera.

This remains the original thrust-based launch controller: it does not hold
altitude during tracking. `GUID_OPTIONS` bit 8 selects direct thrust; without it
ArduPilot interprets 0.5 as zero climb rate, so the existing thrust values are
incompatible. Source: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
Tune thrust/gains for the aircraft and validate in SITL and a props-off direction
check before airborne use. A CSRT success flag plus jump/area checks is not proof
of target identity; slow drift onto background can pass these checks. LAND is verified by heartbeat where possible; loss of the link can prevent
confirmation. LAND may drift when horizontal position is unavailable. Existing autopilot failsafes still apply.


### Camera servoing without GPS (this patch)

The live path selects an ArduPilot heartbeat, requests ATTITUDE and VFR_HUD
at the tracking rate, and uses GUIDED_NOGPS. It never requires LOCAL_POSITION_NED,
GPS coordinates, or an EKF horizontal position. Simulated `app.py` still uses NED.
VFR_HUD altitude is captured before arming; altitude above that baseline controls
the existing POST_TAKEOFF_ALT_M thrust phase. This is not altitude hold while tracking.
All existing gains and direct-thrust settings are retained. GUID_OPTIONS bit 8
must remain set. No parameter is changed automatically.

Click, observe LOCK READY, then Enter confirms launch. DRY_RUN=True remains the
shipped default; set DRY_RUN=False to enable flight. Confirmed selection freezes
reselection; the preview shows STARTUP until an armed heartbeat is received.

On Q/Esc/Ctrl+C, camera failure, target loss, or stale telemetry, attitude commands
stop and the app requests LAND if still in GUIDED_NOGPS after an arm request.
It checks for a LAND heartbeat for up to 3 seconds. LAND can drift without
horizontal navigation. This replaces the previous stop-position-hold behavior.
Pilot mode changes are preserved. Hardware and airborne behavior are untested;
verify in SITL and on a propeller-free bench before flight.
