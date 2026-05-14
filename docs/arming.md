# Arming with a dedicated RC switch

This is the flow we use for hardware tests and race day: the companion
computer handles mode (OFFBOARD), the pilot holds physical arm/kill
authority via dedicated RC switches.

## Required PX4 parameters

Set these via QGroundControl ("Parameters" → search) or MAVLink shell
(`param set ...`). Values are starting points — adjust channel numbers to
your transmitter's wiring.

| Parameter            | Value            | Why                                      |
| -------------------- | ---------------- | ---------------------------------------- |
| `RC_MAP_ARM_SW`      | channel #        | RC channel mapped to your arm switch     |
| `COM_ARM_SWISBTN`    | 0 (toggle) / 1 (momentary) | Match your switch type         |
| `RC_MAP_KILL_SW`     | channel #        | Kill switch — cuts motors instantly      |
| `COM_RC_ARM_HYST`    | 1000 (default)   | Hold-time before arm registers (ms)      |
| `COM_RCL_EXCEPT`     | 4                | Allow OFFBOARD even if RC link lost      |
| `COM_OF_LOSS_T`      | 1.0              | Setpoint-loss timeout (s) — don't lower  |

After setting, **reboot the FCU** and verify in QGC that the switch
appears in the "Radio" tab and toggles `Armed` state.

## Pre-flight verification

Before props on, with the vehicle on a bench:

1. Power the FCU. Wait for GPS 3D fix (≥6 sats, EKF green).
2. Boot the companion stack: `boot_px4(...)`.
3. Confirm MAVROS is connected: `px4.connected == True`.
4. Start the heartbeat stream **before** mode change.
5. Switch to OFFBOARD via `start_offboard()`. Confirm
   `current_state.mode == "OFFBOARD"`.
6. Watch QGC: it should show OFFBOARD active and "Ready to Arm".
7. Flip the arm switch. Motors arm.
8. Flip the kill switch. Motors disarm instantly.

If any step fails, **do not put props on**. The failure modes with props
attached are dangerous.

## Race-day arming flow

```
companion boots → MAVROS up → heartbeat stream → OFFBOARD → pilot arm switch → mission runs → pilot disarm or kill
```

The companion never calls `arm_vehicle()` in this flow. The pilot is the
arming authority.

## Common gotchas

- **OFFBOARD drops before arm**: setpoints stopped flowing for >1s. Make
  sure `start_offboard_stream_background()` is running before the arm
  wait, not just inside it.
- **Switch flipped but nothing happens**: `RC_MAP_ARM_SW` not set, or set
  to the wrong channel. Check the "Radio" tab in QGC to confirm the
  channel moves when you flip the switch.
- **Arms then immediately disarms**: pre-arm check failure that only
  triggers after arming briefly. Usually low battery or EKF reject.
  Check `/mavros/statustext`.
- **Won't arm while in OFFBOARD**: `COM_ARM_SWISBTN` mismatch with switch
  type (toggle vs momentary), or `RC_MAP_ARM_SW` not pointing at a free
  channel.
