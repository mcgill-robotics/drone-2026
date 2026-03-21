# ArduPilot Integration Guide

This document describes how the mission controller integrates with ArduPilot autopilots via pymavlink.

## Architecture

```
┌─────────────────────────────────────────────┐
│   mission_controller (FSM, objectives)      │
├─────────────────────────────────────────────┤
│         stubs.py (flight commands)          │
├─────────────────────────────────────────────┤
│   ardupilot_interface.py (MAVLink wrapper)  │
├─────────────────────────────────────────────┤
│    pymavlink (MAVLink protocol library)     │
├─────────────────────────────────────────────┤
│  ArduPilot Flight Controller (via network)  │
└─────────────────────────────────────────────┘
```

## Initialization

### SITL (Software-In-The-Loop) Testing

For development/testing without real hardware:

```python
from mission_controller import init_autopilot

# Start ArduPilot SITL simulator
# $ sim_vehicle.py -v ArduCopter --console

# Connect from Python
autopilot = init_autopilot("udp:127.0.0.1:14550")
```

### Real Hardware via Serial

For Pixhawk/Cube autopilot connected via USB:

```python
autopilot = init_autopilot("/dev/ttyUSB0")  # Linux
autopilot = init_autopilot("COM3", baud=921600)  # Windows
autopilot = init_autopilot("/dev/ttyAMA0")  # Raspberry Pi
```

### Real Hardware via WiFi

For telemetry radio or WiFi connection:

```python
autopilot = init_autopilot("udp:192.168.1.100:14550")
```

## Flight Control Functions

All flight control is implemented in `stubs.py` using `ardupilot_interface.py`:

### Basic Flight Operations

```python
from mission_controller import takeoff_drone, land_drone, goto_drone

# Takeoff to 50 meters
takeoff_drone(altitude=50)

# Navigate to GPS location
goto_drone({
    'lat': 37.786885,
    'lon': -122.394105,
    'alt': 50
})

# Land at current location
land_drone()
```

### Telemetry Access

```python
from mission_controller import get_autopilot

autopilot = get_autopilot()

# Get current location
location = autopilot.get_location()
print(f"Lat: {location['lat']}, Lon: {location['lon']}, Alt: {location['alt']}")

# Get altitude only
alt = autopilot.get_altitude()

# Get battery status
battery = autopilot.get_battery_status()
print(f"Voltage: {battery['voltage']}V, Current: {battery['current']}A")

# Check if armed
if autopilot.is_armed():
    print("Vehicle is armed")
```

### Flight Modes

Supported modes include:
- `STABILIZE` - Manual stabilization
- `GUIDED` - Autonomous navigation
- `AUTO` - Waypoint following
- `LAND` - Automatic landing
- `RTL` - Return to Launch
- `LOITER` - Hover in place

```python
autopilot.change_mode("GUIDED", timeout=30)
```

## Pymavlink Reference

The underlying communication uses pymavlink, which provides:

- **MAVLink Message Handling**: Automatic parsing of telemetry messages
- **Message Sending**: Command and control message formatting
- **System/Component IDs**: Addressing specific vehicle systems
- **Mode Mapping**: Translation between mode names and numeric IDs

### Common MAVLink Messages Used

| Message | Purpose | Access Method |
|---------|---------|----------------|
| HEARTBEAT | System status | `mav.messages['HEARTBEAT']` |
| GLOBAL_POSITION_INT | GPS location/altitude | `mav.messages['GLOBAL_POSITION_INT']` |
| BATTERY_STATUS | Battery information | `mav.messages['BATTERY_STATUS']` |
| ATTITUDE | Vehicle orientation | `mav.messages['ATTITUDE']` |
| VFR_HUD | Airspeed, groundspeed | `mav.messages['VFR_HUD']` |

### Common MAVLink Commands

| Command | Purpose | Usage |
|---------|---------|-------|
| MAV_CMD_COMPONENT_ARM_DISARM | Arm/disarm vehicle | `command_long_send(...)` |
| MAV_CMD_NAV_TAKEOFF | Takeoff command | `command_long_send(...)` |
| SET_MODE | Change flight mode | `set_mode_send(...)` |
| SET_POSITION_TARGET_GLOBAL_INT | Navigate to GPS | `set_position_target_global_int_send(...)` |
| RC_CHANNELS_OVERRIDE | Override RC channels | `rc_channels_override_send(...)` |

## Implementing Team Functions

Several functions in `stubs.py` require team implementation beyond basic flight control:

### 1. Boustrophedon Search Pattern (`generate_print_pattern`)

Generates parallel lines across a search area:

```python
def generate_print_pattern(start, goal, waypoint_spacing=10):
    """
    Generate back-and-forth pattern waypoints
    
    Requirements:
    - Calculate line orientation based on start->goal vector
    - Generate parallel lines perpendicular to that vector
    - Maintain waypoint_spacing between lines
    - Return list of GPS Points
    """
    # TODO: Implement pattern generation
```

### 2. Payload Release Mechanism (`drop_payload`)

Control servo or valve for payload release:

```python
def drop_payload(target):
    """
    Requirements:
    - Identify which RC channel controls payload servo (typically channel 8)
    - Set PWM to release position (usually 2000 microseconds)
    - Wait for mechanical action
    - Return to neutral (1000 microseconds)
    """
    autopilot = get_autopilot()
    autopilot.set_rc_channel(8, 2000)  # Release
    time.sleep(1)
    autopilot.set_rc_channel(8, 1000)  # Neutral
```

### 3. Sprayer/Extinguisher Control (`extinguish_fire`)

Control spray pump or solenoid valve:

```python
def extinguish_fire(location):
    """
    Requirements:
    - Navigate to fire location using goto_drone()
    - Activate sprayer (may be RC channel, GPIO, or PWM)
    - Spray for appropriate duration
    - Deactivate sprayer
    """
    autopilot = get_autopilot()
    autopilot.set_rc_channel(7, 2000)  # Activate sprayer
    time.sleep(5)  # Spray for 5 seconds
    autopilot.set_rc_channel(7, 1000)  # Deactivate
```

### 4. Vision-Based Detection

Functions like `boustrophedon_search()` and `pad_has_extinguisher()` require:

- **Camera Integration**: OpenCV or similar for image processing
- **Detection Algorithms**: Object detection (landing pad, extinguisher, fire)
- **Onboard Processing**: Must run on Jetson with reasonable latency

Example skeleton:

```python
import cv2
from jetson_inference import detectNet

def pad_has_extinguisher(pad_location):
    """Detect if extinguisher is on landing pad"""
    frame = grab_camera_frame()  # Capture from onboard camera
    detections = detectNet.Detect(frame, 'extinguisher')
    return len(detections) > 0
```

## Connection Strings

Format: `<protocol>:<address>:<port>`

| Connection Type | String | Notes |
|-----------------|--------|-------|
| SITL via UDP | `udp:127.0.0.1:14550` | Default for sim_vehicle.py |
| Remote UDP | `udp:192.168.x.x:14550` | Telemetry radio or WiFi |
| Serial USB | `/dev/ttyUSB0` | Linux USB connection |
| Serial UART | `/dev/ttyAMA0` | Raspberry Pi built-in UART |
| COM Port | `COM3` | Windows COM port |
| TCP | `tcp:192.168.1.100:5760` | TCP telemetry connection |

## Jetson-Specific Notes

### Optimization for Jetson Nano/Xavier

1. **Serial Connections Performance**
   - Jetson built-in UART: `/dev/ttyAMA0` at 921600 baud
   - Most reliable for production

2. **UDP Telemetry**
   - WiFi can introduce latency
   - Recommended baud equivalent: 921600 on serial
   - UDP can drop packets under load

3. **CPU/Memory Constraints**
   - Mission thread: ~50ms update cycle (safe on Jetson)
   - Vision processing: Consider threaded processing
   - Log files: Store to external USB for large missions

4. **Power Management**
   - Monitor battery via telemetry regularly
   - Jetson needs stable 5V power (external battery recommended)
   - Avoid excessive CPU load during landing/critical maneuvers

### Example Jetson Setup

```python
# Serial connection to Pixhawk on Jetson
if __name__ == "__main__":
    # Connect via built-in UART
    autopilot = init_autopilot(
        "/dev/ttyAMA0",  # Jetson built-in UART
        baud=921600,      # High speed serial
        timeout=30
    )
```

## Testing

### Unit Testing (No Hardware)

```python
# Test functions that check conditions
def test_at_position():
    from mission_controller import at_position, Point
    assert at_position(Point(0, 0, 0), tolerance=0.1) == True/False
```

### SITL Testing

```bash
# Terminal 1: Start SITL simulator
sim_vehicle.py -v ArduCopter --console

# Terminal 2: Run mission controller
python mission_controller.py
```

### Hardware Testing

```bash
# Connect to real vehicle telemetry
# Then run missions with real autopilot
python mission_controller.py
```

## Troubleshooting

### Connection Issues

```python
# Check if autopilot is None before using
if not get_autopilot():
    print("Autopilot not connected")
    # Start SITL or check hardware connection
```

### Timeout Errors

```python
# Increase timeout for slow connections
autopilot = init_autopilot("udp:192.168.1.100:14550", timeout=60)
```

### MAVLink Mode Errors

```python
# Verify mode name is correct for vehicle type
# ArduCopter modes: STABILIZE, ALT_HOLD, LOITER, LAND, RTL, GUIDED, etc.
# ArduPlane modes: MANUAL, CIRCLE, AUTO, RTL, LAND, FBWA, etc.
```

## References

- [PyMAVLink Documentation](https://dronekit-python.readthedocs.io/en/latest/automodule.html)
- [ArduCopter Flight Modes](https://ardupilot.org/copter/docs/flight-modes.html)
- [MAVLink Protocol](https://mavlink.io/en/)
- [ArduPilot SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

