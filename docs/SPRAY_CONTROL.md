# Spray Control Implementation Guide

This document describes the implementation of the spray mechanic for the drone control UI.

## Overview

The spray control system is implemented using a **pushbutton mechanic** where:
- **Hold down the button** → Spray pump activates (actuator value set to 1.0)
- **Release the button** → Spray pump deactivates (actuator value set to 0.0)

The system communicates via HTTP API with a Flask backend that sends PX4 actuator commands to the flight controller through MAVROS.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│ manual_controller/ (Frontend UI)                        │
│  - index.html (3 buttons: Screenshot, Payload, Spray)  │
│  - main.js (HTTP API calls)                            │
│  - styles.css (UI styling)                             │
└──────────────┬──────────────────────────────────────────┘
               │
              │ HTTP POST /spray
              │ (JSON: {"action": "activate|deactivate", "channel": 1, "value": 1.0})
               ▼
┌──────────────────────────────────────────────────────────┐
│ mission_controller/api_server.py (Flask HTTP Server)     │
│  - Listens on localhost:5000                             │
│  - Handles /spray, /health, /telemetry endpoints        │
└──────────────┬──────────────────────────────────────────┘
               │
               │ ROS 2 Service calls
               │ CommandLong service
               ▼
┌──────────────────────────────────────────────────────────┐
│ mission_controller/px4_interface.py (PX4 Interface)     │
│  - Wraps MAVROS communication                            │
│  - activate_spray() / deactivate_spray() methods        │
└──────────────┬──────────────────────────────────────────┘
               │
               │ MAV_CMD_DO_SET_ACTUATOR
               │ command_long_client.call_async()
               ▼
┌──────────────────────────────────────────────────────────┐
│ MAVROS (ROS 2 ↔ Flight Controller)                       │
│  - /mavros/cmd/command service                           │
│  - Sends actuator commands to PX4 autopilot             │
└──────────────┬──────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────┐
│ PX4 Flight Controller                                    │
│  - Receives MAV_CMD_DO_SET_ACTUATOR                     │
│  - Controls outputs mapped to Actuator Set 1..6         │
│  - Use Peripheral via Actuator Set 1 for the sprayer     │
└──────────────────────────────────────────────────────────┘
```

## Components

### 1. Backend: PX4 Interface Enhancement

**File:** `mission_controller/px4_setters.py`

Added two new methods:

#### `activate_spray(actuator_slot=1, actuator_value=1.0, timeout=10)`
Activates the spray pump by sending a PX4 actuator command.
- **actuator_slot**: Actuator slot (1 = Actuator Set 1 by default)
- **actuator_value**: Normalized actuator value (1.0 = on)
- **Returns**: True on success, False on failure

#### `deactivate_spray(actuator_slot=1, actuator_value=0.0, timeout=10)`
Deactivates the spray pump by setting the actuator value to zero.
- **actuator_value**: Normalized actuator value (0.0 = off)
- **Returns**: True on success, False on failure

Also updated `px4_getters.py` to include the `CommandLong` service client for sending actuator commands.

### 2. Backend: API Server

**File:** `mission_controller/api_server.py`

A lightweight Flask HTTP server that provides endpoints for the UI to control drone functions:

#### Key Endpoints

**POST /spray** - Control spray pump
```json
Request:
{
    "action": "activate" | "deactivate",
    "channel": 1,    // Optional: actuator slot (default 1 for Actuator Set 1)
    "value": 1.0     // Optional: normalized actuator value for activate (default 1.0)
}

Response:
{
    "success": true,
    "action": "activate",
   "channel": 1,
   "value": 1.0,
    "message": "Spray activated"
}
```

**GET /health** - Health check
```json
Response:
{
    "status": "ok",
    "connected": true,
    "timestamp": 1234567890
}
```

**GET /spray/status** - Spray system status
**GET /telemetry/position** - Drone position
**GET /telemetry/status** - Drone status

#### Usage

```bash
# Run API server with defaults (SITL on localhost:14540)
python3 mission_controller/api_server.py --sitl

# Run with hardware (serial connection)
python3 mission_controller/api_server.py --fcu-url "serial:///dev/ttyTHS1:921600"

# Run on custom port
python3 mission_controller/api_server.py --sitl --port 5000

# Skip PX4 boot (assume already running)
python3 mission_controller/api_server.py --sitl --no-boot
```

### 3. Frontend: UI Button Integration

**File:** `manual_controller/main.js`

Implemented **pushbutton mechanics** with the following features:

#### Spray State Management
```javascript
let sprayState = {
    isActive: false,
    isPushbuttonMode: true,  // Hold to spray
  channel: 1,              // Actuator slot
  valueOn: 1.0,            // Active actuator value
  valueOff: 0.0            // Inactive actuator value
};
```

#### Button Behavior

**Mousedown/Touchstart:**
- Activates spray pump
- Shows visual feedback (button highlighted)
- Shows toast notification

**Mouseup/Touchend:**
- Deactivates spray pump
- Removes visual feedback
- Shows completion notification

**Keyboard Support (Ctrl+W):**
- Ctrl+W pressed: Activate spray
- Ctrl+W released: Deactivate spray

#### Error Handling
- Checks API connection on page load
- Shows warning if API server is not running
- Displays error toasts if spray commands fail
- Gracefully handles network errors

#### Visual Feedback
- Button shows glowing animation when active
- Color changes to cyan when spray is on
- Pulsing effect indicates active state
- Toast messages confirm actions

### 4. Styling: Active Button State

**File:** `manual_controller/styles.css`

Added `.active` class styling for the spray button:
- Glowing effect with pulsing animation
- Color change to indicate active state
- Enhanced visual feedback for pushbutton interaction

## Installation

### Prerequisites
- Python 3.7+
- Flask and Flask-CORS
- ROS 2 and MAVROS
- Mission controller environment set up

### Setup Steps

1. **Install Flask dependencies:**
```bash
pip install flask flask-cors
```

2. **Ensure MAVROS is running with PX4:**
```bash
# For SITL
ros2 launch mavros px4.launch fcu_url:="udp://127.0.0.1:14540"

# For Hardware
ros2 launch mavros px4.launch fcu_url:="serial:///dev/ttyTHS1:921600"

In QGroundControl, map the sprayer output to `Peripheral via Actuator Set 1`
instead of `RC AUX1`. PX4 will reject the command if the output is still bound
to an RC passthrough or motor function.
```

3. **Start the API server:**
```bash
cd /home/drone/drone-2026
python3 mission_controller/api_server.py --sitl
```

4. **Open the UI in a browser:**
```
Open manual_controller/index.html in a web browser
```

## Configuration

### Actuator Slot
The default actuator slot is **1 (Actuator Set 1)**. To change it:

**Backend:**
```python
# In api_server.py, modify the POST /spray endpoint:
channel = data.get('channel', 1)  # Change default here
```

**Frontend:**
```javascript
// In main.js, modify sprayState:
sprayState.channel = 1;  // Change to your actuator slot
```

### Actuator Values
Customize actuator values for your sprayer:

```javascript
// In main.js, sprayState:
valueOn: 1.0,   // Actuator value when spray is active
valueOff: 0.0   // Actuator value when spray is off
```

**Common actuator values:**
- 0.0: Off
- 1.0: Full on
- -1.0..1.0: Normalized control range if your output function supports it

### Button Behavior Mode
Switch between pushbutton (hold) and toggle (click) modes:

```javascript
// In main.js, sprayState:
isPushbuttonMode: true   // true = hold to spray, false = click to toggle
```

## Testing

### Unit Testing
Test spray methods directly:

```python
# In a Python script
from mission_controller.px4_interface import init_px4

px4 = init_px4(fcu_url="udp://127.0.0.1:14540")
px4.activate_spray(actuator_slot=1, actuator_value=1.0)
px4.deactivate_spray(actuator_slot=1)
```

### API Testing
Use `curl` to test endpoints:

```bash
# Health check
curl http://localhost:5000/health

# Activate spray
curl -X POST http://localhost:5000/spray \
  -H "Content-Type: application/json" \
  -d '{"action":"activate","channel":1,"value":1.0}'

# Deactivate spray
curl -X POST http://localhost:5000/spray \
  -H "Content-Type: application/json" \
   -d '{"action":"deactivate","channel":1}'

# Get spray status
curl http://localhost:5000/spray/status
```

### UI Testing
1. Open `manual_controller/index.html` in browser
2. Hold down the "Spray Water" button → Should show activation toast
3. Release the button → Should show deactivation toast
4. Check browser console for debug messages
5. Verify API calls in network tab (F12 → Network)

## Troubleshooting

### API Server Won't Start
- Check if port 5000 is available: `lsof -i :5000`
- Ensure ROS 2 and MAVROS are running
- Check Flask installation: `pip show flask`

### Spray Command Fails
- Verify the output is mapped to `Peripheral via Actuator Set 1`
- Check PX4 logs for `MAV_CMD_DO_SET_ACTUATOR` errors
- Ensure MAVROS CommandLong service is available

### Frontend Can't Connect to API
- Check API server is running: `curl http://localhost:5000/health`
- Verify browser console for CORS errors
- Check firewall settings (port 5000 must be accessible)
- Modify `API_CONFIG` in main.js if using different host/port

### Button Doesn't Respond
- Check browser console (F12) for JavaScript errors
- Verify API server health check passes
- Check that PX4 is armed (can spray while disarmed in SITL for testing)
- Try refreshing the page

## Future Enhancements

1. **Multiple Servo Channels:**
   - Support controlling multiple actuators (payload release, etc.)
   - Add channel selection in UI

2. **Status Feedback:**
   - Real-time servo position feedback
   - Motor current monitoring
   - Tank level indication

3. **Timing Control:**
   - Spray duration presets
   - Burst spray mode
   - Continuous vs. pulsed spray

4. **Integration with Mission Controller:**
   - Automated spray during missions
   - Target-based spraying
   - Integration with strategy system

## References

- MAVROS Documentation: http://wiki.ros.org/mavros
- MAV_CMD_DO_SET_ACTUATOR: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ACTUATOR
- PX4 Flight Controller: https://px4.io/
- Flask Documentation: https://flask.palletsprojects.com/
