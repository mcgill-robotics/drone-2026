# Spray Control Implementation Guide

This document describes the implementation of the spray mechanic for the drone control UI.

## Overview

The spray control system is implemented using a **pushbutton mechanic** where:
- **Hold down the button** → Spray pump activates (PWM set to 1900)
- **Release the button** → Spray pump deactivates (PWM set to 1500/neutral)

The system communicates via HTTP API with a Flask backend that sends servo commands to the PX4 flight controller through MAVROS.

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
               │ (JSON: {"action": "activate|deactivate", "channel": 3})
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
               │ MAV_CMD_DO_SET_SERVO
               │ command_long_client.call_async()
               ▼
┌──────────────────────────────────────────────────────────┐
│ MAVROS (ROS 2 ↔ Flight Controller)                       │
│  - /mavros/cmd/command service                           │
│  - Sends servo commands to PX4 autopilot                │
└──────────────┬──────────────────────────────────────────┘
               │
               ▼
┌──────────────────────────────────────────────────────────┐
│ PX4 Flight Controller                                    │
│  - Receives MAV_CMD_DO_SET_SERVO                         │
│  - Controls servo/motor on specified channel             │
│  - Channel 3 = AUX1 (typically spray pump motor)        │
└──────────────────────────────────────────────────────────┘
```

## Components

### 1. Backend: PX4 Interface Enhancement

**File:** `mission_controller/px4_setters.py`

Added two new methods:

#### `activate_spray(servo_channel=3, pwm_value=1900, timeout=10)`
Activates the spray pump by sending a servo command.
- **servo_channel**: Servo/PWM channel (3 = AUX1 by default)
- **pwm_value**: PWM signal (1900 = full on)
- **Returns**: True on success, False on failure

#### `deactivate_spray(servo_channel=3, timeout=10)`
Deactivates the spray pump by setting servo to neutral.
- **pwm_value**: 1500 (neutral/off position)
- **Returns**: True on success, False on failure

Also updated `px4_getters.py` to include the `CommandLong` service client for sending servo commands.

### 2. Backend: API Server

**File:** `mission_controller/api_server.py`

A lightweight Flask HTTP server that provides endpoints for the UI to control drone functions:

#### Key Endpoints

**POST /spray** - Control spray pump
```json
Request:
{
    "action": "activate" | "deactivate",
    "channel": 3,    // Optional: servo channel (default 3)
    "pwm": 1900      // Optional: PWM value for activate (default 1900)
}

Response:
{
    "success": true,
    "action": "activate",
    "channel": 3,
    "pwm": 1900,
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
    channel: 3,              // Servo channel
    pwmOn: 1900,            // Active PWM
    pwmOff: 1500            // Neutral PWM
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
ros2 launch mavros apm.launch fcu_url:="serial:///dev/ttyTHS1:921600"
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

### Servo Channel
The default servo channel is **3 (AUX1)**. To change it:

**Backend:**
```python
# In api_server.py, modify the POST /spray endpoint:
channel = data.get('channel', 3)  # Change default here
```

**Frontend:**
```javascript
// In main.js, modify sprayState:
sprayState.channel = 3;  // Change to your servo channel
```

### PWM Values
Customize PWM values for your servo/motor:

```javascript
// In main.js, sprayState:
pwmOn: 1900,   // PWM when spray is active (range 1000-2000)
pwmOff: 1500   // PWM when spray is off (neutral)
```

**Common PWM ranges:**
- 1000-1500: Reduced speed
- 1500: Neutral/Off
- 1500-2000: Increasing speed
- 1900: Full speed (typically)

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
px4.activate_spray(servo_channel=3, pwm_value=1900)
px4.deactivate_spray(servo_channel=3)
```

### API Testing
Use `curl` to test endpoints:

```bash
# Health check
curl http://localhost:5000/health

# Activate spray
curl -X POST http://localhost:5000/spray \
  -H "Content-Type: application/json" \
  -d '{"action":"activate","channel":3,"pwm":1900}'

# Deactivate spray
curl -X POST http://localhost:5000/spray \
  -H "Content-Type: application/json" \
  -d '{"action":"deactivate","channel":3}'

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
- Verify servo channel is correct for your hardware
- Check PX4 logs for MAV_CMD_DO_SET_SERVO errors
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
- MAV_CMD_DO_SET_SERVO: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO
- PX4 Flight Controller: https://px4.io/
- Flask Documentation: https://flask.palletsprojects.com/
