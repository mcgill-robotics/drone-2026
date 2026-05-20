# Drone Control Panel UI

A modern, responsive web-based drone control panel with three camera/map screens and three control actions.

## Features

### Screens
- **Depth Camera Feed**: Displays depth camera stream from the drone (placeholder)
- **RTSP Camera Feed**: Displays RTSP camera stream (placeholder)
- **Mission Planning Map**: Google Maps integration for target planning with add/remove functionality

### Control Buttons
- **Screenshot**: Capture a screenshot from the active camera feed
- **Release Payload**: Release payload/cargo from the drone
- **Spray Water**: Activate water spray system

### Additional Features
- Tabbed navigation between screens
- Live target management on the map screen
- Toast notifications for all actions
- Keyboard shortcuts (Ctrl+S, Ctrl+P, Ctrl+W)
- Responsive design for various screen sizes
- Professional dark theme optimized for drone operations

## How to Open

### Option 1: Direct File Access
Simply open `index.html` in a web browser:
```bash
# From the drone-2026 directory
cd manual_controller
# Then open index.html in your browser (e.g., double-click or use a local server)
```

### Option 2: Using a Local Server (Recommended)
For better performance and to avoid CORS issues:

```bash
cd manual_controller

# Using Python 3
python3 -m http.server 8000

# Or using Python 2
python -m SimpleHTTPServer 8000

# Or using Node.js (if http-server is installed)
npx http-server
```

Then open your browser and navigate to:
- `http://localhost:8000`

## File Structure
```
manual_controller/
├── index.html       # Main HTML structure
├── styles.css       # Styling and layout
├── main.js         # Interactivity and event handlers
└── README.md       # This file
```

## Usage

1. **Switch Between Screens**: Click the tab buttons at the top (Depth Camera, RTSP Camera, Mission Map)

2. **Add Targets**: 
   - Navigate to the "Mission Map" screen
   - Click the "+ Add Target" button to add targets
   - Targets appear in a list below the map

3. **Remove Targets**: Click the "✕" button next to a target to remove it

4. **Control Actions**:
   - Click the button at the bottom, or use keyboard shortcuts:
     - `Ctrl+S` → Screenshot
     - `Ctrl+P` → Release Payload
     - `Ctrl+W` → Spray Water

5. **Notifications**: All actions show confirmation toasts in the bottom-right corner

## Status Indicator

The green status indicator in the header shows "System Ready" and pulses to indicate the system is active.

## Keyboard Shortcuts

| Action | Shortcut |
|--------|----------|
| Screenshot | `Ctrl + S` |
| Release Payload | `Ctrl + P` |
| Spray Water | `Ctrl + W` |

## Customization

### Colors
Edit the CSS variables in `styles.css` (`:root` section) to customize the color scheme:
```css
--primary-color: #0066cc;
--accent-color: #00d4ff;
--danger-color: #ff3333;
--warning-color: #ffaa00;
--success-color: #00cc66;
```

### Camera/Map Feeds
Replace the placeholder divs with actual video streams or map integrations by modifying the respective sections in `index.html`:
- For video: Use `<video>` or `<iframe>` tags
- For RTSP: Integrate with a streaming library like HLS.js or FFmpeg
- For Maps: Integrate Google Maps API

## Implementation Notes

Currently, this is a **UI prototype**. The following features need implementation:

- **Screenshot Action**: Connect to drone camera system to capture frames
- **Release Payload**: Connect to payload release mechanism/servos
- **Spray Water**: Connect to water spray pump/actuator
- **Depth Camera Feed**: Integrate actual depth camera stream (e.g., ROS2 topics)
- **RTSP Camera Feed**: Integrate RTSP stream (e.g., from RTSP server)
- **Mission Map**: Integrate Google Maps API and connect to mission controller

## Browser Compatibility

- Chrome/Chromium (recommended)
- Firefox
- Safari
- Edge

**Note**: Some features may require HTTPS in production environments (e.g., geolocation for maps).

## License

Part of the Drone-2026 project.
