"""
ArduPilot interface wrapper using pymavlink
This module provides a connection to ArduPilot autopilots via MAVLink
Adapted from ArduPilot's Tools/autotest framework

Note: This module uses lazy imports of pymavlink to avoid hanging on import
if pymavlink is not installed. pymavlink is only required at runtime when
connecting to an actual autopilot.
"""

import time


class ArduPilotInterface:
    """
    Wrapper for communicating with ArduPilot via MAVLink
    Uses pymavlink library for protocol communication
    """
    
    def __init__(self, connection_string="udp:127.0.0.1:14550", baud=115200, timeout=30):
        """
        Initialize connection to ArduPilot autopilot
        
        Args:
            connection_string: MAVLink connection string (e.g., "udp:127.0.0.1:14550", "/dev/ttyUSB0")
            baud: Serial baud rate (default 115200 for most serial connections)
            timeout: Connection timeout in seconds
        """
        self.mav = None
        self.connection_string = connection_string
        self.baud = baud
        self.timeout = timeout
        self.connected = False
        print(f"[ARDUPILOT] Initialized (ready to connect) - {connection_string}")
    
    def connect(self):
        """
        Establish connection to autopilot
        """
        try:
            from pymavlink import mavutil
        except (ImportError, Exception) as e:
            print("[ERROR] pymavlink library not available")
            print("[ERROR] Install with: pip install pymavlink")
            print(f"[ERROR] Details: {str(e)}")
            self.connected = False
            return False
        
        try:
            print(f"[ARDUPILOT] Connecting to {self.connection_string}...")
            self.mav = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baud,
                timeout=self.timeout
            )
            self.mav.wait_heartbeat(timeout=self.timeout)
            print(f"[ARDUPILOT] Connected to autopilot: {self.mav.sysid}")
            self.connected = True
            return True
        except Exception as e:
            print(f"[ARDUPILOT] Connection failed: {str(e)}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close connection to autopilot"""
        if self.mav:
            self.mav.close()
            self.connected = False
            print("[ARDUPILOT] Disconnected")
    
    def is_armed(self):
        """Check if vehicle is armed"""
        if not self.connected or not self.mav:
            return False
        try:
            from pymavlink import mavutil
            msg = self.mav.messages.get('HEARTBEAT')
            if msg:
                return (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        except:
            pass
        return False
    
    def arm_vehicle(self, timeout=20):
        """
        Arm the vehicle (allow motors to spin)
        
        Args:
            timeout: Timeout in seconds
        """
        if not self.connected or not self.mav:
            print("[ARDUPILOT] Not connected, cannot arm")
            return False
        
        if self.is_armed():
            print("[ARDUPILOT] Vehicle already armed")
            return True
        
        print("[ARDUPILOT] Arming vehicle...")
        try:
            from pymavlink import mavutil
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                1,  # arm command
                0, 0, 0, 0, 0, 0
            )
            
            start = time.time()
            while not self.is_armed() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if self.is_armed():
                print("[ARDUPILOT] Vehicle armed successfully")
                return True
            else:
                print("[ARDUPILOT] Arming timeout")
                return False
        except Exception as e:
            print(f"[ARDUPILOT] Arm failed: {str(e)}")
            return False
    
    def disarm_vehicle(self, timeout=20):
        """Disarm the vehicle (prevent motors from spinning)"""
        if not self.connected or not self.mav:
            return False
        
        if not self.is_armed():
            print("[ARDUPILOT] Vehicle already disarmed")
            return True
        
        print("[ARDUPILOT] Disarming vehicle...")
        try:
            from pymavlink import mavutil
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # confirmation
                0,  # disarm command
                0, 0, 0, 0, 0, 0
            )
            
            start = time.time()
            while self.is_armed() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not self.is_armed():
                print("[ARDUPILOT] Vehicle disarmed successfully")
                return True
            else:
                print("[ARDUPILOT] Disarm timeout")
                return False
        except Exception as e:
            print(f"[ARDUPILOT] Disarm failed: {str(e)}")
            return False
    
    def change_mode(self, mode_name, timeout=30):
        """
        Change vehicle flight mode
        
        Args:
            mode_name: Mode name (e.g., "GUIDED", "AUTO", "LAND", "RTL")
            timeout: Timeout in seconds
        """
        if not self.connected or not self.mav:
            print("[ARDUPILOT] Not connected, cannot change mode")
            return False
        
        print(f"[ARDUPILOT] Changing mode to {mode_name}...")
        try:
            from pymavlink import mavutil
            # Get mode ID from mode name
            if mode_name in self.mav.mode_mapping():
                mode_id = self.mav.mode_mapping()[mode_name]
            else:
                print(f"[ARDUPILOT] Unknown mode: {mode_name}")
                return False
            
            # Send set mode command
            self.mav.mav.set_mode_send(
                self.mav.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            start = time.time()
            while True:
                msg = self.mav.messages.get('HEARTBEAT')
                if msg and msg.custom_mode == mode_id:
                    print(f"[ARDUPILOT] Mode changed to {mode_name}")
                    return True
                if (time.time() - start) > timeout:
                    print(f"[ARDUPILOT] Mode change timeout")
                    return False
                time.sleep(0.1)
        except Exception as e:
            print(f"[ARDUPILOT] Mode change failed: {str(e)}")
            return False
    
    def takeoff(self, altitude, timeout=60):
        """
        Perform takeoff to specified altitude
        
        Args:
            altitude: Target altitude in meters
            timeout: Timeout in seconds
        """
        if not self.connected or not self.mav:
            print("[ARDUPILOT] Not connected, cannot takeoff")
            return False
        
        print(f"[ARDUPILOT] Taking off to {altitude}m...")
        
        try:
            from pymavlink import mavutil
            # Arm if not already armed
            if not self.is_armed():
                if not self.arm_vehicle():
                    return False
            
            # Switch to GUIDED mode for takeoff command
            if not self.change_mode("GUIDED"):
                return False
            
            # Send takeoff command
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,  # confirmation
                0,  # pitch (0 = default)
                0,  # empty
                0,  # empty
                0,  # yaw
                0,  # latitude
                0,  # longitude
                altitude  # altitude
            )
            
            # Wait for altitude
            current_alt = 0
            start = time.time()
            while current_alt < (altitude * 0.95):  # 95% of target altitude
                msg = self.mav.messages.get('GLOBAL_POSITION_INT')
                if msg:
                    current_alt = msg.relative_alt / 1000.0  # mm to m
                    print(f"[ARDUPILOT] Current altitude: {current_alt:.1f}m")
                
                if (time.time() - start) > timeout:
                    print("[ARDUPILOT] Takeoff timeout")
                    return False
                time.sleep(0.5)
            
            print(f"[ARDUPILOT] Takeoff complete, reached {current_alt:.1f}m")
            return True
        except Exception as e:
            print(f"[ARDUPILOT] Takeoff failed: {str(e)}")
            return False
    
    def land(self, timeout=60):
        """
        Perform landing sequence
        
        Args:
            timeout: Timeout in seconds
        """
        if not self.connected or not self.mav:
            print("[ARDUPILOT] Not connected, cannot land")
            return False
        
        print("[ARDUPILOT] Landing...")
        
        try:
            # Switch to LAND mode
            if not self.change_mode("LAND"):
                return False
            
            # Wait for landing
            start = time.time()
            while True:
                msg = self.mav.messages.get('GLOBAL_POSITION_INT')
                if msg:
                    current_alt = msg.relative_alt / 1000.0  # mm to m
                    if current_alt < 1.0:  # Close enough to ground
                        print("[ARDUPILOT] Landing complete")
                        return True
                
                if (time.time() - start) > timeout:
                    print("[ARDUPILOT] Landing timeout")
                    return False
                time.sleep(0.5)
        except Exception as e:
            print(f"[ARDUPILOT] Landing failed: {str(e)}")
            return False
    
    def goto_location(self, lat, lon, alt, timeout=60):
        """
        Navigate to a GPS location
        
        Args:
            lat: Latitude
            lon: Longitude
            alt: Altitude in meters
            timeout: Timeout in seconds
        """
        if not self.connected or not self.mav:
            print("[ARDUPILOT] Not connected, cannot navigate")
            return False
        
        print(f"[ARDUPILOT] Navigating to ({lat:.6f}, {lon:.6f}, {alt}m)...")
        
        try:
            from pymavlink import mavutil
            # Switch to GUIDED mode for navigation
            if not self.change_mode("GUIDED"):
                return False
            
            # Send goto location command
            self.mav.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.mav.target_system,
                self.mav.target_component,
                0,  # frame (GLOBAL_INT)
                0b0000111111000111,  # type_mask (position only)
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
                0, 0, 0,  # vx, vy, vz
                0, 0, 0,  # afx, afy, afz
                0, 0  # yaw, yaw_rate
            )
            
            print(f"[ARDUPILOT] Waypoint sent")
            return True
        except Exception as e:
            print(f"[ARDUPILOT] Navigation failed: {str(e)}")
            return False
    
    def get_location(self):
        """Get current vehicle location"""
        if not self.connected or not self.mav:
            return None
        
        try:
            msg = self.mav.messages.get('GLOBAL_POSITION_INT')
            if msg:
                return {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "alt": msg.alt / 1000.0,  # mm to m
                    "relative_alt": msg.relative_alt / 1000.0
                }
        except Exception as e:
            print(f"[ARDUPILOT] Failed to get location: {str(e)}")
        return None
    
    def get_altitude(self):
        """Get current altitude"""
        loc = self.get_location()
        return loc["relative_alt"] if loc else 0
    
    def set_rc_channel(self, channel, pwm_value, timeout=30):
        """
        Set RC channel PWM value (for direct servo/throttle control)
        
        Args:
            channel: Channel number (1-8)
            pwm_value: PWM value (typically 1000-2000 microseconds)
            timeout: Timeout in seconds
        """
        if not self.connected or not self.mav:
            return False
        
        try:
            # Build RC override message
            rc_values = [65535] * 8  # 65535 = no override
            rc_values[channel - 1] = pwm_value
            
            self.mav.mav.rc_channels_override_send(
                self.mav.target_system,
                self.mav.target_component,
                *rc_values
            )
            return True
        except Exception as e:
            print(f"[ARDUPILOT] RC override failed: {str(e)}")
            return False
    
    def get_battery_status(self):
        """Get battery status"""
        if not self.connected or not self.mav:
            return None
        
        try:
            msg = self.mav.messages.get('BATTERY_STATUS')
            if msg:
                return {
                    "voltage": msg.voltages[0] / 1000.0,  # mV to V
                    "current": msg.current_battery / 100.0,  # cA to A
                    "percentage": msg.battery_remaining
                }
        except Exception as e:
            print(f"[ARDUPILOT] Failed to get battery: {str(e)}")
        return None


# Global instance
_autopilot = None


def init_autopilot(connection_string="udp:127.0.0.1:14550"):
    """Initialize global autopilot interface"""
    global _autopilot
    _autopilot = ArduPilotInterface(connection_string)
    if _autopilot.connect():
        return _autopilot
    return _autopilot  # Return even if not connected (may connect later)


def get_autopilot():
    """Get global autopilot interface"""
    global _autopilot
    return _autopilot
