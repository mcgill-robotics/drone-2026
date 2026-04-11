"""
ArduPilot interface wrapper using MAVROS and ROS 2
This module provides connection to ArduPilot via MAVROS middleware
MAVROS must be running to use this interface
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, BatteryStatus
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome
from sensor_msgs.msg import BatteryState


class ArduPilotInterface(Node):
    """
    Wrapper for communicating with ArduPilot via MAVROS
    Requires MAVROS to be running and connected to the flight controller
    """
    
    def __init__(self, node_name="ardupilot_interface", namespace="mavros"):
        """
        Initialize MAVROS interface
        
        Args:
            node_name: ROS 2 node name
            namespace: MAVROS namespace (default "mavros")
        """
        super().__init__(node_name)
        
        self.namespace = namespace
        self.connected = False
        self.current_state = None
        self.current_position = None
        self.battery_status = None
        self.timeout = 30
        
        # Create subscriptions
        # create ROS2 subscriptions
        self.state_sub = self.create_subscription(
            State,
            f"/{namespace}/state",
            #this callback is a function that is continously called from the ros node
            self._state_callback,
            10
        )
        
        self.position_sub = self.create_subscription(
            PoseStamped,
            f"/{namespace}/local_position/pose",
            self._position_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            f"/{namespace}/battery",
            self._battery_callback,
            10
        )
        
        # Create service clients
        self.arming_client = self.create_client(CommandBool, f"/{namespace}/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, f"/{namespace}/cmd/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, f"/{namespace}/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, f"/{namespace}/cmd/land")
        self.home_client = self.create_client(CommandHome, f"/{namespace}/cmd/set_home")
        
        # Create publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            f"/{namespace}/setpoint_position/local",
            10
        )
        
        print(f"[ARDUPILOT] Initialized MAVROS interface (namespace: {namespace})")
    
    def _state_callback(self, msg):
        """Update current vehicle state"""
        self.current_state = msg
        self.connected = msg.connected
    
    def _position_callback(self, msg):
        """Update current vehicle position"""
        self.current_position = msg
    
    def _battery_callback(self, msg):
        """Update battery status"""
        self.battery_status = msg
    
    def _wait_for_connection(self, timeout=None):
        """Wait for MAVROS connection to be established"""
        if timeout is None:
            timeout = self.timeout
        
        start = time.time()
        # continuosly runs rclpy spins which calls state_call and if "connected" changes, its conected
        while not self.connected and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        if self.connected:
            print("[ARDUPILOT] Connected to MAVROS")
            return True
        else:
            print("[ARDUPILOT] Failed to connect to MAVROS")
            return False
    
    def connect(self):
        """
        Connect to MAVROS (wait for connection)
        MAVROS must be running for this to work
        """
        print("[ARDUPILOT] Connecting to MAVROS...")
        # this function waits for a heartbeat as well
        return self._wait_for_connection()
    
    def disconnect(self):
        """Disconnect from MAVROS"""
        print("[ARDUPILOT] Disconnected")
        try:
            self.destroy_node()
            print("[ARDUPILOT] Node destroyed")
        except Exception as e:
            print(f"[ARDUPILOT] Disconnect failed: {str(e)}")
    
    def is_armed(self):
        """Check if vehicle is armed"""
        if not self.connected or not self.current_state:
            return False
        return self.current_state.armed
    
    def arm_vehicle(self, timeout=20):
        """
        Arm the vehicle (allow motors to spin)
        
        Args:
            timeout: Timeout in seconds
        """
        if not self.connected:
            print("[ARDUPILOT] Not connected to MAVROS, cannot arm")
            return False
        
        if self.is_armed():
            print("[ARDUPILOT] Vehicle already armed")
            return True
        
        print("[ARDUPILOT] Arming vehicle...")
        try:
            req = CommandBool.Request()
            req.value = True
            
            future = self.arming_client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            
            if future.done() and future.result().success:
                print("[ARDUPILOT] Vehicle armed successfully")
                return True
            else:
                print("[ARDUPILOT] Arming failed")
                return False
        except Exception as e:
            print(f"[ARDUPILOT] Arm failed: {str(e)}")
            return False
    
    def disarm_vehicle(self, timeout=20):
        """Disarm the vehicle (prevent motors from spinning)"""
        if not self.connected:
            print("[ARDUPILOT] Not connected to MAVROS, cannot disarm")
            return False
        
        if not self.is_armed():
            print("[ARDUPILOT] Vehicle already disarmed")
            return True
        
        print("[ARDUPILOT] Disarming vehicle...")
        try:
            req = CommandBool.Request()
            req.value = False
            
            future = self.arming_client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            
            if future.done() and future.result().success:
                print("[ARDUPILOT] Vehicle disarmed successfully")
                return True
            else:
                print("[ARDUPILOT] Disarming failed")
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
        if not self.connected:
            print("[ARDUPILOT] Not connected to MAVROS, cannot change mode")
            return False
        
        print(f"[ARDUPILOT] Changing mode to {mode_name}...")
        try:
            req = SetMode.Request()
            req.custom_mode = mode_name
            
            future = self.set_mode_client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            
            if future.done() and future.result().mode_sent:
                print(f"[ARDUPILOT] Mode changed to {mode_name}")
                return True
            else:
                print(f"[ARDUPILOT] Mode change failed")
                return False
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
        if not self.connected:
            print("[ARDUPILOT] Not connected to MAVROS, cannot takeoff")
            return False
        
        print(f"[ARDUPILOT] Taking off to {altitude}m...")
        
        try:
            # Arm if not already armed
            if not self.is_armed():
                if not self.arm_vehicle():
                    return False
            
            # Switch to GUIDED mode for takeoff
            if not self.change_mode("GUIDED"):
                return False
            
            # Send takeoff command
            req = CommandTOL.Request()
            req.altitude = float(altitude)
            
            future = self.takeoff_client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.current_position:
                    current_alt = self.current_position.pose.position.z
                    if current_alt >= (altitude * 0.95):  # 95% of target
                        print(f"[ARDUPILOT] Takeoff complete, reached {current_alt:.1f}m")
                        return True
                
                time.sleep(0.5)
            
            print("[ARDUPILOT] Takeoff timeout")
            return False
        except Exception as e:
            print(f"[ARDUPILOT] Takeoff failed: {str(e)}")
            return False
    
    def land(self, timeout=60):
        """
        Perform landing sequence
        
        Args:
            timeout: Timeout in seconds
        """
        if not self.connected:
            print("[ARDUPILOT] Not connected to MAVROS, cannot land")
            return False
        
        print("[ARDUPILOT] Landing...")
        
        try:
            # Send land command
            req = CommandTOL.Request()
            req.altitude = 0  # Land at current location
            
            future = self.land_client.call_async(req)
            
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.current_position:
                    current_alt = self.current_position.pose.position.z
                    if current_alt < 0.1:  # Close to ground
                        print("[ARDUPILOT] Landing complete")
                        return True
                
                time.sleep(0.5)
            
            print("[ARDUPILOT] Landing timeout")
            return False
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
        if not self.connected:
            print("[ARDUPILOT] Not connected to MAVROS, cannot navigate")
            return False
        
        print(f"[ARDUPILOT] Navigating to ({lat:.6f}, {lon:.6f}, {alt}m)...")
        
        try:
            # Switch to GUIDED mode for navigation
            if not self.change_mode("GUIDED"):
                return False
            
            # Send setpoint position (local NED frame)
            # Note: This uses local position, not GPS
            # For GPS waypoints, use AUTO mode with mission waypoints
            setpoint = PoseStamped()
            setpoint.header.stamp = self.get_clock().now().to_msg()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = lat  # These should be local offsets
            setpoint.pose.position.y = lon
            setpoint.pose.position.z = alt
            
            self.setpoint_pub.publish(setpoint)
            print(f"[ARDUPILOT] Waypoint sent")
            return True
        except Exception as e:
            print(f"[ARDUPILOT] Navigation failed: {str(e)}")
            return False
    
    def get_location(self):
        """Get current vehicle location"""
        if not self.connected or not self.current_position:
            return None
        
        try:
            pos = self.current_position.pose.position
            return {
                "x": pos.x,
                "y": pos.y,
                "z": pos.z,
            }
        except Exception as e:
            print(f"[ARDUPILOT] Failed to get location: {str(e)}")
        return None
    
    def get_altitude(self):
        """Get current altitude"""
        loc = self.get_location()
        return loc["z"] if loc else 0
    
    def set_rc_channel(self, channel, pwm_value, timeout=30):
        """
        Set RC channel PWM value (for direct servo/throttle control)
        
        Args:
            channel: Channel number (1-8)
            pwm_value: PWM value (typically 1000-2000 microseconds)
            timeout: Timeout in seconds
        
        Note: MAVROS RC override topic may be used for this
        """
        print(f"[ARDUPILOT] RC override not yet implemented in MAVROS wrapper")
        return False
    
    def get_battery_status(self):
        """Get battery status"""
        if not self.connected or not self.battery_status:
            return None
        
        try:
            return {
                "voltage": self.battery_status.voltage,
                "current": self.battery_status.current,
                "percentage": self.battery_status.percentage
            }
        except Exception as e:
            print(f"[ARDUPILOT] Failed to get battery: {str(e)}")
        return None


# Global instance
_autopilot = None


def init_autopilot(node_name="ardupilot_interface", namespace="mavros"):
    """Initialize global autopilot interface"""
    global _autopilot
    
    # Initialize ROS 2 if not already done
    if not rclpy.ok():
        rclpy.init()
    
    _autopilot = ArduPilotInterface(node_name=node_name, namespace=namespace)
    if _autopilot.connect():
        return _autopilot
    return _autopilot  # Return even if not connected (may connect later)


def get_autopilot():
    """Get global autopilot interface"""
    global _autopilot
    return _autopilot
