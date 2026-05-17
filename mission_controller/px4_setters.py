"""
PX4 control / setter interface using MAVROS and ROS 2

Responsibilities:
- Call MAVROS services (arming, mode changes, takeoff, landing)
- Publish setpoint commands (position, velocity)
- Provide write/control APIs for the drone

This file does NOT own telemetry subscriptions.
It assumes the main interface object already created:
- service clients
- publishers
- cached state like current_position and current_state
"""

import time
import rclpy
import threading
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamGet, ParamPull, ParamSet
from mavros_msgs.msg import ParamValue


FT_TO_M = 0.3048
DEFAULT_MAX_ALT_FT = 400  # FAA Part 107 ceiling


class PX4Setters:
    def __init__(self, **kwargs):
        """Initialize threads and state, then pass control to parent class"""
        self._stream_thread = None
        self._stream_running = False
        self._stream_lock = threading.Lock()
        self._last_user_publish_time = 0  # Track when user last published a command
        self._last_command_message = None  # Track last published message (PoseStamped or TwistStamped)
        self._last_command_publisher = None  # Track which publisher to use for heartbeat

        # Altitude limit (software-side clamp). None = no limit until set.
        self._max_alt_m = None
        super().__init__(**kwargs)

    def set_altitude_limit_ft(self, feet=DEFAULT_MAX_ALT_FT, timeout=10):
        if not self.connected:
            print("[PX4] Not connected, cannot set altitude limit")
            return False

        meters = float(feet) * FT_TO_M
        self._max_alt_m = meters

        if not self.param_set_client.wait_for_service(timeout_sec=5):
            print("[PX4] /param/set service unavailable; software clamp set, fence NOT set")
            return False

        # ArduPilot fence params:
        # FENCE_ALT_MAX (m), FENCE_TYPE bit0 = max alt, FENCE_ENABLE = 1
        params = [
            ("FENCE_ALT_MAX", 0, meters),
            ("FENCE_TYPE", 1, 0.0),     # bit0 = altitude
            ("FENCE_ENABLE", 1, 0.0),
        ]

        for name, integer, real in params:
            req = ParamSet.Request()
            req.param_id = name
            req.value = ParamValue(integer=int(integer), real=float(real))
            future = self.param_set_client.call_async(req)
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not (future.done() and future.result() and future.result().success):
                print(f"[PX4] Failed to set {name}")
                return False
            print(f"[PX4] {name} set ({integer if integer else real})")

        print(f"[PX4] Altitude limit: {feet} ft ({meters:.2f} m)")
        return True

    def get_param(self, name, timeout=5, quiet=False):
        """Read a PX4/MAVROS parameter by name. Returns ParamValue or None."""
        if not self.connected:
            if not quiet:
                print(f"[PX4] Not connected, cannot get {name}")
            return None

        if not self.param_get_client.wait_for_service(timeout_sec=5):
            if not quiet:
                print("[PX4] /param/get service unavailable")
            return None

        req = ParamGet.Request()
        req.param_id = str(name)

        try:
            future = self.param_get_client.call_async(req)
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

            if future.done() and future.result() and future.result().success:
                return future.result().value

            if not quiet:
                print(f"[PX4] Failed to get {name}")
            return None
        except Exception as e:
            if not quiet:
                print(f"[PX4] Failed to get {name}: {str(e)}")
            return None

    def pull_params(self, timeout=30, force=True):
        """Ask MAVROS to pull/sync the full PX4 parameter table."""
        if not self.connected:
            print("[PX4] Not connected, cannot pull params")
            return False

        if not self.param_pull_client.wait_for_service(timeout_sec=5):
            print("[PX4] /param/pull service unavailable")
            return False

        req = ParamPull.Request()
        req.force_pull = bool(force)

        try:
            print("[PX4] Pulling MAVROS parameter cache...")
            future = self.param_pull_client.call_async(req)
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

            if not future.done():
                print(f"[PX4] Param pull timed out after {timeout}s")
                return False

            result = future.result()
            if result and result.success:
                received = getattr(result, "param_received", "unknown")
                print(f"[PX4] ✓ MAVROS param pull complete ({received} params)")
                return True

            print("[PX4] MAVROS param pull failed")
            return False
        except Exception as e:
            print(f"[PX4] Param pull failed: {str(e)}")
            return False

    def wait_for_params(self, names, timeout=30, poll_interval=1.0):
        """Block until MAVROS can read all requested PX4 parameters."""
        pending = {str(name) for name in names}
        start = time.time()

        self.pull_params(timeout=min(30, timeout), force=True)

        print(f"[PX4] Waiting for MAVROS param sync: {', '.join(sorted(pending))}")
        while pending and (time.time() - start) < timeout:
            for name in list(pending):
                if self.get_param(name, timeout=2, quiet=True) is not None:
                    pending.remove(name)

            if not pending:
                print("[PX4] ✓ MAVROS params available")
                return True

            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(poll_interval)

        print(f"[PX4] Timed out waiting for params: {', '.join(sorted(pending))}")
        return False

    def set_param(self, name, value, timeout=10, integer=False):
        """Set a PX4/MAVROS parameter by name."""
        if not self.connected:
            print(f"[PX4] Not connected, cannot set {name}")
            return False

        if not self.param_set_client.wait_for_service(timeout_sec=5):
            print("[PX4] /param/set service unavailable")
            return False

        req = ParamSet.Request()
        req.param_id = str(name)
        req.value = ParamValue(
            integer=int(value) if integer else 0,
            real=0.0 if integer else float(value),
        )

        try:
            future = self.param_set_client.call_async(req)
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

            if future.done() and future.result() and future.result().success:
                print(f"[PX4] {name} set to {value}")
                return True

            print(f"[PX4] Failed to set {name}")
            return False
        except Exception as e:
            print(f"[PX4] Failed to set {name}: {str(e)}")
            return False

    def _clamp_alt(self, z):
        """Clamp a requested z (meters) against the configured limit. Warns on clamp."""
        if self._max_alt_m is None or z is None:
            return z
        if z > self._max_alt_m:
            print(f"[PX4][WARN] Setpoint z={z:.2f}m exceeds limit {self._max_alt_m:.2f}m; clamping")
            return self._max_alt_m
        return z

    def arm_vehicle(self, timeout=20):
        #Arm the vehicle (allow motors to spin)
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot arm")
            return False

        if self.is_armed():
            print("[PX4] Vehicle already armed")
            return True

        print("[PX4] Arming vehicle...")
        try:
            req = CommandBool.Request() #this just creates a service message that has the arm state
            req.value = True #we edit the message so we want it to be true (to arm)

            future = self.arming_client.call_async(req) #we send the request. 
            #arming_client comes from the getters, where the services are called

            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                #we spin until we get a response
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            #future.done() checks if smt received, .result() checks if something is inside, .success is state
            if future.done() and future.result() and future.result().success:
                print("[PX4] Vehicle armed successfully")
                return True
            else:
                print("[PX4] Arming failed")
                return False
        except Exception as e:
            print(f"[PX4] Arm failed: {str(e)}")
            return False

    def disarm_vehicle(self, timeout=20):
        """
        Disarm the vehicle (prevent motors from spinning)
        we will probably never use this
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot disarm")
            return False

        if not self.is_armed():
            print("[PX4] Vehicle already disarmed")
            return True

        print("[PX4] Disarming vehicle...")
        try:
            req = CommandBool.Request() #create req message
            req.value = False #set the content of the req to false

            future = self.arming_client.call_async(req)

            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            if future.done() and future.result() and future.result().success:
                print("[PX4] Vehicle disarmed successfully")
                return True
            else:
                print("[PX4] Disarming failed")
                return False
        except Exception as e:
            print(f"[PX4] Disarm failed: {str(e)}")
            return False

    def change_mode(self, mode_name, timeout=30):
        """
        Change vehicle flight mode
        mode_name: Mode name (e.g., "GUIDED", "AUTO", "LAND", "RTL", "OFFBOARD")
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot change mode")
            return False

        # Check if service is available
        if not self.set_mode_client.wait_for_service(timeout_sec=5):
            print("[PX4] Service /set_mode NOT available after 5 seconds")
            return False

        print(f"[PX4] Changing mode to {mode_name}...")
        try:
            # this message type is just to set mode
            req = SetMode.Request()
            req.custom_mode = mode_name #set the specific mode we want in req

            future = self.set_mode_client.call_async(req)

            start = time.time()
            elapsed = 0
            while not future.done() and elapsed < timeout:
                # CRITICAL: Keep publishing setpoints during mode change
                # PX4 OFFBOARD mode requires continuous setpoint stream (>2Hz)
                # If we stop publishing, PX4 may reject the mode change
                if mode_name == "OFFBOARD":
                    self.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
                
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
                elapsed = time.time() - start
                
                if int(elapsed) % 5 == 0 and elapsed < int(elapsed) + 0.1:
                    print(f"[PX4] Waiting for mode change response... ({int(elapsed)}s)")

            if not future.done():
                print(f"[PX4] Mode change request TIMED OUT after {timeout}s")
                return False
            
            result = future.result()
            if result and result.mode_sent:
                print(f"[PX4] Mode changed to {mode_name}")
                return True
            else:
                print(f"[PX4] Mode change REJECTED by PX4 (mode_sent=False)")
                if result:
                    print(f"[PX4] Response: {result}")
                return False
        except Exception as e:
            print(f"[PX4] Mode change EXCEPTION: {str(e)}")
            import traceback
            traceback.print_exc()
            return False

    def takeoff(self, altitude, timeout=60):
        """
        Perform takeoff to specified altitude using position setpoint in OFFBOARD mode.
        
        Sends a position setpoint with the target altitude. The heartbeat will continuously
        republish this position, maintaining the climb until the target altitude is reached.
        Locks in the current yaw heading to prevent unwanted rotation.
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot takeoff")
            return False

        altitude = self._clamp_alt(float(altitude))
        
        # Get current position
        if not self.current_position:
            print("[PX4] Cannot takeoff: current position unavailable")
            return False
        
        current = self.current_position.pose.position
        current_orientation = self.current_position.pose.orientation
        target_alt = current.z + altitude
        
        # Extract yaw from current orientation quaternion
        # For a quaternion (x, y, z, w), yaw = atan2(2*(w*z + x*y), 1 - 2*(y*z + z*z))
        # Simplified for roll=0, pitch=0: yaw = atan2(2*w*z, 1 - 2*z*z)
    
        qx = current_orientation.x
        qy = current_orientation.y
        qz = current_orientation.z
        qw = current_orientation.w
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        print(f"[PX4] Taking off to {target_alt:.2f}m (current: {current.z:.2f}m, climbing {altitude:.2f}m)...")
        print(f"[PX4] Locking yaw heading: {math.degrees(yaw):.1f}°")
        
        try:
            # Arm if not already armed
            if not self.is_armed():
                if not self.arm_vehicle():
                    return False
            
            # Send position setpoint with current yaw locked (heartbeat will maintain it)
            if not self.send_position_setpoint(current.x, current.y, target_alt, yaw=yaw, yaw_from_direction=True):
                print("[PX4] Failed to send takeoff position setpoint")
                return False
            
            # Wait until target altitude is reached
            start = time.time()
            while (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.current_position:
                    current_alt = self.current_position.pose.position.z
                    if current_alt >= (target_alt * 0.95):  # 95% of target
                        print(f"[PX4] Takeoff complete, reached {current_alt:.2f}m")
                        return True
                
                time.sleep(0.5)
            
            print("[PX4] Takeoff timeout")
            return False
        except Exception as e:
            print(f"[PX4] Takeoff failed: {str(e)}")
            return False

    def land(self, timeout=60):
        """
        Land using PX4's built-in landing behavior via MAVROS.

        This sends MAV_CMD_NAV_LAND through /cmd/land instead of commanding an
        OFFBOARD position setpoint to local z=0. PX4 then owns descent,
        touchdown detection, and landing-specific limits.
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot land")
            return False

        if not self.land_client.wait_for_service(timeout_sec=5):
            print("[PX4] /cmd/land service unavailable")
            return False

        print("[PX4] Requesting PX4 landing mode...")
        try:
            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 0.0

            future = self.land_client.call_async(req)
            start = time.time()
            while not future.done() and (time.time() - start) < 5:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            if future.done():
                result = future.result()
                if not (result and result.success):
                    print("[PX4] Landing command rejected")
                    return False
                print("[PX4] Landing command accepted")
            else:
                print("[PX4] Landing command response timed out; monitoring landing state")

            start = time.time()
            while (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.is_landed():
                    print("[PX4] Landing complete, vehicle reports landed")
                    return True

                if self.current_position:
                    current_alt = self.current_position.pose.position.z
                    if current_alt < 0.1:
                        print(f"[PX4] Landing complete, reached {current_alt:.2f}m")
                        return True

                time.sleep(0.5)

            print("[PX4] Landing timeout")
            return False
        except Exception as e:
            print(f"[PX4] Landing failed: {str(e)}")
            return False

    # =========================================================
    # Publisher-based control APIs
    # These do NOT use services.
    # They publish command messages continuously or on demand.
    #OFFBOARD is stream based, not req/response, so we need these publishes
    # =========================================================
    def send_position_setpoint(self, x, y, z, yaw=None, yaw_from_direction=False):
        """
        Publish a local position setpoint with optional yaw heading.

        NOTE:
        This is a topic publish , not a service call. (messages vs services)
        In other words, the following functions below just send a command,
        but do not expect a response.
        
        When a background heartbeat is running, this setpoint is stored and
        republished by the heartbeat thread at its frequency. This ensures the
        position target is maintained without being overwritten by other messages.
        
        Args:
            x, y, z: Position in local NED frame (meters)
            yaw: Heading in radians (optional). If None and yaw_from_direction=False, doesn't set orientation.
            yaw_from_direction: If True, automatically calculate yaw from current position towards target (x, y).
                               This makes the drone face the direction it's moving. Overrides explicit yaw parameter.
        """
        try:
            # Record that user just published (heartbeat will skip if recent)
            self._last_user_publish_time = time.time()
            
            x_clamped = float(x)
            y_clamped = float(y)
            z_clamped = self._clamp_alt(float(z))
            
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = x_clamped
            msg.pose.position.y = y_clamped
            msg.pose.position.z = z_clamped
            
            # Calculate yaw from direction if requested
            calculated_yaw = None
            if yaw_from_direction and self.current_position:
                # Get current position
                current = self.current_position.pose.position
                # Calculate vector from current to target
                dx = x_clamped - current.x
                dy = y_clamped - current.y
                # Calculate yaw from direction (atan2 gives angle from +x axis)
                calculated_yaw = math.atan2(dy, dx)
            
            # Determine which yaw to use: calculated > explicit > none
            final_yaw = calculated_yaw if calculated_yaw is not None else yaw
            
            # Set orientation (yaw) if provided
            if final_yaw is not None:
                # Convert yaw to quaternion (roll=0, pitch=0, yaw=yaw)
                # Using euler to quaternion conversion: q = [qx, qy, qz, qw]
                yaw_f = float(final_yaw)
                half_yaw = yaw_f / 2.0
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = math.sin(half_yaw)
                msg.pose.orientation.w = math.cos(half_yaw)
            else:
                # Default orientation (no rotation)
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0
            
            # Store message and publisher for heartbeat to republish
            self._last_command_message = msg
            self._last_command_publisher = self.setpoint_pub
            
            self.setpoint_pub.publish(msg)
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to publish position setpoint: {str(e)}")
            return False

    def send_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """
        Publish a velocity setpoint immediately.

        When background heartbeat is running, this publishes your desired velocity.
        The heartbeat will then republish this velocity command at its frequency,
        maintaining your movement command without interference.

        IMPORTANT:
        - Call this as often as you need in your mission logic
        - The heartbeat automatically repeats your last command
        - For continuous movement, keep calling this at your desired rate
        """
        try:
            vx_f = float(vx)
            vy_f = float(vy)
            vz_f = float(vz)
            yaw_f = float(yaw_rate)
            
            # Record that user just published (heartbeat will skip if recent)
            self._last_user_publish_time = time.time()
            
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = vx_f
            msg.twist.linear.y = vy_f
            msg.twist.linear.z = vz_f
            msg.twist.angular.z = yaw_f
            
            # Store message and publisher for heartbeat to republish
            self._last_command_message = msg
            self._last_command_publisher = self.velocity_setpoint_pub
            
            self.velocity_setpoint_pub.publish(msg)
            
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to set velocity setpoint: {str(e)}")
            return False
        

    def send_position_setpoint_gps(self, current_lat, current_lon, current_alt, target_lat, target_lon, target_alt, yaw_rate=0.0, yaw_from_direction=False):
        """
        Convert GPS coordinates to local NED and send as position setpoint.
        
        Converts absolute GPS target coordinates to local NED position relative to HOME.
        The position setpoint in the map frame is relative to the home position where
        the drone armed, not the current position.
        
        Args:
            current_lat, current_lon, current_alt: Current GPS location (used only for yaw direction)
            target_lat, target_lon, target_alt: Target GPS location
            yaw_rate: Deprecated parameter (kept for compatibility)
            yaw_from_direction: If True, automatically face towards the target direction
        
        Returns:
            Tuple of (north, east, down, distance) in meters
        """
        # Get home position (origin of map frame)
        home = self.get_home_location()
        if not home:
            print("[PX4] Cannot send GPS setpoint - home position not available yet")
            return None
        
        home_lat = home["latitude"]
        home_lon = home["longitude"]
        home_alt = home["altitude"]
        
        # Calculate displacement from HOME to TARGET (not from current to target)
        # The map frame is centered at home, so we need home-relative coordinates
        latitude1_rad = math.radians(home_lat)  # Use HOME latitude for cosine calculation
        
        dlat = target_lat - home_lat  # Displacement from home to target
        dlong = target_lon - home_lon
        dalt = target_alt - home_alt

        north = dlat * 111320
        east = dlong * 111320 * math.cos(latitude1_rad)
        down = -dalt #negative down = up

        distance = math.sqrt(north**2 + east**2 + down**2)
        self.send_position_setpoint(north, east, down, yaw_from_direction=yaw_from_direction)
        
        return north, east, down, distance

    def hold_current_position(self):
        """Publish a position setpoint equal to the current local pose"""
        if not self.current_position:
            print("[PX4][WARN] Cannot hold current position because local pose is unavailable")
            return False

        try:
            pos = self.current_position.pose.position
            return self.send_position_setpoint(pos.x, pos.y, pos.z)
        except Exception as e:
            print(f"[PX4][ERROR] Failed to hold current position: {str(e)}")
            return False

    def start_offboard(self, warmup_count=20, warmup_dt=0.05):
        """
        Warm up setpoint stream and switch to OFFBOARD.

        PX4 usually expects setpoints to already be flowing before
        OFFBOARD is accepted.
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot start OFFBOARD")
            return False

        print("[PX4] Warming up OFFBOARD setpoints...")
        for _ in range(warmup_count):
            self.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(warmup_dt)

        return self.change_mode("OFFBOARD")

    def maintain_offboard_stream(self, get_velocity=None, duration=None, rate_hz=50):
        """
        Maintain OFFBOARD mode by continuously publishing velocity setpoints.

        CRITICAL: Once in OFFBOARD mode, you MUST call this (or another continuous
        publishing function) to keep setpoints flowing at >2Hz. If publishing stops,
        PX4 will timeout OFFBOARD and switch to failsafe mode.

        Args:
            get_velocity: Callable that returns (vx, vy, vz, yaw_rate) tuple.
                         If None, publishes zero velocity (hover/stay in place).
                         Example: lambda: (1.0, 0.0, 0.0, 0.0) for forward motion
            duration: Duration to maintain stream in seconds. If None, runs until
                     interrupted (KeyboardInterrupt) or exception.
            rate_hz: Publishing frequency in Hz (default 50 = 20ms interval)

        Returns:
            True if completed normally, False if error occurred

        Usage Examples:
            # Hover in place for 5 seconds
            px4.maintain_offboard_stream(duration=5.0)

            # Fly forward at 1 m/s until interrupted
            px4.maintain_offboard_stream(get_velocity=lambda: (1.0, 0.0, 0.0, 0.0))

            # Fly forward for 10 seconds
            px4.maintain_offboard_stream(
                get_velocity=lambda: (1.0, 0.0, 0.0, 0.0),
                duration=10.0
            )
        """
        if not self.connected:
            print("[PX4] Not connected, cannot maintain OFFBOARD stream")
            return False

        if get_velocity is None:
            get_velocity = lambda: (0.0, 0.0, 0.0, 0.0)  # Default: hover

        dt = 1.0 / rate_hz
        start = time.time()

        try:
            while True:
                # Check duration limit
                if duration and (time.time() - start) > duration:
                    return True

                # Get velocity from user function
                vx, vy, vz, yaw_rate = get_velocity()
                
                # Publish setpoint
                self.send_velocity_setpoint(vx, vy, vz, yaw_rate)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(dt)

        except KeyboardInterrupt:
            print("[PX4] Stream interrupted by user")
            return True
        except Exception as e:
            print(f"[PX4] Error maintaining OFFBOARD stream: {str(e)}")
            return False

    def start_offboard_stream_background(self, rate_hz=10):
        """
        Start a background thread that continuously publishes a heartbeat message.

        CRITICAL: OFFBOARD mode requires continuous message publishing (>2Hz).
        This method starts a lightweight heartbeat thread in the background,
        allowing setpoint publishing to be independent and only occur on-demand.

        Args:
            rate_hz: Heartbeat frequency in Hz (default 10 = 100ms interval)

        Returns:
            True if thread started successfully, False if already running

        Usage:
            # Start background thread
            px4.start_offboard_stream_background()

            # Now you can just switch to OFFBOARD mode
            px4.start_offboard()

            # Do whatever you want - thread keeps heartbeat alive in background
            time.sleep(5)
            px4.send_velocity_setpoint(1.0, 0.0, 0.0)
            px4.send_velocity_setpoint(0.0, 0.0, 0.0)

            # Stop the thread when done
            px4.stop_offboard_stream_background()
        """
        with self._stream_lock:
            if self._stream_running:
                print("[PX4] Heartbeat stream already running")
                return False

            self._stream_running = True
            self._stream_thread = threading.Thread(
                target=self._heartbeat_worker,
                args=(rate_hz,),
                daemon=False
            )
            self._stream_thread.start()
            print("[PX4] ✓ Background heartbeat stream started")
            return True

    def stop_offboard_stream_background(self, timeout=5):
        """
        Stop the background heartbeat publishing thread.

        Args:
            timeout: Timeout in seconds to wait for thread to stop

        Returns:
            True if stopped successfully, False if error or timeout
        """
        with self._stream_lock:
            if not self._stream_running:
                print("[PX4] Heartbeat stream not running")
                return True

            self._stream_running = False

        if self._stream_thread:
            self._stream_thread.join(timeout=timeout)
            if self._stream_thread.is_alive():
                print("[PX4] [WARN] Background thread did not stop within timeout")
                return False

        self._stream_thread = None
        print("[PX4] ✓ Background heartbeat stream stopped")
        return True

    def wait_for_arm_with_heartbeat(self, timeout=60, heartbeat_rate=10):
        """
        Wait for drone to be armed while maintaining OFFBOARD mode via manual heartbeat.
        
        This is useful when you need to wait for manual arming (RC or button) without
        using a background thread. It publishes heartbeat setpoints to keep OFFBOARD
        mode alive while blocking on the arm check.
        
        Args:
            timeout: Maximum time to wait in seconds (default 60)
            heartbeat_rate: Heartbeat publishing rate in Hz (default 10)
        
        Returns:
            True if armed within timeout, False if timeout occurred
        
        Usage:
            px4.start_offboard()  # Switch to OFFBOARD mode
            if px4.wait_for_arm_with_heartbeat(timeout=60):
                # Drone is armed, proceed with commands
                px4.send_velocity_setpoint(0, 0, 0.5)
            else:
                # Timeout - not armed
                return False
        """
        heartbeat_interval = 1.0 / heartbeat_rate
        start_time = time.time()
        heartbeat_count = 0
        last_log = 0
        
        print(f"[PX4] Waiting for arm (timeout={timeout}s, heartbeat={heartbeat_rate}Hz)...")
        print(f"[PX4] DEBUG: current_state at start = {self.current_state}")
        
        while (time.time() - start_time) < timeout:
            # Publish heartbeat to maintain OFFBOARD mode
            self.send_velocity_setpoint(0.0, 0.0, 0.0, 0.0)
            heartbeat_count += 1

            # Spin multiple times to ensure messages are processed
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.02)
            
            # Check if armed
            is_armed_now = self.is_armed()
            if is_armed_now:
                elapsed = time.time() - start_time
                print(f"[PX4] ✓ Vehicle armed in {elapsed:.1f}s ({heartbeat_count} heartbeats)")
                return True
            
            # Log progress every second
            current_time = time.time() - start_time
            if int(current_time) != last_log:
                last_log = int(current_time)
                print(f"[PX4] DEBUG: Waiting {int(timeout - current_time)}s... current_state={self.current_state}, is_armed={is_armed_now}")
            
            time.sleep(heartbeat_interval)
        
        print(f"[PX4] ✗ Arm timeout after {timeout}s ({heartbeat_count} heartbeats)")
        print(f"[PX4] DEBUG: Final current_state = {self.current_state}")
        return False

    def _heartbeat_worker(self, rate_hz):
        """
        Worker thread function that publishes a lightweight heartbeat message.

        This runs in the background and keeps OFFBOARD mode alive by repeatedly
        publishing the last command the user sent. The heartbeat behavior is simple:
        - Whatever the user sent last (position or velocity) gets republished
        - If user sent a position setpoint, heartbeat republishes that position
        - If user sent a velocity command, heartbeat republishes that velocity
        - If no command sent yet, heartbeat publishes zero velocity (hover)
        
        The heartbeat SKIPS publishing if the user has published a command recently
        (within the last heartbeat interval). This prevents rapid updates from being
        disrupted by the heartbeat republishing while the user is actively sending.
        
        The heartbeat publishes at a low frequency (default 10Hz) to keep PX4
        aware the system is alive.
        """
        dt = 1.0 / rate_hz
        publish_count = 0
        log_interval = int(5 * rate_hz)  # Log every 5 seconds at specified rate

        try:
            while self._stream_running:
                # Check if user published recently (within last heartbeat interval)
                time_since_user_publish = time.time() - self._last_user_publish_time
                
                # Only publish heartbeat if user hasn't published recently
                # This prevents heartbeat from interfering with rapid user commands
                if time_since_user_publish > dt:
                    # If there's a stored command, republish it
                    if self._last_command_message is not None and self._last_command_publisher is not None:
                        # Update timestamp to current time
                        self._last_command_message.header.stamp = self.get_clock().now().to_msg()
                        self._last_command_publisher.publish(self._last_command_message)
                    else:
                        # No command yet; publish zero velocity (hover)
                        msg = TwistStamped()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.twist.linear.x = 0.0
                        msg.twist.linear.y = 0.0
                        msg.twist.linear.z = 0.0
                        msg.twist.angular.z = 0.0
                        self.velocity_setpoint_pub.publish(msg)
                    
                    publish_count += 1
                    
                    # Log progress every 5 seconds
                    if publish_count % log_interval == 0:
                        print(f"[PX4] Heartbeat alive - published {publish_count} messages")
                
                time.sleep(dt)
        except Exception as e:
            print(f"[PX4] Heartbeat worker error: {str(e)}")
            with self._stream_lock:
                self._stream_running = False
