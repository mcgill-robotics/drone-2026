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
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamSet
from mavros_msgs.msg import ParamValue


FT_TO_M = 0.3048
DEFAULT_MAX_ALT_FT = 400  # FAA Part 107 ceiling


class PX4Setters:
    """
    Mixin containing control / setter APIs.

    This class assumes the object already has:
    - connected
    - current_position
    - arming_client
    - set_mode_client
    - takeoff_client
    - land_client
    - setpoint_pub
    - velocity_setpoint_pub
    - get_clock()
    - is_armed()
    """

    def __init__(self, **kwargs):
        """Initialize threads and state, then pass control to parent class"""
        self._stream_thread = None
        self._stream_running = False
        self._stream_lock = threading.Lock()
        self._last_user_publish_time = 0  # Track when user last published a command

        # Altitude limit (software-side clamp). None = no limit until set.
        # Firmware fence is set independently via set_altitude_limit_ft().
        self._max_alt_m = None

        # Call parent class __init__ (PX4Getters -> Node)
        super().__init__(**kwargs)

    def set_altitude_limit_ft(self, feet=DEFAULT_MAX_ALT_FT, timeout=10):
        """
        Configure altitude limit in feet. Sets the ArduPilot fence on the FCU
        AND records the limit locally for software-side setpoint clamping.

        Call once after connect, before takeoff. ArduPilot params persist on
        the FCU, so re-calling per-command is unnecessary.
        """
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

    def _clamp_alt(self, z):
        """Clamp a requested z (meters) against the configured limit. Warns on clamp."""
        if self._max_alt_m is None or z is None:
            return z
        if z > self._max_alt_m:
            print(f"[PX4][WARN] Setpoint z={z:.2f}m exceeds limit {self._max_alt_m:.2f}m; clamping")
            return self._max_alt_m
        return z

    def arm_vehicle(self, timeout=20):
        """
        Arm the vehicle (allow motors to spin)

        Args:
            timeout: Timeout in seconds
        """
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
            #future.done() checks if smt received, .result() checks if something is in, .success is state
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
        """Disarm the vehicle (prevent motors from spinning)"""
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

        Args:
            mode_name: Mode name (e.g., "GUIDED", "AUTO", "LAND", "RTL", "OFFBOARD")
            timeout: Timeout in seconds
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot change mode")
            return False

        # Check if service is available
        print(f"[PX4] Checking if set_mode service is available...")
        if not self.set_mode_client.wait_for_service(timeout_sec=5):
            print("[PX4] Service /set_mode NOT available after 5 seconds")
            print("[PX4] Make sure MAVROS is running: ros2 launch mavros px4.launch fcu_url:=...")
            return False
        print("[PX4] ✓ Service is available")

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
                print(f"[PX4] ✓ Mode changed to {mode_name}")
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

    def wait_for_mode(self, mode_name, timeout=5):
        """Poll until current_state.mode equals mode_name, or timeout.

        `change_mode` only confirms the request was accepted (`mode_sent`),
        not that PX4 actually entered the mode. PX4 can reject a mode
        transition after accepting the request (e.g. OFFBOARD without
        enough setpoints flowing). Use this after `change_mode` to verify.
        """
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_state and self.current_state.mode == mode_name:
                return True
            time.sleep(0.1)
        actual = self.current_state.mode if self.current_state else "unknown"
        print(f"[PX4] wait_for_mode: wanted {mode_name}, got {actual} after {timeout}s")
        return False

    def takeoff(self, altitude, timeout=60):
        """
        Perform takeoff to specified altitude

        Args:
            altitude: Target altitude in meters
            timeout: Timeout in seconds
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot takeoff")
            return False

        altitude = self._clamp_alt(float(altitude))
        print(f"[PX4] Taking off to {altitude}m...")

        try:
            # Arm if not already armed
            if not self.is_armed():
                if not self.arm_vehicle():
                    return False

            # Some setups use GUIDED, some use OFFBOARD.
            # Keep this comment because it is useful during debugging.
            if not self.change_mode("GUIDED"):
                print("[PX4][WARN] GUIDED mode failed. If using PX4, you may need OFFBOARD instead.")
                return False

            req = CommandTOL.Request() # takeoff and land request
            req.altitude = float(altitude)

            future = self.takeoff_client.call_async(req)

            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                #here, we dont need to check if we received a message, because
                #the drone checks for altitude, so its actually already robust
                if self.current_position:
                    current_alt = self.current_position.pose.position.z
                    if current_alt >= (altitude * 0.95):  # 95% of target
                        print(f"[PX4] Takeoff complete, reached {current_alt:.1f}m")
                        return True

                time.sleep(0.5)

            print("[PX4] Takeoff timeout")
            return False
        except Exception as e:
            print(f"[PX4] Takeoff failed: {str(e)}")
            return False

    def land(self, timeout=60):
        """
        Perform landing sequence

        Args:
            timeout: Timeout in seconds
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot land")
            return False

        print("[PX4] Landing...")

        try:
            req = CommandTOL.Request()
            req.altitude = 0  # Land at current location

            future = self.land_client.call_async(req)

            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.current_position:
                    current_alt = self.current_position.pose.position.z
                    if current_alt < 0.1:  # Close to ground
                        print("[PX4] Landing complete")
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
    # =========================================================

    def goto_location(self, lat, lon, alt, timeout=60):
        """
        Navigate to a location using local setpoint publishing.

        Args:
            lat: Placeholder local x position
            lon: Placeholder local y position
            alt: Local z altitude in meters
            timeout: Timeout in seconds

        NOTE:
        This currently publishes a local position setpoint.
        It does NOT send a true GPS waypoint mission.
        """
        if not self.connected:
            print("[PX4] Not connected to MAVROS, cannot navigate")
            return False

        alt = self._clamp_alt(float(alt))
        print(f"[PX4] Navigating to ({lat:.6f}, {lon:.6f}, {alt}m)...")

        try:
            if not self.change_mode("GUIDED"):
                print("[PX4][WARN] GUIDED mode failed. If using PX4, you may need OFFBOARD instead.")
                return False

            setpoint = PoseStamped()
            setpoint.header.stamp = self.get_clock().now().to_msg() #set all the fields
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = lat
            setpoint.pose.position.y = lon
            setpoint.pose.position.z = alt

            self.setpoint_pub.publish(setpoint)
            print("[PX4] Waypoint sent")
            return True
        except Exception as e:
            print(f"[PX4] Navigation failed: {str(e)}")
            return False

    def send_position_setpoint(self, x, y, z):
        """
        Publish a local position setpoint.

        NOTE:
        This is a topic publish , not a service call. (messages vs services)
        In other words, the following functions below just send a command,
        but do not expect a response.
        """
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = self._clamp_alt(float(z))
            self.setpoint_pub.publish(msg)
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to publish position setpoint: {str(e)}")
            return False

    def send_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """
        Publish a velocity setpoint immediately.

        When background heartbeat is running, this publishes your desired velocity
        on-demand without interference from the heartbeat. The heartbeat only publishes
        if no user command has been sent recently.

        IMPORTANT:
        - Call this as often as you need - it won't be overwritten by the heartbeat
        - The heartbeat publishes zero velocity only when user isn't actively commanding
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
            self.velocity_setpoint_pub.publish(msg)
            
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to set velocity setpoint: {str(e)}")
            return False

    def send_velocity_setpoint_gps(self, lat, lon, alt, yaw_rate=0.0):
        """
        Update the desired velocity setpoint in GPS coordinates. This will make traveling algs easier
        we cannot run this in the second mission, as there isnt consistent access to the GPS
        """
    # =========================================================
    # High-level helpers built on top of setters
    # =========================================================

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

        This runs in the background and keeps OFFBOARD mode alive by publishing
        zero velocity setpoints. However, it SKIPS publishing if the user has
        published a command recently (within the last heartbeat interval).

        This prevents the heartbeat from overwriting user commands.
        
        The heartbeat publishes at a low frequency (default 10Hz) to keep PX4
        aware the system is alive, while allowing user commands to take effect
        without interference.
        """
        dt = 1.0 / rate_hz
        publish_count = 0
        log_interval = int(5 * rate_hz)  # Log every 5 seconds at specified rate

        try:
            while self._stream_running:
                # Check if user published recently (within last heartbeat interval)
                time_since_user_publish = time.time() - self._last_user_publish_time
                
                # Only publish heartbeat if user hasn't published recently
                # This prevents heartbeat from overwriting user commands
                if time_since_user_publish > dt:
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

    def set_rc_channel(self, channel, pwm_value, timeout=30):
        """
        Set RC channel PWM value (for direct servo/throttle control)

        Args:
            channel: Channel number (1-8)
            pwm_value: PWM value (typically 1000-2000 microseconds)
            timeout: Timeout in seconds

        Note: MAVROS RC override topic may be used for this
        """
        print("[PX4] RC override not yet implemented in MAVROS wrapper")
        return False