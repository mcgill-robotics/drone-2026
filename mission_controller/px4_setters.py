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
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


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

        print(f"[PX4] Changing mode to {mode_name}...")
        try:
            # this message type is just to set mode
            req = SetMode.Request()
            req.custom_mode = mode_name #set the specific mode we want in req

            future = self.set_mode_client.call_async(req)

            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)

            if future.done() and future.result() and future.result().mode_sent:
                print(f"[PX4] Mode changed to {mode_name}")
                return True
            else:
                print("[PX4] Mode change failed")
                return False
        except Exception as e:
            print(f"[PX4] Mode change failed: {str(e)}")
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

            req = CommandTOL.Request()
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
            msg.pose.position.z = float(z)
            self.setpoint_pub.publish(msg)
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to publish position setpoint: {str(e)}")
            return False

    def send_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """
        Publish a local velocity setpoint.

        IMPORTANT:
        PX4 generally requires continuous setpoint publishing (>2 Hz)
        when operating in OFFBOARD mode.
        """
        try:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(vx)
            msg.twist.linear.y = float(vy)
            msg.twist.linear.z = float(vz)
            msg.twist.angular.z = float(yaw_rate)
            self.velocity_setpoint_pub.publish(msg)
            return True
        except Exception as e:
            print(f"[PX4][ERROR] Failed to publish velocity setpoint: {str(e)}")
            return False

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