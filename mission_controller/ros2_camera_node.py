#!/usr/bin/env python3
"""
ROS2 node for capturing images from the drone's RGB camera.
Includes a tkinter GUI with live camera feed.

Topics subscribed:
    /camera/image_raw  (sensor_msgs/Image)

Topics published:
    /drone/capture_trigger  (std_msgs/Bool)

Services:
    /drone/capture_image  (std_srvs/Trigger)
"""

import os
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage, ImageTk
import tkinter as tk
from datetime import datetime


SAVE_DIR = os.path.expanduser("~/drone_images")


# ──────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ──────────────────────────────────────────────────────────────────────────────

class DroneCamera(Node):
    def __init__(self):
        super().__init__("drone_camera_node")

        os.makedirs(SAVE_DIR, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_count = 0
        self._lock = threading.Lock()

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("auto_capture_interval", 0.0)

        camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        interval = self.get_parameter("auto_capture_interval").get_parameter_value().double_value

        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )
        self.trigger_sub = self.create_subscription(
            Bool, "/drone/capture_trigger", self.trigger_callback, 10
        )
        self.capture_srv = self.create_service(
            Trigger, "/drone/capture_image", self.capture_service_callback
        )

        self._auto_timer = None
        if interval > 0.0:
            self._auto_timer = self.create_timer(interval, self.save_frame)

        self.get_logger().info(f"Subscribed to: {camera_topic}")
        self.get_logger().info(f"Saving images to: {SAVE_DIR}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self.latest_frame = frame
                self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    def trigger_callback(self, msg: Bool):
        if msg.data:
            self.save_frame()

    def capture_service_callback(self, request, response):
        path = self.save_frame()
        response.success = path is not None
        response.message = f"Saved: {path}" if path else "No frame available"
        return response

    def get_frame(self):
        """Thread-safe frame access for the GUI."""
        with self._lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def save_frame(self) -> str | None:
        frame = self.get_frame()
        if frame is None:
            self.get_logger().warn("No frame to save yet")
            return None
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(SAVE_DIR, f"frame_{timestamp}.jpg")
        cv2.imwrite(path, frame)
        self.get_logger().info(f"Saved: {path}")
        return path

    def set_auto_capture(self, interval: float):
        if self._auto_timer:
            self._auto_timer.cancel()
            self._auto_timer = None
        if interval > 0.0:
            self._auto_timer = self.create_timer(interval, self.save_frame)

    def stop_auto_capture(self):
        if self._auto_timer:
            self._auto_timer.cancel()
            self._auto_timer = None


# ──────────────────────────────────────────────────────────────────────────────
# GUI
# ──────────────────────────────────────────────────────────────────────────────

class DroneCameraGUI:
    FEED_W, FEED_H = 800, 500
    REFRESH_MS = 33  # ~30 fps

    def __init__(self, root: tk.Tk, node: DroneCamera):
        self.root = root
        self.node = node
        self.auto_active = False

        root.title("Drone Camera Feed")
        root.configure(bg="#1a1a2e")
        root.resizable(False, False)

        self._build_ui()
        self._poll()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        PAD = 10
        BG = "#1a1a2e"
        PANEL = "#16213e"
        ACCENT = "#0f3460"
        GREEN = "#00d26a"
        RED = "#e94560"
        FG = "#e0e0e0"

        # ── Header ──
        header = tk.Frame(self.root, bg=ACCENT, pady=6)
        header.pack(fill="x")
        tk.Label(header, text="DRONE CAMERA", font=("Courier", 14, "bold"),
                 bg=ACCENT, fg=GREEN).pack()

        # ── Main area ──
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=PAD, pady=PAD)

        # Camera feed
        feed_frame = tk.Frame(main, bg=PANEL, bd=2, relief="sunken")
        feed_frame.pack(side="left")

        self.canvas = tk.Canvas(feed_frame,
                                width=self.FEED_W, height=self.FEED_H,
                                bg="#000", highlightthickness=0)
        self.canvas.pack()

        # Waiting label shown until first frame arrives
        self._waiting_text = self.canvas.create_text(
            self.FEED_W // 2, self.FEED_H // 2,
            text="Waiting for camera...",
            fill=GREEN, font=("Courier", 14)
        )

        # Sidebar controls
        sidebar = tk.Frame(main, bg=BG, padx=PAD)
        sidebar.pack(side="left", fill="y")

        def label(parent, text, **kw):
            tk.Label(parent, text=text, bg=BG, fg=FG,
                     font=("Courier", 9), **kw).pack(anchor="w", pady=(10, 2))

        def btn(parent, text, cmd, color=ACCENT):
            tk.Button(parent, text=text, command=cmd,
                      bg=color, fg=FG, activebackground=GREEN,
                      font=("Courier", 10, "bold"),
                      relief="flat", padx=10, pady=6,
                      width=18).pack(fill="x", pady=2)

        # Status
        label(sidebar, "STATUS")
        self.status_var = tk.StringVar(value="Connecting...")
        tk.Label(sidebar, textvariable=self.status_var,
                 bg=BG, fg=GREEN, font=("Courier", 10, "bold")).pack(anchor="w")

        self.frame_var = tk.StringVar(value="Frames: 0")
        tk.Label(sidebar, textvariable=self.frame_var,
                 bg=BG, fg=FG, font=("Courier", 9)).pack(anchor="w")

        # Capture
        label(sidebar, "CAPTURE")
        btn(sidebar, "[ S ] Save Frame", self._capture, color=ACCENT)

        # Auto-capture
        label(sidebar, "AUTO-CAPTURE INTERVAL (s)")
        self.interval_var = tk.DoubleVar(value=1.0)
        interval_spin = tk.Spinbox(
            sidebar, from_=0.5, to=10.0, increment=0.5,
            textvariable=self.interval_var, width=6,
            font=("Courier", 10), bg=PANEL, fg=FG,
            buttonbackground=ACCENT
        )
        interval_spin.pack(anchor="w", pady=2)

        self.auto_btn = tk.Button(
            sidebar, text="START AUTO",
            command=self._toggle_auto,
            bg=GREEN, fg="#000",
            activebackground=RED,
            font=("Courier", 10, "bold"),
            relief="flat", padx=10, pady=6, width=18
        )
        self.auto_btn.pack(fill="x", pady=2)

        # Save location
        label(sidebar, "SAVE DIRECTORY")
        tk.Label(sidebar, text=SAVE_DIR, bg=BG, fg=FG,
                 font=("Courier", 8), wraplength=170,
                 justify="left").pack(anchor="w")

        # Keybinds hint
        label(sidebar, "KEYBINDS")
        for hint in ["s — save frame", "q — quit"]:
            tk.Label(sidebar, text=hint, bg=BG, fg="#888",
                     font=("Courier", 9)).pack(anchor="w")

        # Quit
        tk.Frame(sidebar, bg=BG, height=20).pack()
        btn(sidebar, "[ Q ] Quit", self.root.destroy, color=RED)

        # ── Status bar ──
        self.statusbar = tk.Label(
            self.root, text="Ready", bd=1, relief="sunken",
            anchor="w", bg=PANEL, fg=FG, font=("Courier", 9)
        )
        self.statusbar.pack(fill="x", side="bottom")

        # Keybinds
        self.root.bind("<s>", lambda _: self._capture())
        self.root.bind("<q>", lambda _: self.root.destroy())

    # ── Poll / update ─────────────────────────────────────────────────────────

    def _poll(self):
        frame = self.node.get_frame()

        if frame is not None:
            if self._waiting_text is not None:
                self.canvas.delete(self._waiting_text)
                self._waiting_text = None

            # Resize to fit canvas
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img = PILImage.fromarray(rgb).resize(
                (self.FEED_W, self.FEED_H), PILImage.LANCZOS
            )
            self._tk_img = ImageTk.PhotoImage(pil_img)
            self.canvas.create_image(0, 0, anchor="nw", image=self._tk_img)

            count = self.node.frame_count
            self.status_var.set("LIVE")
            self.frame_var.set(f"Frames: {count}")

        self.root.after(self.REFRESH_MS, self._poll)

    # ── Actions ───────────────────────────────────────────────────────────────

    def _capture(self):
        path = self.node.save_frame()
        msg = f"Saved: {os.path.basename(path)}" if path else "No frame available"
        self.statusbar.config(text=msg)

    def _toggle_auto(self):
        if self.auto_active:
            self.node.stop_auto_capture()
            self.auto_btn.config(text="START AUTO", bg="#00d26a", fg="#000")
            self.statusbar.config(text="Auto-capture stopped")
            self.auto_active = False
        else:
            interval = self.interval_var.get()
            self.node.set_auto_capture(interval)
            self.auto_btn.config(text="STOP AUTO", bg="#e94560", fg="#fff")
            self.statusbar.config(text=f"Auto-capture every {interval}s")
            self.auto_active = True


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DroneCamera()

    # Spin ROS2 in a background thread so the GUI stays responsive
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    DroneCameraGUI(root, node)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
