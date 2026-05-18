#!/usr/bin/env python3
"""
Mission 1 RTSP Camera Viewer

Displays live video feed from an RTSP camera attached to the drone via ethernet.
Can run independently in a separate terminal while mission is executing.

This connects to an RTSP stream and displays it in real-time using OpenCV.
Works for any phase of the mission (laps, boustrophedon, etc.).

Usage:
    # In one terminal, start the mission:
    python3 tests/mission1/test_mission1.py
    
    # In another terminal, view the camera:
    python3 tests/mission1/test_mission1_camera_viewer.py --rtsp-url rtsp://192.168.1.100:8554/live
    
    Or with defaults (assumes camera on local drone):
    python3 tests/mission1/test_mission1_camera_viewer.py
"""

import sys
import os
import cv2
import argparse
import time
from pathlib import Path

# Add parent paths
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(REPO_ROOT))


class RTSPCameraViewer:
    """Display RTSP camera stream in OpenCV window"""
    
    def __init__(self, rtsp_url, window_name="Mission 1 - Front Camera", reconnect_interval=5):
        """
        Initialize camera viewer.
        
        Args:
            rtsp_url: RTSP stream URL (e.g., rtsp://192.168.1.100:8554/live)
            window_name: OpenCV window title
            reconnect_interval: Seconds to wait before reconnecting if stream fails
        """
        self.rtsp_url = rtsp_url
        self.window_name = window_name
        self.reconnect_interval = reconnect_interval
        self.cap = None
        self.frame_count = 0
        self.last_frame = None
        
    def connect(self):
        """Connect to RTSP stream"""
        if self.cap is not None:
            self.cap.release()
        
        print(f"[CAMERA] Connecting to RTSP stream: {self.rtsp_url}")
        
        try:
            # Create video capture object with RTSP URL
            self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            
            # Set some buffer properties for lower latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Try to read first frame to verify connection
            ret, frame = self.cap.read()
            
            if ret:
                print(f"[CAMERA] ✓ Connected to RTSP stream")
                print(f"[CAMERA] Resolution: {frame.shape[1]}x{frame.shape[0]}")
                self.last_frame = frame
                self.frame_count = 1
                return True
            else:
                print(f"[CAMERA] ✗ Failed to read from RTSP stream (cannot retrieve frame)")
                if self.cap:
                    self.cap.release()
                self.cap = None
                return False
                
        except Exception as e:
            print(f"[CAMERA] ✗ Failed to connect: {str(e)}")
            if self.cap:
                self.cap.release()
            self.cap = None
            return False
    
    def display_frame(self, frame):
        """Display a frame with FPS and connection info"""
        try:
            # Add FPS counter
            if hasattr(self, 'last_frame_time'):
                fps = 1.0 / (time.time() - self.last_frame_time)
                cv2.putText(
                    frame,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
            
            # Add frame count
            cv2.putText(
                frame,
                f"Frames: {self.frame_count}",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            
            # Add status
            cv2.putText(
                frame,
                "STREAMING",
                (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            
            # Display the frame
            cv2.imshow(self.window_name, frame)
            self.last_frame_time = time.time()
            
        except Exception as e:
            print(f"[CAMERA] Error displaying frame: {str(e)}")
    
    def run(self):
        """Main viewer loop - continuously display frames"""
        print(f"[CAMERA] Starting camera viewer (press 'q' to quit)")
        print(f"[CAMERA] Window: {self.window_name}")
        
        last_connect_attempt = 0
        
        try:
            while True:
                # Try to connect if not connected
                if self.cap is None:
                    now = time.time()
                    if now - last_connect_attempt >= self.reconnect_interval:
                        if not self.connect():
                            last_connect_attempt = now
                            print(f"[CAMERA] Retrying in {self.reconnect_interval}s...")
                            # Display last frame or placeholder
                            if self.last_frame is not None:
                                frame = self.last_frame.copy()
                                cv2.putText(
                                    frame,
                                    "DISCONNECTED - Reconnecting...",
                                    (50, frame.shape[0]//2),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0,
                                    (0, 0, 255),
                                    2
                                )
                                cv2.imshow(self.window_name, frame)
                            
                            # Check for quit key even when disconnected
                            key = cv2.waitKey(100) & 0xFF
                            if key == ord('q'):
                                print("[CAMERA] Quit requested")
                                break
                            continue
                        last_connect_attempt = now
                
                # Try to read frame from stream
                try:
                    ret, frame = self.cap.read()
                    
                    if not ret:
                        print("[CAMERA] Stream ended or disconnected, will reconnect...")
                        self.cap.release()
                        self.cap = None
                        continue
                    
                    self.frame_count += 1
                    self.last_frame = frame.copy()
                    
                    # Display the frame with overlays
                    self.display_frame(frame)
                    
                    # Check for quit key (press 'q' to exit)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("[CAMERA] Quit requested")
                        break
                    
                except Exception as e:
                    print(f"[CAMERA] Error reading frame: {str(e)}")
                    if self.cap:
                        self.cap.release()
                    self.cap = None
        
        except KeyboardInterrupt:
            print("\n[CAMERA] Interrupted by user")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Release resources"""
        print("[CAMERA] Cleaning up...")
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        print("[CAMERA] ✓ Camera viewer closed")


def main():
    parser = argparse.ArgumentParser(
        description="Display RTSP camera feed from drone during Mission 1"
    )
    parser.add_argument(
        "--rtsp-url",
        type=str,
        default="rtsp://192.168.1.100:8554/live",
        help="RTSP stream URL (default: rtsp://192.168.1.100:8554/live)",
    )
    parser.add_argument(
        "--window-name",
        type=str,
        default="Mission 1 - Front Camera",
        help="OpenCV window title",
    )
    parser.add_argument(
        "--reconnect-interval",
        type=int,
        default=5,
        help="Seconds to wait before reconnecting if stream fails (default: 5)",
    )
    
    args = parser.parse_args()
    
    print("[CAMERA] Mission 1 RTSP Camera Viewer")
    print(f"[CAMERA] RTSP URL: {args.rtsp_url}")
    print("[CAMERA] Press 'q' or Ctrl+C to quit")
    print()
    
    # Check if OpenCV is available
    try:
        import cv2
    except ImportError:
        print("[CAMERA] Error: OpenCV (cv2) is required but not installed")
        print("[CAMERA] Install with: pip install opencv-python")
        return False
    
    viewer = RTSPCameraViewer(
        rtsp_url=args.rtsp_url,
        window_name=args.window_name,
        reconnect_interval=args.reconnect_interval,
    )
    
    viewer.run()
    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
