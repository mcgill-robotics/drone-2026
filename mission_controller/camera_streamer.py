"""
Frame streamer for sending RealSense D455 frames from Jetson to computer over Ethernet.
Run this on the Jetson.
"""

import socket
import cv2
import numpy as np
import threading
import struct
import pyrealsense2 as rs
from typing import Optional


class D455Streamer:
    """Streams RGB and depth frames from RealSense D455 over TCP."""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 5005, include_depth: bool = False):
        """
        Initialize the streamer.
        
        Args:
            host: Host to bind to (0.0.0.0 for all interfaces)
            port: Port to listen on
            include_depth: Whether to also stream depth frames
        """
        self.host = host
        self.port = port
        #true = stream RBG+Depth (2 freames/itertaion), otherwise 1 frame/iteration
        self.include_depth = include_depth
        self.running = False
        self.client_socket: Optional[socket.socket] = None
        
        # RealSense setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)
        
    def start(self):
        """Start the camera and server."""
        # Configure RealSense
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        if self.include_depth:
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        self.pipeline.start(self.config)
        self.running = True
        
        print(f"Camera streamer listening on {self.host}:{self.port}")
        
        # Start server thread
        server_thread = threading.Thread(target=self._server_loop, daemon=True)
        server_thread.start()
    
    def _server_loop(self):
        """Accept client connections and stream frames."""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        
        while self.running:
            try:
                print("Waiting for client connection...")
                self.client_socket, addr = server_socket.accept()
                print(f"Client connected from {addr}")
                self._stream_frames()
            except Exception as e:
                print(f"Connection error: {e}")
            finally:
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
    
    def _stream_frames(self):
        """Stream frames to connected client."""
        while self.running and self.client_socket:
            try:
                frames = self.pipeline.wait_for_frames()
                frames = self.align.process(frames)
                
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                color_data = np.asanyarray(color_frame.get_data())
                
                # Encode RGB frame as JPEG
                _, rgb_encoded = cv2.imencode('.jpg', color_data, [cv2.IMWRITE_JPEG_QUALITY, 80])
                
                # Send frame with header: [frame_type(1)] [size(4)] [data]
                frame_type = 1  # RGB frame
                frame_size = len(rgb_encoded)
                header = struct.pack('>BI', frame_type, frame_size)
                
                self.client_socket.sendall(header + rgb_encoded.tobytes())
                
                # Optionally send depth frame
                if self.include_depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        depth_data = np.asanyarray(depth_frame.get_data()).astype(np.uint16)
                        # Convert to 8-bit for visualization
                        depth_vis = (depth_data / depth_data.max() * 255).astype(np.uint8)
                        
                        _, depth_encoded = cv2.imencode('.jpg', depth_vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        
                        frame_type = 2  # Depth frame
                        frame_size = len(depth_encoded)
                        header = struct.pack('>BI', frame_type, frame_size)
                        
                        self.client_socket.sendall(header + depth_encoded.tobytes())
                        
            except (BrokenPipeError, ConnectionResetError):
                print("Client disconnected")
                break
            except Exception as e:
                print(f"Frame streaming error: {e}")
                break
    
    def stop(self):
        """Stop the streamer."""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        self.pipeline.stop()
        print("Streamer stopped")


def start_streaming(host: str = "0.0.0.0", port: int = 5005, include_depth: bool = False):
    """
    Convenience function to start streaming frames.
    
    Args:
        host: Host to bind to
        port: Port to listen on  
        include_depth: Whether to stream depth frames
        
    Returns:
        D455Streamer instance
    """
    streamer = D455Streamer(host=host, port=port, include_depth=include_depth)
    streamer.start()
    return streamer


if __name__ == "__main__":
    # Start streaming
    streamer = start_streaming(include_depth=False)
    
    try:
        while True:
            pass
    except KeyboardInterrupt:
        streamer.stop()
