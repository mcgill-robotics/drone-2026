"""
Frame receiver for displaying RealSense D455 frames sent from Jetson.
Run this on your computer to display the stream.
"""

import socket
import cv2
import numpy as np
import struct
from typing import Tuple


class D455Receiver:
    """Receives and displays frames from RealSense D455 streamer on Jetson."""
    
    def __init__(self, jetson_ip: str, port: int = 5005):
        """
        Initialize the receiver.
        
        Args:
            jetson_ip: IP address of the Jetson (e.g., "192.168.1.100")
            port: Port to connect to
        """
        self.jetson_ip = jetson_ip
        self.port = port
        self.socket: socket.socket = None
        self.running = False
    
    def connect(self):
        """Connect to the Jetson streamer."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            print(f"Connecting to {self.jetson_ip}:{self.port}...")
            self.socket.connect((self.jetson_ip, self.port))
            print("Connected!")
            self.running = True
        except ConnectionRefusedError:
            print(f"Failed to connect to {self.jetson_ip}:{self.port}")
            print("Make sure the streamer is running on the Jetson")
            raise
    
    def _receive_exact(self, num_bytes: int) -> bytes:
        """Receive exactly num_bytes from socket."""
        data = b''
        while len(data) < num_bytes:
            chunk = self.socket.recv(num_bytes - len(data))
            if not chunk:
                raise ConnectionResetError("Connection closed by server")
            data += chunk
        return data
    
    def receive_frame(self) -> Tuple[int, np.ndarray]:
        """
        Receive a frame from the streamer.
        
        Returns:
            Tuple of (frame_type, frame_data)
            frame_type: 1 for RGB, 2 for depth
            frame_data: Decoded image as numpy array
        """
        try:
            # Receive header: [frame_type(1 byte)] [size(4 bytes)]
            header = self._receive_exact(5)
            frame_type, frame_size = struct.unpack('>BI', header)
            
            # Receive frame data
            frame_data = self._receive_exact(frame_size)
            
            # Decode JPEG
            frame_array = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
            
            return frame_type, frame
        except Exception as e:
            print(f"Error receiving frame: {e}")
            self.running = False
            return None, None
    
    def display_stream(self):
        """Display the received frames."""
        self.connect()
        
        print("Displaying stream... Press 'q' to quit")
        
        while self.running:
            frame_type, frame = self.receive_frame()
            
            if frame is None:
                break
            
            # Display frame
            if frame_type == 1:
                cv2.imshow("RealSense - RGB", frame)
            elif frame_type == 2:
                cv2.imshow("RealSense - Depth", frame)
            
            # Check for quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.close()
    
    def close(self):
        """Close the connection."""
        self.running = False
        if self.socket:
            self.socket.close()
        cv2.destroyAllWindows()
        print("Disconnected")


def display_stream(jetson_ip: str, port: int = 5005):
    """
    Convenience function to connect and display the stream.
    
    Args:
        jetson_ip: IP address of the Jetson
        port: Port to connect to (default 5005)
    """
    receiver = D455Receiver(jetson_ip, port)
    receiver.display_stream()


if __name__ == "__main__":
    # Configure your Jetson's IP address here
    JETSON_IP = "192.168.1.100"  # Change this to your Jetson's IP
    
    display_stream(JETSON_IP)
