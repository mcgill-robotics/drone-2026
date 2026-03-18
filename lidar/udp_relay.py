#!/usr/bin/env python3
"""TCP-to-UDP relay for LiDAR data inside Docker on macOS.

Receives Velodyne packets tunneled over TCP from the host-side socat,
and re-emits them as UDP on localhost:2368 for velodyne_driver_node.

Not needed on Linux (host networking gives direct UDP access).

Host side: socat -u UDP4-RECV:2368,reuseaddr TCP4:127.0.0.1:12368
"""
import socket
import sys
import threading

TCP_PORT = 12368
UDP_TARGET = ("127.0.0.1", 2368)
VLP16_PACKET_SIZE = 1206


def handle_client(conn, addr, udp_sock):
    print(f"[relay] Connection from {addr}", flush=True)
    buf = b""
    packets = 0
    try:
        while True:
            data = conn.recv(65536)
            if not data:
                break
            buf += data
            while len(buf) >= VLP16_PACKET_SIZE:
                udp_sock.sendto(buf[:VLP16_PACKET_SIZE], UDP_TARGET)
                buf = buf[VLP16_PACKET_SIZE:]
                packets += 1
                if packets % 5000 == 0:
                    print(f"[relay] {packets} packets forwarded", flush=True)
    except Exception as e:
        print(f"[relay] Error: {e}", flush=True)
    finally:
        conn.close()
        print(f"[relay] Disconnected {addr} ({packets} packets)", flush=True)


def main():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_sock.bind(("0.0.0.0", TCP_PORT))
    tcp_sock.listen(1)
    print(f"[relay] Listening TCP:{TCP_PORT} -> UDP:{UDP_TARGET}", flush=True)

    while True:
        conn, addr = tcp_sock.accept()
        threading.Thread(target=handle_client, args=(conn, addr, udp_sock), daemon=True).start()


if __name__ == "__main__":
    main()
