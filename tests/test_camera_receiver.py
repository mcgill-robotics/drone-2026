"""Tests for camera_receiver.D455Receiver"""
import struct
from unittest.mock import MagicMock, patch

import pytest

from camera_receiver import D455Receiver


class TestD455ReceiverInit:
    def test_default_port(self):
        r = D455Receiver("192.168.0.1")
        assert r.jetson_ip == "192.168.0.1"
        assert r.port == 5005
        assert r.socket is None
        assert r.running is False

    def test_custom_port(self):
        r = D455Receiver("1.2.3.4", port=9999)
        assert r.port == 9999


class TestConnect:
    def test_connect_success_sets_running(self):
        r = D455Receiver("1.2.3.4")
        fake_sock = MagicMock()
        with patch("camera_receiver.socket.socket", return_value=fake_sock):
            r.connect()
        fake_sock.connect.assert_called_once_with(("1.2.3.4", 5005))
        assert r.running is True
        assert r.socket is fake_sock

    def test_connect_refused_reraises(self):
        r = D455Receiver("1.2.3.4")
        fake_sock = MagicMock()
        fake_sock.connect.side_effect = ConnectionRefusedError
        with patch("camera_receiver.socket.socket", return_value=fake_sock):
            with pytest.raises(ConnectionRefusedError):
                r.connect()
        assert r.running is False


class TestReceiveExact:
    def test_reads_full_amount_across_multiple_chunks(self):
        r = D455Receiver("1.2.3.4")
        fake_sock = MagicMock()
        # socket returns 3 bytes, then 2 bytes (total 5)
        fake_sock.recv.side_effect = [b"abc", b"de"]
        r.socket = fake_sock
        assert r._receive_exact(5) == b"abcde"

    def test_raises_on_closed_connection(self):
        r = D455Receiver("1.2.3.4")
        fake_sock = MagicMock()
        fake_sock.recv.return_value = b""  # connection closed
        r.socket = fake_sock
        with pytest.raises(ConnectionResetError):
            r._receive_exact(5)


class TestReceiveFrame:
    def test_returns_frame_type_and_decoded_image(self):
        r = D455Receiver("1.2.3.4")
        # Header: frame_type=1, size=4. Payload=4 bytes.
        header = struct.pack(">BI", 1, 4)
        payload = b"\x01\x02\x03\x04"

        fake_sock = MagicMock()
        fake_sock.recv.side_effect = [header, payload]
        r.socket = fake_sock

        fake_image = MagicMock()
        with patch("camera_receiver.cv2.imdecode", return_value=fake_image):
            frame_type, frame = r.receive_frame()
        assert frame_type == 1
        assert frame is fake_image

    def test_returns_none_on_exception(self):
        r = D455Receiver("1.2.3.4")
        fake_sock = MagicMock()
        fake_sock.recv.side_effect = RuntimeError("broken pipe")
        r.socket = fake_sock
        r.running = True

        frame_type, frame = r.receive_frame()
        assert frame_type is None
        assert frame is None
        assert r.running is False


class TestClose:
    def test_close_shuts_down_socket_and_flag(self):
        r = D455Receiver("1.2.3.4")
        r.socket = MagicMock()
        r.running = True
        with patch("camera_receiver.cv2.destroyAllWindows"):
            r.close()
        assert r.running is False
        r.socket.close.assert_called_once()

    def test_close_without_socket_does_not_raise(self):
        r = D455Receiver("1.2.3.4")
        with patch("camera_receiver.cv2.destroyAllWindows"):
            r.close()
        assert r.running is False
