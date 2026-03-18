#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

OS="$(uname -s)"
SERVICE=""
RELAY_PID=""

cleanup() {
    echo "Stopping..."
    [ -n "$RELAY_PID" ] && kill "$RELAY_PID" 2>/dev/null
    docker compose --profile mac --profile linux down 2>/dev/null
    exit 0
}
trap cleanup INT TERM

case "$OS" in
    Linux)
        SERVICE="lidar-linux"
        echo "=== Linux detected: using host networking + native display ==="
        xhost +local:docker 2>/dev/null || true
        docker compose --profile linux up -d --build "$SERVICE"
        ;;
    Darwin)
        SERVICE="lidar-mac"
        echo "=== macOS detected: using TCP relay + Foxglove ==="

        # Check socat
        if ! command -v socat &>/dev/null; then
            echo "Installing socat..."
            brew install socat
        fi

        docker compose --profile mac up -d --build "$SERVICE"

        # Start UDP relay inside container
        echo "Starting UDP relay in container..."
        docker exec -d suas-lidar python3 /home/rosuser/ros_ws/src/lidar/udp_relay.py
        sleep 2

        # Tunnel LiDAR UDP traffic into the container over TCP.
        echo "Starting host-side socat relay..."
        socat -u UDP4-RECV:2368,reuseaddr TCP4:127.0.0.1:12368 &
        RELAY_PID=$!
        sleep 1
        ;;
    *)
        echo "Unsupported OS: $OS"
        exit 1
        ;;
esac

# Launch live ingest and the visualization bridge.
echo "Starting LiDAR driver + Foxglove bridge..."
docker exec -d suas-lidar bash -c "source /opt/ros/humble/setup.bash && \
    ros2 launch /home/rosuser/ros_ws/src/lidar/launch_vlp16.py"

sleep 5

echo ""
echo "============================================="
echo "  LiDAR ingest stack is live."
echo "  Foxglove Studio: ws://localhost:8765"
if [ "$OS" = "Linux" ]; then
echo "  RViz2:  docker exec -it suas-lidar bash"
echo "          ros2 run rviz2 rviz2"
fi
echo "============================================="
echo ""
echo "Press Ctrl+C to stop."
wait
