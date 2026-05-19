# Use the official OSRF ROS 2 Humble Desktop Full image.
# This base image includes Ubuntu 22.04 (Jammy) and all core ROS 2 tools,
# including RViz and rqt, which saves significant build time.
FROM ros:humble-ros-base

# Define arguments for creating a non-root user (best practice)
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# --------------------------------------------------------------------------------
# 1. Setup Non-Root User
# --------------------------------------------------------------------------------
# Create the user with the specified UID/GID (usually matching the host user)
USER root
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Grant devuser passwordless sudo access
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# --------------------------------------------------------------------------------
# 2. Install GUI and Development Dependencies
# --------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # libgl1 is crucial for OpenGL rendering used by RViz
    libgl1-mesa-glx \
    xterm \
    git \
    build-essential \
    cmake \
    python3-pip \
    # Cleanup to reduce image size
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------------
# 3. Environment Configuration
# --------------------------------------------------------------------------------
# Switch to the non-root user for all subsequent commands
USER $USERNAME
WORKDIR /home/$USERNAME/ros_ws

# Source the ROS 2 environment automatically when the container starts a shell
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Set the entrypoint to ensure the environment is sourced for all executions
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]

# Set the default command if none is provided
CMD ["/bin/bash"]
