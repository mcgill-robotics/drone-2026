FROM osrf/ros:humble-desktop-full-jammy

ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

USER root
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    python3-pip \
    iproute2 \
    iputils-ping \
    tcpdump \
    net-tools \
    # Velodyne VLP-16 driver stack
    ros-humble-velodyne \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    ros-humble-velodyne-laserscan \
    ros-humble-velodyne-msgs \
    # Pointcloud processing
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    # Transforms
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    # Diagnostics
    ros-humble-diagnostic-updater \
    # Foxglove bridge for platform-independent visualization
    ros-humble-foxglove-bridge \
    # SLAM (rtabmap for 3D lidar SLAM)
    ros-humble-rtabmap-ros \
    # Octomap (3D occupancy grid)
    ros-humble-octomap \
    ros-humble-octomap-server \
    ros-humble-octomap-mapping \
    ros-humble-octomap-rviz-plugins \
    # Sensor fusion (EKF for IMU + odometry)
    ros-humble-robot-localization \
    # MAVLink bridge (flight controller integration)
    ros-humble-mavros \
    ros-humble-mavros-extras \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# MAVros requires GeographicLib datasets
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

COPY --chown=$USERNAME:$USERNAME entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x /home/$USERNAME/entrypoint.sh

USER $USERNAME
WORKDIR /home/$USERNAME/ros_ws

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

EXPOSE 8765
ENTRYPOINT ["/home/rosuser/entrypoint.sh"]
CMD ["/bin/bash"]
