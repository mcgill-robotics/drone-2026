FROM ros2_image:latest

ENV DEBIAN_FRONTEND=noninteractive

RUN sudo apt update && sudo apt install -y \
    ros-humble-cv-bridge \
    python3-tk

COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

WORKDIR /app

# Source ROS2 on every shell session
RUN echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]
