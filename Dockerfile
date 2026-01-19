# -----------------------------------------------------------
# Base Image (Ubuntu 22.04 + CUDA 12.4 for RTX 5050)
# -----------------------------------------------------------
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# -----------------------------------------------------------
# Install ROS 2 Humble & Build Tools
# -----------------------------------------------------------
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release locales software-properties-common git vim python3-pip && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# -----------------------------------------------------------
# Install Robotis & Audio Dependencies
# -----------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-argcomplete \
    ros-dev-tools \
    # --- AUDIO TOOLS ---
    alsa-utils espeak-ng pulseaudio-utils libsndfile1 libasound2-plugins \
    # --- OFFICIAL ROBOTIS DEPENDENCIES ---
    ros-humble-gazebo-* \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gripper-controllers \
    ros-humble-moveit* \
    # Compilation tools
    build-essential cmake \
    libcudnn9-cuda-12 libcudnn9-dev-cuda-12 \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------
# Install AI Libraries (Llama/Whisper)
# -----------------------------------------------------------
RUN python3 -m pip install --upgrade pip
RUN pip3 install --upgrade packaging setuptools wheel scikit-build-core cmake ninja
RUN pip3 install "numpy<2.0.0" --ignore-installed
RUN pip3 install --ignore-installed sympy==1.13.1
RUN pip3 install faster-whisper
ENV CMAKE_ARGS="-DGGML_CUDA=on"
RUN pip3 install llama-cpp-python --no-cache-dir --verbose

# Cleanup for ROS
RUN pip3 install setuptools==58.2.0
RUN pip3 uninstall -y cmake

# -----------------------------------------------------------
# Build Robotis Source Packages & Apply Patch
# -----------------------------------------------------------
WORKDIR /root/turtlebot3_ws/src

# Clone Repos
RUN git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# --- [PATCH]
RUN printf '#!/usr/bin/env python3\n\
import os\n\
from ament_index_python.packages import get_package_share_directory\n\
from launch import LaunchDescription\n\
from launch.actions import DeclareLaunchArgument\n\
from launch.actions import IncludeLaunchDescription\n\
from launch.launch_description_sources import PythonLaunchDescriptionSource\n\
from launch.substitutions import PathJoinSubstitution\n\
from launch_ros.substitutions import FindPackageShare\n\
\n\
def generate_launch_description():\n\
    ld = LaunchDescription()\n\
    launch_dir = os.path.join(get_package_share_directory("turtlebot3_manipulation_moveit_config"), "launch")\n\
    gazebo_package_dir = get_package_share_directory("turtlebot3_manipulation_gazebo")\n\
    gazebo_launch_dir = os.path.join(gazebo_package_dir, "launch")\n\
\n\
    rviz_launch = IncludeLaunchDescription(\n\
        PythonLaunchDescriptionSource([launch_dir, "/moveit_rviz.launch.py"])\n\
    )\n\
    ld.add_action(rviz_launch)\n\
\n\
    move_group_launch = IncludeLaunchDescription(\n\
            PythonLaunchDescriptionSource([launch_dir, "/move_group.launch.py"]),\n\
            launch_arguments={"use_sim_time": "true"}.items(),\n\
        )\n\
    ld.add_action(move_group_launch)\n\
\n\
    rviz_arg = DeclareLaunchArgument("start_rviz", default_value="false", description="Whether execute rviz2")\n\
    ld.add_action(rviz_arg)\n\
\n\
    gazebo_control_launch = IncludeLaunchDescription(\n\
        PythonLaunchDescriptionSource([gazebo_launch_dir, "/gazebo.launch.py"]),\n\
        launch_arguments={"x_pose": "0.0", "y_pose": "0.0", "z_pose": "0.0", "roll": "0.0", "pitch": "0.0", "yaw": "0.0"}.items(),\n\
        )\n\
    ld.add_action(gazebo_control_launch)\n\
    return ld\n' > /root/turtlebot3_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_moveit_config/launch/moveit_gazebo.launch.py

WORKDIR /root/turtlebot3_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# -----------------------------------------------------------
# Environment Configuration
# -----------------------------------------------------------
# Audio Bridge
RUN echo "pcm.!default { type pulse fallback \"sysdefault\" }" > /etc/asound.conf
RUN echo "ctl.!default { type pulse fallback \"sysdefault\" }" >> /etc/asound.conf

# Gazebo Environment
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc

# [FIX] Force Gazebo to see the Robot Meshes
RUN echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/turtlebot3_ws/install/turtlebot3_description/share:/root/turtlebot3_ws/install/turtlebot3_manipulation_description/share' >> /root/.bashrc

# Standard ROS Environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/turtlebot3_ws/install/setup.bash" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle_pi" >> /root/.bashrc
RUN echo "export ROS_DOMAIN_ID=30" >> /root/.bashrc

CMD ["/bin/bash"]
