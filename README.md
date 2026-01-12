# 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

## Programming Assignment: Milestone 1 (V1.3)

**Authors:** Minhyuk Park and Tsz-Chiu Au

## Introduction

Welcome to CS 4379K / CS 5342. We prepared a series of programming assignments to teach you how to program a mobile manipulator to perform a task. Each assignment is a milestone towards programming a robot to perform the task. In the first milestone, we intend to give you an idea of how to interact with ROBOTIS’ Turtlebot3 waffle Pi with a Manipulator Arm using the Robot Operating System 2 (ROS2).

The first milestone is about simulating the mobile base (Turtlebot 3 Waffle Pi) in Gazebo, and running teleoperation, SLAM, and Navigation nodes. This verifies you have a working remote-pc setup for simulating turtlebot3, as well as controlling the physical robot in the future milestone assignments. This environment should also allow you to explore communication interfaces in ROS2 that will be useful in future milestones assignments. 

To do this, you will deploy our pre-configured Docker container that sets up all the software that is required for the assignment.

You might find the official tutorial on ROS2 Foxy useful in this course. <https://docs.ros.org/en/foxy/Tutorials.html>

You might find this video a useful overview of requirements.
<https://www.youtube.com/watch?v=8w3xhG1GPdo>

### Assignment Requirement

You need to demonstrate that you have a working setup and can operate the turtlebot in simulation by making a video. This will also demonstrate that you have a working setup for working with a physical turtlebot in the next milestone assignment. Refer to the demo requirement section at the end of the milestone assignment on what to include in the video. Once your group is done with the video demonstration that satisfies the demo requirement outlined at the end, please submit it to Canvas. Each group will submit one video. 

### Major Change Log

#### v1.2-> v1.3 
- Instruction updated to support the latest Nvidia GPUs (SM120) using a Docker container.

#### v1.1-> v1.2 
- Instruction updated to clarify how to modify parameters for SLAM and Navigation.
- Fix done to the provided NUC to support modifying parameters without sudo.
- Added sudo password for NUC. Please do not abuse this privilege.

#### v1.0-> v1.1 
- Styling updates
- Part 1 - Remote PC Setup

--- 

# Part 1 - Environment Setup

We need you to prepare a Ubuntu 22.04 Docker environment for the course milestones and the final project. From now onwards, it will be called ‘Remote PC’. It will provide you with a simulated environment for the robot, necessary Artificial Intelligence softwares and the necessary visualization tools you need to interface with both the simulated and the physical robot. We highly recommend that you to use the provided laptop. Nevertheless, you should be able to deploy this Docker container on your own computer, provided you have the necessary hardware, such as an Nvidia GPU.

The following instructions are based on a manual provided by the manufacturer, Robotis. We are using the ROS 2 version Humble.  Please select Humble on the website.

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup>

The above manual might help you if you are not comfortable with the provided instructions.

## Preparing ROS 2 environment for the provided Laptop as the ‘Remote PC’.

Please log in to the account corresponding to your group number. You would be asked to set up a first-time password.

The account would not have sudo access. However, all the programs you need are already preinstalled. You just need to set up the following for your group’s account.

IN PROGRESS

Chage your nvidia drivers by going into software and updates, settings and pro, additional drivers, nvidia cooperation: unknown and using nvidia driver (open kernel).... (proprietary)

# 1. Configure the repository
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
          && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Install the toolkit
        sudo apt-get update
        sudo apt-get install -y nvidia-container-toolkit

# 3. Configure Docker to use it and restart Docker
        sudo nvidia-ctk runtime configure --runtime=docker
        sudo systemctl restart docker

Ubuntu 24 install

        https://docs.docker.com/engine/install/ubuntu

        sudo apt remove $(dpkg --get-selections docker.io docker-compose docker-compose-v2 docker-doc podman-docker containerd runc | cut -f1)

# Add Docker's official GPG key:
        sudo apt update
        sudo apt install ca-certificates curl
        sudo install -m 0755 -d /etc/apt/keyrings
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
        sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
        Types: deb
        URIs: https://download.docker.com/linux/ubuntu
        Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
        Components: stable
        Signed-By: /etc/apt/keyrings/docker.asc
        EOF

        sudo apt update
        
        sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        
        sudo docker run hello-world

only if you have sudo

        sudo usermod -aG docker $USER

Setup dockerfile

mkdir turltebot_docker
cd turtlebot_docker
vi Dockerfile

# Step 1: Base Image with CUDA 12.1 support for Ubuntu 20.04
# We use the "devel" version to get the compiler (nvcc) needed for llama-cpp-python
FROM nvidia/cuda:12.1.1-devel-ubuntu20.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive

# Step 2: Install Basic Tools & Audio/AI Dependencies
# ffmpeg is required for Whisper; espeak-ng for TTS; python3-pip for libraries
RUN apt-get update && apt-get install -y \
    vim \
    net-tools \
    openssh-server \
    curl \
    git \
    locales \
    software-properties-common \
    python3-pip \
    python3-venv \
    espeak-ng \
    ffmpeg \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

# Step 3: Locale Setup
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Step 4: Install Python AI Libraries (Whisper & Llama)
# Upgrade pip first to avoid build issues
RUN python3 -m pip install --upgrade pip

# Install OpenAI Whisper
RUN pip3 install openai-whisper setuptools-rust

# Install llama-cpp-python with CUDA support enabled
# We set CMAKE_ARGS to force it to use the GPU
ENV CMAKE_ARGS="-DGGML_CUDA=on"
RUN pip3 install llama-cpp-python

# Step 5: Install ROS 2 Foxy
# Add keys and repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and Install Foxy Desktop + Dev Tools
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    python3-argcomplete \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Step 6: Install TurtleBot 3 Dependencies
RUN apt-get update && apt-get install -y \
    ros-foxy-gazebo-* \
    ros-foxy-cartographer \
    ros-foxy-cartographer-ros \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-dynamixel-sdk \
    ros-foxy-turtlebot3-msgs \
    ros-foxy-turtlebot3 \
    && rm -rf /var/lib/apt/lists/*

# Step 7: Install Open Manipulator & Build Workspace
RUN apt-get update && apt-get install -y \
    ros-foxy-ros2-control \
    ros-foxy-ros2-controllers \
    ros-foxy-gripper-controllers \
    ros-foxy-moveit \
    ros-foxy-moveit-servo \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/turtlebot3_ws/src
RUN git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git

WORKDIR /root/turtlebot3_ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

# Step 8: Update .bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "export ROS_DOMAIN_ID=30" >> /root/.bashrc
RUN echo "source /root/turtlebot3_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]

building docker image 

docker build -t ros-foxy-turtlebot .


create docker-compose file to mount working folder and add gui support

        mkdir my_code
        vi docker-compose.yml

Change yml file to the following

        services:
          remote_pc:
            build: .
            image: ros-foxy-ai-gpu
            container_name: remote_pc_foxy
            network_mode: host
            privileged: true
            
            # Enable NVIDIA GPU support
            deploy:
              resources:
                reservations:
                  devices:
                    - driver: nvidia
                      count: all
                      capabilities: [gpu]
        
            environment:
              - DISPLAY=${DISPLAY}
              - QT_X11_NO_MITSHM=1
              - NVIDIA_VISIBLE_DEVICES=all
              - NVIDIA_DRIVER_CAPABILITIES=all
            
            volumes:
              - /tmp/.X11-unix:/tmp/.X11-unix:rw
              # Your local code (and GGUF models) map here
              - ./my_code:/root/turtlebot3_ws/src/my_code
            
            command: tail -f /dev/null
            stdin_open: true
            tty: true

# Useful commands

enable gui permissions every boot

        xhost +local:root

build container

        docker compose up -d --build

enter container

        docker exec -it remote_pc_foxy bash

test your container

        xhost +local:root

printenv | grep ROS
# Should show ROS_DISTRO=foxy and ROS_DOMAIN_ID=30
        ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py

        sudo docker compose down
        sudo HOST_UID=$(id -u) USER_HOME=$HOME docker compose up -d --build
        sudo docker exec -it remote_pc_foxy bash

        espeak "Audio check successful" --stdout | paplay
        printenv | grep ROS


# Part 2 - Testing Your Setup Through Simulation

Next, we need you to demonstrate your setup with teleoperation and running SLAM and Navigation.

## Teleoperation Simulation

This manual is based on the following manual for Foxy.

**(Wayback Machine)**
<https://web.archive.org/web/20240309203141/https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

This manual assumes you have completed Part 1 on setting up your remote PC. You may also use the provided NUC.

**[Remote PC]** Bring up the TurtleBot3 with OpenMANIPULATOR-X into the Gazebo world with the following command.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** To control the TurtleBot3 in the Gazebo simulation, the servo server node of MoveIt must be launched first.

    ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
        
 **[Remote PC]** Launch the keyboard teleoperation node.

    ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop
        
The following keys are used to control the TurtleBot3. Try moving the turtlebot in the simulated space. Use O, K, L, and; keys to drive the TurtleBot3 platform.

    Use o|k|l|; keys to move the turtlebot base and use the 'space' key to stop the base        
    Use s|x|z|c|a|d|f|v keys to Cartesian jog
    Use 1|2|3|4|q|w|e|r keys to joint jog.
    'ESC' to quit.

## SLAM Simulation

This manual is based on the following manual for ROS2 Foxy.
**(Wayback Machine)**
<https://web.archive.org/web/20240309203141/https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

Note: Simulation for vanilla turtlebot 3 and turtlebot 3 with manipulator arm uses different software packages. However, this earlier section of the manual provides a better overview of SLAM. You might want to read up on the SLAM and Simulation section. Select Foxy for specific instructions for foxy.

**(Wayback Machine)**
<https://web.archive.org/web/20240309202817/https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node>

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/>

Close all terminals if you are coming from previous sections.

**[Remote PC]** Bringup the TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** Launch the slam node using the following command.

    ros2 launch turtlebot3_manipulation_cartographer cartographer.launch.py
        
**[Remote PC]** Launch the keyboard teleoperation node. Use O, K, L, ; keys to drive the TurtleBot3 platform to create a good “map” of the environment.

    ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop

**[Remote PC]** Open a new terminal on Remote PC. Run the nav2_map_server to save the current map on RViz.

    ros2 run nav2_map_server map_saver_cli -f ~/map

You might find it necessary to tune the SLAM parameters. Please read the tuning guide before trying to tune. For people using their own PC, please also refer to the last section of the appendix.

**[Remote PC]** The configuration location can be accessed by following the terminal command. For NUC users, go to this folder and look for your group’s Lua script.

    cd /home/chiu/turtlebot3_ws/install/turtlebot3_manipulation_cartographer/share/turtlebot3_manipulation_cartographer/config

**[Remote PC]**  You can load your custom Lua script by following the command.

**[Remote PC]** Close all terminals. Bring up the TurtleBot3 with OpenMANIPULATOR-X into the Gazebo world with the following command. 

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** For NUC users, modify your own group’s Lua script. File permissions are set so that you cannot view or modify other groups’ files.

    # change index according to your group name 
    vi group1.lua 

**[Remote PC]** Launch your modified Lua file by changing the launch command

    # change index according to your group name 
    ros2 launch turtlebot3_manipulation_cartographer cartographer.launch.py configuration_basename:=’group1.lua’

## Slam Tuning Guide

Reference:

**(Wayback Machine)**:
<https://web.archive.org/web/20240309202817/https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node>

**(Original URL)**:
<https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/>

In this assignment, you will demonstrate understanding of parameters in ROS2 software. Parameters are variables that can either be loaded during the launch process or changed during runtime, that alter the behavior of ROS2 software.

The SLAM in ROS2 uses Cartographer ROS, which provides configuration options via a Lua file.

Below options are defined in turtlebot3_cartographer/config/turtlebot3_lds_2d.lua file. (Note: Exact file name and file location might differ). For more details about each options, please refer to the Cartographer ROS official documentation.

* `MAP_BUILDER.use_trajectory_builder_2d`
    * This option sets the type of SLAM.

* `TRAJECTORY_BUILDER_2D.min_range`
    * This option sets the minimum usable range of the lidar sensor.

* `TRAJECTORY_BUILDER_2D.max_range`
    * This option sets the maximum usable range of the lidar sensor.

* `TRAJECTORY_BUILDER_2D.missing_data_ray_length`
    * In 2D, Cartographer replaces ranges further than `max_range` with this value.

* `TRAJECTORY_BUILDER_2D.use_imu_data`
    * If you use 2D SLAM, range data can be handled in real-time without an additional source of information, so you can choose whether you’d like Cartographer to use an IMU or not.

* `TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching`
    * **Local SLAM:** The `RealTimeCorrelativeScanMatcher` can be toggled depending on the reliability of the sensor.

* `TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians`
    * **Local SLAM:** To avoid inserting too many scans per submap, a scan is dropped if the motion does not exceed a certain angle.

* `POSE_GRAPH.optimize_every_n_nodes`
    * **Global SLAM:** Setting this to `0` is a handy way to disable global SLAM and concentrate on the behavior of local SLAM.

* `POSE_GRAPH.constraint_builder.min_score`
    * **Global SLAM:** Threshold for the scan match score below which a match is not considered. Low scores indicate that the scan and map do not look similar.

* `POSE_GRAPH.constraint_builder.global_localization_min_score`
    * **Global SLAM:** Threshold below which global localizations are not trusted.
      
## Navigation Simulation

This manual is based on the following manual for ROS2 Foxy.

**(Wayback Machine)**
<https://web.archive.org/web/20240309203141/https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

Note: Simulation for vanilla turtlebot 3 and turtlebot 3 with manipulator arm uses different software packages. However, this earlier section of the manual provides a better overview of Navigation. You might want to read up on the Navigation and Simulation section. Select Foxy for specific instructions for foxy.

**(Wayback machine)**
<https://web.archive.org/web/20240309202259/https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#run-navigation-nodes>

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/>

Close all terminals if you are coming from previous sections.

**[Remote PC]** Bringup the TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

 **[Remote PC]** Open a terminal on Remote PC. Launch the navigation file using the following command. Note that you are referring to map.yaml file created in the previous step for SLAM.

    ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=$HOME/map.yaml

 **[Remote PC]** Click on the 2D Nav Goal button in the RViz menu GUI.

 **[Remote PC]** Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing. 

This green arrow is a marker that can specify the destination of the robot.
The root of the arrow is the x, y coordinates of the destination, and the angle  is determined by the orientation of the arrow.
As soon as x, y, and targets are set, TurtleBot3 will immediately start moving to the destination.

 Recall from SLAM that you can tune parameters. You can also tune Navigation parameters by modifying the appropriate YAML file. Please read the tuning guide before trying to tune. For people using their own PC, please also refer to the last section of the appendix.

**[Remote PC]** For NUC users, go to this folder to look for your group’s YAML file.

    cd /home/chiu/turtlebot3_ws/install/turtlebot3_manipulation_navigation2/share/turtlebot3_manipulation_navigation2/param

 **[Remote PC]**  You can load your custom configuration YAML file by following the command.

Close all terminals if you are coming from previous sections.

**[Remote PC]** Bringup the TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** For NUC users, change your own group’s Lua script. File permissions are set so that you cannot view or modify other groups’ files.

    # change index according to your group name 
    vi group1_turtlebot3.yaml

**[Remote PC]** Load your custom configuration YAML file by changing the launch command

    # change index according to your group name 
    ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=$HOME/map.yaml params_file:=group1_turtlebot3.yaml

## Navigation Tuning Guide

Just like SLAM, you will demonstrate your understanding of parameters in ROS2 software with Navigation2.

The Navigation2 stack has many unique parameters that change performance for different robots. Although it’s similar to the ROS1 Navigation, please refer to the Configuration Guide of Navigation2 or the ROS Navigation Tuning Guide by Kaiyu Zheng for more details.

Reference:
<https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation>

Note: The YAML file location is different in this guide
 
### Costmap Parameters
*Parameters defined in: `turtlebot3_navigation2/param/${TB3_MODEL}.yaml`*

* `inflation_layer.inflation_radius`
  * This parameter creates an inflation area around the obstacle. The path is planned so that it does not cross this area. It is safe to set this larger than the robot radius. For more information, please refer to the `costmap_2d` wiki.

* `inflation_layer.cost_scaling_factor`
  * This is an inverse proportional factor multiplied by the value of the costmap. If this parameter is increased, the costmap value decreases.
  * The optimal path for the robot to pass through obstacles is to take a median path between them. Setting a smaller value for this parameter will create a path farther from the obstacles.

### dwb_controller
*Parameters defined in: `turtlebot3_navigation2/param/${TB3_MODEL}.yaml`*

* `max_vel_x`
  * Sets the maximum value of translational velocity.

* `min_vel_x`
  * Sets the minimum value of translational velocity. If set negative, the robot can move backwards.

* `max_vel_y`
  * The maximum y velocity for the robot in m/s.

* `min_vel_y`
  * The minimum y velocity for the robot in m/s.

* `max_vel_theta`
  * Actual value of the maximum rotational velocity. The robot cannot be faster than this.

* `min_speed_theta`
  * Actual value of the minimum rotational speed. The robot cannot be slower than this.

* `max_speed_xy`
  * The absolute value of the maximum translational velocity for the robot in m/s.

* `min_speed_xy`
  * The absolute value of the minimum translational velocity for the robot in m/s.

* `acc_lim_x`
  * The x acceleration limit of the robot in m/s².

* `acc_lim_y`
  * The y acceleration limit of the robot in m/s².

* `acc_lim_theta`
  * The rotational acceleration limit of the robot in rad/s².

* `decel_lim_x`
  * The deceleration limit of the robot in the x direction in m/s².

* `decel_lim_y`
  * The deceleration limit of the robot in the y direction in m/s².

* `decel_lim_theta`
  * The deceleration limit of the robot in the theta direction in rad/s².

* `xy_goal_tolerance`
  * The x,y distance allowed when the robot reaches its goal pose.

* `yaw_goal_tolerance`
  * The yaw angle allowed when the robot reaches its goal pose.

* `transform_tolerance`
  * Allows latency for tf messages.

* `sim_time`
  * Sets the forward simulation time in seconds.
  * Setting this too small makes it difficult for the robot to pass narrow spaces, while a large value limits dynamic turns. *You can observe the differences in the length of the yellow line in the simulation path.*

# Video Demo Requirements (Approximately 2.5 Minutes)

Your group will upload one or more video clips (e.g., in MP4 format) to Canvas. The maximum total length of the video clips is approximately two and a half minutes. One group member should narrate the video, explaining each step as it's performed. At the beginning of the first video clip, please clearly state the names of all group members.

Your recording setup should be organized to show all relevant windows at once: the terminal(s) used for launching nodes, the Gazebo simulation window, and the RViz visualization window.

The demonstration must clearly show the successful completion of the following four parts in order. 

## Part A: Teleoperation 

The goal here is to show you can manually control both the robot's base 

- Launch Simulation: Start the Gazebo world and the necessary nodes for teleoperation
- Base Movement: Using the keyboard teleoperation node, demonstrate control of the mobile base. You must show the robot moving forward, backward, and turning both left and right.


## Part B: SLAM (Simultaneous Localization and Mapping) 

This part demonstrates your ability to map an unknown environment.

- Launch SLAM: With the simulation running, launch the Cartographer SLAM node.
- Build the Map: Use the teleoperation node to drive the robot around the Gazebo environment. Your video must show the map being constructed in real-time within RViz. Drive enough to create a reasonably complete and accurate map of the world.
- Save the Map: Once the map is complete, execute the map_saver_cli command in a new terminal to save your map.
- Verify Files: Briefly open your file manager and show the map files that were generated in your home directory.

## Part C: Autonomous Navigation 

This final part shows you can use your generated map to have the robot navigate autonomously to specific goals.

Launch Navigation: Relaunch the simulation and start the Navigation2 stack, making sure to load the map you saved in Part B.
Localize the Robot: In RViz, confirm the robot's initial position is correct. If it's not, use the "2D Pose Estimate" tool to set its correct starting position and orientation on the map.

- Navigate to Goals: Use the "2D Nav Goal" tool in RViz to assign two sequential goals for the robot:
- Goal 1: Set a destination in a clear, open area.
- Goal 2: Set a destination that requires the robot to navigate around at least one obstacle 
- Show the Path: For each goal, ensure the video clearly shows the global path planned by the navigation stack and the robot smoothly following that path to arrive at its destination. 

## Part D: Parameter Investigation 

This section demonstrates that you have explored the configuration files and understand how tuning parameters can affect robot behavior.

- Choose a Parameter: Select one parameter from either the SLAM configuration (turtlebot3.lua) or the Navigation2 configuration (turtlebot3.yaml). Note: Exact Lua and YAML file names are different.
- Modify It: Change its value significantly (e.g., double or halve the default value).
- Demonstrate the Effect: Briefly re-run the relevant task (SLAM or Navigation) and show the resulting change in the robot's behavior.

Explain Your Findings: In your narration, clearly state:

- Which file and parameter did you modify?
- The original value and the new value.
- A brief explanation of the observed effect (e.g., "By increasing the inflation_radius from 0.2 to 1.0, the robot now plans paths that stay much further away from walls, treating them as larger obstacles.").

# Appendix

While we highly recommend that you use the provided NUC, you can choose to use your own portable PC for simulations and physical robot control.

The following instructions are based on a manual provided by the manufacturer, Robotis. We are using the ROS 2 version Foxy.  Please select Foxy on the website.

**(Web Archive)**
<https://web.archive.org/web/20240309202506/https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup>

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup>

The above manual might help you if you are not comfortable with the provided instructions.



## Preparing ROS 2 environment for your PC as ‘Remote PC’ without Docker. (Note: We will not support this approach anymore)

1) Install Ubuntu 20.04.2.0 LTS (Focal Fossa, 64bit, Desktop). Note that Virtual Machines and WSL2 environments are not officially supported by our class. 

    <https://releases.ubuntu.com/focal/> 

The installation file is  ubuntu-20.04.2.0-desktop-amd64.iso

2) Install some software on the remote PC:

      sudo apt -y install vim
      sudo apt -y install net-tools
      sudo apt -y install openssh-server
      sudo apt -y install curl

You may want to remap the shortcut keys of Copy and Paste in your terminal.


3) Set up Network:

Set up wifi connection settings to TXST-Bobcats wifi (for internet) and Small_Blue_Wifi (for local network environment for Turtlebot 3’s Single Board Computer)

Run ifconfig to see the IP of remote-pc while being connected to Small_Blue_Wifi. Remember the IP as IP_OF_REMOTE_PC.

4) Update Ubuntu software:

      sudo apt-get update
      sudo apt-get upgrade

5) Install ROS 2 Foxy (full version):

Please refer to <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>

    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    
    sudo apt upgrade
    
    sudo apt install ros-foxy-desktop python3-argcomplete
    
    sudo apt install ros-dev-tools
    
    source /opt/ros/foxy/setup.bash
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

6) Install TurtleBot 3 ROS 2 Packages for foxy:

Please refer to 

**(Web Archive)**
<https://web.archive.org/web/20240309202506/https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>
**(Current URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup.>

Dependencies

    sudo apt-get install ros-foxy-gazebo-*
    sudo apt install ros-foxy-cartographer
    sudo apt install ros-foxy-cartographer-ros
    sudo apt install ros-foxy-navigation2
    sudo apt install ros-foxy-nav2-bringup

Turtlebot 3 Packages

    source ~/.bashrc
    sudo apt install ros-foxy-dynamixel-sdk
    sudo apt install ros-foxy-turtlebot3-msgs
    sudo apt install ros-foxy-turtlebot3

Environmental configuration

    echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
    source ~/.bashrc

Note: You can ignore the following warning

    bash: /home/{$YOUR_ACCOUNT}/turtlebot3_ws/install/setup.bash: No such file or directory

7) Install Open Manipulator ROS2 packages for Foxy. 

Please refer to these links for more information
**(Web Archive)**
<https://web.archive.org/web/20240309203141/https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#turtlebot3-with-openmanipulator>
**(Current URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/>

Refer to 7. Manipulation -> 7.1 TB3 & OpenMANIPULATOR -> 2. Software Setup -> Remote PC

    sudo apt install ros-foxy-dynamixel-sdk ros-foxy-ros2-control ros-foxy-ros2-controllers ros-foxy-gripper-controllers ros-foxy-moveit
    cd ~/
    mkdir turtlebot3_ws
    cd turtlebot3_ws
    mkdir src
    cd ~/turtlebot3_ws/src/
    git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
    cd ~/turtlebot3_ws && colcon build --symlink-install

8) Set up the source for turtlebot ws

    source ~/turtlebot3_ws/install/setup.bash
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc 

9) Install foxy-moveit-servo

    sudo apt install ros-foxy-moveit-servo

10) You may ignore warnings such as these whenever you load a new terminal window.

    not found: "/home/chiu/turtlebot3_ws/install/turtlebot3_fake_node/share/turtlebot3_fake_node/local_setup.bash"
    not found: "/home/chiu/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/local_setup.bash"




## Notes on running changing SLAM and Navigation parameters on your PC

### SLAM Simulation Steps 5 and 6

 **[Remote PC]**  You might want to look at the tuning guide to tune your map, should you find it necessary while performing steps on navigation. 

For people following the Appendix to set up their own system

    cd ~ /turtlebot3_ws/install/turtlebot3_manipulation_cartographer/share/turtlebot3_manipulation_cartographer/config

 **[Remote PC]**  You can load your custom Lua script by following the command.

Close all terminals. Bring up the TurtleBot3 with OpenMANIPULATOR-X into the Gazebo world with the following command. 

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

For people who are using their own pc, you can change the configuration file itself.

    vi turtlebot3_2d.lua
    
    ros2 launch turtlebot3_manipulation_cartographer cartographer.launch.py 

Note: Alternatively, in order for your custom Lua script loading to work, you need the configuration to be in your ros2 package’s configuration folder and use colcon build to let the ROS2 configuration file resolver know that it exists. (Step 7 in Appendix). This has been done for you on the NUC we provide. If you want to make a custom Lua file,  refer to the following steps

    cd ~ /turtlebot3_ws/install/turtlebot3_manipulation_cartographer/share/turtlebot3_manipulation_cartographer/config
    
    cp turtlebot3_2d.lua custom.lua
    
    cd ~/turtlebot3_ws && colcon build --symlink-install


    ros2 launch turtlebot3_manipulation_cartographer cartographer.launch.py configuration_basename:=’custom.lua’

### Navigation Simulation Steps 5 and 6

 **[Remote PC]** Recall from SLAM that you can tune SLAM parameters. You can also tune Navigation parameters by modifying the appropriate YAML file. 

Go to 

    cd ~ /turtlebot3_ws/install/turtlebot3_manipulation_navigation2/share/turtlebot3_manipulation_navigation2/param

**[Remote PC]**  You can load your custom configuration YAML file by following the command.

Close all terminals if you are coming from previous sections.

**[Remote PC]** Bringup the TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

For people who are using their own pc, you can change the configuration file itself.

    vi turtlebot3.yaml 
    
    ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=$HOME/map.yaml 

Note: Alternatively, in order for your custom YAML loading to work, you need the configuration to be in your ros2 package’s configuration folder and use colcon build to let the ROS2 configuration file resolver know that it exists. (Step 7 in Appendix). This has been done for you on the NUC we provide. If you want to make a custom YAML file,  refer to the following steps

    cd ~ /turtlebot3_ws/install/turtlebot3_manipulation_navigation2/share/turtlebot3_manipulation_navigation2/param
    
    cp turtlebot3.yaml custom.yaml
    
    cd ~/turtlebot3_ws && colcon build --symlink-install
    
    ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=$HOME/map.yaml params_file:=custom.yaml


