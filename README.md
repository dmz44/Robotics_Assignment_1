# 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

## Programming Assignment: Milestone 1 (V1.0)

**Authors:** Minhyuk Park and Tsz-Chiu Au

## Introduction

Welcome to CS 4379K / CS 5342. We prepared a series of programming assignments to teach you how to program a mobile manipulator to perform a task. Each assignment is a milestone towards programming a robot to perform the task. In the first milestone, we intend to give you an idea of how to interact with ROBOTIS’ Turtlebot3 Waffle Pi with a Manipulator Arm using the Robot Operating System 2 (ROS2).

The first milestone is about simulating the mobile base (Turtlebot 3 Waffle Pi) in Gazebo, and running teleoperation, SLAM, and Navigation nodes. This verifies you have a working remote PC setup for simulating turtlebot3, as well as controlling the physical robot in future milestone assignments. This environment should also allow you to explore communication interfaces in ROS2 that will be useful in future milestones assignments. 

To do this, you will deploy our pre-configured Docker container that sets up all the software that is required for the assignment.
Please refer to the following video for an explanation of what a Docker container environment is. 

[https://www.youtube.com/watch?v=Gjnup-PuquQ](https://www.youtube.com/watch?v=Gjnup-PuquQ)

Robot Operating System version is associated with Ubuntu Long-Term Support Versions (e.g. Ubuntu 22.04 with Humble). We are using **ROS 2 Humble in a Docker environment**. You might find the official tutorial on ROS 2 Humble useful in this course:

[https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)

You might find this video a useful overview of the requirements of Milestone 1.

<https://www.youtube.com/watch?v=8w3xhG1GPdo>

For all questions regarding milestone assignments and the robot, **you should contact the Doctoral Instructor Assistant via direct message on Slack**. Please do not contact the Instructor with questions regarding the milestone assignments. This is the URL for Slack for this course. 

<https://spring2026txstrobot.slack.com/>

We use vim (vi) for text editing in a terminal environment. Please refer to the tutorial for Vim if you are not familiar with vim environment for editing documents. 

https://opensource.com/article/19/3/getting-started-vim

### Assignment Requirement

You need to demonstrate that you have a working setup and can operate the turtlebot in simulation by making videos. This will also demonstrate that you have a working setup for working with a physical turtlebot in the next milestone assignment. Refer to the demo requirement section at the end of the milestone assignment on what to include in the video. Once your group is done with the video demonstration that satisfies the demo requirement outlined at the end, please submit it to Canvas. Only one member from each group will submit all videos necessary to demonstrate that your group has completed all requirements. Please refer to the rules for the Robot room usage if you are using our laptop for the assignment requirement. 

### Major Change Log

#### v1.0
- Instruction updated to support the latest Nvidia GPUs (SM120) using a Docker container.
- **Major Update:** Migrated remote PC environment to Ubuntu 22.04 and ROS 2 Humble.

--- 

# Part 1 - Environment Setup

We need you to prepare a **Ubuntu 22.04 Docker** environment for the course milestones and the final project. From now onwards, it will be called ‘Remote PC’. It will provide you with a simulated environment for the robot, the necessary Artificial Intelligence software, and the necessary visualization tools you need to interface with both the simulated and the physical robot. We highly recommend that you use the provided laptop. Nevertheless, you should be able to deploy this Docker container on your own computer, provided you have the necessary hardware, such as an Nvidia GPU. Please refer to the appendix for instructions on how to setup 'Remote PC' environment on your computer. 

The following instructions for environment setup are of our own, based on a manual provided by the manufacturer, Robotis. Please select Humble on the website.
**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup>

The above manual might help you if you are not comfortable with the provided instructions.

## Preparing ROS 2 environment for the provided Laptop as the ‘Remote PC’.

Please log in to the account corresponding to your group number. You would be asked to set up a first-time password.

For the provided laptop, all the programs you need are preinstalled. However, you still need to set up a Docker environment for your account. For your first login, you will set the password for the account. Please do not attempt to set a password or log onto a group number other than your assigned group number. 

# Part 1: Container Setup and How to Use Docker

## Container Setup

1. **Clone the course Docker repository:**
```bash
mkdir -p ~/txst_robotics
cd ~/txst_robotics
mkdir -p my_code
git clone https://github.com/dmz44/Robotics_Assignment_1.git.

```

2. **Build and Start the Container:**
We have provided a `docker-compose.yml` file that automates the build process and sets up the necessary volume mappings (shared folders) and display settings.
```bash
# Build and start the container in detached mode
sudo HOST_UID=$(id -u) USER_HOME=$HOME docker compose up -d --build

```

*Note: This process may take a long time depending on your internet speed as it downloads ROS 2 Humble and builds the simulation packages.*

## How to use Docker

You should start the container itself every time you reboot the computer. Please refer to **Build and Start the Container:**.

Once the container is running, you can enter it and run the simulation examples. You can enter it in multiple terminal windows to get multiple terminal windows of the container.

*For those not familiar with Docker, you need to enter the container and use the container's terminal/shell, or the software given in milestones will not run!*

1. **Enable GUI Permissions:**
Since the simulation runs inside Docker but displays on your host screen, you need to allow local connections to the X server:
```bash
xhost +local:root

```
*You need to run this command again if you restart your computer.*

2. **Start the Container. You only need to do it once every boot unless you stop the environment by composing down:**

```bash
docker compose up -d
```

3. **Enter the Container:**
```bash
docker exec -it remote_pc_humble bash

```

4. **Verify ROS 2 Environment:**
Inside the container, verify that ROS 2 Humble is active:
```bash
printenv | grep ROS
# Expected output:
# ROS_DISTRO=humble
# ROS_DOMAIN_ID=30 (or similar)

```

5. **Stopping the Environment:**
When you are finished, you can stop the container from your host terminal:
```bash
docker compose down

```
6. **Critical Concept: The Container is Temporary (Immutable)**

It is vital to understand that a Docker container is **ephemeral**. This means it resets to its original "factory settings" every time you delete it (via `docker compose down`) and restart it.

* **What is lost:** If you run `sudo apt install <package>` or create a file inside the container's home folder (e.g., `/root/`), those changes **will vanish** when the container is stopped.
* **What is safe:** Only files stored in the **Shared Folder** (see Section 8) are safe.
* **Temporary Testing:** It is perfectly fine to install a package manually or edit a system config file inside the container to test a fix. Just remember that you must repeat that step next time, or make the change permanent (see Section 9).

7. **Restarting the Container**

If you just want to "pause" your work without losing the container's temporary state, you can use `docker compose stop` and `docker compose start`. However, for this course, we generally recommend fully shutting down (`down`) to clear simulation glitches.

8. **The Shared Folder: Where to Save Your Code**

To prevent losing your homework, we use a feature called **Volume Mapping** (Shared Folders). This creates a direct "tunnel" between a specific folder on your real computer (Host) and a folder inside the Docker container.

* **On your Host:** The folder is `./cs4379k_ws/my_code` (or similar, depending on where you cloned the repo).
* **Inside Docker:** The folder is mapped to `/root/turtlebot3_ws/src/my_code`.

**How to use it:**

1) Create your Python scripts and ROS packages **inside this folder**.
2) If you edit a file in this folder on your laptop (using VS Code, Sublime, etc.), the change appears **instantly** inside the Docker container.
3) Even if you delete the container completely, files in this folder remain safe on your laptop.

9. **Advanced: Modifying the Docker Image (Rebuilding)**

Let's say you need a new system library (e.g., `scipy` or a new `apt` package) permanently on your Docker Image. We can rebuild the Docker image by the following:

1. **Edit the Dockerfile:** Open the `Dockerfile` in your host text editor. Add the installation command (e.g., `RUN pip3 install scipy`) in the appropriate section.
2. **Rebuild the Container:** You must tell Docker to rebuild the image based on your changes. Run the following command from your host terminal:

```bash
docker compose up -d --build

```

The `--build` flag forces Docker to read the `Dockerfile` again and install the new software.

# Part 2 - Testing Your Setup Through Simulation

Next, we need you to demonstrate your setup with teleoperation and running SLAM and Navigation.

## Teleoperation Simulation

This manual is based on the following manual for Humble.

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

This manual assumes you have completed Part 1 on setting up your remote PC. Please enter the container and work within the container.

**[Remote PC]** **Enter the Container:**

```bash
docker exec -it remote_pc_humble bash

```

Verify that you have docker shell, e.g. root@remote-pc-humble. You will not be able to execute simulation outside docker shell. 

**[Remote PC]** Bring up the TurtleBot3 with OpenMANIPULATOR-X into the Gazebo world with the following command.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** To control the TurtleBot3 in the Gazebo simulation, the servo server node of MoveIt must be launched first. Open another Docker shell by repeating entering the container on another terminal window and type the following command.

    ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
        
 **[Remote PC]** Launch the keyboard teleoperation node. Open another Docker shell by repeating entering the container on another terminal window and type the following command.

    ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop
        
The following keys are used to control the TurtleBot3. Try moving the turtlebot in the simulated space. Use O, K, L, and; keys to drive the TurtleBot3 platform.

    Use o|k|l|; keys to move the turtlebot base and use the 'space' key to stop the base        
    Use s|x|z|c|a|d|f|v keys to Cartesian jog
    Use 1|2|3|4|q|w|e|r keys to joint jog.
    'ESC' to quit.

## SLAM Simulation

This manual is based on the following manual for ROS2 Humble.

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

Note: Simulation for vanilla turtlebot 3 and turtlebot 3 with manipulator arm uses different software packages. However, this earlier section of the manual provides a better overview of SLAM. You might want to read up on the SLAM and Simulation section. 

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/>

Close all terminals if you are coming from previous sections.

**[Remote PC]** **Enter the Container:**

```bash
docker exec -it remote_pc_humble bash

```

Verify that you have docker shell, e.g. root@remote-pc-humble. You will not be able to execute simulation outside docker shell. 

**[Remote PC]** Bringup the TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** Launch the slam node using the following command. Open another Docker shell by repeating entering the container on another terminal window and type the following command.

    ros2 launch turtlebot3_manipulation_cartographer cartographer.launch.py
        
**[Remote PC]** Launch the keyboard teleoperation node. 

Open another Docker shell by repeating entering the container on another terminal window and type the following command.

Use O, K, L, ; keys to drive the TurtleBot3 platform to create a good “map” of the environment.

    ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop

**[Remote PC]** Open a new terminal and another Docker shell on Remote PC. Run the nav2_map_server to save the current map on RViz.

    ros2 run nav2_map_server map_saver_cli -f ~/map

## How to Launch with Modified SLAM Parameters

Further along the road, you might not like how the SLAM software behaves and want to change its behavior by modifying certain SLAM parameters. Please read the tuning guide before trying to follow this section. For people using their own PC, please also refer to the last section of the appendix.

**[Remote PC]** Close all terminals. Open another Docker shell. 

```bash
docker exec -it remote_pc_humble bash

```

Verify that you have Docker shell, e.g. root@remote-pc-humble. You will not be able to execute the simulation outside docker shell. 

**[Remote PC]** The configuration location can be accessed by following the terminal command. Go to the folder below inside the Docker container and look for the appropriate Lua script. Note that you have to change the path to your group's corresponding number.

    cd ~/turtlebot3_ws/install/turtlebot3_manipulation_cartographer/share/turtlebot3_manipulation_cartographer/config
    vi turtlebot3_2d.lua

**[Remote PC]** After editing the lua script, bring up the TurtleBot3 with OpenMANIPULATOR-X into the Gazebo world with the following command. 

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** Launch another Docker shell and launch cartographer with your modified Lua file.

    ros2 launch turtlebot3_manipulation_cartographer cartographer.launch.py 

## Slam Tuning Guide

Reference:

**(Original URL)**:
<https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/>

In this assignment, you will demonstrate understanding of parameters in ROS2 software. Parameters are variables that can either be loaded during the launch process or changed during runtime, that alter the behavior of ROS2 software.

The SLAM in ROS2 uses Cartographer ROS, which provides configuration options via a Lua file.

The options are defined in turtlebot3_cartographer/config/turtlebot3_lds_2d.lua file. (Note: Exact file name and file location might differ.) For more details about each option, please refer to the Cartographer ROS official documentation.

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

This manual is based on the following manual for ROS2 Humble.

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation>

Note: Simulation for vanilla turtlebot 3 and turtlebot 3 with manipulator arm uses different software packages. However, you might want to read up on the Navigation and Simulation section for base turtlebot 3. 

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/>

Close all terminals if you are coming from previous sections.

**[Remote PC]** **Enter the Container:**

```bash
docker exec -it remote_pc_humble bash

```

Verify that you have Docker shell, e.g. root@remote-pc-humble. You will not be able to execute the simulation outside docker shell. 

**[Remote PC]** Bringup the TurtleBot3 with OpenMANIPULATOR-X into the Gazebo world with the following command inside the Docker shell.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

 **[Remote PC]** Open another Docker shell on Remote PC. Launch the navigation file using the following command. Note that you are referring to map.yaml file created in the previous step for SLAM.

    ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=$HOME/map.yaml

 **[Remote PC]** Click on the 2D Nav Goal button in the RViz menu GUI.

 **[Remote PC]** Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing. 

This green arrow is a marker that can specify the destination of the robot.
The root of the arrow is the x, y coordinates of the destination, and the angle  is determined by the orientation of the arrow.
As soon as x, y, and targets are set, TurtleBot3 will immediately start moving to the destination.

Recall from SLAM that you can tune parameters. You can also tune Navigation parameters by modifying the appropriate YAML file. Please read the tuning guide before trying to tune. For people using their own PC, please also refer to the last section of the appendix.

## How to Launch with Modified Navigation 2 Parameters

Further along the road, you might not like how the Navigation software behaves and want to change its behavior by modifying certain Navigation 2 parameters. Please read the tuning guide before trying to follow this section. 

**[Remote PC]** **Enter the Container:**

```bash
docker exec -it remote_pc_humble bash

```

Verify that you have Docker shell, e.g., root@remote-pc-humble. You will not be able to execute the simulation outside docker shell. 

**[Remote PC]** The configuration location can be accessed by following the terminal command inside the Docker shell. Go to the folder below and look for the appropriate Lua script. Note that you have to change the path to your group's corresponding number.

    cd ~ /turtlebot3_ws/install/turtlebot3_manipulation_navigation2/share/turtlebot3_manipulation_navigation2/param

**[Remote PC]**   Bringup the TurtleBot3 with OpenMANIPULATOR-X into Gazebo world with the following command after opening another terminal window and entering another window of Docker shell.

    ros2 launch turtlebot3_manipulation_bringup gazebo.launch.py

**[Remote PC]** Change your own group’s Lua script.

    vi turtlebot3.yaml 

**[Remote PC]** Inside the Docker shell, load your custom configuration YAML file by changing the launch command

    ros2 launch turtlebot3_manipulation_navigation2 navigation2.launch.py map_yaml_file:=$HOME/map.yaml 

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

# Video Demo Requirements 

Your group will upload one or more video clips (e.g., in MP4 format) to Canvas. The estimated total length of the video clips is approximately two and a half minutes, but  going over that time is fine. One group member should narrate the video, explaining each step as it's performed. At the beginning of the first video clip, please show every group member's face and state the names of all group members.

Your recording setup should be organized to show all relevant windows at once: the terminal(s) used for launching nodes, the Gazebo simulation window, and the RViz visualization window.

You do not need to edit the videos, and uploading raw videos will suffice. You can split the video into multiple videos and submit them separately if the video is too long.  
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

This part shows you can use your generated map to have the robot navigate autonomously to specific goals.

Launch Navigation: Relaunch the simulation and start the Navigation2 stack, making sure to load the map you saved in Part B.
Localize the Robot: In RViz, confirm the robot's initial position is correct. If it's not, use the "2D Pose Estimate" tool to set its correct starting position and orientation on the map.

- Navigate to Goals: Use the "2D Nav Goal" tool in RViz to assign two sequential goals for the robot:
- Goal 1: Set a destination in a clear, open area.
- Goal 2: Set a destination that requires the robot to navigate around at least one obstacle 
- Show the Path: For each goal, ensure the video clearly shows the global path planned by the navigation stack and the robot smoothly following that path to arrive at its destination. 

## Part D: Parameter Investigation 

This final section demonstrates that you have explored the configuration files and understand how tuning parameters can affect robot behavior.

- Choose a Parameter: Select one parameter from either the SLAM configuration (turtlebot3.lua) or the Navigation2 configuration (turtlebot3.yaml). Note: Exact Lua and YAML file names are different.
- Modify It: Change its value significantly (e.g., double or halve the default value).
- Demonstrate the Effect: Briefly re-run the relevant task (SLAM or Navigation) and show the resulting change in the robot's behavior.

Explain Your Findings: In your narration, clearly state:

- Which file and parameter did you modify?
- The original value and the new value.
- A brief explanation of the observed effect (e.g., "By increasing the inflation_radius from 0.2 to 1.0, the robot now plans paths that stay much further away from walls, treating them as larger obstacles.").

# Appendix

# [Optional] How to Install Required Software on Your Own PC [Not Supported]

While we highly recommend that you use the provided Laptop, you can choose to use your own portable PC for simulations and physical robot control. This might be beneficial for you, as access to the robot is limited to support multiple student groups. 

The hardware requirement is hardware capable of supporting the Ubuntu 24 Operating System with a relatively modern Nvidia GPU, a microphone, and a speaker.

The following instructions are based on a manual provided by the manufacturer, Robotis. We are using the ROS 2 version Humble.  Please select Humble on the website.

**(Original URL)**
<https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup>

For your information, you can also get away from using Docker by following the instructions from the above URL to install robot packages locally on **Ubuntu 22.04** PC for milestones 1 to 3. However, Artificial Intelligence materials from Milestones 4 to 6 are not supported by the local installation. 

## Preparing ROS 2 Environment for Your PC as ‘Remote PC’ with Docker. 

### (Note: if you choose to develop with your own PC, we will not troubleshoot or support your environment)

1) **Install Ubuntu 24.04 LTS (64bit, Desktop)**. We recommend not using WSL2 or any other virtualization.

```bash
[<https://releases.ubuntu.com/focal/> ](https://ubuntu.com/download/desktop)
```

2) **Install essential software on your PC**:

```bash
sudo apt -y install vim
sudo apt -y install net-tools
sudo apt -y install openssh-server
sudo apt -y install curl
```

You may want to remap the shortcut keys of Copy and Paste in your terminal.

3) **Set up Network**:

Set up wifi connection settings to your internet, such as TXST-Bobcats wifi and Small_Blue_Wifi (for the local network environment for Turtlebot 3’s Single Board Computer)

Run ifconfig to see the IP of remote-pc while being connected to Small_Blue_Wifi. Remember the IP as IP_OF_REMOTE_PC.

4) **Update Ubuntu software**:

```bash
sudo apt-get update
sudo apt-get upgrade
```

Now, we need to set up the Docker engine on your host machine. Follow these steps to install Docker from the official repository.
    [<https://docs.docker.com/engine/install/ubuntu//> ](https://docs.docker.com/engine/install/ubuntu/)


5) **Remove conflicting packages (if any):**

```bash
sudo apt-get remove docker docker-engine docker.io containerd runc
```

6) **Set up the repository:**

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

```

7) **Add the repository to Apt sources:**

```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

```

8) **Install Docker packages:**

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

```


9) **Manage Docker as a non-root user:**
To avoid typing `sudo` for every Docker command, add your user to the Docker group:

```bash
sudo usermod -aG docker $USER
newgrp docker

```


10) **Verify Installation:**

```bash
docker run hello-world

```

11) **Configure the repository for NVIDIA Container Toolkit:**
To allow the Docker container to access your GPU (essential for Gazebo simulation and AI tasks), you must install the NVIDIA Container Toolkit.

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

```

12) **Install the NVIDIA toolkit:**
```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

```

13) **Configure Docker runtime and restart:**
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

```

14) **Clone the course Docker repository:**
*(Replace the URL below with the actual course repository link)*
```bash
mkdir -p ~/txst_robotics
cd ~/txst_robotics
mkdir -p ~/my_code
git clone https://github.com/dmz44/Robotics_Assignment_1.git.

```

15) **Build and Start the Container:**
We have provided a `docker-compose.yml` file that automates the build process and sets up the necessary volume mappings (shared folders) and display settings.
```bash
# Build and start the container in detached mode
docker compose up -d --build

```


*Note: This process may take a long time depending on your internet speed as it downloads ROS 2 Humble and builds the simulation packages.*

16) **Enable GUI Permissions:**
Since the simulation runs inside Docker but displays on your host screen, you need to allow local connections to the X server:
```bash
xhost +local:root

```
*You may need to run this command again if you restart your computer.*

Once the container is running, you can enter it and run the simulation examples.




