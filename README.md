# Controlling Synapticon Devices Using ROS2 Package

## Description

This repository provides an example of using Synapticon drives (SOMANET Node, SOMANET Circulo and SOMANET Integro) in CSP, CSV, and CST modes using the ROS2 package. It utilizes `SOEM Ethercat Master`. 
ROS2 package was originally developed by Andy Zelenak. Synapticon GmbH adds examples, simulation and adds the extended instructions for easier installation as well as support for containerization using Docker.

![Alt text](docs/images/rviz.png)

## Table of Contents

1. [Intention](#intention)
2. [Overview](#overview)
   - [Hardware](#hardware)
   - [Software](#software)
      - [Ubuntu 22.04 with ROS2](#ubuntu-2204-with-ros2)
         - [ROS2 Installation](#ros2-installation)
         - [Synapticon Package Installation](#synapticon-package-installation)
         - [Demo](#demo)
      - [Isolated Environment (Docker)](#isolated-environment-docker)
         - [Docker Installation](#docker-installation)
         - [Synapticon Package Installation](#synapticon-package-installation-docker)
         - [Demo](#demo-docker)
3. [Disclaimer](#disclaimer)



## Intention

The intention of this document is to provide instructions on how to quickly start using Synapticon Devices with ROS2 package using Synapticon library.

Additionally, in order to make it compatible with other Linux distributions, we provide a Docker file. You can specify your ethernet device name via a launch argument.

## Overview

### Hardware 

In the figure below, a block diagram of the wiring used in this setup is given. The provided package assumes that the laptop on which the setup is used has only one Ethernet port. Hardware can be used once the parameters are configured with [OBLAC tools](https://www.synapticon.com/en/products/oblac-drives). Detailed instructions and wiring diagrams for all the devices are available at our [official web page](https://www.synapticon.com/en/support/dokumentation) documentation. Software allows daisy chaining of all the Synapticon drives in any order.

![Alt text](docs/images/hardware.jpg)


### Software

In this demo, we consider two scenarios:
- Ubuntu 22.04 is installed on the system and ROS Humble and Synapticon package will be installed on that system
- User wants to run the package in isolated environment (possibly because a different distribution of Linux is installed)


#### Ubuntu 22.04 with ROS2

To install ROS2 on your Ubuntu machine, follow the steps from the [official website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and install the full version. After the installation, some configuration steps as described [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) are needed. For the completeness of the demo, the commands in the following subsection are copied from the official website and should be executed for the ROS2 installation.

##### ROS2 Installation

To make sure that locale supports UTF-8, run the following commands:
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
Enable Ubuntu Universe repository:
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Add GPG key:
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Add repository to the sources:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Update apt cache:
```bash
sudo apt update
```
This command is for updating the packages on your system and if the commands after it will work, we recommend skipping it:
```bash
sudo apt upgrade
```
Finally, install ROS and compilers:
```bash
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```
After the installation is complete, add the following line to the end of `/home/USER/.bashrc` file:
```bash
source /opt/ros/humble/setup.bash
```
In order for ROS2 not to interfere with communication on other ports, we need to set Domain ID (detailed information is available [here](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html)). In our case, we just used 1. To do so, add the following at the end of `/home/YOUR_USER/.bashrc`
```bash
export ROS_DOMAIN_ID=1
```
After this, restart all your terminals for the source command to be active.
To verify the installation, open two terminals and run:
```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```
If the nodes are communicating, the installation was successful.

##### Synapticon Package Installation

OPTION 1: Installing from Source

Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
Clone the Synapticon package:
```bash
git clone https://github.com/synapticon/synapticon_ros2_control
```
Install build tools:
```bash
sudo apt install python3-colcon-common-extensions
```
Build the package:
```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build
```
Additionally, you can source the workspace by adding the following line to the `/home/USER/.bashrc` file, but above the line where you sourced the ROS installation (above this line: `source /opt/ros/humble/setup.bash`):
```bash
source /home/USER/ros2_ws/install/setup.bash
```

OPTION 2: Binary Installation

If needed, add the ROS repository (this is done only once):

```bash
sudo apt install software-properties-common 
sudo add-apt-repository universe 
sudo apt update
```
Install Synapticon package:

```bash
sudo apt install ros-humble-synapticon-ros2-control
```

Make sure your rosdep is initialized and updated:

```bash
sudo rosdep init 
rosdep update
```

Install its dependencies:

```bash
rosdep install synapticon_ros2_control
```
The package will get installed to `/opt/ros/humble/share/synapticon_ros2_control/`.

VERIFICATION

To check if the master could be run and if the slaves are found, in the container terminal execute the following.
If you installed from source:
```bash
sudo ./home/YOUR_USER/ros2_ws/install/synapticon_ros2_control/bin/torque_control_executable YOUR_ETHERNET_INTERFACE
```
or if you installed using binary installation:
```bash
sudo ./opt/ros/humble/share/synapticon_ros2_control/bin/torque_control_executable YOUR_ETHERNET_INTERFACE
```
Before running other scripts, stop this one by CTRL+C (or wait, it will shutdown automatically after a while).

##### Demo
For turning the motor in different modes, you will need 5 terminals and in all of them execute:
```bash
sudo -i
source ~/.bashrc
```
- Terminal 1:

If you are running demo with one motor: 
```bash
ros2 launch synapticon_ros2_control elevated_permissions_1_dof.launch.py
```
If you are running demo with two motors:
```bash
ros2 launch synapticon_ros2_control elevated_permissions_2_dof.launch.py
```
- Terminal 2:

If you are running demo with one motor: 
```bash
ros2 launch synapticon_ros2_control single_dof.launch.py eth_device:=YOUR_ETHERNET_DEVICE
```
If you are running demo with two motors:
```bash
ros2 launch synapticon_ros2_control two_dof.launch.py eth_device:=YOUR_ETHERNET_DEVICE
```
- Terminal 3 - to show the running controllers 
```bash
ros2 control list_controllers
```
(Information does not automatically refresh - it can be refreshed each M seconds 
using `watch -n M ros2 control list_controllers`, but the output format might be ugly)
- Running motors with different controllers:

CSV (Cyclic Sync Velocity) mode:

Terminal 4 to turn on the controller :
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: []}"
```
Terminal 5 to create a publisher:

If you are running demo with one motor:
```bash
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray data:\ [100]
```
If you are running demo with two motors:
```bash
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray data:\ [100,100]
```
Stopping it: CTRL+C on Terminal 5 and in Terminal 4:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['quick_stop_controller'], deactivate_controllers: ['forward_velocity_controller']}"
```
- CSP (Cyclic Sync Position) mode:

Terminal 4 to turn on the controller :	
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_position_controller'], deactivate_controllers: [quick_stop_controller]}"
```
Terminal 5 to create a publisher:

If you are running demo with one motor:
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray data:\ [140]
```
If you are running demo with two motors:
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray data:\ [140, 140]
```
Stopping it: CTRL+C on Terminal 5 and in Terminal 4:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['quick_stop_controller'], deactivate_controllers: ['forward_position_controller']}"
```
	
- CST (Cyclic Sync Torque) mode:

Terminal 4 to turn on the controller :
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_torque_controller'], deactivate_controllers: [quick_stop_controller]}"	
```
Terminal 5 to create a publisher (value is in per mille of torque):

If you are running demo with one motor:
```bash
ros2 topic pub /forward_torque_controller/commands std_msgs/msg/Float64MultiArray data:\ [100]	
```
If you are running demo with two motors:
```bash
ros2 topic pub /forward_torque_controller/commands std_msgs/msg/Float64MultiArray data:\ [100, 100]	
```
Stopping it: CTRL+C on Terminal 5 and in Terminal 4:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['quick_stop_controller'], deactivate_controllers: ['forward_torque_controller']}"
```
##### Running Without Sudo (Optional)

If you want to run the example without using `sudo`, you need to create:
```bash
sudo touch /etc/systemd/system/ros2_control_node.service
```
and use text editor to paste in that file the following:

```bash
[Unit]
Description=Launch ros2_control_node with socket permissions

[Service]
Type=simple
User=YOUR_USER
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash; source /home/YOUR_USER/.bashrc; source /home/YOUR_USER/ros2_ws/install/setup.bash'
# Write the user environment to file, for debugging
#ExecStartPre=/bin/bash -c 'env > /home/YOUR_USER/Documents/ros_env_before_start.txt'

# This is essentially a copy of my normal user env
Environment="AMENT_PREFIX_PATH=/home/YOUR_USER/ros2_ws/install/synapticon_ros2_control:/opt/ros/humble"
Environment="HOME=/home/YOUR_USER"
Environment="LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib"
Environment="PATH=/opt/ros/humble/bin:/usr/lib/ccache:/home/your_user/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin"
Environment="PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
Environment="ROS_DISTRO=humble"
Environment="ROS_DOMAIN_ID=1"
Environment="ROS_PYTHON_VERSION=3"
Environment="ROS_VERSION=2"
Environment="ROSCONSOLE_FORMAT=[${severity}] - ${node}: [${time}] ${message}"
Environment="USER=YOUR_USER"
Environment="USERNAME=YOUR_USER"

ExecStart=/opt/ros/humble/bin/ros2 launch synapticon_ros2_control elevated_permissions_X_dof.launch.py
AmbientCapabilities=CAP_NET_RAW

[Install]
WantedBy=multi-user.target
```
After pasting, do not forget to replace `YOUR_USER` with your username and `X_dof` with 1 or 2 in the line saying which launch file needs to be executed. Save the file, restart the daemon:
```bash
sudo systemctl daemon-reload
```
and start the service:
```bash
sudo systemctl restart ros2_control_node.service
```
If you want to check the service status and see the ROS console logging:
```bash
sudo systemctl status ros2_control_node.service
```
Now, the example can be run by these two commands:
```bash
sudo systemctl restart ros2_control_node.service
```
and, if running demo with one motor:
```bash
ros2 launch synapticon_ros2_control single_dof.launch.py eth_device:=YOUR_ETHERNET_DEVICE
```
If you are running demo with two motors:
```bash
ros2 launch synapticon_ros2_control two_dof.launch.py eth_device:=YOUR_ETHERNET_DEVICE
```
Changing the controllers and publishing the desired position/velocity/torque can be now executed without sudo.
To stop the `ros2_control_node`:
```bash
sudo systemctl stop ros2_control_node.service
```

#### Isolated Environment (Docker)

For users with different Linux distributions or those preferring isolated environment, Docker can be used. Installation steps can be found in the [Docker Documentation](https://docs.docker.com/engine/install/ubuntu/). For the completeness of the documentation, we provide those steps here also:

##### Docker Installation

Install Docker and add the user to the Docker group:
```bash
sudo apt update
sudo apt install -y docker.io
sudo groupadd docker
sudo usermod -aG docker $USER
```
##### Synapticon Package Installation
Copy the `Dockerfile` and `synapticon_ros2_controller_build.sh` script from the `ros2_humble_docker` to the same folder on your machine. With the following command, you will run the bash script that will automatically install the ROS2 and package into Docker image, detect the hardware and replace the ethernet adapter name in the necessary files. In case that the script does not detect your hardware properly, you will need to follow the instructions from its output to build Docker image. In the folder where you placed two aforementioned files run:
```bash
bash ./synapticon_ros2_controller_build.sh
```
To allow Docker containers to output the screen on your system (this is required for RViZ), execute this on the host system:
```bash
xhost +
```
For the first execution of the program, we build container named `ros2_container` from the image `ros2_humble_synapticon` that we generated by bash script from the Dockerfile using the following command (this command is executed only once):
```bash
docker run -it -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket -v /tmp/.X11-unix:/tmp/.X11-unix --ipc=host -e DISPLAY=$DISPLAY  --network=host --env QT_X11_NO_MITSHM=1 --privileged --name ros2_container ros2_humble_synapticon
```
Now we have our container running. Each other time, we start container using: 
```bash
docker start ros2_container
```
For opening a new terminal in the running container, use:
```bash
docker exec -it ros2_container bash
```
and, once it opens, source ROS2 environment using
```bash
source /root/.bashrc
```
To check if the master could be run and if the slaves are found, in the container terminal execute:
```bash
./install/synapticon_ros2_control/bin/torque_control_executable
```
Before running other scripts, stop this one by CTRL+C (or wait, it will shutdown automatically after a while).

##### Demo

Connect Synapticon device configured with OBLAC Tools to your ethernet port as shown in Figure 1. For the demo, run 5 terminals in the container (`docker exec -it ros2_container bash` and `source /root/.bashrc`)

- Terminal 1
If you are running demo with one motor:
```bash
ros2 launch synapticon_ros2_control elevated_permissions_1_dof.launch.py
```
If you are running demo with two motors:
```bash
ros2 launch synapticon_ros2_control elevated_permissions_2_dof.launch.py
```
- Terminal 2 - this one will open RViZ (if it fails, you forgot to execute `xhost +` on your host machine). If you spin the motor by hand, you should see the movement in RViZ.
If you are running demo with one motor:
```bash
ros2 launch synapticon_ros2_control single_dof.launch.py eth_device:=YOUR_ETHERNET_DEVICE
```
If you are running demo with two motors:
```bash
ros2 launch synapticon_ros2_control two_dof.launch.py eth_device:=YOUR_ETHERNET_DEVICE
```
- Terminal 3 - to show the running controllers 
```bash
ros2 control list_controllers
```
(Information does not automatically refresh - it can be refreshed each M seconds 
using `watch -n M ros2 control list_controllers`, but the output might be ugly)

- Running motors with different controllers:
CSV (Cyclic Sync Velocity) mode:

Terminal 4 to turn on the controller :
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_velocity_controller'], deactivate_controllers: []}"
```
Terminal 5 to create a publisher:
If you are running demo with one motor:
```bash
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray data:\ [100]
```
If you are running demo with two motors:
```bash
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray data:\ [100,100]
```
Stopping it: CTRL+C on Terminal 5 and in Terminal 4:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['quick_stop_controller'], deactivate_controllers: ['forward_velocity_controller']}"
```
- CSP (Cyclic Sync Position) mode:

Terminal 4 to turn on the controller :	
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_position_controller'], deactivate_controllers: [quick_stop_controller]}"
```
Terminal 5 to create a publisher:
If you are running demo with one motor:
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray data:\ [140]
```
If you are running demo with two motors:
```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray data:\ [140, 140]
```
Stopping it: CTRL+C on Terminal 5 and in Terminal 4:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['quick_stop_controller'], deactivate_controllers: ['forward_position_controller']}"
```
	
- CST (Cyclic Sync Torque) mode:

Terminal 4 to turn on the controller :
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_torque_controller'], deactivate_controllers: [quick_stop_controller]}"	
```
Terminal 5 to create a publisher (value is in per mille of torque):
If you are running demo with one motor:
```bash
ros2 topic pub /forward_torque_controller/commands std_msgs/msg/Float64MultiArray data:\ [100]	
```
If you are running demo with two motors:
```bash
ros2 topic pub /forward_torque_controller/commands std_msgs/msg/Float64MultiArray data:\ [100, 100]	
```
Stopping it: CTRL+C on Terminal 5 and in Terminal 4:
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['quick_stop_controller'], deactivate_controllers: ['forward_torque_controller']}"
```

## Disclaimer

This repository is an example of using SOMANET drives with ROS2 Humble. It does not guarantee compatibility with the latest ROS versions or SOMANET firmware. The included code is for demonstration purposes only. Synapticon GmbH refuses any responsibility for any problem or damage by the use of the example configuration and code!



