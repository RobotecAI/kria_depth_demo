# Kria K260 depth demo

This demo presents an appication of [AMD Kria K260](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit.html) to computations of stereo-vision based depth. It also showcases a Hardware-in-the-loop (HIL) simulation with [Open 3D Engine (O3DE)](www.o3de.org) and the use of Robot Operating System (ROS) interfaces.

The key computation is offloaded to FPGA (Field-programmable gate array) for efficiency and determinism.

In this simple project number of technologies are utilized:
- [AMD Kria 260](https://xilinx.github.io/kria-apps-docs/home/build/html/index.html), a board which is well-suited for robotics.
- [AMD Vitis :tm:](https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vitis.html), a platform to develop solutions for FPGA.
- [Kria Robotics stack](https://xilinx.github.io/KRS/sphinx/build/html/index.html), a ROS 2 set of tools, nodes, and libraries to deploy hardware-accelerated solutions to Kria SOMs.

## Prerequisites
- Host machine (X86) with Ubuntu 22.04 and plenty of free space (~300 Gb), primarily for Vitis. This machine will also run the HIL simulation.
- KR260 board with Ubuntu 22.04, which you set up following [this guide](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit/getting-started/setting-up-the-sd-card-image.html).
- [ROS 2 Humble installed](https://docs.ros.org/en/humble/Installation.html) on KR260 and x86.

## Install Vitis (Vivado) on the host machine

Download [Vivado 2022.1](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive.html), preferably [Xilinx_Unified_2022.1](https://www.xilinx.com/member/forms/download/xef.html?filename=Xilinx_Unified_2022.1_0420_0327.tar.gz). The file is quite large, so do not forget to check MD5 checksum before installation. 
Install Vitis at default location `/tools/Xilinx`.  

#### Troubleshooting

If you experience freezing, follow the [support recommendation](https://support.xilinx.com/s/question/0D54U00005astbhSAA/vivado-gets-stuck-or-takes-more-than-1-to-15-days-in-final-processing-ie-generating-installed-device-list-when-trying-to-install-in-ubuntu-2204?language=en_US) and install these:
```bash 
sudo apt-get install libtinfo5 libncurses5
```

## Install ROS 2 Humble on KRIA

Setup Locale:
   ```bash
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```
Add Sources & Repo Key:
   ```bash
   sudo apt update && sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   ```
Add ROS 2 Repository:
   ```bash
   sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
   ```

Install ROS 2 Humble (full-desktop, 1.7 GB download):**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

Install additional tools and `rmw-cyclonedds-cpp` for ROS:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-argcomplete ros-humble-rmw-cyclonedds-cpp
   ```

## Build workspace on KRIA

```bash
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv
```

```bash
###################################################
# 2. create a new ROS 2 workspace with examples and
#    firmware for KR260
###################################################
mkdir -p ~/krs_ws/src; cd ~/krs_ws
```

```bash

###################################################
# 3. Create file with KRS 1.0 additional repos
###################################################
cat << 'EOF' > krs_humble.repos
repositories:  
  perception/image_pipeline:
    type: git
    url: https://github.com/ros-acceleration/image_pipeline
    version: ros2

  tracing/tracetools_acceleration:
    type: git
    url: https://github.com/ros-acceleration/tracetools_acceleration
    version: humble

  firmware/acceleration_firmware_kr260:
    type: zip
    url: https://github.com/ros-acceleration/acceleration_firmware_kr260/releases/download/v1.1.1/acceleration_firmware_kr260.zip

  acceleration/adaptive_component:
    type: git
    url: https://github.com/ros-acceleration/adaptive_component
    version: humble
  acceleration/ament_acceleration:
    type: git
    url: https://github.com/ros-acceleration/ament_acceleration
    version: humble
  acceleration/ament_vitis:
    type: git
    url: https://github.com/ros-acceleration/ament_vitis
    version: humble
  acceleration/colcon-hardware-acceleration:
    type: git
    url: https://github.com/colcon/colcon-hardware-acceleration
    version: main
  acceleration/ros2_kria:
    type: git
    url: https://github.com/ros-acceleration/ros2_kria
    version: main
  acceleration/ros2acceleration:
    type: git
    url: https://github.com/ros-acceleration/ros2acceleration
    version: humble
  acceleration/vitis_common:
    type: git
    url: https://github.com/ros-acceleration/vitis_common
    version: humble
  acceleration/acceleration_examples:
    type: git
    url: https://github.com/ros-acceleration/acceleration_examples
    version: main
EOF
###################################################
# 4. import repos of KRS 1.0 release
###################################################
vcs import src --recursive < krs_humble.repos 

```
Build workspace

```bash
unset RMW_IMPLEMENTATION
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
sudo ls -l # Hack to give sudo access to shell, else build may hang.
colcon build --merge-install --packages-select acceleration_firmware_kr260 vitis_common colcon-hardware-acceleration ros2acceleration ros2_kria colcon-hardware-acceleration ament_vitis ament_acceleration
```

Add to bash.rc
```
source /opt/ros/humble/setup.bash
source /home/mpelka/krs_ws/install/setup.bash
```

## Build workspace on X86

Enter the 'krs_ws' workspace. Setup environment variable by adding the following line to your `~.bashrc`:
```bash 
export KRS_WS=/home/$USER/kria_depth_demo/krs_ws/
```

Install prerequisites (according to [Install KRS](https://xilinx.github.io/KRS/sphinx/build/html/docs/install.html))
```bash 
sudo apt-get install ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds* 
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv gcc-multilib
sudo apt-get -y install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
sudo apt-get install qemu-user-static
sudo apt-get install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-msgs

```

Install repositories that are part of [Kria Robotic Stack](https://xilinx.github.io/KRS/sphinx/build/html/index.html).

Note!
Before proceed with cross-compilation, make sure that RMW_IMPLEMENTATION is not set [#KRS/84](https://github.com/Xilinx/KRS/issues/97):
```bash 
unset RMW_IMPLEMENTATION
```

```bash 
cd $KRS_WS
vcs import src --recursive < krs_humble.repos 
```

Source Vitis and ROS 2 and build tools for x86.
Note that the building needs superuser access. 
```bash 
cd $KRS_WS
unset RMW_IMPLEMENTATION
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
sudo ls -l # Hack to give sudo access to shell, else build may hang.
colcon build --merge-install --packages-select acceleration_firmware_kr260 vitis_common colcon-hardware-acceleration ros2acceleration ros2_kria colcon-hardware-acceleration ament_vitis ament_acceleration
``` 

## Prepare system for cross compilation

```bash 
sudo ln -s $KRS_WS/install/../acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0 /usr/lib/aarch64-linux-gnu/libpython3.10.so -f

```
## Build accelerated node

With `colcon` select the build target (kr260):
```bash 
cd $KRS_WS
unset RMW_IMPLEMENTATION
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
source ./install/setup.bash  # Source KRS
export PATH="/usr/bin":$PATH
colcon acceleration select kr260
```
Build only the depth node (takes about half an hour on Ryzen 7 9700x):
```bash 
cd $KRS_WS
unset RMW_IMPLEMENTATION
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
source ./install/setup.bash  # Source KRS
export PATH="/usr/bin":$PATH
rm -r build-kr260-ubuntu/stereolbm_accel/
colcon build --executor sequential --event-handlers console_direct+ --build-base=build-kr260-ubuntu --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --cmake-args -DNOKERNELS=false --packages-select stereolbm_accel
```

Next, you need to copy to the board (assuming that Kria is configured as host in your `.ssh/config`, and the user name is 'ubuntu'):

```bash
cd $KRS_WS
scp -r install-kr260-ubuntu/lib/stereolbm_accel  ubuntu@kria:/home/ubuntu/
```

Next on the board (after `ssh kria`):
```bash
sudo cp -r /home/$USER/stereolbm_accel /usr/lib/firmware/xilinx
sudo xmutil listapps #Queries on target FW resource manager daemon of pre-built app bitstreams available on the platform and provides summary to CLI.
sudo xmutil unloadapp #Removes application bitstream. (Takes slot number, default 0)
sudo xmutil loadapp stereolbm_accel #Loads requested application configuration bitstream to programmable logic if the device is available.
```

Next, source KRS and run accelerated node:
```
source /home/$USER/krs_ws/install/setup.bash
cd /home/$USER/stereolbm_accel
./stereolbm_accel_tb
```

## DDS tunning
We have a peer-to-peer connection with `cyclone-dds`.

Please create a file with the DDS configuration on the x86 and Kria board.
This configuration assumes that peer addresses are:
 - 192.168.99.1 
 - 192.168.99.2
Do not forget to adjust the interface name, substitute 'enp46s0' for your network interface.
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">    
	<General>
		<Interfaces>
			<!--- Interface name below --->
			<NetworkInterface name="enp46s0"/>
		</Interfaces>
      	<AllowMulticast>default</AllowMulticast>
      	<MaxMessageSize>65500B</MaxMessageSize>
    	</General>
	<Internal>
		<SocketReceiveBufferSize min="10MB" max="default" />
		<Watermarks>
        		<WhcHigh>500kB</WhcHigh>
      		</Watermarks>
	</Internal>
	<Discovery>
		<Peers>
			<Peer address="192.168.99.1"/>
			<Peer address="192.168.99.2"/>
		</Peers>

      		<ParticipantIndex>auto</ParticipantIndex>
      		<MaxAutoParticipantIndex>1000</MaxAutoParticipantIndex>
    	</Discovery>
    </Domain>
</CycloneDDS>
```

Now, add reference to created file (e.g. `~/kria_cyclone.xml`) in `~/.bashrc`:
```bash 
export CYCLONEDDS_URI=file:///home/$USER/kria_cyclone.xml

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export ROS_DOMAIN_ID=34
```

Finally, set the network stack on x86 and Kria:
```
sudo vim /etc/sysctl.d/10-cyclone-max.conf
```
and insert:
```
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
net.core.rmem_max=2147483647
```

Reboot both x86 and Kria.

### Download simulator

The simulator is available under this repository [ROSCon2023Demo](https://github.com/RobotecAI/ROSCon2023Demo).

[Download prebuilt package for ROS 2 humble](https://robotecai-my.sharepoint.com/:u:/g/personal/michal_pelka_robotec_ai/EYMO9TR5EI9CrU8M2l2lPTwBWJ9xf4QGXQCONQ0THzncDQ?e=J8E3uX)

Install prerequisites:
```bash
sudo apt install ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-control-toolbox ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-ur-msgs ros-${ROS_DISTRO}-moveit-servo ros-${ROS_DISTRO}-moveit-visual-tools ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-pilz-industrial-motion-planner ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-ur-client-library ros-${ROS_DISTRO}-nav2-common ros-${ROS_DISTRO}-navigation2 python3-rosdep2 
```

Initialize rosdep
```bash
rosdep update
```

Create SIM_PACKAGE environment variable (adjust accordingly to the chosen directory)
```bash
export SIM_PACKAGE=~/rosconpkg
```

After downloading the package and unzipping it under ${SIM_PACKAGE} on X86 you need to build Simulator's ROS 2 workspace:
```bash
cd ${SIM_PACKAGE}/ros2_ws
rosdep install --ignore-src --from-paths src/Universal_Robots_ROS2_Driver -y
colcon build --symlink-install
```


### Running HIL simulation

On Kria:
```
source /home/${USER}/krs_ws/install/setup.bash
cd /home/${USER}/stereolbm_accel
./stereolbm_accel_tb
```

On x86:
```
source ${SIM_PACKAGE}/ros2_ws/install/setup.bash
${SIM_PACKAGE}/ROSCon2023DemoGamePackage/ROSCon2023Demo.GameLauncher -r_fullscreen=false -bg_ConnectToAssetProcessor=0 -r_width=2560 -r_height=1440 -r_resolutionMode=1
```
