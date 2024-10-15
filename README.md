# Kria depth demo

This demo presents a usecase for FPGA SOC Kria K260.
FPGA (Field programmable gateway array) is technology that allows to create custom hardware to fullfil complex task in parallel and deterministic manner.
In contrast to programming processor, designing FPGA solution, one is not limited to architecture constrains of given platform.

In this simple project number of technologies are utilized:
- [Kria 260](https://xilinx.github.io/kria-apps-docs/home/build/html/index.html) \
It is a system-on-chip solution that can be integrated in robotics design.
- [Vitis](https://www.amd.com/en/products/software/adaptive-socs-and-fpgas/vitis.html)
It is a platform to develop solutions deployed in FPGA.
- [Kria Robotics stack](https://xilinx.github.io/KRS/sphinx/build/html/index.html)
It is a ROS 2 set of tools, nodes and libraries to depoloy harware-accelerated solutions to Kria SOMs.


## Nomenclature

- [KR260](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit.html) - AMD board with SOM platform
- x68 - Host machine, PC, or laptop

## Prerequisites
- x68 machine with Ubuntu 22.04 and plenty of free space (~300 Gb)

- Kria KR260 board with Ubuntu 22.04 \
  Refer to [Getting Started](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit/getting-started/setting-up-the-sd-card-image.html)

- ROS 2 Humble installed on KR260 and x86. Refer to [Installation](https://docs.ros.org/en/humble/Installation.html)

## Install Vitis

Vitis 2022.1 needs to be installed on x86. Please download Vitis 2022.1 from [Download](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive.html).  I recommend to use [Xilinx_Unified_2022.1](https://www.xilinx.com/member/forms/download/xef.html?filename=Xilinx_Unified_2022.1_0420_0327.tar.gz). File is quite huge, so do not forget to check MD5 checksum before installation. 
Install Vitis at default location `/tools/Xilinx`. I've experienced some freezing and needed to install those packages (according to recommendation found at [Support](https://support.xilinx.com/s/question/0D54U00005astbhSAA/vivado-gets-stuck-or-takes-more-than-1-to-15-days-in-final-processing-ie-generating-installed-device-list-when-trying-to-install-in-ubuntu-2204?language=en_US)):
```bash 
sudo apt-get install libtinfo5 libncurses5
```


## Build workspace on KRIA

Note : It is not tested yet

```bash
sudo apt-get -y install curl build-essential libssl-dev git wget \
                          ocl-icd-* opencl-headers python3-vcstool \
                          python3-colcon-common-extensions python3-colcon-mixin \
                          kpartx u-boot-tools pv gcc-multilib
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
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /home/mpelka/krs_ws/install/setup.bash
```

## Build workspace on x86

Enter the 'krs_ws' workspace. For convience setup environment variable:
```bash 
export KRS_WS=/home/$USER/kria_depth_demo/krs_ws/
```

Install prerequisits (according to [Install KRS](https://xilinx.github.io/KRS/sphinx/build/html/docs/install.html))
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
Note that the building needs super user access. 
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

```bash 
cd $KRS_WS
unset RMW_IMPLEMENTATION
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
source ./install/setup.bash  # Source KRS
export PATH="/usr/bin":$PATH
colcon acceleration select kr260
```

```bash 
cd $KRS_WS
unset RMW_IMPLEMENTATION
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
source ./install/setup.bash  # Source KRS
export PATH="/usr/bin":$PATH

colcon build --executor sequential --event-handlers console_direct+ --build-base=build-kr260-ubuntu --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --cmake-args -DNOKERNELS=false

```
To build only depth node:
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

Next, you need to copy to board (assuming that kria is configured as host in your `.ssh/config`, and user name is 'ubuntu'):
```bash
cd $KRS_WS
scp -r install-kr260-ubuntu/lib/stereolbm_accel  kria:/home/ubuntu/
```

Next on the board (after `ssh kria`):
```bash
sudo cp -r /home/$USER/stereolbm_accel /usr/lib/firmware/xilinx
sudo xmutil listapps #Queries on target FW resource manager daemon of pre-built app bitstreams available on the platform and provides summary to CLI.
sudo xmutil unloadapp #Removes application bitstream. (Takes slot number, default 0)
sudo xmutil loadapp stereolbm_accel #Loads requested application configuration bitstream to programmable logic if the device is available.
```


Rest can be done as user
```
source /home/$USER/krs_ws/install/setup.bash
cd /home/$USER/stereolbm_accel
./stereolbm_accel_tb
```

## DDS tunning
We have a peer-to-peer connection with `cyclone-dds`.

Please create a file with the dds configuration on both the x86 and Kria board.
This configuration assumes that peers addresses are:
 - 192.168.99.1 
 - 192.168.99.2
Do not forget to adjust inteface name:
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

Next add reference created file (e.g. `~/kria_cyclone.xml`) in `~/.bashrc`:
```bash 
export CYCLONEDDS_URI=file:///home/$USER/kria_cyclone.xml

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export ROS_DOMAIN_ID=34
```

Finally set the network stack on both x86 and Kria:


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

### Simulator 
Grab project from [ROSCon2023Demo/test_stereo](https://github.com/RobotecAI/ROSCon2023Demo/tree/mp/test_stereo).
Make sure to use `test_stereo` and build following instructions.

### Running system

On Kria:
```
source /home/${USER}/krs_ws/install/setup.bash
cd /home/${USER}/stereolbm_accel
./stereolbm_accel_tb
```

On x86:
```
RosCon2023.GameLaucher -r_fullscreen=false -bg_ConnectToAssetProcessor=0 -r_width=2560 -r_height=1440 -r_resolutionMode=1
```
