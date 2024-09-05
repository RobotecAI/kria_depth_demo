# Kria depth demo

## Nomenclature

- KR260 - AMD board with SOM platform
- x68 - Host machine, PC or laptop

## Prerequisites
- x68 machine with Ubuntu 22.04 and plenty of free space (~300 Gb)

- Kria KR260 board with Ubuntu 22.04 \
  Refer to [Getting Started](https://www.amd.com/en/products/system-on-modules/kria/k26/kr260-robotics-starter-kit/getting-started/setting-up-the-sd-card-image.html)

- ROS 2 Humble installed on KR260 and x86. Refer to [Installation](https://docs.ros.org/en/humble/Installation.html)

## Install Vitis

Vitis 2022.1 needs to be installed on x86. Please download Vitis 2022.1 from [Download](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive.html).  I recommend to use [Xilinx_Unified_2022.1](https://www.xilinx.com/member/forms/download/xef.html?filename=Xilinx_Unified_2022.1_0420_0327.tar.gz). File is quite huge, so do not forget to check MD5 checksum before installation. 
Install Vitis at default location `/tools/Xilinx`. I've experience some freezing and needed to install those packages (according to recommendation found at [Support](https://support.xilinx.com/s/question/0D54U00005astbhSAA/vivado-gets-stuck-or-takes-more-than-1-to-15-days-in-final-processing-ie-generating-installed-device-list-when-trying-to-install-in-ubuntu-2204?language=en_US)):
```
sudo apt-get install libtinfo5 libncurses5
```

## Build workspace on x86


Enter to 'krs_ws' workspace. For convience setup envioroment variable:
```
export KRS_WS=/home/michal/kria_depth_demo/krs_ws/
```

Install prerequisits (according to [Install KRS](https://xilinx.github.io/KRS/sphinx/build/html/docs/install.html))
```
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
Before proceed with cross compilation, make sure that RMW_IMPLEMENTATION is not set [#KRS/84](https://github.com/Xilinx/KRS/issues/97):
```
unset $RMW_IMPLEMENTATION
```

```
cd $KRS_WS
vcs import src --recursive < krs_humble.repos 
```

Source Vitis and ROS 2 and build tools for x86.
Note that building needs super user acces. 
```
cd $KRS_WS
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
export PATH="/usr/bin":$PATH  # FIXME: adjust path for CMake 3.5+
sudo ls -l # Hack to give sudo access to shell, else build may hang.
colcon build --merge-install  # about 18 mins in an AMD Ryzen 5 PRO 4650G
```

## Prepare system for cross compilation

```
sudo ln -s ~/krs_ws/install/../acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0 /usr/lib/aarch64-linux-gnu/libpython3.10.so -f

```
## Build accelerated node


```
cd $KRS_WS
unset $RMW_IMPLEMENTATION
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
source ./install/setup.bash  # Source KRS
export PATH="/usr/bin":$PATH
colcon acceleration select kr260
```


```
cd $KRS_WS
unset $RMW_IMPLEMENTATION
source /tools/Xilinx/Vitis/2022.1/settings64.sh  # source Xilinx tools
source /opt/ros/humble/setup.bash  # Sources system ROS 2 installation.
source ./install/setup.bash  # Source KRS
export PATH="/usr/bin":$PATH

colcon build --executor sequential --event-handlers console_direct+ --build-base=build-kr260-ubuntu --install-base=install-kr260-ubuntu --merge-install --mixin kr260 --cmake-args -DNOKERNELS=false --packages-select stereolbm_accel
```
