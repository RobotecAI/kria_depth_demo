cmake_minimum_required(VERSION 3.5)
project(stereolbm_accel)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE Release)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Threads REQUIRED)
find_package(ament_vitis)

# add_executable(accelerated_vadd_new
#   src/host_new.cpp)
# target_include_directories(accelerated_vadd_new PUBLIC include)
# target_link_libraries(accelerated_vadd_new
#   ${OpenCL_LIBRARY}
#   pthread)

  find_package(vitis_common REQUIRED)
  find_package(OpenCL REQUIRED)

  # accelerated_vadd
  add_executable(stereolbm_accel_tb
    src/accelerated_node.cpp
    src/main.cpp
  )
  MESSAGE("env $ENV{XILINX_HLS}")
  target_include_directories(stereolbm_accel_tb PUBLIC 
	   include
	   $ENV{XILINX_HLS}/common/technology/autopilot
       $ENV{XILINX_HLS}/include
     /home/michal/krs_ws/acceleration/firmware/kr260/sysroots/aarch64-xilinx-linux/usr/include/xrt/
	   )

  target_link_libraries(stereolbm_accel_tb
    ${OpenCL_LIBRARY}
          uuid
          xrt_core xrt_coreutil

  )
  ament_target_dependencies(stereolbm_accel_tb
	  rclcpp
	  rclcpp_components 
	  std_msgs 
	  cv_bridge 
	  image_transport 
	  vitis_common  )

if (ROS_VITIS)
  # resize kernel
  vitis_acceleration_kernel(
    NAME stereolbm_accel
    FILE src/xf_stereolbm_accel.cpp
    CONFIG cfg/kr260.cfg
    INCLUDE
      include
      ${CMAKE_INSTALL_PREFIX}/include
      ${CMAKE_INSTALL_PREFIX}/include/vitis_common
    TYPE
      # sw_emu
      # hw_emu
      hw
    LINK
    PACKAGE
    )

endif()  # ROS_VITIS

  install(TARGETS
    stereolbm_accel_tb
    # accelerated_vadd_new
    DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
