cmake_minimum_required(VERSION 3.5)
project(stereolbm_cpu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(Threads REQUIRED)


# add_executable(cpuerated_vadd_new
#   src/host_new.cpp)


  # cpuerated_vadd
  add_executable(stereolbm_cpu_tb
    src/cpu_node.cpp
    src/main.cpp
  )

  target_include_directories(stereolbm_cpu_tb PUBLIC include)

  target_link_libraries(stereolbm_cpu_tb
    ${OpenCL_LIBRARY}
    pthread
  )
  ament_target_dependencies(stereolbm_cpu_tb
	  rclcpp
	  rclcpp_components 
	  std_msgs 
	  cv_bridge 
	  image_transport
      OpenCV
  )


ament_package()
