// Copyright (c) 2021, Xilinx®.
// All rights reserved
//
// Inspired by the Vector-Add example.
// See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis
//

#include <chrono>  // NOLINT
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cpu_node.hpp"


using namespace std::chrono_literals;  // NOLINT

int main(int argc, char * argv[]) {
  // ROS 2 abstractions
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  rclcpp::NodeOptions node_options;
  auto node_accelerated				= std::make_shared<CPUNode>(node_options);

  executor.add_node(node_accelerated);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
