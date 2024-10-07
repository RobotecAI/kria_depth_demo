/*
    Modification Copyright (c) 2023, Acceleration Robotics®
    Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
    Based on:
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2023, Xilinx®.
    \   \   \/    Author: Jasvinder Khurana <jasvinder.khurana@amd.com>
     \   \
     /   /        Licensed under the Apache License, Version 2.0 (the "License");
    /___/   /\    you may not use this file except in compliance with the License.
    \   \  /  \   You may obtain a copy of the License at
     \___\/\___\            http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

    Inspired by resize.cpp authored by Kentaro Wada, Joshua Whitley
*/

#include <memory>
#include <mutex>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>


#include <vitis_common/common/xf_headers.hpp>
#include <vitis_common/common/utilities.hpp>
//#include "image_proc/xf_resize_config.h"
#include "xf_stereolbm_config.h"

#include "tracetools_image_pipeline/tracetools.h"
#include <rclcpp/serialization.hpp>
#include "accelerated_node.hpp"


#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <chrono>



using namespace message_filters;

// Forward declaration of utility functions included at the end of this file
std::vector<cl::Device> get_xilinx_devices();
char* read_binary_file(const std::string &xclbin_file_name, unsigned &nb);



#define _TEXTURE_THRESHOLD_ 20
#define _UNIQUENESS_RATIO_ 100
#define _PRE_FILTER_CAP_ 31
#define _MIN_DISP_ 0

#define XCLBIN_NAME "/lib/firmware/xilinx/stereolbm_accel/stereolbm_accel.xclbin"
#define KERNEL_NAME "stereolbm_accel"

double GetExecutionTime(const struct timespec& start, const struct timespec& end)
{
  const double seconds = end.tv_sec - start.tv_sec;
  const double nanoseconds = end.tv_nsec - start.tv_nsec;
  return (seconds + nanoseconds * 1e-9) * 1e3;
}

void AcceleratedNode::InitKernel()
{

  device = xrt::device(0);
  std::cout << "Found Device" << device.get_info<xrt::info::device::name>() << std::endl;
  auto uuid = device.load_xclbin("stereolbm_accel.xclbin");
  std::cout << "Loaded xclbin with uuid" << uuid << std::endl;
  stereo_accel = xrt::kernel(device, uuid.get(), "stereolbm_accel");

  left_bo = xrt::bo (device, image_in_size_bytes, stereo_accel.group_id(0));
  right_bo = xrt::bo (device,image_in_size_bytes, stereo_accel.group_id(1));
  params_bo = xrt::bo (device,vec_in_size_bytes, stereo_accel.group_id(2));
  out_img_bo = xrt::bo (device,image_out_size_bytes, xrt::bo::flags::cacheable, stereo_accel.group_id(3));

  result_hls_8u.create(rows, cols, CV_8UC1);

}



void AcceleratedNode::ExecuteKernel()
{

  struct timespec begin_cpu, end_cpu, begin_hw, end_hw;
  struct timespec t_memin, t_krnl_exec;

  auto left_map = left_bo.map();
  auto right_map = right_bo.map();
  auto param_map = params_bo.map();


  bm_state_params[0] = _PRE_FILTER_CAP_;
  bm_state_params[1] = _UNIQUENESS_RATIO_;
  bm_state_params[2] = _TEXTURE_THRESHOLD_;
  bm_state_params[3] = _MIN_DISP_;


  assert(cv_ptr_left->image.rows == rows);
  assert(cv_ptr_left->image.cols == cols);
  assert(cv_ptr_right->image.rows == rows);
  assert(cv_ptr_right->image.cols == cols);


  //Load left image in bo_map

  clock_gettime(CLOCK_REALTIME, &begin_cpu);
  std::memcpy((unsigned char*)left_map, cv_ptr_left->image.data, image_in_size_bytes);
  std::memcpy((unsigned char*)right_map, cv_ptr_right->image.data, image_in_size_bytes);
  std::memcpy((unsigned char*)param_map,bm_state_params.data(), vec_in_size_bytes );
  clock_gettime(CLOCK_REALTIME, &end_cpu);

  if (thread_.joinable())
  {
    thread_.join();
  }

  //DMA from host to device
  clock_gettime(CLOCK_REALTIME, &begin_hw);
  left_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  right_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);
  params_bo.sync(XCL_BO_SYNC_BO_TO_DEVICE);

  clock_gettime(CLOCK_REALTIME, &t_memin);
  //Execute the HLS kernel
  auto run = stereo_accel(left_bo, right_bo, params_bo, out_img_bo, rows, cols );
  run.wait();
  clock_gettime(CLOCK_REALTIME, &t_krnl_exec);

  //Copy the output buffer to cv:mat to compare and save the img
  //std::memcpy(result_hls_8u.data, out_img_bo.map(), image_out_size_bytes);
  
  clock_gettime(CLOCK_REALTIME, &end_hw);
  //Total hw time
  const double cpu_time = GetExecutionTime(begin_cpu, end_cpu);
  const double hw_time = GetExecutionTime(begin_hw, end_hw);
  const double memin_time = GetExecutionTime(begin_hw, t_memin);
  const double krnl_exec_time = GetExecutionTime(t_memin, t_krnl_exec);
  const double memout_time = GetExecutionTime(t_krnl_exec, end_hw);

  std::cout.precision(3);
  std::cout << std::fixed;
  std::cout << "HLS time is " << hw_time  << "ms" << "\n";
  std::cout << "CPU time (data prep) is " << cpu_time  << "ms" << "\n";
  std::cout << "Memory in time " << memin_time << "ms | Kernel Execution time is " << krnl_exec_time << "ms | Memory out time is: " << memout_time << "ms" << "\n";

  result_hls_8u = cv::Mat(rows,cols, CV_8UC1, out_img_bo.map());
}


AcceleratedNode::AcceleratedNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: rclcpp::Node("AcceleratedNode", options)
{
  	rclcpp::QoS  qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // rclcpp::SensorDataQoS();

	const rmw_qos_profile_t my_qos =
	{
		RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		2,
		RMW_QOS_POLICY_RELIABILITY_RELIABLE,
		RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
		RMW_QOS_DEADLINE_DEFAULT,
		RMW_QOS_LIFESPAN_DEFAULT,
		RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
		RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
		false
	};

	interpolation_ 		= this->declare_parameter("interpolation", 1);
	use_scale_ 		= this->declare_parameter("use_scale", true);
	scale_height_ 		= this->declare_parameter("scale_height", 1.0);
	scale_width_ 		= this->declare_parameter("scale_width", 1.0);
	height_ 		= this->declare_parameter("height", -1);
	width_ 			= this->declare_parameter("width", -1);
	profile_ 		= this->declare_parameter("profile", true);
  baseline_ = this->declare_parameter("baseline", 0.025);
  depth_to_close_ = this->declare_parameter("depthToClose", 1.25);
  depth_warn_ = this->declare_parameter("depthWarn", 3.0);
  roi_start_x_ = this->declare_parameter("roi_start_x", 100);
  roi_end_x_ = this->declare_parameter("roi_end_x", 1280-100);

  roi_start_y_ = this->declare_parameter("roi_start_y", 0);
  roi_end_y_ = this->declare_parameter("roi_end_y", 720-50);
  assert(roi_start_x_ < roi_end_x_);
  assert(roi_start_y_ < roi_end_y_);
  
 	this->set_parameter(rclcpp::Parameter("use_sim_time", true));
	// Create image pub
	publisher_ 		= this->create_publisher<sensor_msgs::msg::Image>("disparity_map", qos_profile);

  roi_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("roi_image", qos_profile);


  // stop publihser
  publisher_distance_ = this->create_publisher<std_msgs::msg::Int32>("distance_error", qos_profile);
	subscriber_left.subscribe(this, "left", my_qos);
        subscriber_right.subscribe(this, "right", my_qos);

	sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), subscriber_left, subscriber_right));
	sync_->registerCallback(std::bind(&AcceleratedNode::imageCbSync, this, std::placeholders::_1, std::placeholders::_2));

  sub_info_left = this->create_subscription<sensor_msgs::msg::CameraInfo>("left/camera_info", 10, [this](sensor_msgs::msg::CameraInfo msg){
      RCLCPP_INFO(this->get_logger(), "Camera (left) Info received");
      left_info_ = msg;

      // compute disparities for warning and to close depths
      const auto &focal_length = left_info_.k[4];
      disparity_warn_ = std::round( focal_length * baseline_ / depth_warn_ );
      disparity_toClose_ = std::round( focal_length * baseline_ / depth_to_close_);
      std::cout << "Disparities :" << std::endl;
      std::cout << "\t Warn    : " << (int) disparity_warn_ << std::endl;
      std::cout << "\t ToClose : " << (int) disparity_toClose_ << std::endl;
      sub_info_left.reset();
  });

  cv::Mat roi_image = cv::Mat::zeros(rows, cols, CV_8UC1);
  for (int i = roi_start_y_; i < roi_end_y_; i++)
  {
      for (int j = roi_start_x_;  j < roi_end_x_ ;  j++)
      {
          roi_image.at<uint8_t>(i,j) = 255;
      }
  }
  // save the roi image for debug purposes
  cv::imwrite("roi_image.png", roi_image);

	InitKernel();
}

void AcceleratedNode::imageCbSync(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& right_msg)
{

	static int count = 0;
	static auto nodeStart = std::chrono::high_resolution_clock::now();
	auto now =  std::chrono::high_resolution_clock::now();
	count +=1;
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - nodeStart).count();
	double time = (duration) / count;
	std::cout << "Average frame to frame time: " << time << " ms " << "\n";
	try
	{
		cv_ptr_left 	= cv_bridge::toCvCopy(left_msg);
		cv_ptr_right 	= cv_bridge::toCvCopy(right_msg);
	}
	catch (cv_bridge::Exception & e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}


	ExecuteKernel();

  double focal_length = left_info_.k[4];
  std::thread t1 ([focal_length, result_hls_8u_copy = result_hls_8u, this](){
    // parallel, CPU postprocessing image
    
    #define POSTPROCESS
    #ifdef POSTPROCESS
    struct timespec begin, end;
    clock_gettime(CLOCK_REALTIME, &begin);
    int pointsToClose = 0;
    int pointsWarn = 0;


    for (int i = roi_start_y_; i < roi_end_y_; i++)
    {
        for (int j = roi_start_x_;  j < roi_end_x_ ;  j++)
        {
            auto &disparity = result_hls_8u_copy.at<uint8_t>(i,j);
            if (disparity  > disparity_toClose_ )
            {
              pointsToClose ++;
              continue;
            }
            if (disparity > disparity_warn_)
            {
              pointsWarn ++;
            }
        }
    }
    clock_gettime(CLOCK_REALTIME, &end);
    const double postprocess_time = GetExecutionTime(begin, end);
    std::cout << "Postprocess time is " << postprocess_time << "ms \n";
    std::cout << "pointsToClose " << pointsToClose << "\n";
    std::cout << "pointsToClose " << pointsWarn << "\n";

    const int onePercentPixel = rows*cols *0.01;

    std_msgs::msg::Int32 status;
    status.data = 0;

    if (pointsWarn > onePercentPixel )
    {
        status.data = 1;
    }

    if (pointsToClose > onePercentPixel )
    {
        status.data = 2;
    }

    publisher_distance_->publish(status);
    #endif
    cv_bridge::CvImage 	output_image;
    output_image.header     = cv_ptr_left->header;
    output_image.encoding   = "mono8";
    output_image.image = result_hls_8u_copy;
    sensor_msgs::msg::Image::SharedPtr msg_ = output_image.toImageMsg();
    publisher_->publish(*msg_.get());
  });
  std::swap(thread_ , t1);

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(AcceleratedNode)
