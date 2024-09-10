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

void AcceleratedNode::InitKernel()
{
	cl_int err;
	unsigned fileBufSize;

	// Get the device:
	std::vector<cl::Device> devices 	= get_xilinx_devices();
	cl::Device device 			= devices[0];


	// Context, command queue and device name:
	OCL_CHECK(err, context_ 		= new cl::Context(device, NULL, NULL, NULL, &err));
	OCL_CHECK(err, queue_ 			= new cl::CommandQueue(*context_, device, CL_QUEUE_PROFILING_ENABLE, &err));
	OCL_CHECK(err, std::string device_name 	= device.getInfo<CL_DEVICE_NAME>(&err));
	
	std::cout << "INFO: Device found - " << device_name << std::endl;

	char* fileBuf 				= read_binary_file(XCLBIN_NAME, fileBufSize);

	cl::Program::Binaries bins{{fileBuf, fileBufSize}};
	devices.resize(1);
	OCL_CHECK(err, cl::Program program(*context_, devices, bins, NULL, &err));

	// Create a kernel:
	OCL_CHECK(err, krnl_ 			= new cl::Kernel(program, KERNEL_NAME, &err));

}

void AcceleratedNode::ExecuteKernel()
{
	int rows = 720;
	int cols = 1280;

	// OpenCL section:
	cl_int err;

	// OpenCL section:
	std::vector<unsigned char> bm_state_params(4);
	bm_state_params[0] = _PRE_FILTER_CAP_;
	bm_state_params[1] = _UNIQUENESS_RATIO_;
	bm_state_params[2] = _TEXTURE_THRESHOLD_;
	bm_state_params[3] = _MIN_DISP_;

	size_t image_in_size_bytes = rows * cols * sizeof(unsigned char);
	size_t vec_in_size_bytes = bm_state_params.size() * sizeof(unsigned char);
	size_t image_out_size_bytes = rows * cols * sizeof(unsigned char);

	// result_hls.create(rows, cols, CV_16UC1);
	result_hls_8u.create(rows, cols, CV_8UC1);

	// Allocate the buffers:
	OCL_CHECK(err, cl::Buffer buffer_inImageL(*context_, CL_MEM_READ_ONLY, image_in_size_bytes, NULL, &err));
	OCL_CHECK(err, cl::Buffer buffer_inImageR(*context_, CL_MEM_READ_ONLY, image_in_size_bytes, NULL, &err));
	OCL_CHECK(err, cl::Buffer buffer_inVecBM(*context_, CL_MEM_READ_ONLY, vec_in_size_bytes, NULL, &err));
	OCL_CHECK(err, cl::Buffer buffer_outImage(*context_, CL_MEM_WRITE_ONLY, image_out_size_bytes, NULL, &err));

	// Set kernel arguments:
	OCL_CHECK(err, err = krnl_->setArg(0, buffer_inImageL));
	OCL_CHECK(err, err = krnl_->setArg(1, buffer_inImageR));
	OCL_CHECK(err, err = krnl_->setArg(2, buffer_inVecBM));
	OCL_CHECK(err, err = krnl_->setArg(3, buffer_outImage));
	OCL_CHECK(err, err = krnl_->setArg(4, rows));
	OCL_CHECK(err, err = krnl_->setArg(5, cols));


	auto start_time = std::chrono::high_resolution_clock::now();
	//std::cout << "JASSI DEBUG 3  " << std::endl;
	// Initialize the buffers:
	cl::Event event;

	OCL_CHECK(err, queue_->enqueueWriteBuffer(buffer_inImageL,     // buffer on the FPGA
				CL_TRUE,             // blocking call
				0,                   // buffer offset in bytes
				image_in_size_bytes, // Size in bytes
				cv_ptr_left->image.data,       // Pointer to the data to copy
				nullptr, &event));

	//std::cout << "JASSI DEBUG 4 " << std::endl;
	OCL_CHECK(err, queue_->enqueueWriteBuffer(buffer_inImageR,     // buffer on the FPGA
				CL_TRUE,             // blocking call
				0,                   // buffer offset in bytes
				image_in_size_bytes, // Size in bytes
				cv_ptr_right->image.data,      // Pointer to the data to copy
				nullptr, &event));

	OCL_CHECK(err, queue_->enqueueWriteBuffer(buffer_inVecBM,         // buffer on the FPGA
				CL_TRUE,                // blocking call
				0,                      // buffer offset in bytes
				vec_in_size_bytes,      // Size in bytes
				bm_state_params.data(), // Pointer to the data to copy
				nullptr, &event));

	auto tsBufferWrite1 = std::chrono::high_resolution_clock::now();
	
	//std::cout << "JASSI DEBUG 5  " << std::endl;
	// Execute the kernel:
	OCL_CHECK(err, err = queue_->enqueueTask(*krnl_));
	// Make it blocking by waiting for all commands to complete
	OCL_CHECK(err, err = queue_->finish());

	auto tsKernel2 = std::chrono::high_resolution_clock::now();

	// std::cout << "JASSI DEBUG 6  " << std::endl;
	// Copy Result from Device Global Memory to Host Local Memory
	queue_->enqueueReadBuffer(buffer_outImage, // This buffers data will be read
			CL_TRUE,         // blocking call
			0,               // offset
			image_out_size_bytes,
			result_hls_8u.data, // Data will be stored here
			nullptr, nullptr);

	auto tsBufferRead3 = std::chrono::high_resolution_clock::now();

	{
		using namespace std::chrono;
		auto durationWrite = duration_cast<microseconds>(tsBufferWrite1 - start_time).count();
		auto durationKernel = duration_cast<microseconds>(tsKernel2 - tsBufferWrite1).count();
		auto durationRead = duration_cast<microseconds>(tsBufferRead3 - tsKernel2).count();
		printf(" Write : %f, Kernel : %f, Read : %f,   ms \n", 0.001f*durationWrite, 0.001f*durationKernel, 0.001f*durationRead );
	}
    
//	std::cout << "JASSI DEBUG 7  " << std::endl;
	// Clean up:
	queue_->finish();


	// Convert 16U output to 8U output:
<<<<<<< HEAD
	result_hls.convertTo(result_hls_8u, CV_8U, 1.f/16.f);
=======
	//result_hls.convertTo(result_hls_8u, CV_8U, (256.0 / NO_OF_DISPARITIES) / (16.));


>>>>>>> d3c04ba (Switched in FPGA to convert to 8 bit)
	output_image.header     = cv_ptr_left->header;
    output_image.encoding   = "mono8";
    output_image.image = cv::Mat{ rows,cols, CV_8U, result_hls_8u.data};

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

 	this->set_parameter(rclcpp::Parameter("use_sim_time", true));
	// Create image pub
	publisher_ 		= this->create_publisher<sensor_msgs::msg::Image>("disparity_map", qos_profile); 

	subscriber_left.subscribe(this, "left", my_qos);
        subscriber_right.subscribe(this, "right", my_qos);

	sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), subscriber_left, subscriber_right));
	sync_->registerCallback(std::bind(&AcceleratedNode::imageCbSync, this, std::placeholders::_1, std::placeholders::_2));


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
	// Get subscribed image
	//-------------------------------------------------------------------------------------------------------
	

	// Converting ROS image messages to OpenCV images, for diggestion
	// with the Vitis Vision Library
	// see http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

	//std::cout << "INside imageCbSync function " << std::endl;

	try 
	{
		cv_ptr_left 	= cv_bridge::toCvCopy(left_msg);
		cv_ptr_right 	= cv_bridge::toCvCopy(right_msg);
		
		// cv::cvtColor(cv_ptr_left->image, cv_ptr_left->image, cv::COLOR_BGR2GRAY);
		// cv::cvtColor(cv_ptr_right->image, cv_ptr_right->image, cv::COLOR_BGR2GRAY);
		


	} 
	catch (cv_bridge::Exception & e) 
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}



	// Process Image and run the accelerated Kernel
	//-------------------------------------------------------------------------------------------------------

	//this->get_parameter("profile", profile_);  // Update profile_

	// std::cout << "JASSI DEBUG 2  " << std::endl;
	ExecuteKernel();


 	sensor_msgs::msg::Image::SharedPtr msg_ = output_image.toImageMsg();

	// Publish processed image
	//-------------------------------------------------------------------------------------------------------
	publisher_->publish(*msg_.get());

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(AcceleratedNode)
