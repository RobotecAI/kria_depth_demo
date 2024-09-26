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

#include <opencv2/calib3d.hpp>



#include <rclcpp/serialization.hpp>
#include "cpu_node.hpp"


#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <chrono>
using namespace message_filters;


char* read_binary_file(const std::string &xclbin_file_name, unsigned &nb);



CPUNode::CPUNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: rclcpp::Node("CPUNode", options)
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
	sync_->registerCallback(std::bind(&CPUNode::imageCbSync, this, std::placeholders::_1, std::placeholders::_2));

}

void CPUNode::imageCbSync(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
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

	} 
	catch (cv_bridge::Exception & e) 
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	// Process the image
        //-------------------------------------------------------------------------------------------------------
        cv::Ptr<cv::StereoBM> stereobm = cv::StereoBM::create(32, 11);

        cv::Mat output_image;
        stereobm->compute(cv_ptr_left->image, cv_ptr_right->image, output_image);

        for (int i = 0; i < output_image.rows; i++)
        {
            for (int j = 0; j < output_image.cols; j++)
            {
                output_image.at<short>(i, j) = output_image.at<short>(i, j) >> 4;
            }
        }
        cv::Mat filteredImage;
        cv::medianBlur(output_image, filteredImage, 5);
        cv_bridge::CvImage cv_image;
        cv_image.encoding = sensor_msgs::image_encodings::MONO16;
        cv_image.image = filteredImage;

	// Publish processed image
	//-------------------------------------------------------------------------------------------------------
	publisher_->publish( *cv_image.toImageMsg());

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component
// to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(CPUNode)
