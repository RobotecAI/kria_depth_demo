
#ifndef IMAGE_PROC_RESIZE_FPGA_HPP_
#define IMAGE_PROC_RESIZE_FPGA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <vitis_common/common/ros_opencl_120.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <xrt/xrt_bo.h>

class AcceleratedNode  : public rclcpp::Node
{
public:
  AcceleratedNode(const rclcpp::NodeOptions& options);

protected:

  int 			interpolation_;
  bool 			use_scale_;
  bool 			profile_;
  double 		scale_height_;
  double 		scale_width_;
  int 			height_;
  int 			width_;

  xrt::device device;
  xrt::kernel stereo_accel;
  xrt::bo left_bo;
  xrt::bo right_bo;
  xrt::bo params_bo;
  xrt::bo out_img_bo;

  constexpr  static int rows = 720;
  constexpr  static int cols = 1280;
  constexpr static size_t image_in_size_bytes = rows * cols * sizeof(unsigned char);
  constexpr static size_t image_out_size_bytes = rows * cols * sizeof(unsigned char);
  constexpr static size_t ParamCount = 4;
  using ParameterVector = std::array<unsigned char,ParamCount>;
  ParameterVector bm_state_params;
  constexpr static size_t vec_in_size_bytes = ParamCount * sizeof(unsigned char);
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

 
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_left;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_right;
 
 

  void connectCb();
  size_t get_msg_size(sensor_msgs::msg::Image::ConstSharedPtr image_msg);
  size_t get_msg_size(sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

  void imageCbLeft(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void imageCbRight(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void imageCbSync(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
                     const sensor_msgs::msg::Image::ConstSharedPtr& right_msg);


  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
  
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

private:

  cv_bridge::CvImagePtr cv_ptr_left;		//Stores input image from img_msg in  subscriber callback
  cv_bridge::CvImagePtr cv_ptr_right;		//Stores input image from img_msg in  subscriber callback
  cv::Mat  		result_hls;	// stores result after kernel execution on FPGA
  cv::Mat  		result_hls_8u;	// stores 8 bit result after kernel execution on FPGA
  cv_bridge::CvImage 	output_image; 	// Create CV image from result_hls, required to publish image msg.

  float baseline_; //Baseline between left and right camera
  sensor_msgs::msg::CameraInfo left_info; //Stores left camera info
  
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_left;	//Subscriber for left camera info
  
  void InitKernel();
  void ExecuteKernel();
};


#endif  // IMAGE_PROC_RESIZE_FPGA_HPP_
