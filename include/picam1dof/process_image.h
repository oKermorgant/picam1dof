#ifndef PROCESSIMAGE_H
#define PROCESSIMAGE_H

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <picam1dof/msg/cmd.hpp>
#include <picam1dof/srv/detect.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <image_transport/publisher.hpp>
#include <opencv2/core.hpp>

#include <picam1dof/color_detector.h>
#include <picam1dof/flow_detector.h>


namespace picam1dof
{

using picam1dof::msg::Cmd;
using picam1dof::srv::Detect;

enum class DetectMode {COLOR, FLOW};

class ProcessImage : public rclcpp::Node
{

public:
  ProcessImage(rclcpp::NodeOptions options);

private:

  // params
  double gain;
  OnSetParametersCallbackHandle::SharedPtr cb_handle;
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::TimerBase::SharedPtr refresh_timer;
  void process();

  DetectMode detect_mode = DetectMode::FLOW;
  rclcpp::Service<Detect>::SharedPtr detect_srv;

  Cmd cmd;
  rclcpp::Publisher<Cmd>::SharedPtr cmd_pub;

  image_transport::Subscriber im_sub;
  image_transport::Publisher im_pub;
  cv::Mat img;

  ColorDetector color_detector;
  FlowDetector flow_detector;



};
}

#endif
