#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <picam1dof/process_image.h>

using  namespace std::chrono_literals;

namespace picam1dof
{

ProcessImage::ProcessImage(rclcpp::NodeOptions options) : rclcpp::Node("process_image", options)
{
  // control gain
  gain = declare_parameter("gain", 1.5);

  // image I/O
  im_sub = image_transport::create_subscription(this, "image", [&](const sensor_msgs::msg::Image::ConstSharedPtr &msg)
  {img = cv_bridge::toCvCopy(msg, "bgr8")->image;}, "raw");
  im_pub = image_transport::create_publisher(this, "image_proc");

  cmd_pub = create_publisher<Cmd>("angle_cmd", 10);
  cmd.mode = cmd.VELOCITY;

  refresh_timer = create_wall_timer( 50ms,    // rate
                                     [&]()
  {process();});

  detect_srv = create_service<Detect>("detect",
                                      [&](const Detect::Request::SharedPtr req, const Detect::Response::SharedPtr res)
  {
    if(req->mode == req->FLOW)
      detect_mode = DetectMode::FLOW;
    else
    {
      flow_detector.init();
      if(req->mode == req->COLOR)
      {
        detect_mode = DetectMode::COLOR;
        color_detector.detectColor(req->rgb[0], req->rgb[1], req->rgb[2]);
        (void) res;
      }
      else
      {
        detect_mode = DetectMode::FACE;
      }
    }
  });

  // defaut Saturation / Value thresholds for color detection
  color_detector.setValue(declare_parameter("value_threshold", 120));
  color_detector.setValue(declare_parameter("saturation_threshold", 150));
  color_detector.fitCircle();

  // defaut outlier threshold
  flow_detector.setThreshold(declare_parameter("outlier_threshold", 50.f));

  cb_handle = add_on_set_parameters_callback(
        std::bind(&ProcessImage::parametersCallback, this, std::placeholders::_1));
}

void ProcessImage::process()
{
  static sensor_msgs::msg::Image im_msg;

  if(img.empty())
    return;

  float x;
  bool success;
  switch (detect_mode) {
  case DetectMode::FLOW:
    success = flow_detector.process(img);
    x = flow_detector.x();
    break;
  case DetectMode::COLOR:
    success = color_detector.process(img);
    x = color_detector.x();
    break;
  default:
    success = face_detector.process(img);
    x = face_detector.x();
    break;
  }

  cv::imshow("Processed", img);
  cv::waitKey(1);

  cv_bridge::CvImage(im_msg.header, "bgr8", img).toImageMsg(im_msg);
  im_msg.header.stamp = now();

  im_pub.publish(im_msg);

  if(!success)
    return;

  // normalize and publish vel cmd
  x = std::isnan(x) ? 0 : x/img.cols-.5;
  cmd.cmd = gain * x;
  cmd_pub->publish(cmd);
}

rcl_interfaces::msg::SetParametersResult ProcessImage::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for(const auto &parameter: parameters)
  {
    const auto &name(parameter.get_name());
    const auto &type(parameter.get_type());
    if(name == "value_threshold" || name == "saturation_threshold")
    {
      if(type == rclcpp::ParameterType::PARAMETER_INTEGER && parameter.as_int() > 0 && parameter.as_int() < 256)
      {
        if(name == "value_threshold")
          color_detector.setValue(parameter.as_int());
        else
          color_detector.setSaturation(parameter.as_int());
      }
      else
      {
        result.successful = false;
        result.reason = name + " has to be between 0-255";
      }
    }
    else if(name == "gain")
    {
      if(type == rclcpp::ParameterType::PARAMETER_DOUBLE && parameter.as_double() > 0)
        gain = parameter.as_double() ;
      else
      {
        result.successful = false;
        result.reason = "gain has to be positive";
      }
    }
    else  // outlier threshold
    {
      if(type == rclcpp::ParameterType::PARAMETER_DOUBLE && parameter.as_double() > 0)
        flow_detector.setThreshold(parameter.as_double());
      else
      {
        result.successful = false;
        result.reason = "outlier threshold has to be positive";
      }
    }
  }
  return result;
}


/*private:
  ImgProc im_proc;
  int process_type;
  image_transport::Subscriber im_sub;
  cv::Mat im;
  bool im_received = false;

  rclcpp::TimerBase::SharedPtr process_timer;

  void process()
  {
    if(!im_received)
      return;

    switch (process_type)
    {
    case 1:
      im_proc.BGRHSV(im);
      break;

    case 2:
      im_proc.Contours(im);
      break;
    case 3:
      im_proc.Flow(im);
    default:
      1;
    }
    cv::waitKey(1);
  }*/

}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(picam1dof::ProcessImage)
