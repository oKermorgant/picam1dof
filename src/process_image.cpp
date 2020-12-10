#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

namespace picam1dof
{

class ProcessImage : public rclcpp::Node
{
public:
  ProcessImage(rclcpp::NodeOptions options) : rclcpp::Node("process_image", options)
  {
    /*process_type = declare_parameter<int>("type", 3);

    im_sub = image_transport::create_subscription(this, "image_raw", [&](const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {im_received = true;
     im = cv_bridge::toCvCopy(msg, "bgr8")->image;}, "raw");

    process_timer = create_wall_timer( 50ms,    // rate
                                            [&]()
                                            {process();});*/
  }


private:
 /* ImgProc im_proc;
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
};

}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(picam1dof::ProcessImage)
