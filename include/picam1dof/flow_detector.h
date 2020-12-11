#ifndef FLOWDETECTOR_H
#define FLOWDETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <picam1dof/flow_point.h>

namespace picam1dof
{

class FlowDetector
{
public:
    FlowDetector()
    {
      flow_points.reserve(MAX_POINTS);
      init();
    }

    void setThreshold(float thr) {FlowPoint::outlier_threshold = thr;}

    void init() {flow_points.clear();}

    bool process(cv::Mat &img);

    float x() const {return cog.x;}
    float y() const {return cog.y;}


protected:
  std::vector<FlowPoint> flow_points;
  cv::Point2f cog;
  static constexpr int MAX_POINTS = 200;

};
}

#endif
