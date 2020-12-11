#ifndef FLOWPOINT_H
#define FLOWPOINT_H

#include <opencv2/core.hpp>

namespace picam1dof
{

struct FlowPoint : public cv::Point2f
{
  using Points = std::vector<cv::Point2f>;
  using FlowPoints = std::vector<FlowPoint>;

  FlowPoint(float x = 0, float y = 0) : cv::Point2f(x, y) {}
  FlowPoint(const cv::Point2f &point) : cv::Point2f(point) {}

  cv::Point2f flow;
  bool valid = false;
  bool outlier = false;

  void update(const cv::Point2f &point, bool _valid = false)
  {
    valid = _valid;
    if(valid)
    {
      flow.x = point.x - x;
      flow.y = point.y - y;
      x = point.x;
      y = point.y;
    }
  }

  bool isOutlierFrom(const cv::Point2f &median_flow) const;

  void diplay(cv::Mat &img) const;

  static std::pair<size_t,size_t> computeOutliers(FlowPoints &points);
  static float outlier_threshold;
};
}





#endif
