#include <picam1dof/flow_point.h>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <iostream>


/*template<typename T=picam1dof::FlowPoint>
struct cv::Type
{ enum { value = cv::DataType<float>::type }; };
*/
namespace picam1dof
{

using Point = cv::Point2f;
float FlowPoint::outlier_threshold = 20.;
bool compareFlowX(const FlowPoint &p1, const FlowPoint &p2)
{
  return p1.flow.x < p2.flow.x;
}
bool compareFlowY(const FlowPoint &p1, const FlowPoint &p2)
{
  return p1.flow.y < p2.flow.y;
}
bool compareX(const FlowPoint &p1, const FlowPoint &p2)
{
  return p1.x < p2.x;
}
bool compareY(const FlowPoint &p1, const FlowPoint &p2)
{
  return p1.y < p2.y;
}
Point medianPoint(FlowPoint::FlowPoints &points, const FlowPoint::FlowPoints::iterator &valid, size_t median_index)
{
  Point median;

  std::nth_element(points.begin(), points.begin()+median_index, valid, compareX);
  median.x = points[median_index].flow.x;

  std::nth_element(points.begin(), points.begin()+median_index, valid, compareY);
  median.y = points[median_index].flow.y;
  return median;
}
Point medianFlow(FlowPoint::FlowPoints &points, const FlowPoint::FlowPoints::iterator &valid, size_t median_index)
{
  Point median;

  std::nth_element(points.begin(), points.begin()+median_index, valid, compareFlowX);
  median.x = points[median_index].flow.x;

  std::nth_element(points.begin(), points.begin()+median_index, valid, compareFlowY);
  median.y = points[median_index].flow.y;
  return median;
}

inline float square(float f){return f*f;}
inline float squaredDist(const Point &p1, const Point &p2)
{
  return square(p1.x-p2.x)  + square(p1.y-p2.y);
}
inline float meanDist(const FlowPoint::FlowPoints &points, size_t max_idx, const Point &ref)
{
  float mean(0);
  for(size_t i = 0; i < max_idx; ++i)
    mean += squaredDist(points[i], ref);
  return mean/max_idx;
}

void FlowPoint::diplay(cv::Mat &img) const
{
  cv::Scalar color{0,255,0};
  if(!outlier)
    color = {0,0,255};

  cv::line(img, *this, *this-flow, color, valid+1);
}

inline bool FlowPoint::isOutlierFrom(const Point &median_flow) const
{
  return squaredDist(flow, median_flow) > outlier_threshold;
}

void setOutlier(FlowPoint &point) {point.outlier = true;}
void setNotOutlier(FlowPoint &point) {point.outlier = false;}
bool isOutlier(const FlowPoint &point) {return point.outlier;}

std::pair<size_t, size_t> FlowPoint::computeOutliers(FlowPoints &points)
{
  // get all valid points at the front
  auto valid_it = std::partition(points.begin(), points.end(), [](const auto &point)
  {return point.valid;});

  // get median flow
  const auto valid(std::distance(points.begin(), valid_it));
  const Point median_flow(medianFlow(points, valid_it, valid/2));

  // analyze outliers
  auto outliers_it = std::partition(points.begin(), valid_it, [&](const auto &point){return point.isOutlierFrom(median_flow);});
  auto outlier_count = std::distance(points.begin(), outliers_it);



  if(outlier_count < 5)
  {
    // just keep from last processing
    outliers_it = std::partition(points.begin(), valid_it, isOutlier);
    outlier_count = std::distance(points.begin(), outliers_it);
  }

  // refine from median
  const auto median(medianPoint(points, outliers_it, outlier_count/2));
  const auto twice_mean(2*meanDist(points, outlier_count, median));
  outliers_it = std::partition(points.begin(), outliers_it,
                               [&](const auto &point){return squaredDist(point, median) < twice_mean;});
  std::for_each(points.begin(), outliers_it, setOutlier);
  std::for_each(outliers_it, valid_it, setNotOutlier);
  outlier_count = std::distance(points.begin(), outliers_it);

  return {outlier_count, valid};
}

}
