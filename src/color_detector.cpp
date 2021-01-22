
#include <picam1dof/color_detector.h>
#include <iostream>
namespace picam1dof
{

using std::vector;

void ColorDetector::detectColor(int r, int g, int b)
{
  ccolor = {float(b),float(g),float(r)};

  // convert color to HSV
  const float cmax = std::max(r, std::max(g,b));
  const float cmin = std::min(r, std::min(g,b));
  const float d = cmax - cmin;

  int h = 0;
  if(d)
  {
    if(cmax == r)
      h = 30*(fmod((g-b)/d,6));
    else if(cmax == g)
      h = 30*((b-r)/d + 2);
    else
      h = 30*((r-g)/d + 4);
  }

  // build inRange bounds for hue
  int hthr = 10;
  hue_[0] = {std::max(h-hthr,0),std::min(h+hthr,179)};
  hue_[1] = {0,0};

  // other segmentation for h
  if(h < hthr)
    hue_[1] = {179+h-hthr,179};
  else if(h+hthr > 179)
    hue_[1] = {0, h+hthr-179};
}


std::vector<cv::Point> ColorDetector::findMainContour(const cv::Mat &im)
{
  static cv::Mat im_work, seg1, seg2;

  cv::cvtColor(im, im_work, cv::COLOR_BGR2HSV);
  cv::GaussianBlur(im_work, im_work, cv::Size(9,9), 2);

  // segment for detection of given RGB (from Hue)
  cv::inRange(im_work, cv::Scalar(hue_[0].first, sat_, val_),
      cv::Scalar(hue_[1].second, 255, 255), seg1);
  // add for 2nd detection if near red
  if(hue_[1].first != hue_[1].second)
  {
    cv::inRange(im_work, cv::Scalar(hue_[1].first, sat_, val_),
        cv::Scalar(hue_[1].second, 255, 255), seg2);
    seg1 += seg2;
  }

  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours( seg1, contours, hierarchy, CV_RETR_CCOMP,
                    CV_CHAIN_APPROX_SIMPLE);

  // pop all children
  bool found = true;
  while(found)
  {
    found = false;
    for(unsigned int i=0;i<hierarchy.size();++i)
    {
      if(hierarchy[i][3] > -1)
      {
        found = true;
        hierarchy.erase(hierarchy.begin()+i,hierarchy.begin()+i+1);
        contours.erase(contours.begin()+i, contours.begin()+i+1);
        break;
      }
    }
  }

  if(contours.size())
  {
    // get largest contour
    auto largest = std::max_element(
          contours.begin(), contours.end(),
          [](const vector<cv::Point> &c1, const vector<cv::Point> &c2)
    {return cv::contourArea(c1) < cv::contourArea(c2);});
    int idx = std::distance(contours.begin(), largest);

    return contours[idx];
  }
  return std::vector<cv::Point>();
}

bool ColorDetector::process(cv::Mat &im)
{
  auto contour = findMainContour(im);

  if(!contour.size())
    return false;

  if(fit_circle_)
  {
    cv::Point2f pt;float radius;
    cv::minEnclosingCircle(contour, pt, radius);

    // filter output
    x_ = .5*(x_ + pt.x);
    y_ = .5*(y_ + pt.y);
    area_ = .5*(area_ + radius*radius*M_PI);

    cv::circle(im, pt, radius, ccolor, 2);
  }
  else
  {
    cv::Moments m = cv::moments(contour, false);
    // filter output
    x_ = 0.5*(x_ + m.m10/m.m00);
    y_ = 0.5*(y_ + m.m01/m.m00);
    area_ = 0.5*(area_ + m.m00);

    std::vector<std::vector<cv::Point>> conts(1, contour);
    cv::drawContours(im, conts, 0, ccolor, 2);
  }
  return true;
}
}




/*#include <ecn_common/color_detector.h>
#include <math.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>


namespace ecn
{

using std::vector;



}
*/
