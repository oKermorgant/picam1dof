#ifndef COLORDETECTOR_H
#define COLORDETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace picam1dof
{

class ColorDetector
{
public:
  ColorDetector(int r=255, int g=0, int b=0)
  {
    detectColor(r, g, b);
  }

  // tuning
  void setSaturation(int sat)
  {
    sat_ = sat;
  }
  void setValue(int value)
  {
    val_ = value;
  }
  void detectColor(int r, int g, int b);
  inline void fitCircle() {fit_circle_ = true;}

  // processing functions
  std::vector<cv::Point> findMainContour(const cv::Mat &_im);

  bool process(cv::Mat &im);

  // get last info
  inline float x() const {return x_;}
  inline float y() const {return y_;}
  inline float area() const {return area_;}

protected:    
  float x_=0, y_=0, area_=0;
  std::array<std::pair<int,int>, 2> hue_;
  int sat_=130, val_=95;
  cv::Scalar ccolor;
  bool fit_circle_ = false;
};

}

#endif // COLORDETECTOR_H
