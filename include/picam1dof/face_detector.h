#ifndef FACE_DETECTOR_H
#define FACE_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>

namespace picam1dof
{


class FaceDetector
{
public:
  FaceDetector();

  bool process(cv::Mat &im, double scale = 1);
  double x() const {return x_;}


protected:
  double x_;


  cv::CascadeClassifier cascade, nestedCascade;
};



}



#endif // FACE_DETECTOR_H
