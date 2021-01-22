#include <picam1dof/face_detector.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
namespace picam1dof
{

using namespace cv;


FaceDetector::FaceDetector()
{
  const auto picam_share = ament_index_cpp::get_package_share_directory("picam1dof");

  nestedCascade.load( picam_share + "/haar/haarcascade_eye_tree_eyeglasses.xml");
  cascade.load(picam_share +  "/haar/haarcascade_frontalcatface.xml");
}

bool FaceDetector::process( Mat& im, double scale)
{
  std::vector<Rect> faces, faces2;
  static Mat gray, smallImg;

  cvtColor( im, gray, COLOR_BGR2GRAY ); // Convert to Gray Scale
  double fx = 1 / scale;

  // Resize the Grayscale Image
  resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR );
  equalizeHist( smallImg, smallImg );

  // Detect faces of different sizes using cascade classifier
  cascade.detectMultiScale( smallImg, faces, 1.1,
                            2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

  if(faces.size() == 0)
    return false;

  // keep largest one
  std::cout << faces.size() << std::endl;
  auto face = std::max_element(faces.begin(), faces.end(), [](Rect &r1, Rect &r2)
  {return r1.area() < r2.area();});

  x_ = face->x + 0.5*face->width;
  Mat smallImgROI;
  std::vector<Rect> nestedObjects;
  Point center;
  Scalar color = Scalar(255, 0, 0); // Color for Drawing tool
  int radius;

  double aspect_ratio = (double)face->width/face->height;
  if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
  {
    const float height_fact =1.3;
    const float width_fact = 0.9;

    center.x = cvRound((face->x + face->width*0.5)*scale);
    center.y = cvRound((face->y + face->height*0.5 * height_fact)*scale);
    radius = cvRound((face->width + face->height)*0.25*scale);

    ellipse(im, center, {(int) (width_fact*face->width), (int) (height_fact*face->height)}, 0, 0, 360, color, 2);


//    circle( im, center, radius, color, 3, 8, 0 );
  }
  else
    rectangle( im, cv::Point(face->x*scale, face->y*scale),
               cv::Point(cvRound((face->x + face->width-1)*scale),
                         cvRound((face->y + face->height-1)*scale)), color, 3, 8, 0);

  smallImgROI = smallImg(*face);

  // Detection of eyes int the input image
  nestedCascade.detectMultiScale( smallImgROI, nestedObjects, 1.1, 2,
                                  0|CASCADE_SCALE_IMAGE, Size(30, 30) );

  // Draw circles around eyes
  for ( size_t j = 0; j < nestedObjects.size(); j++ )
  {
    Rect nr = nestedObjects[j];
    center.x = cvRound((face->x + nr.x + nr.width*0.5)*scale);
    center.y = cvRound((face->y + nr.y + nr.height*0.5)*scale);
    radius = cvRound((nr.width + nr.height)*0.25*scale);
    circle( im, center, radius, color, 3, 8, 0 );
  }

  return true;
}









}
