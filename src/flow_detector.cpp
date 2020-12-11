#include <picam1dof/flow_detector.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>


namespace picam1dof
{

void drawRotatedRectangle(cv::Mat &img, const cv::RotatedRect &rect)
{
  static cv::Point2f vertices2f[4];
  rect.points(vertices2f);
  cv::line(img, vertices2f[0], vertices2f[1], {0,255,0});
  cv::line(img, vertices2f[1], vertices2f[2], {0,255,0});
  cv::line(img, vertices2f[2], vertices2f[3], {0,255,0});
  cv::line(img, vertices2f[3], vertices2f[0], {0,255,0});
}

bool FlowDetector::process(cv::Mat &img)
{
  static cv::Mat gray;
  static cv::Mat prevGray;
  static FlowPoint::Points points, prev_points, outlier_points;
  static cv::Size subPixWinSize(10,10), winSize(31,31);
  static cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

  bool update_cog(false);

  if(flow_points.size() < 3*MAX_POINTS/4)
  {
    int to_detect(MAX_POINTS - flow_points.size());
    FlowPoint::Points new_points;
    static cv::Mat track_mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    track_mask = 255;
    for(const auto &p: flow_points)
      cv::circle(track_mask, p, 10, 0, -1);
    cv::goodFeaturesToTrack(gray, new_points, to_detect, 0.01, 10, track_mask);
    cv::cornerSubPix(gray,  new_points, subPixWinSize, cv::Size(-1,-1), termcrit);
    for(const auto &point: new_points)
      flow_points.emplace_back(point);
    std::swap(prevGray, gray);

    if(to_detect == MAX_POINTS)
      return false;
  }

  if(!flow_points.empty())
  {
    std::vector<uchar> status;
    std::vector<float> err;
    if(prevGray.empty())
      gray.copyTo(prevGray);

    prev_points.resize(flow_points.size());
    std::copy(flow_points.begin(), flow_points.end(), prev_points.begin());
    cv::calcOpticalFlowPyrLK(prevGray, gray, prev_points, points, status, err, winSize,
                             3, termcrit, 0, 0.001);
    size_t i, k;
    const auto tracked(points.size());

    // update all flow points
    for( i = k = 0; i < tracked; i++)
      flow_points[i].update(points[i], status[i]);

    auto [outliers, valid] = FlowPoint::computeOutliers(flow_points); {}
    flow_points.resize(valid);

    if(outliers)
    {
    outlier_points.resize(outliers);
    std::copy(flow_points.begin(), flow_points.begin()+outliers, outlier_points.begin());
    static cv::RotatedRect rect;
    rect = cv::minAreaRect(outlier_points);
    cog = rect.center;
    drawRotatedRectangle(img, rect);
    update_cog = true;
    }

    for(const auto &point: flow_points)
      point.diplay(img);
  }
  cv::swap(prevGray, gray);
  return update_cog;
}
}

/*#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>

using namespace cv;

struct Listener
{
  Listener(ros::NodeHandle&nh) : itrans(nh)
  {
    sub = itrans.subscribe("/image", 1, &Listener::imRead, this);
  }
  void imRead(const sensor_msgs::ImageConstPtr &msg)
  {
    im = cv_bridge::toCvCopy(msg, "bgr8")->image;
    ok = true;
  }
  void get(cv::Mat &_im)
  {
    im.copyTo(_im);
  }

  bool ok = false;
  image_transport::Subscriber sub;
  cv::Mat im;
  image_transport::ImageTransport itrans;
};

Point2f point;
bool addRemovePt = false;


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "optic_flow");
  ros::NodeHandle nh;
  Listener listener(nh);

  ros::Rate loop(20);

  cv::namedWindow( "LK Demo", 1 );
   setMouseCallback( "LK Demo", onMouse, 0 );

     cv::Mat gray, prevGray, image, frame;
     std::vector<cv::Point2f> points[2];
     bool nightMode(false), needToInit(true);
     const int MAX_COUNT(500);
     Size subPixWinSize(10,10), winSize(31,31);
     TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

     while(ros::ok())
     {
       if(listener.ok)
       {

         listener.get(image);
         cvtColor(image, gray, cv::COLOR_BGR2GRAY);

         if( nightMode )
             image = cv::Scalar::all(0);

         if( needToInit )
         {
             // automatic initialization
             goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10);
             cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
             addRemovePt = false;
         }
         else if( !points[0].empty() )
         {
             std::vector<uchar> status;
             std::vector<float> err;
             if(prevGray.empty())
                 gray.copyTo(prevGray);
             calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                  3, termcrit, 0, 0.001);
             size_t i, k;
             for( i = k = 0; i < points[1].size(); i++ )
             {
                 if( addRemovePt )
                 {
                     if( norm(point - points[1][i]) <= 5 )
                     {
                         addRemovePt = false;
                         continue;
                     }
                 }

                 if( !status[i] )
                     continue;

                 points[1][k++] = points[1][i];
                 circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
             }
             points[1].resize(k);
         }

         if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
         {
             std::vector<Point2f> tmp;
             tmp.push_back(point);
             cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
             points[1].push_back(tmp[0]);
             addRemovePt = false;
         }

         needToInit = false;
         imshow("LK Demo", image);

         char c = (char)cv::waitKey(10);
         if( c == 27 )
             break;
         switch( c )
         {
         case 'r':
             needToInit = true;
             break;
         case 'c':
             points[0].clear();
             points[1].clear();
             break;
         case 'n':
             nightMode = !nightMode;
             break;
         }

         std::swap(points[1], points[0]);
         cv::swap(prevGray, gray);
       }
       ros::spinOnce();
       loop.sleep();
     }
}
*/
