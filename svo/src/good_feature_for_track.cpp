/*  
 *  Aug 7 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  detect and show good features for track 
 *
 * */

#include <string.h>
// #include <svo/global.h>
// #include <svo/config.h>
// #include <svo/frame.h>
// #include <svo/feature_detection.h>
// #include <svo/depth_filter.h>
// #include <svo/feature.h>
#include <sensor_msgs/Image.h>
// #include <vikit/timer.h>
// #include <vikit/vision.h>
// #include <vikit/abstract_camera.h>
// #include <vikit/camera_loader.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

using namespace std; 

// vk::AbstractCamera* gcam_; 

void imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat _img;
  try {
    _img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  
  // svo::FramePtr frame(new svo::Frame(gcam_, img, 0.0)); 
  
  // corner detection 
  // svo::Features fts;
  // svo::feature_detection::FastDetector fast_detector(
  //     img.cols, img.rows, svo::Config::gridSize(), svo::Config::nPyrLevels());
  // fast_detector.detect(frame.get(), frame->img_pyr_, svo::Config::triangMinCornerScore(), fts); 
  // printf("Fast %zu corners detected!\n", fts.size()); 
  
  // Histogram Equalization 
  cv::Mat img; 
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  clahe->apply(_img, img); 
  
  // detect good features 
  vector<cv::Point2f> corners; 
  int MAX_CNT = 150; 
  float quality_level = 0.1; 
  float MIN_DIST = 30; 
  cv::goodFeaturesToTrack(img, corners, MAX_CNT, quality_level, MIN_DIST);
 
  // show it 
  cv::Mat rgb = cv::Mat(img.size(), CV_8UC3); 
  cv::cvtColor(img, rgb, CV_GRAY2RGB); 
  // std::for_each(fts.begin(), fts.end(), [&](svo::Feature* i){
      // cv::circle(rgb, cv::Point2f(i->px[0], i->px[1]), 4*(i->level+1), cv::Scalar(0,255,0), 1); });
  for(int i=0; i<corners.size(); i++)
  {
    cv::circle(rgb, corners[i], 2, cv::Scalar(255, 0, 0), 2);
  }
  
  cv::imshow("rgb_good_features", rgb); 
  cv::waitKey(10); 
  // std::for_each(fts.begin(), fts.end(), [&](svo::Feature* i){delete i;});
  return ; 
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "good_feature");
  ros::NodeHandle nh;

  // Create Camera
  // if(!vk::camera_loader::loadFromRosNs("svo", gcam_))
    // throw std::runtime_error("Camera model not correctly specified.");

  // subscribe to cam msgs
  ros::NodeHandle np("~"); 
  std::string cam_topic("camera/image_raw"); 
  np.param("cam_topic", cam_topic, cam_topic);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &imgCb);

  // start processing callbacks
  while(ros::ok())
  {
    ros::spinOnce();
    usleep(100); 
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("good_feature_detector terminated.\n");

  return 0; 
}






