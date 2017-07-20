/*
 *  July 20 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  VIO = svo + imu preintegration, loosely couple 
 *  dpt means initialization is completed using the depth data 
 *
 * */

#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/feature_detection.h>
#include "visualizer.h"
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>
#include <vikit/blender_utils.h>
#include <fstream>

using namespace std; 
using namespace svo; 

void vio_node_dpt(); 

bool LoadRGBDIr2(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<string> &vstrIr1, vector<string>& strIr2, vector<double> &vTimestamps);

// extract features and add depth to each feature 
int addDpt(FramePtr& frame, vk::AbstractCamera* cam, cv::Mat& dpt); 

int main(int argc, char* argv[])
{  
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  
  vio_node_dpt(); 

  ROS_INFO("vio_node_dpt.cpp: FINISH!"); 
  return 0; 
}

void vio_node_dpt()
{
  // 1. load data and setup 
  ros::NodeHandle np("~"); 
  string data_dir = ""; 
  np.param("data_dir", data_dir, data_dir); 
  string imu_file = data_dir + "/imu_vn100.log"; 
  string img_log_file = data_dir + "/timestamp.txt"; 
  vector<double> mvTimeImg; 
  vector<string> mvRgb; 
  vector<string> mvDpt; 
  vector<string> mvIr1;
  vector<string> mvIr2;

  // img index 
  if(!LoadRGBDIr2(img_log_file, mvRgb, mvDpt, mvIr1, mvIr2, mvTimeImg))
  {
    ROS_ERROR("vio_node_dpt.cpp: failed to load img file %s ", img_log_file.c_str()); 
    return ; 
  }
  
  // imu 
  /*
  gtsam::imuBias::ConstantBias prior_bias; 
  double dt = 0.005; // 200 hz
  CImuVn100 * imu = new CImuVn100(dt, prior_bias); 
  if(!imu->readImuData(imu_file))
  {
    ROS_ERROR("vio_node_dpt.cpp: failed to load imu file %s", imu_file.c_str()); 
    return ; 
  }*/
  
  // cam 
  vk::AbstractCamera* cam_ = NULL ;
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // visualizer 
  svo::Visualizer visualizer_; 
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  svo::FrameHandlerMono* vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();

  cv::Mat rgb; 
  cv::Mat gray; 
  cv::Mat dpt; 

  // iterate IMG file
  for(int i=0; i<mvRgb.size() && ros::ok(); i++)
  {
    rgb = cv::imread(data_dir + "/" + mvRgb[i], -1); 
    if(rgb.empty())
    {
      ROS_ERROR("rgb is empty at i = %d", i); 
      break; 
    }
    
    // convert to gray image 
    gray = rgb; 
    cvtColor(gray,gray,CV_RGB2GRAY);

    if(i==0) // first img, initialization 
    {
      dpt = cv::imread(data_dir + "/" + mvDpt[i], -1); 
      if(dpt.empty())
      {
        ROS_ERROR("first dpt is empty %s", mvDpt[i].c_str()); 
        break; 
      }

      FramePtr  frame_ref(new Frame(cam_, gray, mvTimeImg[i])); 
      Sophus::SE3 T_w_gt; 
      frame_ref->T_f_w_ = T_w_gt.inverse(); 

      // add depth to the first frame 
      int N_added = addDpt(frame_ref, cam_, dpt); 

      SVO_INFO_STREAM( "Detected "<<frame_ref->nObs()<<" features, Added "<<N_added<<" 3d pts to the reference frame."); 
      vo_->setFirstFrame(frame_ref); 

    }else // default process 
    {
      vo_->addImage(gray, mvTimeImg[i]); 
      visualizer_.publishMinimal(gray, vo_->lastFrame(), *vo_, mvTimeImg[i]);
      if(vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
        visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
    }

    visualizer_.exportToDense(vo_->lastFrame()); 

    ros::spinOnce(); 
    usleep(100); 
  }

  return ; 
}

// extract features and add depth to each feature 
int addDpt(FramePtr& frame_ref, vk::AbstractCamera* cam_, cv::Mat& dpt)
{
  int ret = 0; 
  // extract features, generate features with 3D points
  svo::feature_detection::FastDetector detector(
      cam_->width(), cam_->height(), svo::Config::gridSize(), svo::Config::nPyrLevels()); 
  detector.detect(frame_ref.get(), frame_ref->img_pyr_, svo::Config::triangMinCornerScore(), frame_ref->fts_); 

    std::for_each(frame_ref->fts_.begin(), frame_ref->fts_.end(), [&](Feature* ftr)
    {
      int x = ftr->px[0]; 
      int y = ftr->px[1]; 
      float depth_value = dpt.at<float>(ftr->px[1], ftr->px[0]); 
      if(depth_value > 0.03 && depth_value < 7.0) // valid depth 
      {
        // Eigen::Vector3d pt_pos_cur = ftr->f*depthmap.at<float>(ftr->px[1], ftr->px[0]); 
        Eigen::Vector3d v3 = cam_->cam2world(x, y); 
        Eigen::Vector2d uv = v3.head<2>()/v3[2]; 
        float ray_len = depth_value * (sqrt(uv[0]*uv[0] + uv[1]*uv[1] + 1.0)); 
        Eigen::Vector3d pt_pos_cur = ftr->f*ray_len; 
        Eigen::Vector3d pt_pos_world = frame_ref->T_f_w_.inverse()*pt_pos_cur;
        svo::Point* point = new svo::Point(pt_pos_world, ftr); 
        ftr->point = point; 
        ++ret;
      }else{
        ftr->point = NULL; 
      }
   });
    return ret; 
}

bool LoadRGBDIr2(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<string> &vstrIr1, vector<string>& vstrIr2, vector<double> &vTimestamps)
{
  ifstream fAssociation;
  fAssociation.open(strAssociationFilename.c_str());
  if(!fAssociation.is_open()) 
    return false; 
  string s; 
  getline(fAssociation, s); // exclude the first line 
  while(!fAssociation.eof())
  {
    string s;
    getline(fAssociation,s);
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      double t;
      string sRGB, sD, sIr1, sIr2;
      ss >> t;
      // printf("load Images line: %s\n", ss.str().c_str());
      // printf("loadImages t = %.9lf\n", t);
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenamesRGB.push_back(sRGB);
      ss >> t;   ss >> sD;
      vstrImageFilenamesD.push_back(sD);
      ss >> t;   ss >> sIr1;
      vstrIr1.push_back(sIr1); 
      ss >> t;   ss >> sIr2; 
      vstrIr2.push_back(sIr2); 
    }
  }
  return true; 
}





