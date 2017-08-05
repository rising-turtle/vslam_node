/*
 *  July 26 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  msckf + svo, loosely couple 
 *  svo publish pose msg, 
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "sensor_fusion_comm/InitScale.h"
#include "visualizer.h"
#include "imu_pub.h"

using namespace std; 
using namespace svo; 

#define R2D(r) (((r)*180.)/M_PI)

void msckf_node_dpt(); 

namespace
{
  ros::Time compT(string curr_t)
  {
    string secs = curr_t.substr(0, 10); 
    string nanosecs = curr_t.substr(11, 9); 
    ros::Time rost(std::stoi(secs), std::stoi(nanosecs)); 
    return rost; 
  }
}

// subscribe EKF's output 
bool g_TF_predict_flag = false; 
tf::Transform g_TF_predict; 
ros::Time g_TF_predict_timestamp;

bool g_TF_update_flag = false; 
tf::Transform g_TF_update; 
ros::Time g_TF_update_timestamp; 

void EKF_predictCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_predict); 
void EKF_updateCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_update); 

void waitForEKFUpdate(ros::Time t); 
tf::Transform waitForEKFPredict(ros::Time t); 

// publish SVO's PCE to EKF
Sophus::SE3 fromTFRot(const tf::Transform& t); 
void publishVOEstimate(ros::Publisher& pub, tf::Transform& Ts2c, FramePtr fr, ros::Time& rt, bool goodVO); 
void publishZeroPose(ros::Publisher& pub, ros::Time& rt); 
void publishTf(ros::Publisher& pub, tf::Transform& T, ros::Time rt); 

// send for visualization, and record result
string g_traj_out_file("/home/davidz/work/ros/indigo/src/vslam_node/msckf/result/msckf_svo.log");
void handleResult(tf::Transform T_w_f_ori, ros::Time timstamp);

bool LoadRGBDIr2(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<string> &vstrIr1, vector<string>& strIr2, vector<ros::Time> &vTimestamps);

// extract features and add depth to each feature 
int addDpt(FramePtr& frame, vk::AbstractCamera* cam, cv::Mat& dpt); 

// whether two ros time almost the same
bool rosTimeEqual(ros::Time t1, ros::Time t2);
void print_tf(ostream& out, tf::Transform tT, string s="");

// pose msg seq 
int vo_msg_seq = 0; 

int main(int argc, char* argv[])
{  
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  
  // subscribe EKF's output 
  ros::Subscriber EKF_predict_sub = nh.subscribe("/msf_core/pose", 100, EKF_predictCb);
  ros::Subscriber EKF_update_sub = nh.subscribe("/msf_core/pose_after_update", 100, EKF_updateCb); 

  msckf_node_dpt(); 

  ROS_INFO("vio_node_dpt.cpp: FINISH!"); 
  return 0; 
}

void msckf_node_dpt()
{
  // 1. load data and setup 
  ros::NodeHandle np("~"); 
  string data_dir = "/home/davidz/work/data/ETAS_2F_640_30"; 
  np.param("data_dir", data_dir, data_dir); 
  string imu_file = data_dir + "/imu_vn100.log"; 
  string img_log_file = data_dir + "/timestamp.txt"; 
  // vector<double> mvTimeImg; 
  vector<ros::Time> mvTimeImg; 
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
  CImuPub imu("/vn100_imu0"); 
  if(!imu.readImuData(imu_file))
  {
    ROS_ERROR("%s: failed to load imu file %s", __FILE__, imu_file.c_str());
    return ; 
  }
  
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

  // graph gtsam 
  tf::Vector3 Ts2c_v(0.063, -0.001, 0.018); 
  tf::Quaternion Ts2c_q(0, 0, 0, 1); 
  tf::Transform Ts2c(Ts2c_q, Ts2c_v); 
  tf::Transform Tc2s = Ts2c.inverse();
  // ROS_WARN("Ts2c and Tc2s"); 
  // print_tf(cout, Ts2c); 
  // print_tf(cout, Tc2s); 

  // vo publisher, send vo's estimate to EKF
  ros::NodeHandle n; 
  ros::Publisher vo_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/msf_updates/pose_with_covariance_input", 20); 
  
  // rosservice to call initScale which will initializes msckf 
  ros::ServiceClient init_client = n.serviceClient<sensor_fusion_comm::InitScale>("/msf_svo/pose_sensor/initialize_msf_scale"); 
    
  // Init VO and start
  svo::FrameHandlerMono* vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();

  cv::Mat rgb, gray, dpt; 
  
  // iterate IMG file
  for(int i=0; i<mvRgb.size() && ros::ok() && i < 7000; i++)
  {
    rgb = cv::imread(data_dir + "/" + mvRgb[i], -1); 
    if(rgb.empty())
    {
      ROS_ERROR("rgb is empty at i = %d", i); 
      break; 
    }
    
    // current timestamp 
    ros::Time curT = mvTimeImg[i]; 
    SVO_INFO_STREAM("Handle current data with stamp : "<<curT);

    // convert to gray image 
    gray = rgb; 
    cvtColor(gray,gray,CV_RGB2GRAY);
    
    cv::imshow("gray", gray); 
    if(i == 0)
    {
      cv::waitKey(0); // In the begining, ros publisher and subscriber needs time to build connection

      // send initial Tcv before msckf sensor_manager.initScale()
      tf::Transform Tini; 
      if(imu.computeInitT(Tini))
      {
        publishTf(vo_pub, Tini, imu.getFirstStamp()); 
        ROS_WARN("msckf_node_dpt.cpp: succeed to compute and send Tini"); 
      }
      else
      {
        ROS_ERROR("msckf_node_dpt.cpp: failed to compute Tini, use default!"); 
      }
      
      // wait for msckf call sensor_manager.initScale()
      // see instrunctions in http://wiki.ros.org/ethzasl_sensor_fusion/Tutorials/Introductory%20Tutorial%20for%20Multi-Sensor%20Fusion%20Framework
      // $rosrun rqt_reconfigure rqt_reconfigure 
      // or here, we call rosservice to initialize it
      usleep(100*1000);
      
      sensor_fusion_comm::InitScale srv; 
      srv.request.scale = 1.0; 
      if(init_client.call(srv))
      {
        ROS_WARN_STREAM("msckf_node_dpt.cpp: client response: "<<srv.response.result); 
      }else
      {
        ROS_ERROR("msckf_node_dpt.cpp: rosservice call failed !"); 
      }

      cv::imshow("gray", gray); 
      cv::waitKey(0); 
    }
    else
      cv::waitKey(10);

    if(i==0) // first img, initialization 
    {
      dpt = cv::imread(data_dir + "/" + mvDpt[i], -1); 
      if(dpt.empty())
      {
        ROS_ERROR("first dpt is empty %s", mvDpt[i].c_str()); 
        break; 
      }

      FramePtr frame_ref(new Frame(cam_, gray, mvTimeImg[i].toSec())); 
      Sophus::SE3 T_w_gt; 
      frame_ref->T_f_w_ = T_w_gt.inverse(); 

      // add depth to the first frame 
      int N_added = addDpt(frame_ref, cam_, dpt); 

      SVO_INFO_STREAM( "Detected "<<frame_ref->nObs()<<" features, Added "<<N_added<<" 3d pts to the reference frame."); 
      vo_->setFirstFrame(frame_ref); 

      // set start point of imu
      // imu.setStartIndex(mvTimeImg[i]);
      imu.publishToTime(curT);   
      
      // publish Zero position 
      SVO_INFO_STREAM(" publish ZeroPose at stamp: "<<curT); 
      publishZeroPose(vo_pub, curT); 
      {
        // init the first pose 
        tf::Quaternion q(0 ,0 ,0 ,1); 
        tf::Vector3 t(0, 0, 0);
        g_TF_update = tf::Transform(q,t);
      }
    
      ROS_INFO("msckf_node_dpt.cpp: finish first image!"); 
    }else // default process 
    {
      // 1. publish IMU data for EKF to handle 
      imu.publishToTime(curT); 

      // 2. get relative pose change from EKF prediction 
      tf::Transform inc_imu = waitForEKFPredict(imu.getLastPubTime()); 
      tf::Transform inc_cam = Tc2s * inc_imu * Ts2c; 
      // ROS_WARN("inc_imu and inc_cam: ");
      // print_tf(cout, inc_imu); 
      // print_tf(cout, inc_cam); 
      Sophus::SE3 inc_cam_se3 = fromTFRot(inc_cam); // only rotation 
      vo_->addIncPrior(inc_cam_se3); 

      // 3. svo produces vo estimation 
      // vo_->addImage(gray, curT.toSec()); 
      FrameHandlerMono::UpdateResult dropout = vo_->addImageFront(gray, curT.toSec()); 
      vo_->addImageBack(dropout); 

      // 4. publish vo's result 
      publishVOEstimate(vo_pub, Ts2c, vo_->lastFrame(), curT, !(dropout == FrameHandlerBase::RESULT_FAILURE));
      
      // 5. wait for EKF's updation 
      waitForEKFUpdate(curT);  

      // 6. send for visualization, rqt_vision
      handleResult(g_TF_update, curT); 
    }

    // publish for visualization 
    visualizer_.publishMinimal(gray, vo_->lastFrame(), *vo_, mvTimeImg[i].toSec());
    if(vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
      visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
    visualizer_.exportToDense(vo_->lastFrame()); 

    ros::spinOnce(); 
    usleep(50*1000); 
  }

  return ; 
}

void handleResult(tf::Transform T_w_f_ori, ros::Time timestamp)
{
  static bool first = true; 
  static tf::Transform last_T_w_f_ori; 
  static tf::Transform last_T_w_f; 
  static tf::TransformBroadcaster br_;
  static ofstream* ouf = NULL; 
  if(first) // make it start from Identity 
  {
    last_T_w_f.getOrigin() = tf::Vector3(0, 0, 0); 
    // last_T_w_f.getRotation() = 
    tf::Quaternion q0 = tf::Quaternion(0, 0, 0, 1); 
    last_T_w_f.setRotation(q0); 
    last_T_w_f_ori = T_w_f_ori; 
    ouf = new ofstream(g_traj_out_file.c_str()); 
    if(!ouf->is_open())
    {
      ROS_ERROR("msckf_node_dpt.cpp: failed to open file %s to record", g_traj_out_file.c_str()); 
      delete ouf; 
      ouf = NULL; 
    }
    first = false; 
  }
  
  tf::Transform inc = last_T_w_f_ori.inverse()*T_w_f_ori; 
  tf::Transform T_w_f = last_T_w_f * inc; 
  tf::Transform T_f_w = T_w_f.inverse(); 

  // print_tf(cout, last_T_w_f_ori, "last_T_w_f_ori: ");
  // print_tf(cout, T_w_f_ori, "T_w_f_ori: "); 
  // print_tf(cout, inc, "inc: "); 

  // print_tf(cout, last_T_w_f, "last_T_w_f: ");
  // print_tf(cout, T_w_f, "T_w_f: ");
  // print_tf(cout, T_f_w, "T_f_w: ");

  // since msckf already sendTransform(state) data 
  // br_.sendTransform(tf::StampedTransform(T_f_w, timestamp, "cam_pos", "world"));
 
  // record data 
  if(ouf != NULL)
  {
    tf::Vector3 t = T_w_f.getOrigin(); 
    tfScalar tx = t.getX(); tfScalar ty = t.getY(); tfScalar tz = t.getZ();
    
    (*ouf) << timestamp <<std::fixed<<"\t"<<tx<<"\t"<<ty<<"\t"<<tz<<"\t"<<
      T_w_f.getRotation().x()<<"\t"<<T_w_f.getRotation().y()<<"\t"<<T_w_f.getRotation().z()<<"\t"<<T_w_f.getRotation().w()<<endl;
  }

  last_T_w_f = T_w_f; 
  last_T_w_f_ori = T_w_f_ori; 
  return ; 
}

void print_tf(ostream& out, tf::Transform tT, string prefix)
{
  tfScalar r, p, y, tx, ty, tz;
  tT.getBasis().getEulerYPR(y, p, r); 
  tf::Vector3 t = tT.getOrigin(); 
  tx = t.getX(); ty = t.getY(); tz = t.getZ();
  out<<"msckf_node_dpt.cpp: "<<prefix<<" yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<" qx = "<<
     tT.getRotation().x()<<" qy = "<<tT.getRotation().y()<<" qz= "<<tT.getRotation().z()<<" qw = "<<tT.getRotation().w()<<endl;
  // out<<"msckf_node_dpt.cpp: yaw: "<<R2D(y)<<" pitch: "<<R2D(p)<<" roll: "<<R2D(r)<<" tx: "<<tx<<" ty: "<<ty<<" tz: "<<tz<<endl;
}

bool rosTimeEqual(ros::Time t1, ros::Time t2)
{
  ros::Duration dt = t1 - t2; 
  if(fabs(dt.toSec()) < 1e-5)
    return true; 
  return false; 
}

tf::Transform waitForEKFPredict(ros::Time t)
{
  // while(g_TF_predict_timestamp != t)
  while(!rosTimeEqual(g_TF_predict_timestamp, t))
  {
    if(g_TF_predict_flag)
    {
      // ROS_INFO_STREAM(__FILE__<<" new EKF predict at stamp "<<g_TF_predict_timestamp);
      g_TF_predict_flag = false; 
    }
    usleep(1000); 
    ros::spinOnce(); 
    ROS_WARN_STREAM_THROTTLE(5, "msckf_node_dpt.cpp: wait for EKF predict given t = "<<t); 
  }
  
  // ROS_WARN_STREAM(__FILE__<<" expected EKF predict at stamp "<<g_TF_predict_timestamp); 
  tf::Transform inc_imu = g_TF_update.inverse() * g_TF_predict; 
  // ROS_WARN_STREAM("g_TF_predict: "<<g_TF_predict<<" inc_imu: "<<inc_imu); 
  // print_tf(cout, g_TF_predict); 
  // print_tf(cout, g_TF_update);
  // print_tf(cout, inc_imu);
  return inc_imu; 
}

void waitForEKFUpdate(ros::Time t)
{
  // while(!g_TF_update_flag)
  while(g_TF_update_timestamp < t)
  {
    usleep(1000); 
    ros::spinOnce(); 
    ROS_WARN_STREAM_THROTTLE(5, "msckf_node_dpt.cpp: wait for EKF update given t = "<<t);
  }
  g_TF_update_flag = false;
  // ROS_WARN_STREAM(__FILE__<<" get EKF update at stamp "<<g_TF_update_timestamp); 
  // if(g_TF_update_timestamp != t)
  // if(!rosTimeEqual(g_TF_update_timestamp, t))
  {
    // ROS_ERROR_STREAM(__FILE__<<" not equal expect stamp "<<t);
  }
  return ; 
}

// subscribe EKF's update 
void EKF_predictCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // Get measurements.
  tf::Vector3 z_p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z); 
  tf::Quaternion z_q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w); 
  g_TF_predict = tf::Transform(z_q, z_p); 
  g_TF_predict_timestamp = msg->header.stamp; 
  g_TF_predict_flag = true; 

  // ROS_INFO_STREAM("msckf_node_dpt.cpp: EKF_predictCb receive pose msg at stamp: "<<msg->header.stamp<<" with T_predict: ");
  // print_tf(cout, g_TF_predict); 

  return ; 
}
void EKF_updateCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // Get measurements.
  tf::Vector3 z_p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z); 
  tf::Quaternion z_q( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w); 
  g_TF_update = tf::Transform(z_q, z_p); 
  g_TF_update_timestamp = msg->header.stamp; 
  g_TF_update_flag = true; 
  
  // ROS_WARN_STREAM("msckf_node_dpt.cpp: EKF_updateCb receive pose msg at stamp: "<<msg->header.stamp<<" with T_update: ");
  // print_tf(cout, g_TF_update); 


  return ; 
}

Sophus::SE3 fromTFRot(const tf::Transform& T)
{
  tf::Quaternion R = T.getRotation(); 
  Eigen::Quaternion<double> z_q_ = Eigen::Quaternion<double>(R.w(), R.x(), R.y(), R.z());
  Eigen::Vector3d v(0, 0, 0); 
  Sophus::SE3 ret(z_q_.toRotationMatrix(), v); 
  return ret; 
}

void publishTf(ros::Publisher& pub, tf::Transform& T, ros::Time rt)
{
  geometry_msgs::PoseWithCovarianceStamped msg; 
  tf::Vector3 t = T.getOrigin(); 
  tf::Quaternion q = T.getRotation(); 
  msg.header.stamp = rt; 
  msg.header.seq = vo_msg_seq++;
  msg.pose.pose.position.x = t.x(); 
  msg.pose.pose.position.y = t.y(); 
  msg.pose.pose.position.z = t.z();  

  msg.pose.pose.orientation.x = q.x(); 
  msg.pose.pose.orientation.y = q.y(); 
  msg.pose.pose.orientation.z = q.z(); 
  msg.pose.pose.orientation.w = q.w();
  
  for(int i=0; i<36; i++)
    msg.pose.covariance[i] = 0.; 
  for(int i=0; i<36; i+=7)
    msg.pose.covariance[i] = 1e-10; 

  pub.publish(msg); 
  ros::spinOnce(); 
  usleep(1000); 
  return ; 
 
}

void publishZeroPose(ros::Publisher& pub, ros::Time& rt)
{
  geometry_msgs::PoseWithCovarianceStamped msg; 
  msg.header.stamp = rt; 
  msg.header.seq = vo_msg_seq++;
  msg.pose.pose.position.x = 0; 
  msg.pose.pose.position.y = 0; 
  msg.pose.pose.position.z = 0;  

  msg.pose.pose.orientation.x = 0; 
  msg.pose.pose.orientation.y = 0; 
  msg.pose.pose.orientation.z = 0; 
  msg.pose.pose.orientation.w = 1;
  
  for(int i=0; i<36; i++)
    msg.pose.covariance[i] = 0.; 
  for(int i=0; i<36; i+=7)
    msg.pose.covariance[i] = 1e-10; 

  pub.publish(msg); 
  ros::spinOnce(); 
  usleep(1000); 
  return ; 
}

void publishVOEstimate(ros::Publisher& pub, tf::Transform& Ts2c, FramePtr fr, ros::Time& rt, bool goodVO)
{
  geometry_msgs::PoseWithCovarianceStamped msg; 

  // timestamp 
  msg.header.stamp = rt ; 
  msg.header.seq = vo_msg_seq++;
  // pose 
  Eigen::Vector3d v = fr->T_f_w_.translation(); 
  // tf::Vector3 Tc_v(v(0), v(1), v(2)); 
  
  msg.pose.pose.position.x = v(0); 
  msg.pose.pose.position.y = v(1); 
  msg.pose.pose.position.z = v(2); 

  Eigen::Quaternion<double> q(fr->T_f_w_.rotationMatrix()); 
  tf::Quaternion Tc_q(q.x(), q.y(), q.z(), q.w()); 
  // tf::Transform Tc(Tc_q, Tc_v); 

  // tf::Transform Tw2c = Ts2c*Tc.inverse(); 
  // ROS_WARN("SVO publish Tw2c: ");
  // print_tf(cout, Tw2c);
  // tf::Transform Tc2w = Tw2c.inverse(); 
  
  msg.pose.pose.orientation.x = q.x(); 
  msg.pose.pose.orientation.y = q.y(); 
  msg.pose.pose.orientation.z = q.z(); 
  msg.pose.pose.orientation.w = q.w();

  // msg.pose.pose.position.x = Tc2w.getOrigin().x(); 
  // msg.pose.pose.position.y = Tc2w.getOrigin().y(); 
  // msg.pose.pose.position.z = Tc2w.getOrigin().z();  

  // msg.pose.pose.orientation.x = Tc2w.getRotation().x(); 
  // msg.pose.pose.orientation.y = Tc2w.getRotation().y(); 
  // msg.pose.pose.orientation.z = Tc2w.getRotation().z(); 
  // msg.pose.pose.orientation.w = Tc2w.getRotation().w();

  // covariance 
  if(goodVO)
  {
    for(int i=0; i<36; i++)
    {
      msg.pose.covariance[i] = fr->Cov_(i/6, i%6);
    }
  }else
  {
    for(int i=0; i<36; i++)
      msg.pose.covariance[i] = 0.; 
    for(int i=0; i<36; i+=7)
      msg.pose.covariance[i] = 1000.; 
  }
  pub.publish(msg); 
  ros::spinOnce(); 
  usleep(1000); // sleep 1ms 
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

bool LoadRGBDIr2(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<string> &vstrIr1, vector<string>& vstrIr2, vector<ros::Time> &vTimestamps)
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
      string st; 
      string sRGB, sD, sIr1, sIr2;
      ss >> st;
      // printf("load Images line: %s\n", ss.str().c_str());
      // printf("loadImages t = %.9lf\n", t);
      // string to ros::Time 
      vTimestamps.push_back(compT(st));
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





