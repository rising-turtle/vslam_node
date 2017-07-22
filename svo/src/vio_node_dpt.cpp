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

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include "imu_vn100.h"
#include "graph_gtsam.h"
#include "match_result.h"

using namespace gtsam; 
using namespace std; 
using namespace svo; 

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 
using symbol_shorthand::L; // plane landmark (nv, d)

void vio_node_dpt(); 

Node* createNodeByFrame(const FramePtr& fr, int id, OptionalJacobian<6,6> H1 = boost::none); 
void addSVOResult(CGraph& gt, const FramePtr& fr);
void updateFramePose(FramePtr fr, const Pose3& pose); 
Sophus::SE3 fromPose3(const Pose3& pose); 

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
  string data_dir = "/home/davidz/work/data/ETAS_2F_640_30"; 
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
  gtsam::imuBias::ConstantBias prior_bias; 
  double dt = 0.005; // 200 hz
  CImuVn100 * imu = new CImuVn100(dt, prior_bias); 
  if(!imu->readImuData(imu_file))
  {
    ROS_ERROR("vio_node_dpt.cpp: failed to load imu file %s", imu_file.c_str()); 
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
  CGraph gt; 
  Eigen::Matrix<double ,4,4> ES2C; 
  ES2C << 1.0, 0., 0., 0.063,
          0., 1.0, 0., -0.001,
          0., 0.,  1., 0.018,
          0.0, 0.0, 0.0, 1.;
  gt.setupTsc(ES2C); 
  Pose3 Ts2c = *(gt.mp_s2c); 
  Pose3 Tc2s = Ts2c.inverse();

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
    
    cv::imshow("gray", gray); 
    cv::waitKey(0); 

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

      // add first node to graph 
      Node* n = createNodeByFrame(frame_ref, 0); 
      gt.firstNode(n); 
      
      // set start point of imu
      imu->setStartPoint(mvTimeImg[i]);
        
      ROS_INFO("vio_node_dpt.cpp: finish first image!"); 

    }else // default process 
    {
      int cur_node_id = gt.m_graph_map.size(); 
      // IMU preintegration
      NavState cur_nav, pre_state; 
      bool imu_available = imu->predictNextFlag(mvTimeImg[i], cur_nav); 
      PreintegratedCombinedMeasurements* preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu->mp_combined_pre_imu);
      if(imu_available)
      {
        // add imu measurement into graph 
        CombinedImuFactor imu_factor(X(cur_node_id-1), V(cur_node_id-1), 
            X(cur_node_id),    V(cur_node_id), 
            B(cur_node_id-1), B(cur_node_id), *(preint_imu_combined));

        gt.mp_fac_graph->add(imu_factor); 
        gt.mp_new_fac->add(imu_factor); 
        gt.addToGTSAM(cur_nav, cur_node_id); 
      }
      
      // set prior pose for svo 
      Pose3 pre_imu = gt.mp_node_values->at<Pose3>(X(cur_node_id-1));
      Pose3 cur_imu = gt.mp_node_values->at<Pose3>(X(cur_node_id));
      Pose3 inc_imu = pre_imu.transform_pose_to(cur_imu); 
      Pose3 inc_cam = Tc2s * inc_imu * Ts2c; 
      Point3 t(0,0,0);
      Pose3 inc_cam_rot = Pose3::Create(inc_cam.rotation(), t); 
      // cout <<"imu estimate inc_cam : "<<inc_cam<<endl; 
      // only use rotation as prior info
      Sophus::SE3 inc_cam_se3 = fromPose3(inc_cam_rot); 
      // Sophus::SE3 inc_cam_se3 = fromPose3(inc_cam); 

      vo_->addIncPrior(inc_cam_se3); 

      // run svo 
      vo_->addImage(gray, mvTimeImg[i]); 
      if(vo_->trackingQuality() == FrameHandlerBase::TRACKING_GOOD)
      {
        // add svo's result into graph 
        addSVOResult(gt, vo_->lastFrame()); 

        // optimize graph to update last frame's pose and imu's bias
        gt.optimizeGraphIncremental(); // isam2 
       
        // publish for visualization 
        visualizer_.publishMinimal(gray, vo_->lastFrame(), *vo_, mvTimeImg[i]);
        if(vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
         visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
      }else
      {
        Pose3 cur_pose = gt.mp_node_values->at<Pose3>(X(cur_node_id)); 
        Node* new_node = new Node; 
        *(new_node->mpPose) = cur_pose; 
        new_node->m_id = gt.m_graph_map.size(); 
        gt.m_graph_map[new_node->m_id] = new_node; 
          
        ROS_WARN("svo track insufficient, try to relocalize!"); 
      }

      // imu reset
      imu->resetPreintegrationAndBias(gt.mp_node_values->at<imuBias::ConstantBias>(B(cur_node_id))); 
      pre_state = NavState(gt.mp_node_values->at<Pose3>(X(cur_node_id)), gt.mp_node_values->at<Vector3>(V(cur_node_id))); 
      imu->setState(pre_state); 
      // update svo's pose 
      //  updateFramePose(vo_->lastFrame(), pre_state.pose());   
    }

    visualizer_.exportToDense(vo_->lastFrame()); 

    ros::spinOnce(); 
    usleep(100); 
  }

  return ; 
}

Sophus::SE3 fromPose3(const Pose3& pose)
{
  Sophus::SE3 ret(pose.rotation().matrix(), pose.translation()); 
  return ret; 
}

void updateFramePose(FramePtr fr, const Pose3& pose)
{
  Pose3 T_fw = pose.inverse(); 
  fr->T_f_w_.rotationMatrix() = T_fw.rotation().matrix(); 
  fr->T_f_w_.translation() = T_fw.translation(); 
  return ; 
}

Node* createNodeByFrame(const FramePtr& fr, int id, OptionalJacobian<6,6> H )
{
  Node* ret = new Node; 
  ret->m_id = id; 
  ret->m_seq_id = id; 
  Eigen::Matrix<double, 4, 4> T_fw = Eigen::Matrix<double, 4, 4>::Identity(); 
  T_fw.block<3,3>(0,0) = fr->T_f_w_.rotationMatrix(); 
  T_fw.block<3,1>(0,3) = fr->T_f_w_.translation(); 
  Pose3 PT_fw(T_fw); 
  *(ret->mpPose) = PT_fw.inverse(H); 
  return ret; 
}
void addSVOResult(CGraph& gt, const FramePtr& fr)
{
  Eigen::Matrix<double, 6, 6> HM = Eigen::Matrix<double, 6, 6>::Identity(); 
  OptionalJacobian<6,6> H(HM); 
  int cur_id = gt.m_graph_map.size(); 
  Node* pre_n = gt.m_graph_map[cur_id-1]; 
  Node* cur_n = createNodeByFrame(fr, cur_id, H); 

  // cout <<"*H "<<*H<<endl; 
  // cout <<"(*H).transpose() "<<(*H).transpose()<<endl;
  Eigen::Matrix<double, 6, 6> COV = (*H)*fr->Cov_*((*H).transpose()); 
  
  MatchingResult mr; 
  mr.m_id_from = pre_n->m_id; 
  mr.m_id_to = cur_n->m_id;  

  Pose3 inc_pose = pre_n->mpPose->between(*(cur_n->mpPose)); // H2 = Identity  
  // cout <<"svo result inc_cam: "<<inc_pose<<endl; 
  mr.m_trans = inc_pose.matrix(); 
  mr.m_informationM = COV.inverse(); 
  // cout <<"svo result COV: "<<mr.m_informationM<<endl;

  gt.addToGTSAM(mr, true); 
  gt.m_graph_map[cur_id] = cur_n; 

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





