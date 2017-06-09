/*  
 *  June 8 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  ros node for svo (semi-direct visual odometry) 
 *
 *  with depth data for the first frame to solve for scale
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
// #include <svo_ros/visualizer.h>
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

namespace svo {

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;
  bool quit_;
  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
  bool mb_first_frame; 
  std::ofstream m_dpt_of;         // where the depth data is stored 
  string m_dpt_file;            
  std::ofstream m_est_traj;       // where to save estimated trajectory 
  void recordTraj(const SE3& T_w_f, const double timestamp);
  
};

VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false),
  mb_first_frame(true)
{
  // Start user input thread in parallel thread that listens to console keys
  if(vk::getParam<bool>("svo/accept_console_user_input", true))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();

  // Create Camera
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // open trajectory file for writing 
  std::string est_traj(vk::getParam<string>("svo/trajectory_log", "mono_dpt_trajectory.log")); 
  m_est_traj.open(est_traj.c_str()); 
  if(m_est_traj.fail())
    throw std::runtime_error("Could not create trajectory log file"); 

  // open depth data for solving scale 
  m_dpt_file = string(vk::getParam<string>("svo/dpt_file", "/home/davidz/work/data/sin2_tex2_h1_v8_d/depth/frame_000002_0.depth")); 
  m_dpt_of.open(m_dpt_file.c_str()); 
  if(m_dpt_of.fail())
    throw std::runtime_error("Fail to open Dpt file! run vo_node instead? ");

  // Init VO and start
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();
}

VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

void VoNode::recordTraj(const SE3& T_w_f, const double timestamp)
{
  Quaterniond q(T_w_f.unit_quaternion()); 
  Vector3d p(T_w_f.translation()); 
  m_est_traj.precision(15); 
  m_est_traj.setf(std::ios::fixed, std::ios::floatfield); 
  m_est_traj<<timestamp<<" "; 
  m_est_traj.precision(6); 
  m_est_traj<< p.x() <<" "<<p.y()<<" "<<p.z()<<" "
            << q.x() <<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<< std::endl; 
  return ;
}

void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();

  if(mb_first_frame) // set first frame info 
  {
    cv::Mat depthmap; 
    vk::blender_utils::loadBlenderDepthmap(
        m_dpt_file.c_str(), *cam_, depthmap);

    // set reference frame at ground-truth pose 
    FramePtr frame_ref(new Frame(cam_, img, msg->header.stamp.toSec())); 
    // first pos 0.11314 0.11314 2.00000 0.82266 0.21488 0.00000 0.00000
    Eigen::Vector3d t(0.11314, 0.11314, 2.00000); 
    Eigen::Quaterniond q(0.82266, 0.21488,0.00000,0.00000); 
    Sophus::SE3 T_w_gt(q, t); 
    frame_ref->T_f_w_ = T_w_gt.inverse(); 
   
    // extract features, generate features with 3D points
    svo::feature_detection::FastDetector detector(
        cam_->width(), cam_->height(), svo::Config::gridSize(), svo::Config::nPyrLevels()); 
    detector.detect(frame_ref.get(), frame_ref->img_pyr_, svo::Config::triangMinCornerScore(), frame_ref->fts_); 
    std::for_each(frame_ref->fts_.begin(), frame_ref->fts_.end(), [&](Feature* ftr){
      Eigen::Vector3d pt_pos_cur = ftr->f*depthmap.at<float>(ftr->px[1], ftr->px[0]); 
      Eigen::Vector3d pt_pos_world = frame_ref->T_f_w_.inverse()*pt_pos_cur;
      svo::Point* point = new svo::Point(pt_pos_world, ftr); 
      ftr->point = point; 
    });
      
    SVO_INFO_STREAM("Added "<<frame_ref->nObs()<<" 3d pts to the reference frame."); 
    vo_->setFirstFrame(frame_ref); 
    SVO_INFO_STREAM("Set reference frame."); 
    mb_first_frame = false; 
  }else
  {
    vo_->addImage(img, msg->header.stamp.toSec());
    visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());
    if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
      visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());
  }

  // save pose in trajetory log 
  recordTraj(vo_->lastFrame()->T_f_w_.inverse(), msg->header.stamp.toSec());

  if(publish_dense_input_)
    visualizer_.exportToDense(vo_->lastFrame());

  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo

int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");
  ros::NodeHandle nh;
  std::cout << "create vo_node" << std::endl;
  svo::VoNode vo_node;

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);

  // subscribe to remote input
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  // start processing callbacks
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");
  return 0;
}
