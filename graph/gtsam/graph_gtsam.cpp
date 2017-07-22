/*  
 *  July 21 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  simple graph wrapper based on gtsam 
 *
 * */
#include "graph_gtsam.h"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>
#include "match_result.h"
#include <ros/ros.h>

using namespace gtsam; 
using namespace std; 

using symbol_shorthand::X; // Pose3 (x, y, z, r, p, y)
using symbol_shorthand::V; // Vel (xdot, ydot, zdot)
using symbol_shorthand::B; // Bias (ax, ay, az, gx, gy, gz) 
using symbol_shorthand::L; // Plane landmark (nv, d)
using symbol_shorthand::Q; // Point3 (x, y, z)

Node::Node() : m_id(-1), m_seq_id(-1),
  mpPose(new Pose3)
{}
Node::~Node()
{
  if(mpPose) delete mpPose; 
}

CGraph::CGraph()
{
  mp_fac_graph = new NonlinearFactorGraph; 
  mp_new_fac = new NonlinearFactorGraph; 
  mp_node_values = new Values ;
  mp_new_node = new Values; 

  initISAM2Params(); 
  mp_w2o = new Pose3;
  mp_s2c = new Pose3; 
  mp_prev_bias = new imuBias::ConstantBias; 
  mp_prev_state = new NavState;
  // mb_record_vro_results = true; // TODO: parameterize this one  
  // m_plane_landmark_id = 0; 
  // m_sift_landmark_id =0 ;
}

void CGraph::initISAM2Params()
{
  mp_isam2_param = new ISAM2Params; 
  mp_isam2_param->relinearizeThreshold = 0.1; // 0.3 0.2
  mp_isam2_param->relinearizeSkip = 1; // 2
  mp_isam2 = new ISAM2(*mp_isam2_param); 
}

CGraph::~CGraph()
{
  if(mp_prev_bias != NULL) 
  {
    delete mp_prev_bias; mp_prev_bias = NULL;
  }
  if(mp_prev_state != NULL)
  {
    delete mp_prev_state; mp_prev_state = NULL; 
  }

  if(mp_fac_graph != NULL) 
  {
    delete mp_fac_graph;  mp_fac_graph = NULL; 
  }
  
  if(mp_new_fac != NULL) 
  {
    delete mp_new_fac; mp_new_fac = NULL; 
  }

  if(mp_new_node != NULL)
  {
    delete mp_new_node; mp_new_node = NULL; 
  }

  if(mp_node_values != NULL)
  {
    delete mp_node_values; mp_node_values = NULL; 
  }

  if(mp_w2o != NULL)
  {
    delete mp_w2o;  mp_w2o = NULL; 
  }
  
  if(mp_s2c != NULL)
  {
    delete mp_s2c;  mp_s2c = NULL; 
  }

  if(mp_isam2_param != NULL)
  {
    delete mp_isam2_param; mp_isam2_param = NULL; 
  }

  if(mp_isam2 != NULL)
  {
    delete mp_isam2; mp_isam2 = NULL; 
  }

  for(map<int, Node*>::iterator it = m_graph_map.begin(); 
      it != m_graph_map.end(); ++it)
  {
    delete it->second; 
  }
}

void CGraph::setupTsc(Eigen::Matrix<double, 4,4>& m)
{
  *mp_s2c = Pose3(m); 
}

void CGraph::firstNode(Node* n)
{
  // 1, ids
  n->m_id = m_graph_map.size();
  m_seq_id = 0; 
  n->m_seq_id = m_seq_id++; 

  // 2, first pose and add into graph 
  Pose3 origin_priorMean(Eigen::Matrix4d::Identity()); 
  mp_node_values->insert(X(n->m_id), origin_priorMean); 

  // for isam2 
  mp_new_node->insert(X(n->m_id), origin_priorMean); 

  Vector6 s; 
  s << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7;
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(s);
  mp_fac_graph->add(PriorFactor<Pose3>(X(n->m_id), origin_priorMean, priorNoise));

  // for isam2
  mp_new_fac->add(PriorFactor<Pose3>(X(n->m_id), origin_priorMean, priorNoise));
  m_graph_map[n->m_id] = n; 

  // 3, imu part 
  Vector3 priorVelocity; priorVelocity << 0, 0, 0; 
  mp_node_values->insert(V(n->m_id), priorVelocity); 
  mp_new_node->insert(V(n->m_id), priorVelocity); 

  Vector3 biasAcc(-0.241011, 0.245995, -1.460118);
  Vector3 biasGyro(-0.000420, 0.001610, 0.001331); 
  // imuBias::ConstantBias priorBias(biasAcc, biasGyro); 
  imuBias::ConstantBias priorBias; 
  mp_node_values->insert(B(n->m_id), priorBias); 
  mp_new_node->insert(B(n->m_id), priorBias); 

  // prior noise 
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 1e-3); // m/s 
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);  
  
  mp_fac_graph->add(PriorFactor<Vector3>(V(n->m_id), priorVelocity, velocity_noise_model)); 
  mp_fac_graph->add(PriorFactor<imuBias::ConstantBias>(B(n->m_id), priorBias, bias_noise_model));
  
  // for isam2 
  mp_new_fac->add(PriorFactor<Vector3>(V(n->m_id), priorVelocity, velocity_noise_model)); 
  mp_new_fac->add(PriorFactor<imuBias::ConstantBias>(B(n->m_id), priorBias, bias_noise_model));
  
}

bool CGraph::addToGTSAM(MatchingResult& mr, bool set_estimate)
{
  bool pre_exist = mp_node_values->exists(X(mr.m_id_from)); 
  bool cur_exist = mp_node_values->exists(X(mr.m_id_to));
  Pose3 inc_pose(mr.m_trans); 

  // increse transformation from Cam to IMU
  inc_pose = (*mp_s2c)*inc_pose*(*mp_s2c).inverse(); 
  if(!pre_exist && !cur_exist)
  {
    ROS_ERROR("%s two nodes %i and %i both not exist ", __FILE__, mr.m_id_from, mr.m_id_to); 
    return false; 
  }
  else if(!pre_exist)
  {
    ROS_WARN("this case is weired, has not solved it!");
    Pose3 cur_pose = mp_node_values->at<Pose3>(X(mr.m_id_to)); 
    Pose3 pre_pose = cur_pose * inc_pose.inverse(); 
    mp_node_values->insert(X(mr.m_id_from), pre_pose); 
    mp_new_node->insert(X(mr.m_id_from), pre_pose); 
  }
  else if(!cur_exist)
  {
    Pose3 pre_pose = mp_node_values->at<Pose3>(X(mr.m_id_from)); 
    Pose3 cur_pose = pre_pose * inc_pose;
    mp_node_values->insert(X(mr.m_id_to), cur_pose);
    mp_new_node->insert(X(mr.m_id_to), cur_pose); 
  }else if(set_estimate)
  {  
    // set estimate 
    // ROS_WARN("%s should not arrive here %i to set estimate", __FILE__, __LINE__);
    Pose3 pre_pose = mp_node_values->at<Pose3>(X(mr.m_id_from)); 
    Pose3 cur_pose = pre_pose * inc_pose; 
    mp_node_values->update(X(mr.m_id_to), cur_pose);
    mp_new_node->update(X(mr.m_id_to), cur_pose); 
  }
  
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity()*1000000; 
  Eigen::Matrix<double, 6, 6> Adj_Tuc = (*mp_s2c).AdjointMap(); 
  info = Adj_Tuc * mr.m_informationM * Adj_Tuc.transpose();
  
   noiseModel::Gaussian::shared_ptr visual_odometry_noise = noiseModel::Gaussian::Information(info);

  mp_fac_graph->add(BetweenFactor<Pose3>(X(mr.m_id_from), X(mr.m_id_to), inc_pose, visual_odometry_noise)); 
  mp_new_fac->add(BetweenFactor<Pose3>(X(mr.m_id_from), X(mr.m_id_to), inc_pose, visual_odometry_noise)); 
  return true; 
}

bool CGraph::addToGTSAM(gtsam::NavState& new_state, int vid)
{
  if(!mp_node_values->exists(X(vid)))
  {
    mp_node_values->insert(X(vid), new_state.pose());
    mp_new_node->insert(X(vid), new_state.pose());
  }
  mp_node_values->insert(V(vid), new_state.v()); 
  mp_node_values->insert(B(vid), *mp_prev_bias); 
  mp_new_node->insert(V(vid), new_state.v()); 
  mp_new_node->insert(B(vid), *mp_prev_bias); 
  
  return true; 
}

void CGraph::optimizeGraphIncremental()
{
  mp_isam2->update(*mp_new_fac, *mp_new_node); 
  // mp_isam2->update(); 
  (*mp_node_values) = mp_isam2->calculateEstimate(); 
  // clear graph and nodes 
  mp_new_fac->resize(0); 
  mp_new_node->clear(); 
}

void CGraph::optimizeGraphBatch()
{
   LevenbergMarquardtOptimizer optimizer(*mp_fac_graph, *mp_node_values); 
   (*mp_node_values) = optimizer.optimize(); 
}

