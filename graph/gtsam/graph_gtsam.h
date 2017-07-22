/*  
 *  July 21 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  simple graph wrapper based on gtsam 
 *
 * */

#pragma once

#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <gtsam/base/Matrix.h>
// #include "color.h"
// #include "opencv2/opencv.hpp"

namespace gtsam{
  class NonlinearFactorGraph;
  class Values;
  class Pose3; 
  class NavState;
  class ISAM2Params; 
  class ISAM2; 
  namespace imuBias
  {
    class ConstantBias;
  }
}

namespace cv{
  class Mat; 
}

class MatchingResult; 

class Node
{
  public:
    Node();
    virtual ~Node(); 
    gtsam::Pose3 * mpPose; 
    int m_id; 
    int m_seq_id; 
};


class CGraph 
{
public:
  CGraph();
  virtual ~CGraph(); 
  
  void firstNode(Node* n); 
  bool addToGTSAM(gtsam::NavState& new_state, int vid); 
  bool addToGTSAM(MatchingResult& mr, bool set_estimate = false); 
  void optimizeGraphIncremental(); // isam2  
  void optimizeGraphBatch();       // LM 
  void setupTsc(Eigen::Matrix<double, 4,4>&); 

  int m_seq_id; 
  int m_ver_id; 

  std::map<int, Node*> m_graph_map;  // id -> Node
  gtsam::NonlinearFactorGraph* mp_fac_graph;  // point to the factor graph
  gtsam::Values * mp_node_values;             // point to the node values in the factor graph
  
  gtsam::Pose3 * mp_w2o;        // w: world o:  imu at t=0
  gtsam::Pose3 * mp_s2c;        // c: camera s: imu
  gtsam::imuBias::ConstantBias * mp_prev_bias; 
  gtsam::NavState * mp_prev_state;
 
  // isam2 
  gtsam::ISAM2* mp_isam2; 
  gtsam::ISAM2Params * mp_isam2_param;
  gtsam::NonlinearFactorGraph* mp_new_fac;  // point to the new factor graph
  gtsam::Values * mp_new_node;    // point to the new node values in the factor graph
  void initISAM2Params(); 

};
