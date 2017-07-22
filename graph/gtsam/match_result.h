/*
 *  July 21 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  structure used to store the vo result
 *
 * */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class MatchingResult
{
  public:
    MatchingResult();
    virtual ~MatchingResult(); 
    
    Eigen::Matrix4d m_trans; 
    int m_id_from; 
    int m_id_to; 
    Eigen::Matrix<double, 6, 6> m_informationM; 


  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
