
#pragma once

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include "frame.h"

namespace TEST{

class ORBmatcher
{
public:
    ORBmatcher(float nnratio=0.6); 
    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2,std::vector<cv::Point2f>& vnPreMatches, std::vector<int> &vnMatches12, int windowSize=10);
    int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
    int SearchByPnP(Frame& F1, Frame& F2, std::vector<int>& vnMatches12); 
    int SearchByPnP2(Frame& F1, Frame& F2, std::vector<int>& vnMatches12); 

    float mfNNratio; 
};


}
