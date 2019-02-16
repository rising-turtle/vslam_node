/*
    Sep. 14 2018, He Zhang, hzhang8@vcu.edu 
    
    follow ORB_SLAM's pipeline to process frame 

*/

#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "ORBextractor.h"

using namespace std; 

namespace TEST
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class Frame
{
public:
    Frame(cv::Mat& img, ORB_SLAM2::ORBextractor* extractor, cv::Mat& K, cv::Mat& dist_coeff); 
 
    void AssignFeaturesToGrid();
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    void ComputeImageBounds(const cv::Mat &imLeft);
    void UndistortKeyPoints();

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const;

    ORB_SLAM2::ORBextractor* mpORBExtractor;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;
    
    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mImg;
    cv::Mat mDescriptors;

    // camera intrinsic parameters 
    cv::Mat mK; 
    cv::Mat mDistCoef; 
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    
    // Number of KeyPoints.
    int N;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    static bool mbInitialComputations; 
    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
};


}
