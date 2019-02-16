/*
    Sep. 14 2018, He Zhang, hzhang8@vcu.edu 
    
    follow the ORB_SLAM's pipeline to match orb features given two images 

*/

#include <string.h>
#include <opencv2/opencv.hpp>
// #include "../../../include/System.h"
// #include "../../../include/ORBextractor.h"
#include <iostream>
#include "frame.h"
#include "orb_matcher.h"

using namespace std; 
using namespace cv;
// using namespace ORB_SLAM2; 
ORB_SLAM2::ORBextractor* gpORBextractor; 

string fname1(""); 
string fname2(""); 
string type2str(int type);
void match_orb(); 

int main(int argc, char* argv[])
{
    if(argc <= 2)
    {
	cout<<"usage: ./orb_match image1 image2"<<endl; 
	return 0; 
    }
    fname1 = string(argv[1]); 
    fname2 = string(argv[2]); 

    match_orb(); 
    return 1; 
}

void match_orb()
{	
    double fScaleFactor = 1.2; 
    int nLevels = 8; 
    int fIniThFAST = 7;
    int fMinThFAST = 20; 
    gpORBextractor = new ORB_SLAM2::ORBextractor(2000,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    
    cv::Mat K = cv::Mat::eye(3,3,CV_32F); 
    K.at<float>(0,0) = 190.9785; // fx;
    K.at<float>(1,1) = 190.9733; // fy;
    K.at<float>(0,2) = 254.9317; // cx;
    K.at<float>(1,2) = 256.8974; // cy;

    cv::Mat DistCoef(5,1,CV_32F);
    DistCoef.at<float>(0) = -0.239552 ; // -0.2847798; // fSettings["Camera.k1"];
    DistCoef.at<float>(1) = 0.037056;  // 0.08245052; // fSettings["Camera.k2"];
    DistCoef.at<float>(2) = 3.5763956e-6; // -1.0946156e-6 ; // fSettings["Camera.p1"];
    DistCoef.at<float>(3) = -1.4032145e-5; // 4.78701072e-6;  // fSettings["Camera.p2"];
    DistCoef.at<float>(4) = 0.;// -0.0104085; //k3;

    cv::Mat img1 = cv::imread(fname1.c_str(), IMREAD_GRAYSCALE); 
    cv::Mat img2 = cv::imread(fname2.c_str(), IMREAD_GRAYSCALE); 
    TEST::Frame frame1(img1, gpORBextractor, K, DistCoef); 
    TEST::Frame frame2(img2, gpORBextractor, K, DistCoef); 

    TEST::ORBmatcher matcher(0.6); 
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched(frame2.mvKeysUn.size());
    for(size_t i=0; i<frame2.mvKeysUn.size(); i++)
	mvbPrevMatched[i]=frame2.mvKeysUn[i].pt;

    int nmatches = matcher.SearchForInitialization(frame1,frame2, mvbPrevMatched, mvIniMatches,100);

    cout <<"orb_match.cc: serachForInitialization found nmatches : "<<nmatches<<endl;
    
    // nmatches = matcher.SearchByPnP(frame1, frame2, mvIniMatches); 
    nmatches = matcher.SearchByPnP2(frame1, frame2, mvIniMatches); 
    
    cout <<"orb_match.cc: serachByPnP found nmatches : "<<nmatches<<endl;
}

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


