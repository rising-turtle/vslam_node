#include "orb_matcher.h"
#include <iostream>

using namespace std;
using namespace cv; 

namespace TEST{

#define TH_LOW 70

ORBmatcher::ORBmatcher(float nnratio):mfNNratio(nnratio){}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f>& vbPrevMatched, std::vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);
    
    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);
    
    int cnt_lev = 0; 
    int cnt_empty = 0; 
    int cnt_dist = 0; 
    int cnt_ort = 0;
    int cnt_dist_ratio = 0;

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
	cv::KeyPoint kp1 = F1.mvKeysUn[i1];
	int level1 = kp1.octave;
	if(level1>0)
	{
	    ++cnt_lev; 
	    continue;
	}

	vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

	if(vIndices2.empty())
	{
	    ++cnt_empty; 
	    continue;
	}
	cv::Mat d1 = F1.mDescriptors.row(i1);

	int bestDist = INT_MAX;
	int bestDist2 = INT_MAX;
	int bestIdx2 = -1;

	for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
	{
	    size_t i2 = *vit;

	    cv::Mat d2 = F2.mDescriptors.row(i2);

	    int dist = DescriptorDistance(d1,d2);

	    if(vMatchedDistance[i2]<=dist)
	    {
		continue;
	    }
	    if(dist<bestDist)
	    {
		bestDist2=bestDist;
		bestDist=dist;
		bestIdx2=i2;
	    }
	    else if(dist<bestDist2)
	    {
		bestDist2=dist;
	    }
	}

	if(bestDist<=TH_LOW)
	{
	    if(bestDist<(float)bestDist2*mfNNratio)
	    {
		if(vnMatches21[bestIdx2]>=0)
		{
		    vnMatches12[vnMatches21[bestIdx2]]=-1;
		    nmatches--;
		}
		vnMatches12[i1]=bestIdx2;
		vnMatches21[bestIdx2]=i1;
		vMatchedDistance[bestIdx2]=bestDist;
		nmatches++;
	    }else{
		++cnt_dist_ratio;
	    }
	}else{++cnt_dist;}

    }
    cout<<"ORBmatcher.cc: matches "<<F1.mvKeysUn.size()<<" cnt_lev: "<<cnt_lev<<" cnt_empty: "<<cnt_empty<<" cnt_dist: "<<cnt_dist
	<<" cnt_dist_ratio: "<<cnt_dist_ratio<<" cnt_ort: "<<cnt_ort<<" matches: "<<nmatches<<endl;

    return nmatches;
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

int ORBmatcher::SearchByPnP2(Frame& F1, Frame& F2, std::vector<int>& vnMatches12)
{
    const double ransac_thresh = 4.5f; // RANSAC inlier threshold
    const double nn_match_ratio = 0.9f; // Nearest-neighbour matching ratio

    string name("ORB");
    // Ptr<ORB> orb = ORB::create(name);
    Ptr<FeatureDetector> orb_detect = FeatureDetector::create(name);
    
    // orb->setMaxFeatures(1000);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    cv::Mat cimg1, cimg2; 

    cv::imshow("F1 IMG", F1.mImg);
    cv::waitKey(0); 
    cv::imshow("F2 IMG", F2.mImg); 
    cv::waitKey(0); 

    Mat desc1, desc2;
    vector<KeyPoint> kp1, kp2;
    orb_detect->detect(F1.mImg, kp1); 
    orb_detect->detect(F2.mImg, kp2); 

    Ptr<Feature2D> descriptorExtractor = Feature2D::create("ORB");
    descriptorExtractor->compute(F1.mImg, kp1, desc1); 
    descriptorExtractor->compute(F2.mImg, kp2, desc2);
    cout<<"what after !"<<endl;

    // orb->detectAndCompute(F1.mImg, noArray(), kp1, desc1);
    // orb->detectAndCompute(F2.mImg, noArray(), kp2, desc2); 
    // (*orb)(cimg1, cv::Mat(), kp1, desc1); 
    // (*orb)(cimg2, cv::Mat(), kp2, desc2); 
    // kp1 = F1.mvKeys; kp2 = F2.mvKeys;
    // desc1 = F1.mDescriptors; desc2 = F2.mDescriptors;

    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(desc1, desc2, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++) {
	if(matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
	    matched1.push_back(kp1[matches[i][0].queryIdx]);
	    matched2.push_back(kp2[matches[i][0].trainIdx]);
	}
    }
    Mat inlier_mask, homography;
    vector<KeyPoint> inliers1, inliers2;
    vector<DMatch> inlier_matches;
    vector<Point2f> pts1, pts2; 
    for(int i=0; i<matched1.size(); i++) 
    {
	pts1.push_back(matched1[i].pt); 
    }
    for(int i=0; i<matched2.size(); i++)
    {
	pts2.push_back(matched2[i].pt); 
    }
    if(matched1.size() >= 4) {
	homography = findHomography(pts1, pts2,
		RANSAC, ransac_thresh, inlier_mask);
    }

    for(unsigned i = 0; i < matched1.size(); i++) {
	if(inlier_mask.at<uchar>(i)) {
	    int new_i = static_cast<int>(inliers1.size());
	    inliers1.push_back(matched1[i]);
	    inliers2.push_back(matched2[i]);
	    inlier_matches.push_back(DMatch(new_i, new_i, 0));
	}
    }
    
    Mat res;
    cimg1 = F1.mImg; 
    cimg2 = F2.mImg;
    drawMatches(cimg1, inliers1, cimg2, inliers2,
                inlier_matches, res,
                Scalar(255, 0, 0), Scalar(255, 0, 0));
    imshow("res", res);
    waitKey(0); 

    return inlier_matches.size(); 
}

int ORBmatcher::SearchByPnP(Frame& F1, Frame& F2, std::vector<int>& vnMatches12)
{
    //-- Step 2: Matching descriptor vectors using FLANN matcher
    // cv::FlannBasedMatcher matcher;
    // std::vector< cv::DMatch > matches;
    // matcher.match( F1.mDescriptors, F2.mDescriptors, matches );

    // create BFMatcher object
    // cv::BFMatcher bf = cv::BFMatcher(cv::NORM_HAMMING, true);
    std::vector< cv::DMatch > matches; 
    // bf.match(F1.mDescriptors, F2.mDescriptors, matches); 
    for(int qi=0; qi<F2.mDescriptors.rows; qi++)
    {
	int min_dis = 256; 
	int best_ti = -1; 
	cv::Mat queryR = F2.mDescriptors.row(qi); 
	for(int ti=0; ti<F1.mDescriptors.rows; ti++)
	{
	    cv::Mat trainR = F1.mDescriptors.row(ti); 
	    int dis = DescriptorDistance(queryR, trainR); 
	    if(best_ti < 0 || dis < min_dis)
	    {
		best_ti = ti; 
		min_dis = dis; 
	    }
	}
	if(min_dis < TH_LOW)
	{
	    cv::DMatch m; 
	    m.queryIdx = qi; 
	    m.trainIdx = best_ti; 
	    m.distance = min_dis; 
	    matches.push_back(m); 
	}
    }
    int max_dist = 0; int min_dist = 1000;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < matches.size(); i++ )
    { int dist = (int)matches[i].distance;
	if( dist < min_dist ) min_dist = dist;
	if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %d \n", max_dist );
    printf("-- Min dist : %d \n", min_dist );
    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< cv::DMatch > good_matches;
//    for( int i = 0; i < F1.mDescriptors.rows; i++ )
//    { if( matches[i].distance <= 0.5*max_dist )
//	{ 
//	    good_matches.push_back( matches[i]); 
//	    cout<<"matches[i]: "<<" queryIdx: "<<matches[i].queryIdx<<" trainIdx: "<<matches[i].trainIdx<<" distance "<<matches[i].distance<<endl; 
//	}
//    }
    good_matches = matches; 

    // 
    //-- Draw only "good" matches
    cv::Mat img_matches;
    // cv::drawMatches( F1.mImg, F1.mvKeys, F2.mImg, F2.mvKeys,
	//    good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
	//    vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::imshow("F1 IMG", F1.mImg);
    cv::waitKey(0); 
    cv::imshow("F2 IMG", F2.mImg); 
    cv::waitKey(0); 
    cv::Mat cimg1, cimg2; 
    cv::cvtColor(F1.mImg, cimg1, CV_GRAY2BGR); 
    cv::cvtColor(F2.mImg, cimg2, CV_GRAY2BGR); 

    cv::drawMatches(cimg1, F1.mvKeys, cimg2, F2.mvKeys, good_matches, img_matches, cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 0)); 
    //-- Show detected matches
    cv::imshow( "Good Matches", img_matches );
    for( int i = 0; i < (int)good_matches.size(); i++ )
    { printf( "-- Good Match [%d] queryIdx: %d  -- trainIdx: %d --distance: %d \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx, (int)good_matches[i].distance ); }
    cv::waitKey(0);

    return good_matches.size();
}




}
