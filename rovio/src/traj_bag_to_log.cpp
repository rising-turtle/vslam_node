/*
    July 6th 2018, He Zhang, hzhang8@vcu.edu 
    
    convert the trajectory file from .bag to .log

*/


#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <nav_msgs/Odometry.h>

using namespace std; 

string g_input_bag = ""; 
string g_output_log = "result.log"; 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_bag_to_log");
    ros::NodeHandle nh;

    if(argc >= 2)
    {
	g_input_bag = string(argv[1]); 
	if(argc >= 3)
	    g_output_log = string(argv[2]); 
    }

    cout <<"traj_bag_to_log: convert "<<g_input_bag<<" into "<<g_output_log<<endl; 
    
    // read bag  
    rosbag::Bag bagIn;
    bagIn.open(g_input_bag, rosbag::bagmode::Read);
    
    // setup topic 
    std::vector<string> topics; 
    string odo_topic("/rovio/odometry");
    topics.push_back(odo_topic); 
    rosbag::View view(bagIn, rosbag::TopicQuery(topics)); 
    
    // open output 
    ofstream ouf(g_output_log.c_str()); 
    if(!ouf.is_open())
    {
	cout <<"traj_bag_to_log: failed to open output file: "<<g_output_log<<endl; 
	return -1; 
    }
    
    // iterate the message
    double px, py, pz, qw, qx, qy, qz; 
    long int sec, nsec; 
    for(rosbag::View::iterator it = view.begin();it != view.end() && ros::ok();it++)
    {
	if(it->getTopic() == odo_topic)
	{
	    nav_msgs::Odometry::ConstPtr odoMsg = it->instantiate<nav_msgs::Odometry>(); 
	    px = odoMsg->pose.pose.position.x; 
	    py = odoMsg->pose.pose.position.y; 
	    pz = odoMsg->pose.pose.position.z; 
	    qw = odoMsg->pose.pose.orientation.w; 
	    qx = odoMsg->pose.pose.orientation.x; 
	    qy = odoMsg->pose.pose.orientation.y; 
	    qz = odoMsg->pose.pose.orientation.z; 
	    sec = odoMsg->header.stamp.sec; 
	    nsec = odoMsg->header.stamp.nsec; 
	    ros::Time t(sec, nsec); 
	    ouf<<std::fixed<<t<<","<<px<<","<<py<<","<<pz<<","<<qw<<","<<qx<<","<<qy<<","<<qz<<endl; 
	}
    }

    ouf.close(); 
    return 0; 
}









