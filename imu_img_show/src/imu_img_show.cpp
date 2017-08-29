/*
 *  Aug. 28, 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  display img and imu's gyro 
 *
 * */

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>
#include <string>
#include <Eigen/Core>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

using namespace std;

ros::Publisher g_imu_pub;

void publishRPY(Eigen::Vector3d & rpy);

#define D2R(d) ((d)/180.*M_PI)

// this is just a workbench. most of the stuff here will go into the Frontend class.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_img_show"); 
  ros::NodeHandle n; 
  g_imu_pub = n.advertise<std_msgs::Float32MultiArray>("/euler_msg", 10); 

  if (argc != 2) 
  {
    cerr <<
    "Usage: ./" << argv[0] << " dataset-folder";
    return -1;
  }

  // the folder path
  std::string path(argv[1]);

  // open the IMU file
  std::string line;
  // std::ifstream imu_file(path + "/imu0/data.csv");
  string fimu_name = path + "/imu_vn100.log";
  std::ifstream imu_file(fimu_name.c_str()); 

  if (!imu_file.good()) 
  {
    cerr << "no imu file found at " << fimu_name;
    return -1;
  }
  int number_of_lines = 0;
  while (std::getline(imu_file, line))
    ++number_of_lines;
    cout << "No. IMU measurements: " << number_of_lines-1;
  if (number_of_lines - 1 <= 0) {
    cerr << "no imu messages present in " << fimu_name;
    return -1;
  }
  // set reading position to second line
  imu_file.clear();
  imu_file.seekg(0, std::ios::beg);
  // std::getline(imu_file, line);

  int num_camera_images = 0;
  std::vector < std::string >  image_names;
  // std::string folder(path + "/cam" + std::to_string(i) + "/data");
  std::string folder(path + "/color");

  for (auto it = boost::filesystem::directory_iterator(folder);
      it != boost::filesystem::directory_iterator(); it++) 
  {
    if (!boost::filesystem::is_directory(it->path())) 
    {  //we eliminate directories
      num_camera_images++;
      image_names.push_back(it->path().filename().string());
    } else {
      continue;
    }
  }
  if (num_camera_images == 0) 
  {
    cerr << "no images at " << folder;
    return 1;
  }


  // the filenames are not going to be sorted. So do this here
  std::sort(image_names.begin(), image_names.end());
  
  std::vector<std::string>::iterator cam_iterator = image_names.begin();

  ros::Time start(0, 0); 
  while (true && ros::ok()) 
  {
    // check if at the end
    if (cam_iterator == image_names.end()) 
    {
      std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
      cv::waitKey();
      return 0;
    }

    cv::Mat rgb = cv::imread(path + "/color/" + *cam_iterator, -1); 
    cv::Mat filtered = rgb.clone(); 
      if(rgb.channels() == 3)
        cv::cvtColor(rgb, filtered, CV_BGR2GRAY);
      // cv::imshow("rgb", rgb); 
      // cv::waitKey(3); 
    
      std::string nanoseconds = cam_iterator->substr(
          // cam_iterators.at(i)->size() - 13, 9);
          11, 9); 
      std::string seconds = cam_iterator->substr(
          0, 10); // 13 since we have one more '.' 
      ros::Time t = ros::Time(std::stoi(seconds), std::stoi(nanoseconds));
      if (start == ros::Time(0, 0)) {
        start = t;
      }

      // get all IMU measurements till then
      ros::Time t_imu = start;
      Eigen::Vector3d ypr; 
      do {
        if (!std::getline(imu_file, line)) 
        {
          std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
          cv::waitKey();
          return 0;
        }

        // handle imu data 
        std::stringstream stream(line);
        std::string s;
        // std::getline(stream, s, ',');
        std::getline(stream, s, '\t');
        std::string nanoseconds = s.substr(11, 9);
        // std::string nanoseconds = s.substr(s.size() - 6, 6);
        std::string seconds = s.substr(0, 10);
        // std::string seconds = s.substr(0, s.size() - 7);

        Eigen::Vector3d acc;
        for (int j = 0; j < 3; ++j) {
          // std::getline(stream, s, ',');
          std::getline(stream, s, '\t');
          acc[j] = std::stof(s);
        }

        Eigen::Vector3d gyr;
        for (int j = 0; j < 3; ++j) {
          // std::getline(stream, s, ',');
          std::getline(stream, s, '\t');
          gyr[j] = std::stof(s);
        }

        // obtain angle
        for(int j=0; j<3; ++j)
        {
          std::getline(stream, s, '\t'); 
          ypr[j] = std::stof(s); 
        }

        // t_imu = okvis::Time(std::stoi(seconds), std::stoi(nanoseconds));
        ros::Time t_imu = ros::Time(std::stoi(seconds), std::stoi(nanoseconds));  
        cout <<"t_imu = "<<t_imu<<" t = "<<t<< ((t_imu<=t)?"t_imu <= t":"t_imu > t") <<endl;
        
        if(t_imu > t) break; 

      } while (true); // (t_imu <= t);

      cam_iterator++;

      // display and publish rpy data
      publishRPY(ypr); 
      
      cv::imshow("grey", filtered); 
      cv::waitKey(0); 
  }

  std::cout << std::endl << std::flush;
  return 0;
}


void publishRPY(Eigen::Vector3d & ypr)
{
  static bool first = true; 
  static Eigen::Vector3d ypr_first; 

  if(first)
  {
    ypr_first = ypr; 
    first = false; 
  }

  std_msgs::Float32MultiArray msg; 
  msg.data.resize(3); 
  msg.data[0] = D2R(ypr(2) - ypr_first(2)); msg.data[1] = D2R(ypr(1) - ypr_first(1)); msg.data[2] = D2R(ypr(0) - ypr_first(0)); 
  g_imu_pub.publish(msg); 
  ROS_WARN("%s: publish rpy %f %f %f",__FILE__, ypr(0), ypr(1), ypr(2));
  ros::spinOnce(); 
  return; 
}
