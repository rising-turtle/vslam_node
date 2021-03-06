/*
 *  Nov. 1 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  Input: 
 *    *.cvs: trajectory file generated by vins-mono
 *    [dir]: directory points to color and depth images
 *  Output:
 *    trajectory.ply: trajectory file stored as [.ply] 
 *    map.ply: 3D point cloud 
 *
 * */

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <tf/tf.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std; 

string g_traj_log("/home/davidz/work/ros/indigo/src/vslam_node/VINS-mono/result/ETAS_F4_640_30.csv"); 
string g_img_dir("/home/davidz/work/data/ETAS_F4_640_30"); 
string g_traj_ply("trajectory.ply"); 
string g_map_ply("map.ply"); 

struct trajItem
{
  int id; int sid; // sequence id 
  // double timestamp; 
  string timestamp; 
  float px, py, pz; // euler angle roll, pitch, yaw; 
  float qx, qy, qz, qw; // quaternion
};

struct obsItem
{
  struct trajItem pose; 
  string frgb; // rgb's filename 
  string fdpt; // dpt's filename
};

// typedef pcl::PointXYZ   Point;
typedef pcl::PointXYZRGBA  Point;
typedef pcl::PointCloud<Point>  Cloud;  
typedef typename pcl::PointCloud<Point>::Ptr CloudPtr;

void doIt(); // Let's do it

// load imgs, trajectory 
bool loadTrajLog(vector<trajItem>& );  // load input trajectory log file
bool loadImgFiles(vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt, vector<string>& mv_ir1, vector<string>& mv_ir2);
bool associateItem(vector<obsItem>&, vector<trajItem>& , vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt);
bool associateData(vector<obsItem>& vobs); // load data and associate filename

// generate point cloud 
void generatePC(CloudPtr& all_pc, vector<obsItem>& vobs);
void generatePointCloud(cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, int skip =1); 
tf::Transform computeTrans(trajItem pose); 
void transformAsMatrix(tf::Transform T, Eigen::Matrix4f& eigen_T);
template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size);

// save trajectory and map into *.ply respectively 
void saveTrajPly(vector<obsItem>& vobs); 
void saveMapPly(CloudPtr& pc);
void headerPLY(std::ofstream& ouf, int vertex_number);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vins_mapping"); 
  ros::NodeHandle n; 

  // 1. trajectory 
  ros::NodeHandle np("~"); 
  np.param("input_traj_log", g_traj_log, g_traj_log); 
  np.param("input_img_directory", g_img_dir, g_img_dir); 
  np.param("output_traj_ply", g_traj_ply, g_traj_ply); 
  np.param("output_map_ply", g_map_ply, g_map_ply); 

  /*np.param("downsample_skip", skip, skip); 
  np.param("trajectory_skip", t_skip, t_skip); 
  np.param("top_left_u", su, 0); 
  np.param("top_left_v", sv, 0); 
  np.param("bot_right_u", eu, rs_F200.m_cols); 
  np.param("bot_right_v", ev, rs_F200.m_rows); 
  */
    ROS_WARN("usage: ./vins-mapping [traj_file] [img_dir]"); 

  if(argc >= 2)
    g_traj_log = string(argv[1]);
  if(argc >= 3)
    g_img_dir = string(argv[2]); 

  // 2. ...
  doIt(); 

  return 0; 
}

void doIt() // first test match 
{
  // 1. associate files 
  vector<obsItem> vobs;
  if(!associateData(vobs)) return; 

  // 2. generate point cloud 
  CloudPtr pc(new Cloud);
  generatePC(pc, vobs);

  // 3. save point cloud into *.ply file
  saveTrajPly(vobs); 
  saveMapPly(pc);

  return ; 
}

void saveTrajPly(vector<obsItem>& vobs)
{
  ofstream ouf(g_traj_ply.c_str()); 
  headerPLY(ouf, vobs.size()); 
  for(int i=0; i<vobs.size(); i++)
  {
    ouf<<vobs[i].pose.px<<" "<<vobs[i].pose.py<<" "<<vobs[i].pose.pz<<" 0 255 0"<<endl; // green
  }
  ouf.flush();
  ouf.close();
  return; 
}
void saveMapPly(CloudPtr& pc)
{
  ofstream ouf(g_map_ply.c_str());
  headerPLY(ouf, pc->points.size()); 
  for(int i=0; i<pc->points.size(); i++)
  {
    Point& pt = pc->points[i]; 
    ouf<<pt.x<<" "<<pt.y<<" "<<pt.z<<" "<<(int)pt.r<<" "<<(int)pt.g<<" "<<(int)pt.b<<endl;
  }
  ouf.flush();
  ouf.close();
  return ; 
}

void headerPLY(std::ofstream& ouf, int vertex_number)
{
  ouf << "ply"<<endl
    <<"format ascii 1.0"<<endl
    <<"element vertex "<<vertex_number<<endl
    <<"property float x"<<endl
    <<"property float y"<<endl
    <<"property float z"<<endl
    <<"property uchar red"<<endl
    <<"property uchar green"<<endl
    <<"property uchar blue"<<endl
    <<"end_header"<<endl;
}

void generatePC(CloudPtr& all_pc, vector<obsItem>& vobs)
{
  // 1. load rgb and dpt, and then generate point cloud 
  cv::Mat rgb, dpt; 

  for(int i=0; i<vobs.size() && ros::ok(); i++)
  {
    ROS_INFO("vins-mapping: handle (%d/%d) obs", i, vobs.size());
    obsItem& obs = vobs[i]; 
    string frgb = g_img_dir + "/" + obs.frgb; 
    string fdpt = g_img_dir + "/" + obs.fdpt; 
    rgb = cv::imread(frgb.c_str(), -1);
    // ROS_INFO("load dpt file: %s", fdpt.c_str());
    dpt = cv::imread(fdpt.c_str(), -1); // this depth img has float type
    CloudPtr loc_pc(new Cloud); 
    CloudPtr glo_pc(new Cloud);
    generatePointCloud(rgb, dpt, loc_pc);

    // transform point cloud into global coordinate
    tf::Transform T = computeTrans(obs.pose); 
    Eigen::Matrix4f eigen_T; 
    transformAsMatrix(T, eigen_T);

    // NOTE: Here, eigen_T should include Tu2c, which means p_o = To2u * Tu2c * p_c, since 
    // for the ROBOCANE dataset Ru2c of Tu2c is Identity matrix, t_u2c of Tu2c is [0.063, -0.001, 0.018], 
    // which has not been taken into consideration
    pcl::transformPointCloud(*loc_pc, *glo_pc, eigen_T);
    
    *all_pc += *glo_pc; 
    
    // stringstream ss;
    // ss <<"Node_"<<i<<".pcd";
    // pcl::io::savePCDFile(ss.str().c_str(), *loc_pc);
    // cv::imshow("dpt", dpt); 
    // cv::waitKey(10); 
    // cv::imshow("rgb", rgb);
    // cv::waitKey(0);
    

    // voxel grid filter out all_pc, 
    // CloudPtr filterPC(new Cloud); 
    // filterPointCloud<Point>(all_pc, filterPC, 0.01); 
    // all_pc.swap(filterPC); 
    
  }
 
  // pcl::io::savePCDFile("all_pc.pcd", *all_pc); 
  return ; 
}

void transformAsMatrix(tf::Transform T, Eigen::Matrix4f& eigen_T)
{
  tf::Matrix3x3 R = T.getBasis(); 
  tf::Vector3 t = T.getOrigin(); 
  eigen_T = Eigen::Matrix4f::Identity(); 
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
    {
      eigen_T(i,j) = R[i][j]; 
    }
  eigen_T(0, 3) = t[0]; 
  eigen_T(1, 3) = t[1]; 
  eigen_T(2, 3) = t[2]; 
  // cout <<"eigen_T: "<<endl<<eigen_T<<endl;
  return ; 
}

tf::Transform computeTrans(trajItem p)
{
  tf::Quaternion quat(p.qx, p.qy, p.qz, p.qw);
  tf::Vector3 t(p.px, p.py, p.pz); 
  tf::Transform T(quat, t); 
  return T;
} 

template<typename PointT>
void filterPointCloud(typename pcl::PointCloud<PointT>::Ptr& in,typename pcl::PointCloud<PointT>::Ptr& out, double _voxel_size)
{
  // voxelgrid filter
  pcl::VoxelGrid<PointT> vog;
  vog.setInputCloud(in); 
  vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
  vog.filter(*out);
}

void generatePointCloud(cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pc, int skip ) 
{  
  double z; 
  double px, py, pz; 
  int height = rgb.rows/skip; 
  int width = rgb.cols/skip; 
  int N = (rgb.rows/skip)*(rgb.cols/skip); 
  // CamModel rs_R200(615.426, 625.456, 318.837, 240.594); 
  double fx = 615.426; 
  double fy = 625.456; 
  double cx = 318.837;
  double cy = 240.594;

  pc->points.reserve(N); 
  // pc.width = width; 
  // pc.height = height; 

  unsigned char r, g, b; 
  int pixel_data_size = 3; 
  if(rgb.type() == CV_8UC1)
  {
    pixel_data_size = 1; 
  }
  
  int color_idx; 
  char red_idx = 2, green_idx =1, blue_idx = 0;

  Point pt; 

  for(int v = 0; v<rgb.rows; v+=skip)
  for(int u = 0; u<rgb.cols; u+=skip)
  {
    // Point& pt = pc.points[v*width + u]; 
    z = dpt.at<float>((v), (u));
    if(std::isnan(z) || z <= 0.1 || z >= 5) 
    {
      continue; 
    }

    // compute point 
    pz = z; 
    px = ((u-cx)/fx)*pz;
    py = ((v-cy)/fy)*pz;

    pt.x = px;  pt.y = py;  pt.z = pz; 
    color_idx = (v*rgb.cols + u)*pixel_data_size;
    if(pixel_data_size == 3)
    {
      r = rgb.at<uint8_t>(color_idx + red_idx);
      g = rgb.at<uint8_t>(color_idx + green_idx); 
      b = rgb.at<uint8_t>(color_idx + blue_idx);
    }else{
      r = g = b = rgb.at<uint8_t>(color_idx); 
    }
    pt.r = r; pt.g = g; pt.b = b; 
    pc->points.push_back(pt); 
  }
  pc->height = 1; 
  pc->width = pc->points.size(); 
  return ;
}


bool associateData(vector<obsItem>& vobs)
{
  // 1. load trajectory 
  vector<struct trajItem> vts; 
  if(!loadTrajLog(vts)) return false; 

  // 2. load img files 
  vector<string> mv_rgb; 
  vector<string> mv_timestamp;
  vector<string> mv_dpt; 
  vector<string> mv_ir1; 
  vector<string> mv_ir2; 
  if(!loadImgFiles(mv_timestamp, mv_rgb, mv_dpt, mv_ir1, mv_ir2)) return false;

  // 3. match traj's timestamp with img data's timestamp 
  // vector<obsItem> vobs; 
  return associateItem(vobs, vts, mv_timestamp, mv_rgb, mv_dpt); 
}

inline bool sameTime(string t1, string t2)
{
    double dt1 = atof(t1.c_str()); 
    double dt2 = atof(t2.c_str()); 
    if(fabs(dt1-dt2) < 1e-3) return true; 
    return false;
}

bool associateItem(vector<obsItem>& vobs, vector<trajItem>& vts, vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt)
{
  // test 
  // ofstream ouf("associate_file_name_result.log"); 
  
  for(int i=0, lj = 0; i<vts.size(); i++)
  {
    string ti = vts[i].timestamp; 
    int j = lj; 
    obsItem obs; 
    obs.pose = vts[i]; 
    for(; j<mv_timestamp.size(); j++)
    {
      string tj = mv_timestamp[j]; 
      // if(tj == ti)
      if(sameTime(tj, ti))
      {
        obs.frgb = mv_rgb[j]; 
        obs.fdpt = mv_dpt[j]; 
        // ouf <<ti<<"\t"<<obs.frgb<<"\t"<<obs.fdpt<<endl;
        break; 
      }
      if(j == mv_timestamp.size() - 1)
      {
	ROS_ERROR("ti = %s tj = %s cannot find match !", ti.c_str(), tj.c_str()); 
      }
    }
    lj = j; 
    if(j == mv_timestamp.size())
    {
      ROS_ERROR("vins-mapping: j = mv_timestamp.size() something is wrong!");
      return false; 
    }
    vobs.push_back(obs);
  }
  ROS_INFO("vins-mapping: succeed to associate %d obsItems", vobs.size()); 
  return true; 
}

bool loadTrajLog(vector<struct trajItem>& t)  // load input trajectory log file
{
  ifstream inf(g_traj_log.c_str()); 
  if(!inf.is_open())
  {
    ROS_ERROR("vins-mapping: failed to load input_traj_log: %s", g_traj_log.c_str()); 
    return false; 
  }
/*
  char buf[4096]; 
  char b[21]={0}; 
  while(inf.getline(buf, 4096))
  {
    trajItem ti;
    sscanf(buf, "%s,%f,%f,%f,%f,%f,%f,%f", b, &ti.px, &ti.py, &ti.pz, &ti.qx, &ti.qy, &ti.qz, &ti.qw); 
    ti.timestamp = string(b).substr(0, 10) + "." + string(b).substr(10, 5); // here, discard values < e-5 second
    t.push_back(ti); 
  }*/
  
  char buf[4096]; 

  while(inf.getline(buf, 4096))
  {
    char* p = strtok(buf, " ,"); 
    if(p == NULL)
    {
      break; 
    }
    trajItem ti;
    ti.timestamp = string(p).substr(0, 10) + "." + string(p).substr(10, 4); // here, discard values < e-5 second
    p = strtok(NULL, " ,");  ti.px = atof(p); 
    p = strtok(NULL, " ,");  ti.py = atof(p); 
    p = strtok(NULL, " ,");  ti.pz = atof(p); 
    p = strtok(NULL, " ,");  ti.qw = atof(p); 
    p = strtok(NULL, " ,");  ti.qx = atof(p); 
    p = strtok(NULL, " ,");  ti.qy = atof(p); 
    p = strtok(NULL, " ,");  ti.qz = atof(p); 
    t.push_back(ti);
  }

  ROS_INFO("vins-mapping: obtain %d items", t.size()); 
  return true; 
}

bool loadImgFiles(vector<string>& mv_timestamp, vector<string>& mv_rgb, vector<string>& mv_dpt, vector<string>& mv_ir1, vector<string>& mv_ir2)
{
  ifstream fAssociation;
  string strAssociationFilename = g_img_dir + "/timestamp.txt"; 
  fAssociation.open(strAssociationFilename.c_str());
  if(!fAssociation.is_open())
  {
    ROS_ERROR("vins-mapping: failed to open file %s", strAssociationFilename.c_str()); 
    return false; 
  }
  mv_rgb.clear(); 
  mv_dpt.clear(); 
  mv_ir1.clear(); 
  mv_ir2.clear(); 
  mv_timestamp.clear(); 

  string s;
  getline(fAssociation,s); // delete the first line  

  while(!fAssociation.eof())
  {
    string s;
    getline(fAssociation,s);// 
    if(!s.empty())
    {
      stringstream ss;
      ss << s;
      string t; 
      string sRGB, sD, sr1, sr2;
      ss >> t;
      mv_timestamp.push_back(t.substr(0,15));
      ss >> sRGB;
      mv_rgb.push_back(sRGB);
      ss >> t;
      ss >> sD;
      mv_dpt.push_back(sD);
      ss >> t; 
      ss >> sr1; 
      mv_ir1.push_back(sr1); 
      ss >> t; 
      ss >> sr2; 
      mv_ir2.push_back(sr2); 
    }
  }
  
  ROS_INFO("vins-mapping: succeed to load %d img file names", mv_timestamp.size());
  
  return true; 
}



