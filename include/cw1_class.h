/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <ros/time.h>

// messages includes
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Moveit includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
// TF specific includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
// include services from the spawner package - we will be responding to these
#include "cw1_world_spawner/Task1Service.h"
#include "cw1_world_spawner/Task2Service.h"
#include "cw1_world_spawner/Task3Service.h"

// // include any services created in this package
// #include "cw1_team_x/example.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

class cw1
{
public:

  /* ----- class member functions ----- */

  // constructor
  cw1(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw1_world_spawner::Task1Service::Request &request,
    cw1_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw1_world_spawner::Task2Service::Request &request,
    cw1_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw1_world_spawner::Task3Service::Request &request,
    cw1_world_spawner::Task3Service::Response &response);
  
  // added
  void
  pcCallBack (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);
  bool 
  moveArm(geometry_msgs::Pose target_pose);
  bool
  moveGripper(float width);
  void
  addCollisionObject(std::string object_name,geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,geometry_msgs::Quaternion orientation);
  void
  removeCollisionObject(std::string object_name);
  bool
  pick(geometry_msgs::Point position);
  bool
  place(geometry_msgs::Point position);
  bool
  arm_go(geometry_msgs::Point position);
  int
  getNearestPoint(const PointC& cloud, const pcl::PointXYZRGBA& position);
  void
  pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);
  void
  applyPT (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);
  void
  findNormals (PointCPtr &in_cloud_ptr);
  void
  segPlane (PointCPtr &in_cloud_ptr);
  void
  segCylind (PointCPtr &in_cloud_ptr);
  void
  findCylPose (PointCPtr &in_cloud_ptr);
  bool 
  findColor (PointCPtr &in_cloud_ptr);
 
  // std::string
  // findCylColor(PointCPtr &in_cloud_ptr, geometry_msgs::PointStamped &positon);
  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // added for task 1
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  float gripper_open_ = 80e-3;
  float gripper_closed_ = 0.0;

  double angle_offset_ = 3.14159 / 4.0;
  double z_offset_ = 0.125;
  double approach_distance_ = 0.1;

  //added for task 2
  std::string g_input_pc_frame_id;
  pcl::PCLPointCloud2 g_pcl_pc;
  PointCPtr g_cloud_ptr;
  tf::TransformListener listener_;

  //
  ros::Publisher g_pub_cloud;
  ros::Publisher g_pub_seg1;
  ros::Publisher g_pub_seg2;
  ros::Publisher g_pub_seg3;
  ros::Publisher g_pub_seg4;
  ros::Publisher g_pub_seg5;
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;
  //
  pcl::PassThrough<PointT> g_pt; //Pass Through filter.
  PointCPtr g_cloud_filtered;
  double g_pt_thrs_min, g_pt_thrs_max;
  //
  pcl::NormalEstimation<PointT, pcl::Normal> g_ne;
  pcl::search::KdTree<PointT>::Ptr g_tree;
  double g_k_nn;
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals;
  //   
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> g_seg;  // SAC segmentation.
  pcl::PointIndices::Ptr g_inliers_plane; // Point indices for plane
  pcl::ModelCoefficients::Ptr g_coeff_plane; // Model coefficients for the plane segmentation.
  pcl::ExtractIndices<PointT> g_extract_pc; //Extract point cloud indices.
  PointCPtr g_cloud_plane; // Point cloud to hold plane
  PointCPtr g_cloud_filtered2;
  pcl::ExtractIndices<pcl::Normal> g_extract_normals; //Extract point cloud normal indices.
  pcl::PointCloud<pcl::Normal>::Ptr g_cloud_normals2; //Cloud of normals
  //
  pcl::PointIndices::Ptr g_inliers_cylinder; //Point indices for cylinder
  pcl::ModelCoefficients::Ptr g_coeff_cylinder; //Model coefficients for the culinder segmentation
  PointCPtr g_cloud_cylinder; //Point cloud to hold cylinder points.
  //
  geometry_msgs::PointStamped g_cyl_pt_msg;
  //
  geometry_msgs::Point::Ptr basket_pos1_ptr, basket_pos2_ptr ,basket_pos3_ptr;



};
#endif // end of include guard for CW1_CLASS_H_
