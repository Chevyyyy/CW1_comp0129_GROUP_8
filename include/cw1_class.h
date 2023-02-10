/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW1_CLASS_H_
#define CW1_CLASS_H_

// system includes
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
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
  int
  getNearestPoint(const PointC& cloud, const pcl::PointXYZRGBA& position);

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


};

#endif // end of include guard for CW1_CLASS_H_
