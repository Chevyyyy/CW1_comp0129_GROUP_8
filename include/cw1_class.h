// MIT License

// Copyright (c) [2023] [Chevy WENG]

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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

// include any services created in this package
#include "string"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

class cw1
{
public:
  /*----- class member functions ----- */

  /// constructor
  cw1(ros::NodeHandle nh);

  /// function to solve tasks 1
  bool t1_callback(cw1_world_spawner::Task1Service::Request &request,
                   cw1_world_spawner::Task1Service::Response &response);

  /// function to solve tasks 2
  bool t2_callback(cw1_world_spawner::Task2Service::Request &request,
                   cw1_world_spawner::Task2Service::Response &response);

  /// function to solve tasks 2
  bool t3_callback(cw1_world_spawner::Task3Service::Request &request,
                   cw1_world_spawner::Task3Service::Response &response);

  /*******************************************************************************
   * @brief the callback function for receving the point cloud function
   *
   * @param cloud_input_msg input cloud message from color frame
   ******************************************************************************/
  void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  /*******************************************************************************
   * @brief move the robot arm to target pose
   *
   * @param target_pose the target pose
   * @return true if sucessful
   ******************************************************************************/
  bool moveArm(geometry_msgs::Pose target_pose);

  /*******************************************************************************
   * @brief move the gripper to a certain width
   *
   * @param width the width of the gripper
   * @return true if sucessful
   ******************************************************************************/
  bool moveGripper(float width);

  /*******************************************************************************
   * @brief pickup the cube in certain position
   *
   * @param position the position of cube to be picked up
   * @return true if sucessful
   ******************************************************************************/
  bool pick(geometry_msgs::Point position);

  /*******************************************************************************
   * @brief place the cube into a basket in certain position
   *
   * @param position the position of the basket
   * @return true if sucessful
   ******************************************************************************/
  bool place(geometry_msgs::Point position);

  /*******************************************************************************
   * @brief move arm to a specicfic position
   *
   * @param position the target position of the arm
   * @return true if sucessful
   ******************************************************************************/
  bool armGo(geometry_msgs::Point position);

  /*******************************************************************************
   * @brief get the nearset point's index from the cloud and target position
   *
   * @param cloud the point cloud data
   * @param position the target position to be searched
   * @return the index of the nearest point
   ******************************************************************************/
  int getNearestPoint(const PointC &cloud, const pcl::PointXYZRGBA &position);

  /*******************************************************************************
   * @brief publish the point cloud message using specific publisher
   *
   * @param pc_pub the point cloud publisher
   * @param pc the point cloud
   ******************************************************************************/
  void pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc);

  /*******************************************************************************
   * @brief apply a PT filter to the input point cloud
   *
   * @param in_cloud_ptr input point cloud to be filtered
   * @param out_cloud_ptr output point cloud pointer
   ******************************************************************************/
  void applyPT(PointCPtr &in_cloud_ptr, PointCPtr *out_cloud_ptr);

  /*******************************************************************************
   * @brief find the center of the input point cloud
   *
   * @param in_cloud_ptr input cloud message from color frame
   * @param pose_out ouput pose pointer
   ******************************************************************************/
  void findCenter(PointCPtr &in_cloud_ptr, geometry_msgs::PointStamped *pose_out);

  /*******************************************************************************
   * @brief find the color of the a position in the given point cloud
   *
   * @param cloud input cloud
   * @param loc position to be searched
   * @param move_arm if ture, move arm above the loc to calculate the more accurate color
   * @param cloud_num_thresh the threshold to judge a cloud is empty or not 
   * @return the color (1 for red, 2 for blue, 3 for pink, 4 for empty and -1 for error)
   ******************************************************************************/
  int findColor(const PointC &cloud, const geometry_msgs::PointStamped &loc, bool move_arm = true, int cloud_num_thresh=2000);

  /*******************************************************************************
   * @brief search all the cubes and store their locs and colors 
   * 
   * @return the number of cubes (boxes) 
   ******************************************************************************/
  int searchCubesTask3();

  /*******************************************************************************
   * @brief search all the baskets and store their locs and colors 
   * 
   * @return the number of baskets 
   ******************************************************************************/
  int searchBasketsTask3();
  
  /*******************************************************************************
   * @brief pick and place all the cubes accroding to the searching results 
   * 
   * @param n_cube the number of cubes
   * @param n_basket the number of baskets
   * @return true if sucessful
   ******************************************************************************/
  bool pickPlaceCubes(int n_cube, int n_basket);
 


  /* ----- class member variables ----- */

  ros::NodeHandle nh_;            /// node handle
  ros::ServiceServer t1_service_; /// service of task1
  ros::ServiceServer t2_service_; /// service of task2
  ros::ServiceServer t3_service_; /// service of task3

  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"}; /// arm group ("panda_arm") in moveit
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};     /// arm group ("hand") in moveit

  float gripper_open_ = 80e-3;          /// safe value for the open size of gripper
  float gripper_closed_ = 0.0;          /// safe value for the closed size of gripper
  double angle_offset_ = 3.14159 / 4.0; /// angle offset for grasping orentation

  double z_offset_ = 0.125;        /// z-axis offset for the grasping pose
  double approach_distance_ = 0.1; /// the pre-grasping distance

  std::string fram_id_;            /// frame id for the recevied cloud
  pcl::PCLPointCloud2 pcl_pc_;     /// point cloud data in PCL
  PointCPtr cloud_ptr_;            /// point cloud data pointer
  tf::TransformListener listener_; /// TF listener

  ros::Publisher pub_cloud_; /// for publishing the filtered cloud
  ros::Publisher pub_seg_;  /// for publishing the seg of the first cube

  pcl::PassThrough<PointT> pt_; /// Pass Through filter.
  PointCPtr cloud_filtered_;    /// filtered cloud pointer

  geometry_msgs::PointStamped pose_color;   /// the pose of the seg to be found the center
  geometry_msgs::Point basket_locs[4]; /// store the locations of all the baskets
  geometry_msgs::Point cube_locs[100]; /// store the locations of all the cubes
  int basket_colors[4];                /// store the colors of all the baskets
  int cube_colors[100];                /// store the colors of all the cubes
};
#endif // end of include guard for CW1_CLASS_H_
