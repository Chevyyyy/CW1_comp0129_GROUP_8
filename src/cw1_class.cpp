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

#include <cw1_class.h>

cw1::cw1(ros::NodeHandle nh) : cloud_ptr_(new PointC),
                               cloud_filtered_(new PointC)
{
  /* class constructor */
  nh_ = nh;

  pub_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("cw1/filtered_cloud", 1, true);
  pub_seg_ = nh.advertise<sensor_msgs::PointCloud2>("cw1/seg", 1, true);

  // advertise solutions for coursework tasks
  t1_service_ = nh_.advertiseService("/task1_start",
                                     &cw1::t1_callback, this);
  t2_service_ = nh_.advertiseService("/task2_start",
                                     &cw1::t2_callback, this);
  t3_service_ = nh_.advertiseService("/task3_start",
                                     &cw1::t3_callback, this);
  
  // avoid touching the falling cubes
  geometry_msgs::Point point_worldframe;
  point_worldframe.x = 0.45;
  point_worldframe.y = 0.0;
  armGo(point_worldframe);
  ROS_INFO("cw1 class initialised");
}

bool cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
                      cw1_world_spawner::Task1Service::Response &response)
{
  /* function which should solve task 1 */

  // pick the cube
  geometry_msgs::Point pick_position = request.object_loc.pose.position;
  pick(pick_position);
  // place the cube into basket
  geometry_msgs::Point place_position = request.goal_loc.point;
  place(place_position);

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  return true;
}

bool cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
                      cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  // clear the basket_colours
  response.basket_colours.clear();

  for (int i = 0; i < 4; i++)
  {
    // find the color for given location
    int color_code = findColor(*cloud_filtered_, request.basket_locs[i], true, 2000);
    ROS_WARN("%d",(*cloud_filtered_).size());
    // decode the color index to string
    switch (color_code)
    {
    case 1:
      response.basket_colours.push_back("red");
      ROS_INFO("####################");
      ROS_INFO("Color of basket: red");
      ROS_INFO("####################");
      break;
    case 2:
      response.basket_colours.push_back("blue");
      ROS_INFO("####################");
      ROS_INFO("Color of basket: blue");
      ROS_INFO("####################");
      break;
    case 3:
      response.basket_colours.push_back("pink");
      ROS_INFO("####################");
      ROS_INFO("Color of basket: pink");
      ROS_INFO("####################");
      break;
    case 4:
      response.basket_colours.push_back("empty");
      ROS_INFO("####################");
      ROS_INFO("Color of basket: empty");
      ROS_INFO("####################");
      break;

    default:
      break;
    }
  }

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

bool cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
                      cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  // search cubes
  int n_cube = searchCubesTask3();

  // search baskets
  int n_basket = searchBasketsTask3();

  // place and pick all the cube into baskets
  pickPlaceCubes(n_cube, n_basket);

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

void cw1::pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  fram_id_ = cloud_input_msg->header.frame_id;
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_input_msg, pcl_pc_);
  pcl::fromPCLPointCloud2(pcl_pc_, *cloud_ptr_);

  applyPT(cloud_ptr_, &cloud_filtered_);

  pubFilteredPCMsg(pub_cloud_, *cloud_filtered_);
}

bool cw1::moveArm(geometry_msgs::Pose target_pose)
{
  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.allowReplanning(true);
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

bool cw1::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_)
    width = gripper_open_;
  if (width < gripper_closed_)
    width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // move the gripper joints
  hand_group_.move();

  return success;
}

bool cw1::pick(geometry_msgs::Point position)
{
  /* This function picks up an object using a pose. The given point is where the
  centre of the gripper fingers will converge */

  // define grasping as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the grasping orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion grasp_orientation = tf2::toMsg(q_result);

  // set the desired grasping pose
  geometry_msgs::Pose grasp_pose;
  grasp_pose.position = position;
  grasp_pose.orientation = grasp_orientation;
  grasp_pose.position.z += z_offset_;

  // set the desired pre-grasping pose
  geometry_msgs::Pose approach_pose;
  approach_pose = grasp_pose;
  approach_pose.position.z += approach_distance_;

  /* Now perform the pick */

  bool success = true;

  ROS_INFO("Begining pick operation");

  // move the arm above the object
  success *= moveArm(approach_pose);

  if (not success)
  {
    ROS_ERROR("Moving arm to pick approach pose failed");
    return false;
  }

  // open the gripper
  success *= moveGripper(gripper_open_);

  if (not success)
  {
    ROS_ERROR("Opening gripper prior to pick failed");
    return false;
  }

  // approach to grasping pose
  success *= moveArm(grasp_pose);

  if (not success)
  {
    ROS_ERROR("Moving arm to grasping pose failed");
    return false;
  }

  // grasp!
  success *= moveGripper(gripper_closed_);

  if (not success)
  {
    ROS_ERROR("Closing gripper to grasp failed");
    return false;
  }

  // retreat with object
  success *= moveArm(approach_pose);

  if (not success)
  {
    ROS_ERROR("Retreating arm after picking failed");
    return false;
  }

  ROS_INFO("Pick operation successful");

  return true;
}

bool cw1::place(geometry_msgs::Point position)
{
  /* This function plcae an object to specicfic position. */

  // define placeing as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);

  // determine the placeing orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // set the desired place pose
  geometry_msgs::Pose place_pose;
  place_pose.position = position;
  place_pose.orientation = place_orientation;
  place_pose.position.z = 0.5;

  /* Now perform the place */
  

  bool success = true;

  ROS_INFO("Begining place operation");

  // move the arm above the basket
  success *= moveArm(place_pose);

  // open the gripper
  success *= moveGripper(gripper_open_);

  return true;
}

bool cw1::armGo(geometry_msgs::Point position)
{
  /* This function move arm to a specicfic position. */

  // define placeing as from above
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // determine the placeing orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion place_orientation = tf2::toMsg(q_result);

  // set the desired place pose
  geometry_msgs::Pose place_pose;
  place_pose.position = position;
  place_pose.orientation = place_orientation;
  place_pose.position.z = 0.45;
  /* Now perform the place */
  bool success = true;
  // move the arm above the basket
  success *= moveArm(place_pose);

  ros::Duration(1, 0).sleep();

  ROS_INFO("Arm reach the specific position");

  return true;
}

int cw1::getNearestPoint(const PointC &cloud, const pcl::PointXYZRGBA &position)
{
  // create the kdtree
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  // if the cloud is empty return -1
  if (cloud.size() < 2000)
  {
    return -1;
  }
  // get the nearest point and return
  kdtree.setInputCloud(cloud.makeShared());
  int k = 1;
  std::vector<int> indices(k);
  std::vector<float> distances(k);
  kdtree.nearestKSearch(position, k, indices, distances);

  return indices[0];
}

int cw1::findColor(const PointC &cloud, const geometry_msgs::PointStamped &loc, bool move_arm, int cloud_num_thresh)
{

  // go to target point
  if (move_arm)
  {
    geometry_msgs::Point point_worldframe = loc.point;
    armGo(point_worldframe);
  }
  // tansform to color frame
  geometry_msgs::PointStamped point_cameraframe;

  try
  {
    listener_.transformPoint("color", loc, point_cameraframe);
  }
  catch (tf::TransformException &ex)
  {
    // ROS_WARN ("Received a trasnformation exception: %s", ex.what());
  }

  pcl::PointXYZRGBA pointPCL_cameraframe;
  pointPCL_cameraframe.x = point_cameraframe.point.x;
  pointPCL_cameraframe.y = point_cameraframe.point.y;
  pointPCL_cameraframe.z = point_cameraframe.point.z;

  // find the nearset point index
  int nearestIndex = getNearestPoint(cloud, pointPCL_cameraframe);
  // if the cloud is empty return 4(empty)
  if (nearestIndex == -1)
  {
    return 4;
  }
  PointT pointT_est_cameraframe = (cloud).points[nearestIndex];

  // judge the color (red 1 blue 2 pink 3 empty 4 )
  if (pointT_est_cameraframe.r / pointT_est_cameraframe.g > 5 && pointT_est_cameraframe.b / pointT_est_cameraframe.g < 5)
    return 1;
  else if (pointT_est_cameraframe.b / pointT_est_cameraframe.g > 5 && pointT_est_cameraframe.r / pointT_est_cameraframe.g < 5)
    return 2;
  else if (pointT_est_cameraframe.b / pointT_est_cameraframe.g > 5 && pointT_est_cameraframe.r / pointT_est_cameraframe.g > 5)
    return 3;

  return -1;
}

void cw1::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc)
{
  // Publish the data
  sensor_msgs::PointCloud2 cloud_filtered__msg;
  pcl::toROSMsg(pc, cloud_filtered__msg);
  cloud_filtered__msg.header.frame_id = fram_id_;
  pc_pub.publish(cloud_filtered__msg);

  return;
}

void cw1::applyPT(PointCPtr &in_cloud_ptr, PointCPtr *out_cloud_ptr)
{
  pt_.setInputCloud(in_cloud_ptr);
  pt_.setFilterFieldName("z");
  pt_.setFilterLimits(0, 0.37);
  pt_.filter(**out_cloud_ptr);

  return;
}

void cw1::findCenter(PointCPtr &in_cloud_ptr, geometry_msgs::PointStamped *pose_out)
{
  Eigen::Vector4f centroid_in;
  if (pcl::compute3DCentroid(*in_cloud_ptr, centroid_in) == 0)
  {
    return;
  }

  pose_color.header.frame_id = fram_id_;
  pose_color.header.stamp = ros::Time(0);
  pose_color.point.x = centroid_in[0];
  pose_color.point.y = centroid_in[1];
  pose_color.point.z = centroid_in[2];

  // Transform the point to new frame
  try
  {
    listener_.transformPoint("panda_link0", pose_color, *pose_out);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Received a trasnformation exception: %s", ex.what());
  }
  return;
}

int cw1::searchCubesTask3()
{
  // declare the point in world frame to reach
  geometry_msgs::Point point_worldframe;
  // declare the pose to be found as the center of seg
  geometry_msgs::PointStamped pose_out;

  // move to a position that can see all the cubes
  point_worldframe.x = 0.45;
  point_worldframe.y = 0;
  armGo(point_worldframe);

  // initialize the cluster extration
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_filtered_);

  pcl::EuclideanClusterExtraction<PointT> euclidean_cluster_extraction;
  euclidean_cluster_extraction.setClusterTolerance(0.01); // 2 cm
  euclidean_cluster_extraction.setMinClusterSize(200);
  euclidean_cluster_extraction.setMaxClusterSize(2500000);
  euclidean_cluster_extraction.setSearchMethod(tree);
  euclidean_cluster_extraction.setInputCloud(cloud_filtered_);

  // declare the cluster indices and extract
  std::vector<pcl::PointIndices> cluster_indices;
  euclidean_cluster_extraction.extract(cluster_indices);

  int cube_counter = 0;
  for (const auto &cluster : cluster_indices)
  {
    PointCPtr cloud_cluster(new PointC);
    for (const auto &idx : cluster.indices)
    {
      cloud_cluster->push_back((*cloud_filtered_)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    ROS_WARN("%d",cloud_cluster->width);

    // publish the seg
    pubFilteredPCMsg(pub_seg_, *cloud_cluster);
    // find the center of the seg
    findCenter(cloud_cluster, &pose_out);
    // find the color of the seg
    int color = findColor(*cloud_filtered_, pose_out, false);
    // store the above things
    cube_locs[cube_counter] = pose_out.point;
    cube_colors[cube_counter] = color;

    cube_counter++;
  }
  return cube_counter;
}

int cw1::searchBasketsTask3()
{
  // declare the point in world frame to reach
  geometry_msgs::Point point_worldframe;
  // declare the pose to be found as the center of seg
  geometry_msgs::PointStamped pose_out;

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_filtered_);
  // initialize the cluster extration
  pcl::EuclideanClusterExtraction<PointT> euclidean_cluster_extraction;
  euclidean_cluster_extraction.setClusterTolerance(0.01); // 2 cm
  euclidean_cluster_extraction.setMinClusterSize(2000);
  euclidean_cluster_extraction.setMaxClusterSize(2500000);
  euclidean_cluster_extraction.setSearchMethod(tree);
  euclidean_cluster_extraction.setInputCloud(cloud_filtered_);
  // declare the cluster indices and extract
  std::vector<pcl::PointIndices> cluster_indices;

  int basket_counter = 0;
  for (int k = 0; k < 4; k++)
  {
    // search all the four corners
    switch (k)
    {
    case 0:
      point_worldframe.x = 0.28;
      point_worldframe.y = 0.36;
      break;
    case 1:
      point_worldframe.x = 0.28;
      point_worldframe.y = -0.36;
      break;
    case 2:
      point_worldframe.x = 0.63;
      point_worldframe.y = -0.36;
      break;
    case 3:
      point_worldframe.x = 0.63;
      point_worldframe.y = 0.36;
      break;
    default:
      break;
    }
    armGo(point_worldframe);

    euclidean_cluster_extraction.extract(cluster_indices);

    for (const auto &cluster : cluster_indices)
    {
      PointCPtr cloud_cluster(new PointC);
      for (const auto &idx : cluster.indices)
      {
        cloud_cluster->push_back((*cloud_filtered_)[idx]);
      }
      cloud_cluster->width = cloud_cluster->size();

      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      ROS_WARN("%d",cloud_cluster->width);
      // publish the seg
      pubFilteredPCMsg(pub_seg_, *cloud_cluster);
      // find the center
      findCenter(cloud_cluster, &pose_out);
      // find the color
      int color = findColor(*cloud_filtered_, pose_out, false);
      // store the above things
      basket_locs[basket_counter] = pose_out.point;
      basket_colors[basket_counter] = color;
      basket_counter++;
      break;
    }
  }
  return basket_counter;
}

bool cw1::pickPlaceCubes(int n_cube, int n_basket)
{
  for (int i = 0; i < n_cube; i++)
  {
    // loop over all the cubes
    geometry_msgs::Point pick_position = cube_locs[i];
    // adjust the pick poistion (z)
    pick_position.z = 0.015;
    int cube_color = cube_colors[i];
    int target_basket_index = 0;
    // find the same color basket and read the loc of it
    for (int k = 0; k < n_basket; k++)
    {
      if (basket_colors[k] == cube_color)
      {
        target_basket_index = k;
        break;
      }
      else
      {
        target_basket_index = -1;
      }
    }
    // judge the matching color basket exsit or not
    if (target_basket_index != -1)
    {
      geometry_msgs::Point place_position = basket_locs[target_basket_index];
      // pick the cube
      pick(pick_position);

      // place the cube
      place(place_position);
    }
    else
    {
      ROS_WARN("############################################");
      ROS_WARN("NO match color for the cube, skip it!!!!!!!");
      ROS_WARN("############################################");
    }
  }
  return true;
}