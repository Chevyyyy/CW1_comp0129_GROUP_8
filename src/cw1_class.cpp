#include <cw1_class.h>


cw1::cw1(ros::NodeHandle nh):
  g_cloud_ptr (new PointC),
  g_cloud_filtered (new PointC),
  g_tree (new pcl::search::KdTree<PointT> ()), // KdTree
  g_cloud_normals (new pcl::PointCloud<pcl::Normal>), // segmentation
  g_inliers_plane (new pcl::PointIndices), // plane seg
  g_coeff_plane (new pcl::ModelCoefficients), // plane coeff
  g_cloud_plane (new PointC), // plane point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_normals2 (new pcl::PointCloud<pcl::Normal>),// segmentation
  g_inliers_cylinder (new pcl::PointIndices), // cylidenr seg
  g_coeff_cylinder (new pcl::ModelCoefficients), // cylinder coeff
  g_cloud_cylinder (new PointC)// cylinder point cloud

{
  /* class constructor */
  nh_ = nh;

  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("cw1/filtered_cloud", 1, true);
  g_pub_seg1 = nh.advertise<sensor_msgs::PointCloud2> ("cw1/seg1", 1, true);
  g_pub_seg2 = nh.advertise<sensor_msgs::PointCloud2> ("cw1/seg2", 1, true);
  g_pub_seg3 = nh.advertise<sensor_msgs::PointCloud2> ("cw1/seg3", 1, true);
  g_pub_seg4 = nh.advertise<sensor_msgs::PointCloud2> ("cw1/seg4", 1, true);
  g_pub_seg5 = nh.advertise<sensor_msgs::PointCloud2> ("cw1/seg5", 1, true);

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw1::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw1::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw1::t3_callback, this);

  g_pt_thrs_min = 0.0; // PassThrough min thres
  g_pt_thrs_max = 0.7; // PassThrough max thres
  g_k_nn = 50; // Normals nn size

  ROS_INFO("cw1 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
  cw1_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */
  bool success = true;

  geometry_msgs::Point pick_position = request.object_loc.pose.position;
  success = pick(pick_position);
  geometry_msgs::Point place_position = request.goal_loc.point;
  success = place(place_position);

  // response.success = success;
  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
  cw1_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */
  // geometry_msgs/PointStamped[] basket_locs
  // ---
  // string[] basket_colours

  response.basket_colours.clear();
  for (int i = 0; i < 4; i++) 
  {
    int color_code=findColor(*g_cloud_filtered,request.basket_locs[i]);
    cout<<color_code<<endl;
    switch (color_code)
    {
    case 1:
      response.basket_colours.push_back("red");
      break;
    case 2:
      response.basket_colours.push_back("blue");
      break;
    case 3:
      response.basket_colours.push_back("pink");
      break;
    case 4:
      response.basket_colours.push_back("empty");
      break;
       
    default:
      break;
    }
    
  }

  for (int i = 0; i < 4; i++)
  {
    printf(response.basket_colours[i].data());
    printf("\n");
  }

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
  cw1_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */
  ROS_INFO("The coursework solving callback for task 3 has been triggered");
 
  geometry_msgs::Point point_worldframe;
  geometry_msgs::PointStamped pose_out;

  point_worldframe.x = 0.45;
  point_worldframe.y = 0;

  armGo(point_worldframe);
 
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> euclidean_cluster_extraction;

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (g_cloud_filtered);


  euclidean_cluster_extraction.setClusterTolerance (0.002); // 2 cm
  euclidean_cluster_extraction.setMinClusterSize (200);
  euclidean_cluster_extraction.setMaxClusterSize (2500000);
  euclidean_cluster_extraction.setSearchMethod (tree);
  euclidean_cluster_extraction.setInputCloud (g_cloud_filtered);
  euclidean_cluster_extraction.extract (cluster_indices);

// search cubes
cout<<"search cubes"<<endl;
  int j = 0;   
  for (const auto& cluster : cluster_indices)
  {
    printf("Cluster_index:%d\n",j+1);
    PointCPtr cloud_cluster (new PointC);
    for (const auto& idx : cluster.indices) 
    {
      cloud_cluster->push_back((*g_cloud_filtered)[idx]);
    } 
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
 
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    j++;
    if(j==1)
    {
      pubFilteredPCMsg (g_pub_seg1, *cloud_cluster);
      findCylPose(cloud_cluster,pose_out);
      int color=findColor(*g_cloud_filtered,pose_out,false);
      BasketAndCubeLocation[3]=pose_out.point;
      BasketAndCubeColor[3]=color;
    }
    if(j==2)
    {
      pubFilteredPCMsg (g_pub_seg2, *cloud_cluster);
      findCylPose(cloud_cluster,pose_out);
      int color=findColor(*g_cloud_filtered,pose_out,false);
      BasketAndCubeLocation[4]=pose_out.point;
      BasketAndCubeColor[4]=color;
 
    }
    if(j==3)
    {
      pubFilteredPCMsg (g_pub_seg3, *cloud_cluster);
      findCylPose(cloud_cluster,pose_out);
      int color=findColor(*g_cloud_filtered,pose_out,false);
      BasketAndCubeLocation[5]=pose_out.point;
      BasketAndCubeColor[5]=color;
 
    } 
    if(j==4)
    {
      pubFilteredPCMsg (g_pub_seg4, *cloud_cluster);
      findCylPose(cloud_cluster,pose_out);
      int color=findColor(*g_cloud_filtered,pose_out,false);
      BasketAndCubeLocation[6]=pose_out.point;
      BasketAndCubeColor[6]=color;
 
    }
}
// search baskets
cout<<"search baskets"<<endl;
int corner_count=1;
  for(int k=1; k<4;k++)
{

  cout<<"k"<<k<<endl;
  switch (corner_count)
  {
  case 1:
    point_worldframe.x=0.2;
    point_worldframe.y=0.33;
    break;
  case 2:
    point_worldframe.x=0.2;
    point_worldframe.y=-0.33;
    break;
  case 3:
    point_worldframe.x=0.6;
    point_worldframe.y=-0.33;
    break;
  case 4:
    point_worldframe.x=0.6;
    point_worldframe.y=0.33;
    break;
  default:
    break;
  }
  armGo(point_worldframe);
  corner_count++;
  cout<<(*g_cloud_filtered).size()<<endl;
  if((*g_cloud_filtered).size()<5000)
    {
      k=k-1;
    }
  else
  {
  euclidean_cluster_extraction.setInputCloud (g_cloud_filtered);
  euclidean_cluster_extraction.extract (cluster_indices);

  for (const auto& cluster : cluster_indices)
  {
    PointCPtr cloud_cluster (new PointC);
    for (const auto& idx : cluster.indices) 
    {
      cloud_cluster->push_back((*g_cloud_filtered)[idx]);
    } 
    cloud_cluster->width = cloud_cluster->size ();
    
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
 
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

      pubFilteredPCMsg (g_pub_seg5, *cloud_cluster);
      findCylPose(cloud_cluster,pose_out);
      int color=findColor(*g_cloud_filtered,pose_out,false);
      cout<<"color"<<color<<endl;
      
      BasketAndCubeLocation[k-1]=pose_out.point;
      BasketAndCubeColor[k-1]=color;
    break;
  }
  }
}
  
  
// place and pick all the cube into baskets
for (int i=0;i<4;i++)
{ 
  geometry_msgs::Point pick_position = BasketAndCubeLocation[i+3];
  pick_position.z=0.02;
  pick(pick_position);
  int CubeColor=BasketAndCubeColor[i+3];
  int targetBasketIndex=0;
  for(int k=0;k<3;k++) 
  {
    if(BasketAndCubeColor[k]==CubeColor)
    {
      targetBasketIndex=k;
      break;
    }
  }
  
  geometry_msgs::Point place_position = BasketAndCubeLocation[targetBasketIndex];
  place(place_position);
} 
  return true;
}


void
cw1::pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_frame_id = cloud_input_msg->header.frame_id;
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  applyPT (g_cloud_ptr, g_cloud_filtered);
  
  pubFilteredPCMsg (g_pub_cloud, *g_cloud_filtered);


}


bool 
cw1::moveArm(geometry_msgs::Pose target_pose)
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

bool
cw1::moveGripper(float width)
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

///////////////////////////////////////////////////////////////////////////////

bool
cw1::pick(geometry_msgs::Point position)
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


bool
cw1::place(geometry_msgs::Point position)
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
  place_pose.position.z = 0.4;

  /* Now perform the place */

  bool success = true;

  ROS_INFO("Begining place operation");

  // move the arm above the basket
  success *= moveArm(place_pose);
  
  //open the gripper
  success *= moveGripper(gripper_open_);

  return true;
}

bool
cw1::armGo(geometry_msgs::Point position)
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
  place_pose.position.z = 0.5;
  /* Now perform the place */
  bool success = true;
  // move the arm above the basket
  success *= moveArm(place_pose);
  
  ros::Duration(3, 0).sleep();
  // success *= moveGripper(gripper_open_);
  // success *= moveGripper(gripper_closed_);

  ROS_INFO("Arm reach the specific position");

  return true;
}


////////////////////////////////////////////////////////
//PCL

int
cw1::getNearestPoint(const PointC& cloud, const pcl::PointXYZRGBA& position)
{
  pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
  cout<<cloud.size()<<endl;
  if(cloud.size()<5000)
  {
    return -1;
  }
    kdtree.setInputCloud(cloud.makeShared());
  int k = 1;
  std::vector<int> indices(k);
  std::vector<float> distances(k);
  kdtree.nearestKSearch(position, k, indices, distances);


  return indices[0];
}

int
cw1::findColor(const PointC& cloud, const geometry_msgs::PointStamped &loc,bool move_arm)
{
    
  // go to target point
  if(move_arm)
  {
  geometry_msgs::Point point_worldframe = loc.point;
  armGo(point_worldframe);
  }
  // tansform to color frame 
  geometry_msgs::PointStamped point_cameraframe;
  
  try
  {
    listener_.transformPoint ("color",
                              loc,
                              point_cameraframe);
  }
  catch (tf::TransformException& ex)
  {
    //ROS_WARN ("Received a trasnformation exception: %s", ex.what());
  }

  pcl::PointXYZRGBA pointPCL_cameraframe;
  pointPCL_cameraframe.x = point_cameraframe.point.x;
  pointPCL_cameraframe.y = point_cameraframe.point.y;
  pointPCL_cameraframe.z = point_cameraframe.point.z;

  // find the nearset point index
  int nearestIndex = getNearestPoint(cloud, pointPCL_cameraframe);
  if(nearestIndex==-1)
  {
    return 4;
  }
  PointT pointT_est_cameraframe = (cloud).points[nearestIndex]; 
  printf("r:%d,\n", pointT_est_cameraframe.r);
  printf("g:%d,\n", pointT_est_cameraframe.g);
  printf("b:%d,\n", pointT_est_cameraframe.b);

  // judge the color (red 1 blue 2 pink 3 empty 4 invaild 5)
  if (pointT_est_cameraframe.r/pointT_est_cameraframe.g>5&&pointT_est_cameraframe.b/pointT_est_cameraframe.g<5)
    return 1; 
  else if (pointT_est_cameraframe.b/pointT_est_cameraframe.g>5&&pointT_est_cameraframe.r/pointT_est_cameraframe.g<5)
    return 2; 
  else if (pointT_est_cameraframe.b/pointT_est_cameraframe.g>5&&pointT_est_cameraframe.r/pointT_est_cameraframe.g>5)
    return 3; 

 
}

////////////////////////////////////////////////////////////////////////////////
void
cw1::pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc)
{
  // Publish the data
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  g_cloud_filtered_msg.header.frame_id=g_frame_id;
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}


////////////////////////////////////////////////////////////////////////////////

void
cw1::applyPT (PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  g_pt.setInputCloud (in_cloud_ptr);
  g_pt.setFilterFieldName ("z");
  g_pt.setFilterLimits (0,0.42);
  g_pt.filter (*out_cloud_ptr);
  
  return;
}


////////////////////////////////////////////////////////////////////////////////
void
cw1::findCylPose (PointCPtr &in_cloud_ptr,geometry_msgs::PointStamped &pose_out)
{
  Eigen::Vector4f centroid_in;
  if (pcl::compute3DCentroid(*in_cloud_ptr, centroid_in) == 0)
  {
    printf("empty");
    return;
  }
    
  
  pose_color.header.frame_id = g_frame_id;
  pose_color.header.stamp = ros::Time (0);
  pose_color.point.x = centroid_in[0];
  pose_color.point.y = centroid_in[1];
  pose_color.point.z = centroid_in[2];



  
  // Transform the point to new frame
  try
  {
    listener_.transformPoint ("panda_link0",  // bad styling
                                pose_color,
                                pose_out);
    //ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }

  printf("x:%f\n",pose_out.point.x);
  printf("y:%f\n",pose_out.point.y);
  printf("z:%f\n",pose_out.point.z);

  return;
}
