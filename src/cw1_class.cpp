/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw1_team_<your_team_number> package */

#include <cw1_class.h>
///////////////////////////////////////////////////////////////////////////
using namespace std;


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
  geometry_msgs::PointStamped g_cyl_pt_msg_out;

  point_worldframe.x = 0.45;
  point_worldframe.y = 0;

  arm_go(point_worldframe);
 
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> euclidean_cluster_extraction;

  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (g_cloud_filtered);


  euclidean_cluster_extraction.setClusterTolerance (0.005); // 2 cm
  euclidean_cluster_extraction.setMinClusterSize (200);
  euclidean_cluster_extraction.setMaxClusterSize (2500000);
  euclidean_cluster_extraction.setSearchMethod (tree);
  euclidean_cluster_extraction.setInputCloud (g_cloud_filtered);
  euclidean_cluster_extraction.extract (cluster_indices);

// search cubes
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
      findCylPose(cloud_cluster,g_cyl_pt_msg_out);
      int color=findColor(*g_cloud_filtered,g_cyl_pt_msg_out,false);
      BasketAndCubeLocation[3]=g_cyl_pt_msg_out.point;
      BasketAndCubeColor[3]=color;
    }
    if(j==2)
    {
      pubFilteredPCMsg (g_pub_seg2, *cloud_cluster);
      findCylPose(cloud_cluster,g_cyl_pt_msg_out);
      int color=findColor(*g_cloud_filtered,g_cyl_pt_msg_out,false);
      BasketAndCubeLocation[4]=g_cyl_pt_msg_out.point;
      BasketAndCubeColor[4]=color;
 
    }
    if(j==3)
    {
      pubFilteredPCMsg (g_pub_seg3, *cloud_cluster);
      findCylPose(cloud_cluster,g_cyl_pt_msg_out);
      int color=findColor(*g_cloud_filtered,g_cyl_pt_msg_out,false);
      BasketAndCubeLocation[5]=g_cyl_pt_msg_out.point;
      BasketAndCubeColor[5]=color;
 
    } 
    if(j==4)
    {
      pubFilteredPCMsg (g_pub_seg4, *cloud_cluster);
      findCylPose(cloud_cluster,g_cyl_pt_msg_out);
      int color=findColor(*g_cloud_filtered,g_cyl_pt_msg_out,false);
      BasketAndCubeLocation[6]=g_cyl_pt_msg_out.point;
      BasketAndCubeColor[6]=color;
 
    }
}


  return true;
}


void
cw1::pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id = cloud_input_msg->header.frame_id;
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


void
cw1::addCollisionObject(std::string object_name,
  geometry_msgs::Point centre, geometry_msgs::Vector3 dimensions,
  geometry_msgs::Quaternion orientation)
{
  /* add a collision object in RViz and the MoveIt planning scene */

  // create a collision object message, and a vector of these messages
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input header information
  collision_object.id = object_name;
  collision_object.header.frame_id = "panda_link0";

  // define the primitive and its dimensions
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dimensions.x;
  collision_object.primitives[0].dimensions[1] = dimensions.y;
  collision_object.primitives[0].dimensions[2] = dimensions.z;

  // define the pose of the collision object
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x = centre.x;
  collision_object.primitive_poses[0].position.y = centre.y;
  collision_object.primitive_poses[0].position.z = centre.z;
  collision_object.primitive_poses[0].orientation = orientation;

  // define that we will be adding this collision object 
  // hint: what about collision_object.REMOVE?
  collision_object.operation = collision_object.ADD;

  // add the collision object to the vector, then apply to planning scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);

  return;
}

void
cw1::removeCollisionObject(std::string object_name)
{
  /* remove a collision object from the planning scene */

  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> object_vector;
  
  // input the name and specify we want it removed
  collision_object.id = object_name;
  collision_object.operation = collision_object.REMOVE;

  // apply this collision object removal to the scene
  object_vector.push_back(collision_object);
  planning_scene_interface_.applyCollisionObjects(object_vector);
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
cw1::arm_go(geometry_msgs::Point position)
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
  arm_go(point_worldframe);
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
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  g_cloud_filtered_msg.header.frame_id=g_input_pc_frame_id;
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
cw1::findNormals (PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud (in_cloud_ptr);
  g_ne.setSearchMethod (g_tree);
  g_ne.setKSearch (g_k_nn);
  g_ne.compute (*g_cloud_normals);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
cw1::segPlane (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);

  /*Set the relative weight (between 0 and 1) to give to the angular
  distance (0 to pi/2) between point normals and the plane normal.*/ 
  g_seg.setNormalDistanceWeight (0); //bad style

  g_seg.setMethodType (pcl::SAC_RANSAC);

  //Set the maximum number of iterations before giving up. 
  g_seg.setMaxIterations (100);//bad style

  //Distance to the model threshold (user given parameter). 
  g_seg.setDistanceThreshold (0.03); //bad style

  g_seg.setInputCloud (in_cloud_ptr);
  g_seg.setInputNormals (g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment (*g_inliers_plane, *g_coeff_plane);
  
  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud (in_cloud_ptr);
  g_extract_pc.setIndices (g_inliers_plane);
  g_extract_pc.setNegative (false);
  
  // Write the planar inliers to disk
  g_extract_pc.filter (*g_cloud_plane);
  
  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative (true);
  g_extract_pc.filter (*g_cloud_filtered2);
  g_extract_normals.setNegative (true);
  g_extract_normals.setInputCloud (g_cloud_normals);
  g_extract_normals.setIndices (g_inliers_plane);
  g_extract_normals.filter (*g_cloud_normals2);

  //ROS_INFO_STREAM ("Plane coefficients: " << *g_coeff_plane);
  ROS_INFO_STREAM ("PointCloud representing the planar component: "
                   << g_cloud_plane->size ()
                   << " data points.");
}

////////////////////////////////////////////////////////////////////////////////
void
cw1::segCylind (PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for cylinder segmentation
  // and set all the parameters
  g_seg.setOptimizeCoefficients (true);
  g_seg.setModelType (pcl::SACMODEL_CYLINDER);
  g_seg.setMethodType (pcl::SAC_RANSAC);

  g_seg.setNormalDistanceWeight (0); //bad style
  g_seg.setMaxIterations (500); //bad style
  g_seg.setDistanceThreshold (0.03); //bad style
  
  /*Set the minimum and maximum allowable radius limits 
  for the model (applicable to models that estimate a radius) */

  g_seg.setRadiusLimits (0.01, 0.1); //bad style
  g_seg.setInputCloud (g_cloud_filtered2);
  g_seg.setInputNormals (g_cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  g_seg.segment (*g_inliers_cylinder, *g_coeff_cylinder);
  
  // Write the cylinder inliers to disk
  g_extract_pc.setInputCloud (g_cloud_filtered2);
  g_extract_pc.setIndices (g_inliers_cylinder);
  g_extract_pc.setNegative (false);
  g_extract_pc.filter (*g_cloud_cylinder);
  
  ROS_INFO_STREAM ("PointCloud representing the cylinder component: "
                   << g_cloud_cylinder->size ()
                   << " data points.");
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
cw1::findCylPose (PointCPtr &in_cloud_ptr,geometry_msgs::PointStamped &g_cyl_pt_msg_out)
{
  Eigen::Vector4f centroid_in;
  if (pcl::compute3DCentroid(*in_cloud_ptr, centroid_in) == 0)
  {
    printf("empty");
    return;
  }
    
  
  g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id;
  g_cyl_pt_msg.header.stamp = ros::Time (0);
  g_cyl_pt_msg.point.x = centroid_in[0];
  g_cyl_pt_msg.point.y = centroid_in[1];
  g_cyl_pt_msg.point.z = centroid_in[2];



  
  // Transform the point to new frame
  try
  {
    listener_.transformPoint ("panda_link0",  // bad styling
                                g_cyl_pt_msg,
                                g_cyl_pt_msg_out);
    //ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR ("Received a trasnformation exception: %s", ex.what());
  }

  printf("x:%f\n",g_cyl_pt_msg_out.point.x);
  printf("y:%f\n",g_cyl_pt_msg_out.point.y);
  printf("z:%f\n",g_cyl_pt_msg_out.point.z);

  return;
}
