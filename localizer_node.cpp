#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"

// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;
const std::string TOPIC_MAP = "/map";
const std::string TOPIC_INITIAL_POSE = "/initialpose";
const std::string TOPIC_BASE_SCAN = "/base_scan";
const std::string TOPIC_ODOM = "/odom_out";

Eigen::Isometry2f laser_in_world = Eigen::Isometry2f::Identity();

Localizer2D localizer;

int main(int argc, char** argv) {
  // Initialize ROS system
  // TODO
  printf("%s \n","test");
  //echo "Hello";
  //echo "I am here";
  ros::init(argc, argv, "localizer_node");
  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/");
  ROS_INFO("Node initialized.");

  // Create shared pointer for the Map object
  // TODO
  map_ptr = std::make_shared<Map> ();
  //
  /**
   * Subscribe to the topics:
   * /map [nav_msgs::OccupancyGrid]
   * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
   * /base_scan [sensor_msgs::LaserScan]
   * and assign the correct callbacks
   *
   * Advertise the following topic:
   * /odom_out [nav_msgs::Odometry]
   */
  // TODO

  ros::Subscriber map_subscriber = nh.subscribe(TOPIC_MAP, 10, callback_map);
  
  ros::Subscriber initial_pose_subscriber = nh.subscribe(TOPIC_INITIAL_POSE, 10, callback_initialpose);

  ros::Subscriber base_scan_subscriber = nh.subscribe(TOPIC_BASE_SCAN, 10, callback_scan);
  
  pub_odom = nh.advertise<nav_msgs::Odometry>(TOPIC_ODOM, 10);

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.

  // TODO
  
  if(!map_ptr->initialized()){
  map_ptr->loadOccupancyGrid(msg_);
  localizer.setMap(map_ptr);
  }
  
}

void callback_initialpose(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */
   
   // TODO
   
   //Eigen::Isometry2f& _iso;
   
   pose2isometry(msg_->pose.pose, laser_in_world);
   
   localizer.setInitialPose(laser_in_world);
}

void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  // TODO
  //Localizer2D::ContainerType[std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>] loc2d;
  std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f>> loc2d;
  scan2eigen(msg_, loc2d);
  /**
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  // TODO
  localizer.setLaserParams(msg_->range_min, msg_->range_max,
                                   msg_->angle_min, msg_->angle_max,
                                    msg_->angle_increment);
                                 
  localizer.process(loc2d);

  /**
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */
   
  geometry_msgs::TransformStamped tf_msg;
  nav_msgs::Odometry odom_msg;
                                                                //TODO FRAME LASER HERE?
  isometry2transformStamped(laser_in_world, tf_msg, FRAME_WORLD, msg_->header.frame_id, msg_->header.stamp);
   
  static tf2_ros::TransformBroadcaster br;
  // TODO

  br.sendTransform(tf_msg);
  
  //ROS_INFO("Into scan.");

  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
   
  transformStamped2odometry(tf_msg, odom_msg);
  // Send messages
  pub_odom.publish(odom_msg);
  //br.sendTransform(tf_msg);
  // TODO

  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}
