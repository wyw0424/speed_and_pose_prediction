/*
 * pointcloud process ROS.cpp
 * wyw: 2019-08-09
*/
#include <mutex>
#include <queue>
#include <fstream>

#include <boost/circular_buffer.hpp>
#include "ros/ros.h"
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

#include <pthread.h>


class speed_and_pose_prediction_ros_node
{

private:
  ros::NodeHandle node_;

  // Subscriber
  ros::Subscriber odom_sub_;

  // Publisher
  ros::Publisher filter_pub_, odom_pub_, pub_undistorted_pc_;

  // Service
  //ros::ServiceServer clear_num_service_;

  // Params
  size_t sliding_window_size_= 10;
  bool first_odom_;
  double velocity_threshold_;
  double offset_threshold_;
  double acceleration_threshold_;

  std::mutex buffer_lock;
  std::queue<nav_msgs::Odometry::Ptr> odom_buffer_;
  boost::circular_buffer<Eigen::Vector3d> speed_slide_window_;
  nav_msgs::Odometry::Ptr last_odom_;
  nav_msgs::Odometry last_predict_;

  Eigen::Vector3d last_average_speed_;
  Eigen::Vector3d predict_odom_;
public:
  speed_and_pose_prediction_ros_node();
  ~speed_and_pose_prediction_ros_node();

  void update_predict_pose(nav_msgs::Odometry pose_predict_odom);
  nav_msgs::Odometry calculation_from_two_poses(nav_msgs::Odometry::Ptr curr, nav_msgs::Odometry::Ptr last);
  nav_msgs::Odometry publish_odom(Eigen::Vector3d predict_pose, Eigen::Quaterniond predict_quat);
  void posecallback(const nav_msgs::OdometryConstPtr &msg);
  void update();
};

speed_and_pose_prediction_ros_node::speed_and_pose_prediction_ros_node()
: first_odom_(true)
, velocity_threshold_(2.0)
, offset_threshold_(1.0)
, acceleration_threshold_(0.5)
{
  // parameters
  std::string odom_topic;
  ros::param::get("/speed_and_pose_prediction_ros_node/odom_topic", odom_topic);
  ros::param::get("/speed_and_pose_prediction_ros_node/velocity_threshold", velocity_threshold_);
  ros::param::get("/speed_and_pose_prediction_ros_node/offset_threshold", offset_threshold_);
  ros::param::get("/speed_and_pose_prediction_ros_node/acceleration_threshold", acceleration_threshold_);


  std::cout << "velocity_threshold: " << velocity_threshold_ << std::endl;
  std::cout << "offset_threshold: " << offset_threshold_ << std::endl;
  std::cout << "acceleration_threshold: " << acceleration_threshold_ << std::endl;
  // subscribers
  odom_sub_ = node_.subscribe<nav_msgs::Odometry> (odom_topic, 1, &speed_and_pose_prediction_ros_node::posecallback, this);

  // publishers
  odom_pub_ = node_.advertise<nav_msgs::Odometry> ("predict_odom_from_speed", 1);

  // Services
  //clear_num_service_ = node_.advertiseService("/image_process/clear_num_service", &speed_and_pose_prediction_ros_node::clear_num_service, this);
  speed_slide_window_.set_capacity(sliding_window_size_);
  
  last_average_speed_ = Eigen::Vector3d::Zero();
  predict_odom_ = Eigen::Vector3d::Zero();
}

speed_and_pose_prediction_ros_node::~speed_and_pose_prediction_ros_node()
{
  //cloud_buffer_.clear();
}

void speed_and_pose_prediction_ros_node::update_predict_pose(nav_msgs::Odometry pose_predict_odom)
{
  // update predict odom
  Eigen::Vector3d vio_predict_pose;
  Eigen::Vector3d odom_predict_pose;
}

// PointCloud2 call back
void speed_and_pose_prediction_ros_node::posecallback(const nav_msgs::Odometry::ConstPtr& data)
{
  nav_msgs::Odometry::Ptr odom (new nav_msgs::Odometry);
  *odom = *data;
  buffer_lock.lock();
  odom_buffer_.push(odom);
  buffer_lock.unlock();
}

nav_msgs::Odometry speed_and_pose_prediction_ros_node::publish_odom(Eigen::Vector3d predict_pose, Eigen::Quaterniond predict_quat)
{
  nav_msgs::Odometry predict;
  predict.pose.pose.position.x = predict_pose(0);
  predict.pose.pose.position.y = predict_pose(1);
  predict.pose.pose.position.z = predict_pose(2);
  predict.pose.pose.orientation.w = predict_quat.w();
  predict.pose.pose.orientation.x = predict_quat.x();
  predict.pose.pose.orientation.y = predict_quat.y();
  predict.pose.pose.orientation.z = predict_quat.z();

  odom_pub_.publish(predict);
  return predict;
}

nav_msgs::Odometry speed_and_pose_prediction_ros_node::calculation_from_two_poses(nav_msgs::Odometry::Ptr curr, nav_msgs::Odometry::Ptr last)
{
    double delta_t = (curr->header.stamp - last->header.stamp).toSec();
    double delta_x = curr->pose.pose.position.x - last->pose.pose.position.x;
    double delta_y = curr->pose.pose.position.y - last->pose.pose.position.y;
    double delta_z = curr->pose.pose.position.z - last->pose.pose.position.z;


    predict_odom_(0) = last_predict_.pose.pose.position.x + last_average_speed_(0)*delta_t;
    predict_odom_(1) = last_predict_.pose.pose.position.y + last_average_speed_(1)*delta_t;
    predict_odom_(2) = last_predict_.pose.pose.position.z + last_average_speed_(2)*delta_t;

    double offset_x = predict_odom_(0) - curr->pose.pose.position.x;
    double offset_y = predict_odom_(1) - curr->pose.pose.position.y;
    double offset_z = predict_odom_(2) - curr->pose.pose.position.z;


    if(abs(offset_x) > offset_threshold_ || abs(offset_y) > offset_threshold_ || abs(offset_z) > offset_threshold_)
    {
      std::cout << "offset: " << offset_x << ", " << offset_y << ", " << offset_z << std::endl;
      std::cout << "origin: " << curr->pose.pose.position.x << ", " << curr->pose.pose.position.y << ", " << curr->pose.pose.position.z << std::endl;
      std::cout << "predict: " << predict_odom_(0) << ", " << predict_odom_(1) << ", " << predict_odom_(2) << std::endl;
    }
    Eigen::Quaterniond quat;
    quat.w() = curr->pose.pose.orientation.w;
    quat.x() = curr->pose.pose.orientation.x;
    quat.y() = curr->pose.pose.orientation.y;
    quat.z() = curr->pose.pose.orientation.z;
    nav_msgs::Odometry predict_pose = publish_odom(predict_odom_, quat);

    // calculate curr_speed and generate curr_average_speed
    Eigen::Vector3d curr_speed(Eigen::Vector3d::Zero());
    Eigen::Vector3d curr_average_speed(Eigen::Vector3d::Zero());

    if(delta_t>0)
    {
      curr_speed(0) = delta_x/delta_t;
      curr_speed(1) = delta_y/delta_t;
      curr_speed(2) = delta_z/delta_t;
    }
    // update speed sliding window and generate curr_average_speed from speed sliding window
    speed_slide_window_.push_back(curr_speed);

    for(size_t i=0; i<speed_slide_window_.size(); ++i)
    {
      curr_average_speed = curr_average_speed + speed_slide_window_[i];
    }
    curr_average_speed = curr_average_speed/speed_slide_window_.size();
    last_average_speed_ = curr_average_speed;
    return predict_pose;
}

void speed_and_pose_prediction_ros_node::update()
{
  if(odom_buffer_.size() > 0)
  {
    buffer_lock.lock();
    nav_msgs::Odometry::Ptr curr_odom_ = odom_buffer_.front();
    odom_buffer_.pop();
    buffer_lock.unlock();

    if(first_odom_)
    {
      last_odom_ = curr_odom_;
      last_predict_ = *curr_odom_;
      first_odom_ = false;
    }
    nav_msgs::Odometry predict_pose = calculation_from_two_poses(curr_odom_, last_odom_);
    // update predict pose
    //update_predict_pose(predict_pose);
    last_odom_ = curr_odom_;
    last_predict_ = predict_pose;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_and_pose_prediction_ros_node");
  speed_and_pose_prediction_ros_node node;

  ROS_INFO("slam front end ros node started...");

  ros::Rate rate(20);

  while(ros::ok())
  {
    ros::spinOnce();
    node.update();
    rate.sleep();
  }
  return 0;
}


