/**
 * @file pointcloud_2_laserscan.cpp
 * @author Seongchang Park (scsc1125@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-29 11:00
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

struct LaserScanParameter
{
  std::string target_frame = "base_link"; // Leave disabled to output scan in pointcloud frame
  double transform_tolerance = 0.001;
  double min_height = -2.0;
  double max_height =  0.0;

  double angle_min = -3.14; // -3.14
  double angle_max =  3.14; // 3.14
  double angle_increment =  0.00174532889; //[rad]
  double scan_time = 1.0;
  double range_min = 0.01;
  double range_max = 100.0;
  bool use_inf = true;

  /*
  # Concurrency level, affects number of poinclouds queued for processing and number of threads used
  # 0: Detect Number of cored
  # 1: Single threaded
  # 2->inf : Parallelism level
  */
  int concurrency_level = 1;
};

class LaserScanConverter
{
public:
  LaserScanConverter(ros::NodeHandle& nh) : nh_(nh)
  {
    // "cloud_in" -> "keypoints_cloud"
    //   "scan"   -> "keypoints/scan"
    ros::Publisher laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
    ros::Subscriber pointcloud_sub_ = nh_.subscribe("/cloud_in", 1, &LaserScanConverter::cloudCallback, this);

    process();
  }
  ~LaserScanConverter()
  {

  }

  void process()
  {
    ros::Rate r(10);

    while(ros::ok())
    {
      ros::spinOnce();

      // Convert PointCloud to LaserScan



      // Publish LaserScan
      laser_pub_.publish(laserscan_msgs);

      r.sleep();
    }
  }

private:
  LaserScanParameter param_;

  ros::Publisher laser_pub_;
  ros::Subscriber pointcloud_sub_;

  ros::NodeHandle nh_;
  sensor_msgs::PointCloud2 pointcloud_msgs;
  sensor_msgs::LaserScan laserscan_msgs;

  void cloudCallback(const sensor_msgs::PointCloud2::Ptr& msg)
  {
    pointcloud_msgs = *msg;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_2_laserscan");
  ros::NodeHandle nh;

  LaserScanConverter LSC(nh);

  ros::shutdown();

  return 0;
}