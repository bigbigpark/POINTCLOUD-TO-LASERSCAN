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
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>

struct LaserScanParameter
{
  std::string target_frame = "base_link"; // Leave disabled to output scan in pointcloud frame
  double transform_tolerance = 0.001;
  double min_height = -2.0;
  double max_height =  0.0;

  double angle_min = -3.14; // -3.14
  double angle_max =  3.14; // 3.14
  double angle_increment =  0.00174532889; //[rad]
  double scan_time = 1.0; // time between scans [seconds]
  double range_min = 0.01;
  double range_max = 100.0;

  double inf_epsilon = 1.0;
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
      laserscan_msgs.header = pointcloud_msgs.header;

      laserscan_msgs.angle_min = param_.angle_min;
      laserscan_msgs.angle_max = param_.angle_max;
      laserscan_msgs.angle_increment = param_.angle_increment;
      laserscan_msgs.time_increment = 0.0;
      laserscan_msgs.scan_time = param_.scan_time;
      laserscan_msgs.range_min = param_.range_min;
      laserscan_msgs.range_max = param_.range_max;
      
      // Determine amount of rays to create 
      uint32_t ranges_size = std::ceil( (laserscan_msgs.angle_max - laserscan_msgs.angle_min) / laserscan_msgs.angle_increment);

      // Determine if laserscan rays with no obstacle data will evaluate to INF or max_range
      if (param_.use_inf)
      {
        laserscan_msgs.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
      }
      else
      {
        laserscan_msgs.ranges.assign(ranges_size, laserscan_msgs.range_max + param_.inf_epsilon);
      }

      // Iterate through pointcloud
      for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud_msgs, "x"), iter_y(pointcloud_msgs, "y"), iter_z(pointcloud_msgs, "z")
          ; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        // Check NaN value
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        {
          ROS_WARN("Rejected for NaN in points : (%f, %f, %f)", *iter_x, *iter_y, *iter_z);
          continue;
        }

        // Check out of ROI using max_height && min_height
        if (*iter_z > param_.max_height || *iter_z < param_.min_height)
        {
          ROS_WARN("Rejected for out of ROI [height, min, max] : (%f, %f, %f)", *iter_z, param_.min_height, param_.max_height);
          continue;
        }

        double range = hypot(*iter_x, *iter_y);
        if (range > param_.range_max || range < param_.range_min)
        {
          ROS_WARN("Rejected for out of ROI [range, min, max] : (%f, %f, %f)", range, param_.range_min, param_.range_max);
          continue;
        }

        double angle = atan2(*iter_y, *iter_x);
        if (angle > laserscan_msgs.angle_max || angle < laserscan_msgs.angle_min)
        {
          ROS_WARN("Rejected for out of ROI [angle, min, max] : (%f, %f, %f)", angle, laserscan_msgs.angle_min, laserscan_msgs.angle_max);
          continue;
        }

        int index = (angle - laserscan_msgs.angle_min) / laserscan_msgs.angle_increment;
        if (range < laserscan_msgs.ranges[index])
        {
          laserscan_msgs.ranges[index] = range;
        }
      }

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