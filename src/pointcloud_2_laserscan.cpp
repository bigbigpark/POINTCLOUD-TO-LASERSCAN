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

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

struct LaserScanParameter
{
  std::string target_frame = "base_link"; // Leave disabled to output scan in pointcloud frame
  double transform_tolerance = 0.001;
  double min_height = -2.0;
  double max_height =  0.0;

  double angle_min = -3.14; // -3.14
  double angle_max =  3.14; // 3.14
  double angle_increment =  0.0174532889; // 0.00174532889; //[rad]
  double scan_time = 1.0; // time between scans [seconds]
  double range_min = 0.01;
  double range_max = 100.0;

  double inf_epsilon = 1.0;
  bool use_inf = true;
};

class LaserScanConverter
{
public:
  LaserScanConverter(ros::NodeHandle& nh) : nh_(nh)
  {
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
    pointcloud_sub_ = nh_.subscribe("/cloud_in", 1, &LaserScanConverter::cloudCallback, this);

    ros::spin();
  }
  ~LaserScanConverter()
  {

  }

private:
  LaserScanParameter param_;

  ros::Publisher laser_pub_;
  ros::Subscriber pointcloud_sub_;

  ros::NodeHandle nh_;
  sensor_msgs::LaserScan laserscan_msgs;

  void cloudCallback(const sensor_msgs::PointCloud2::Ptr& msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_ptr);

    laserscan_msgs.header = msg->header;

    laserscan_msgs.angle_min = param_.angle_min;
    laserscan_msgs.angle_max = param_.angle_max;
    laserscan_msgs.angle_increment = param_.angle_increment;
    laserscan_msgs.time_increment = 0.0;
    laserscan_msgs.scan_time = param_.scan_time;
    laserscan_msgs.range_min = param_.range_min;
    laserscan_msgs.range_max = param_.range_max;
    
    uint32_t ranges_size = std::ceil( (laserscan_msgs.angle_max - laserscan_msgs.angle_min) / laserscan_msgs.angle_increment );

    if (param_.use_inf)
    {
      laserscan_msgs.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
      laserscan_msgs.ranges.assign(ranges_size, laserscan_msgs.range_max + param_.inf_epsilon);
    }

    // Iterate through pointcloud
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator iter((*cloud_ptr).begin()); iter != (*cloud_ptr).end(); iter++)
    {
      if ( std::isnan(iter->x) || std::isnan(iter->y) || std::isnan(iter->z) )
      {
        // ROS_WARN("Rejected for NaN in points : (%f, %f, %f)", iter->x, iter->y, iter->z);
        continue;
      }

      if ( iter->z > param_.max_height || iter->z < param_.min_height )
      {
        // ROS_WARN("Rejected for out of ROI [height, min, max] : (%f, %f, %f)", iter->z, param_.min_height, param_.max_height);
        continue;
      }

      double range = hypot( iter->x, iter->y );
      if (range > param_.range_max || range < param_.range_min)
      {
        // ROS_WARN("Rejected for out of ROI [range, min, max] : (%f, %f, %f)", range, param_.range_min, param_.range_max);
        continue;
      }

      double angle = atan2( iter->y, iter->x );
      if (angle > laserscan_msgs.angle_max || angle < laserscan_msgs.angle_min)
      {
        // ROS_WARN("Rejected for out of ROI [angle, min, max] : (%f, %f, %f)", angle, laserscan_msgs.angle_min, laserscan_msgs.angle_max);
        continue;
      }

      int index = (angle - laserscan_msgs.angle_min) / laserscan_msgs.angle_increment;
      if (range < laserscan_msgs.ranges[index])
      {
        laserscan_msgs.ranges[index] = range;
      }
    }
    // std::cout << "ranges.size(): " << laserscan_msgs.ranges.size() << std::endl;

    laser_pub_.publish(laserscan_msgs);
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