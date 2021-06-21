/**
 * @brief 发布里程计，构建点云地图，发布同步节点数据。
 * @author rjy
 * @version 0.6
 * @date 2021.06.21
 */

// std & ros
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <queue>

// pcl
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

// ros msgs
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h> //单线激光雷达

// message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>


/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "localization"); //create node
    ros::NodeHandle nh; // node handle

    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}