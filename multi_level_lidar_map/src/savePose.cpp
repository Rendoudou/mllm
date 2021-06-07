/**
 * @brief 保存关键帧位姿
 * @author rjy
 * @version 1.1
 * @date 2021.06.02
 */
// std & ros
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <queue>
#include <fstream>

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

// opencv
#include <opencv2/opencv.hpp>

// params
ros::Subscriber sub_pose;

struct localParam {
    std::string pose_topic = "key_pose_re_pub";
    std::string key_frame_pose_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame_pose/key_frame_pose.yaml";
    cv::FileStorage fs;
} param;

// function
static void initParams(const ros::NodeHandle &nh); // 初始化参数
static void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose);

/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "savePose"); //create node
    ros::NodeHandle nh; // node handle
    initParams(nh);
    sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(param.pose_topic, 5, &poseCallback);
    param.fs.open(param.key_frame_pose_save_path, cv::FileStorage::WRITE);
    while (ros::ok()) {
        ros::spin();
    }
    param.fs.release();
    return 0;
}


/**
 * @brief 参数初始化
 * @param nh 节点句柄
 */
static void initParams(const ros::NodeHandle &nh) {
    nh.getParam("save_path", param.key_frame_pose_save_path);
    return;
}


/**
 * @brief 位姿回调函数
 * @param pose 位姿信息msg
 */
static void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose) {
    static ros::Time stamp;
    stamp = pose->header.stamp;
    param.fs << "KeyStamp" + std::to_string(stamp.toNSec());
    param.fs << "{" <<
    "stamp" << stamp.toSec() <<
    "x" << pose->pose.position.x <<
    "y" << pose->pose.position.y <<
    "z" << pose->pose.position.z <<
    "qx" << pose->pose.orientation.x <<
    "qy" << pose->pose.orientation.y <<
    "qz" << pose->pose.orientation.z <<
    "qw" << pose->pose.orientation.w <<
    "}";
    return;
}
