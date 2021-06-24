/**
 * @brief 保存节点位姿，发布节点标记
 * @author rjy
 * @version 0.7
 * @date 2021.06.24
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
#include <visualization_msgs/MarkerArray.h> // marker

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
ros::Publisher pub_markers;

struct localParam {
    std::string pose_topic = "key_pose_re_pub";
    std::string pose_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame_pose/key_frame_pose.yaml";

    cv::FileStorage fs;

    visualization_msgs::MarkerArray nodes;
    visualization_msgs::Marker node;
    std::string marker_topic = "node_marker";
    std::string marker_frame_id = "map";
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
    pub_markers = nh.advertise<visualization_msgs::MarkerArray>(param.marker_topic, 5);
    param.fs.open(param.pose_save_path, cv::FileStorage::WRITE);
    param.fs << "KeyPose" << "[";
    while (ros::ok()) {
        ros::spin();
    }
    param.fs << "]";
    param.fs.release();

    return 0;
}


/**
 * @brief 参数初始化
 * @param const ros::NodeHandle & nh 节点句柄
 */
static void initParams(const ros::NodeHandle &nh) {
    nh.getParam("pose_save_path", param.pose_save_path);

    return;
}


/**
 * @brief 位姿回调函数，存储位姿到.yaml，发布节点标记
 * @param const geometry_msgs::PoseStampedConstPtr & pose 位姿信息msg
 */
static void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose) {
    static ros::Time stamp;
    static int id = 0;

    // save pose to .yaml
    stamp = pose->header.stamp;
    // param.fs << "KeyStamp" + std::to_string(id);
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

    // show pose with marker
    id++;
    param.node.header.frame_id = param.marker_frame_id;
    param.node.header.stamp = pose->header.stamp;
    param.node.pose = pose->pose;
    param.node.id = id;
    param.node.type = visualization_msgs::Marker::SPHERE; // sphere
    param.node.action = visualization_msgs::Marker::ADD; // action
    param.node.scale.x = 0.25; // scale
    param.node.scale.y = 0.25;
    param.node.scale.z = 0.25;
    param.node.color.a = 0.5; // color
    param.node.color.r = 0;
    param.node.color.g = 0;
    param.node.color.b = 255;
    param.nodes.markers.push_back(param.node);
    pub_markers.publish(param.nodes);

    return;
}
