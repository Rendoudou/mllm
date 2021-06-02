/**
 * @brief 创建节点数据
 * @author rjy
 * @version 1.1
 * @date 2021.06.02
 */

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <queue>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
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

//typedef
typedef pcl::PointXYZ PointT;

//static params
ros::Subscriber sub_scan_pose;
ros::Subscriber sub_scan_filtered;
ros::Subscriber sub_scan_2_pc;
ros::Publisher odom_pub;

struct localParam {
    std::string pose_topic = "pose_stamped";
    std::string scan_2_pc_topic = "scan_2_pc";
    std::string odom_topic = "odom";
    std::string child_frame_id = "base_link";
    std::string odom_frame_id = "odom";

    pcl::PointCloud<PointT>::ConstPtr key_frame = nullptr;
    ros::Time key_frame_stamp;
    ros::Time curr_pose_stamp;

} param;

//static functions
static void initParams(const ros::NodeHandle &nh); // 初始化参数
static void syncPosePCCallback(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr,
                               const sensor_msgs::PointCloud2ConstPtr &scan_2_pc_ptr); // 多消息同步回调函数


/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "createNode"); //create node
    ros::NodeHandle nh; // node handle
    initParams(nh);     // init

    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, param.pose_topic, 1000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::PointCloud2> PC_sub(nh, param.scan_2_pc_topic, 1000, ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), pose_sub, PC_sub);
    sync.registerCallback(boost::bind(&syncPosePCCallback, _1, _2));
    odom_pub = nh.advertise<nav_msgs::Odometry>(param.odom_topic, 5); //发送里程计结果
    ros::spin();
    return 0;
}


/**
 * @brief 参数初始化
 * @param nh 节点句柄
 */
static void initParams(const ros::NodeHandle &nh) {
    if (!nh.getParam("scan_2_pc_topic", param.scan_2_pc_topic)) {
        param.scan_2_pc_topic = "scan_2_pc";
    }
    return;
}


/**
 * @brief pose & pc2的同步回调函数
 * @param pose_stamped_ptr
 * @param scan_2_pc_ptr
 */
static void syncPosePCCallback(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr,
                               const sensor_msgs::PointCloud2ConstPtr &scan_2_pc_ptr){
    //发布里程计
    static nav_msgs::Odometry odom;                         //里程计信息

    // publish the transform 换个数据格式转发odometry
    odom.header.frame_id = param.odom_frame_id;             //节点id odom
    odom.header.stamp = pose_stamped_ptr->header.stamp;     //设置时间戳
    odom.pose.pose.position = pose_stamped_ptr->pose.position; //提取设置位姿
    odom.pose.pose.orientation = pose_stamped_ptr->pose.orientation;
    odom.child_frame_id = param.child_frame_id;             //scan_filtered
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    // publish the transform
    odom_pub.publish(odom);
    param.curr_pose_stamp = pose_stamped_ptr->header.stamp;

    // 转换消息格式
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>); //申请点云空间
    pcl::fromROSMsg(*scan_2_pc_ptr, *cloud_in); //转换消息格式
    param.key_frame = cloud_in;                    //
    param.key_frame_stamp = scan_2_pc_ptr->header.stamp;

    ROS_INFO("pose stamp: %lf, key_frame_stamp: %lf", param.curr_pose_stamp.toSec(), param.key_frame_stamp.toSec());

    return;
}