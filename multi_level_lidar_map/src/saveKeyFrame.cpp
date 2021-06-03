/**
 * @brief 保存关键帧
 * @author rjy
 * @version 1.1
 * @date 2021.06.02
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

// static params
#define N 256
ros::Subscriber sub_pc;
struct localParam {
    std::string pc_topic = "key_pc_re_pub";
    std::string key_frame_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame/";
    std::string key_frame_orb_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame_orb/";
    cv::FileStorage fs;
} param;

// functions
static void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);


/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "saveKeyFrame"); //create node
    ros::NodeHandle nh; // node handle
    sub_pc = nh.subscribe<sensor_msgs::PointCloud2>(param.pc_topic, 5, &pcCallBack);
    ros::spin();
    return 0;
}


/**
 * @brief getKeyFrame
 *        hokuyo 参数
 *        每帧点数 1800
 *
 * @param cloud
 */
static void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    long long stamp = double(cloud_msg->header.stamp.sec) * 1000 + double(cloud_msg->header.stamp.nsec) * 1e-6;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    //定义阈值
    int a[N][N] = { 0 };
    int pa, pb;
    double pc;
    //确定每个栅格内的点云数目
    for (size_t i = 0; i < cloud->points.size(); i++) {
        pa = int((cloud->points[i].x + 32.0) / 0.25);
        pb = int((cloud->points[i].y + 32.0) / 0.25);
        pc = cloud->points[i].z;
        if (pc < -0.5) { continue; }
        if (pa < N && pb < N && pa >= 0 && pb >= 0) {
            a[pa][pb] = a[pa][pb] + 1;
        }
    }
    cv::Mat img = cv::Mat::zeros(N, N, CV_32FC1);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (a[i][j] > 1)
                img.at<float>(i, j) = 255;
        }
    }
    std::stringstream ss;
    ss << param.key_frame_save_path << std::to_string(stamp) << ".jpg";
    cv::imwrite(ss.str(), img);
    return;
}