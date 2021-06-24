/**
 * @brief 接收实时点云，定位
 * @author rjy
 * @version 0.1
 * @date 2021.06.24
 */

// std & ros
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <queue>
#include <algorithm>

// pcl
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// tf2
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

// ros msgs
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
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


// static params
typedef std::pair<float, double> disTimeType;  // 方便查找时间戳
#define N 127
cv::KeyPoint P_Mid(63, 63, 127); // 图像中点
ros::Subscriber scan_sub; // 订阅2维激光点云
ros::Publisher pose_pub;  // 定位结果位姿发布
ros::Publisher marker_pub;// 节点位置发布

struct localParam {
    // sub & pub
    std::string scan_topic = "scan_filtered"; // 接收的激光话题
    std::string pose_topic = "pose_estimate"; // 预测位姿后的发布话题
    std::string pose_frame = "map";           // 位姿所处的坐标系名称

    // 文件数据路径
    std::string pose_key = "KeyPose";         // 存储位姿信息时，vector
    std::string ORB_key = "KeyORB";           // 存储特征信息时，vector
    std::string frame_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame/";
    std::string orb_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame_orb/key_frame_orb.yaml";
    std::string pose_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame_pose/key_frame_pose.yaml";

    // 数据存储
    std::vector<geometry_msgs::PoseStamped> pose_buffer; // 位姿提取及存储
    std::map<double, cv::Mat> ORB_buffer;           // 激光特征提取

    // 展示节点
    visualization_msgs::MarkerArray nodes;               // 节点标记容器
    visualization_msgs::Marker node;                     // 节点标记
    std::string marker_topic = "node_marker";            // 话题名
    std::string marker_frame_id = "map";                 // 坐标系

    // 文件操作
    cv::FileStorage fs;                                  // 文件操作对象

    //ORB
    unsigned int grid_scale = N;
    double max_distance = 5.0;
    double grid_precision = max_distance * 2 / grid_scale;
} param;

// static funcs
static bool initParams(const ros::NodeHandle &nh);     // 初始化参数
static bool loadGridMap();                             // 加载地图
static bool loadNodesPose(const std::string &posePath);// 加载节点位姿
static bool loadNodesORB(const std::string &ORBPath);  // 加载节点ORB信息
static void scanCallback(const sensor_msgs::LaserScanConstPtr &msgs);      // 激光雷达回调函数
static void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg); // 点云回调函数


/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");                   // 避免中文乱码
    ros::init(argc, argv, "localization");  // 创建节点
    ros::NodeHandle nh;                              // 节点句柄
    // init
    initParams(nh);                      // params init
    loadGridMap();                       // 加载栅格地图
    loadNodesPose(param.pose_save_path); // 加载节点位姿
    loadNodesORB(param.orb_save_path);   // 加载节点ORB信息
    // sub and pub
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>(param.scan_topic, 5, &scanCallback); // 接受实时的激光数据
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(param.pose_topic, 5);            // 发布预测的位姿消息
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>(param.marker_topic, 5);   // 发布节点位置
    // ros
    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}


/**
 * @brief 初始化参数
 * @param nh 节点句柄
 * @return true or false
 */
bool initParams(const ros::NodeHandle &nh) {

    return true;
}


/**
 * @brief 加载栅格地图
 * @return true or false
 */
static bool loadGridMap() {
    nav_msgs::OccupancyGridConstPtr map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
            "map"); // wait for map
    return true;
}


/**
 * @brief 加载节点位姿信息
 * @param yamlPath 存储文件路径
 * @return true or false
 */
static bool loadNodesPose(const std::string &posePath) {
    static int id = 0;
    geometry_msgs::PoseStamped temp_pose;

    temp_pose.header.frame_id = "pose_read";
    param.fs.open(posePath, cv::FileStorage::READ); // 打开文件
    cv::FileNode key_pose = param.fs[param.pose_key];               // vector 容器名
    cv::FileNodeIterator it = key_pose.begin();                     // 迭代器名字
    for (it = key_pose.begin(); it != key_pose.end(); it++) {       // 迭代器运行
        id++;
        // poses
        temp_pose.header.stamp = ros::Time((*it)["stamp"]);
        (*it)["x"] >> temp_pose.pose.position.x;
        (*it)["y"] >> temp_pose.pose.position.y;
        (*it)["z"] >> temp_pose.pose.position.z;
        (*it)["qx"] >> temp_pose.pose.orientation.x;
        (*it)["qy"] >> temp_pose.pose.orientation.y;
        (*it)["qz"] >> temp_pose.pose.orientation.z;
        (*it)["qw"] >> temp_pose.pose.orientation.w;
        temp_pose.header.seq = id;
        // markers
        param.node.header.frame_id = param.marker_frame_id;
        param.node.header.stamp = temp_pose.header.stamp;
        param.node.pose = temp_pose.pose;
        param.node.id = id;
        param.node.type = visualization_msgs::Marker::SPHERE; // sphere
        param.node.action = visualization_msgs::Marker::ADD; // action
        param.node.scale.x = 0.125; // scale
        param.node.scale.y = 0.125;
        param.node.scale.z = 0.125;
        param.node.color.a = 0.5; // color
        param.node.color.r = 0;
        param.node.color.g = 0;
        param.node.color.b = 255;
        param.nodes.markers.push_back(param.node);
        // restore pose
        param.pose_buffer.push_back(temp_pose);
        std::cout << "get key pose : " + std::to_string(temp_pose.header.stamp.toSec()) << std::endl;
    }
    param.fs.release();
    return true;
}


/**
 * @brief
 * @param yamlPath
 * @return
 */
static bool loadNodesORB(const std::string &ORBPath) {
    param.fs.open(ORBPath, cv::FileStorage::READ); // 打开文件
    cv::FileNode key_ORB = param.fs[param.ORB_key];                // vector 容器名
    cv::FileNodeIterator it = key_ORB.begin();                     // 迭代器名字
    cv::Mat temp_mat;
    double temp_stamp;
    for (it = key_ORB.begin(); it != key_ORB.end(); it++) {
        (*it)["orb"] >> temp_mat;
        (*it)["key_stamp"] >> temp_stamp;
        param.ORB_buffer[temp_stamp] = temp_mat;
        temp_mat.release();
    }
    param.fs.release();

    return true;
}


/**
 * @brief 快速排序
 * @param a 偏差和时间戳类型
 * @param b 偏差和时间戳类型
 * @return true or false
 */
static bool cmp(const disTimeType &a, const disTimeType &b){
    return a.first < b.first;
}


/**
 * @brief 激光点云回调函数
 * @param msgs
 */
static void scanCallback(const sensor_msgs::LaserScanConstPtr &msgs){
    static sensor_msgs::PointCloud2 cloud_msg;
    static pcl::PointCloud<pcl::PointXYZ> cloud;
    static laser_geometry::LaserProjection projector;
    static tf::TransformListener tf_listener;
    tf_listener.setExtrapolationLimit(ros::Duration(0.1));
    projector.transformLaserScanToPointCloud("laser_frame", *msgs, cloud_msg, tf_listener);
    pcl::fromROSMsg(cloud_msg, cloud); // 点云转换
    /** 节点级定位 **/
    //定义阈值
    int a[N][N] = {0};
    int pa, pb;
    double pc;
    // 确定每个栅格内的点云数目
    for (auto &point : cloud.points) {
        pa = int((point.x + param.max_distance) / param.grid_precision);
        pb = int((point.y + param.max_distance) / param.grid_precision);
        pc = point.z;
        if (pc < -0.5) { continue; }
        if (pa < N && pb < N && pa >= 0 && pb >= 0) {
            a[pa][pb] = a[pa][pb] + 1;
        }
    }
    // 计算灰度图
    cv::Mat img = cv::Mat::zeros(N, N, CV_8UC1);
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            if (a[i][j] > 0)
                img.at<uchar>(i, j) = 255;
        }
    }
    // 计算描述子
    std::vector<cv::KeyPoint> key_points;
    cv::Mat description;
    cv::Ptr<cv::ORB> orb_detector = cv::ORB::create();
    key_points.push_back(P_Mid);
    orb_detector->detectAndCompute(img, cv::Mat(), key_points, description, true); // 计算
    // 匹配
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> matches;
    std::vector<disTimeType> dis_buff;
    for(auto &node : param.ORB_buffer){
        matcher->match(description, node.second, matches, cv::Mat()); //匹配
        dis_buff.emplace_back(matches[0].distance, node.first);
    }
    std::sort(dis_buff.begin(), dis_buff.end(), &cmp); // 排序
    std::cout << "closet node stamp : " << std::to_string(dis_buff[0].second) << std::endl;
    marker_pub.publish(param.nodes);

    return;
}