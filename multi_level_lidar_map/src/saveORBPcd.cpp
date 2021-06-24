/**
 * @brief 保存节点orb与节点点云
 * @author rjy
 * @version 0.7
 * @date 2021.06.24
 */

// std & ros
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <queue>
#include <unistd.h>
#include <cstdio>
#include <dirent.h>
#include <cstring>
#include <sys/stat.h>
#include <stdlib.h>

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
#define N 127
cv::KeyPoint P_Mid(63, 63, 127);
ros::Subscriber sub_pc;
struct localParam {
    std::string pc_topic = "key_pc_re_pub";
    std::string frame_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame/";
    std::string orb_save_path = "/home/RJY/catkin_ws_multi_level_map/src/multi_level_lidar_map/key_frame_orb/key_frame_orb.yaml";

    cv::FileStorage fs;

    unsigned int grid_scale = N;
    double max_distance = 5.0;
    double grid_precision = max_distance * 2 / grid_scale;
} param;

// functions
static void initParams(const ros::NodeHandle &nh); // 初始化参数
static void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
static void getFilePath(const char *path, const char *filename, char *filepath);
static bool deleteFile(const char* path);
static bool detectLines(cv::Mat &img, cv::Mat &res);


/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "saveORBPcd"); //create node
    ros::NodeHandle nh; // node handle
    sub_pc = nh.subscribe<sensor_msgs::PointCloud2>(param.pc_topic, 10, &pcCallBack);
    deleteFile(param.frame_save_path.c_str()); //
    param.fs.open(param.orb_save_path, cv::FileStorage::WRITE);
    param.fs << "KeyORB" << "[";
    while(ros::ok()){
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
static void initParams(const ros::NodeHandle &nh){
    nh.getParam("orb_save_path", param.orb_save_path);
    nh.getParam("frame_save_path", param.frame_save_path);

    return;
}

/**
 * @brief 点云消息回调函数，点云投影为图像，然后转会为灰度图。
 *        ORB计算描述子，
 *        霍夫变换计算直线，
 *        保存图像、ORB描述子、直线的计算结果。
 * @param const sensor_msgs::PointCloud2ConstPtr & cloud_msg 点云消息
 */
static void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    // 转换点云
    static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // 定义阈值
    int a[N][N] = { 0 };
    int pa, pb;
    double pc;
    //确定每个栅格内的点云数目
    for (auto & point : cloud->points) {
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
            if(a[i][j] > 0)
                img.at<uchar>(i, j) = 255;
        }
    }
    // 计算描述子、保存，全局ORB
    std::vector<cv::KeyPoint> key_points;
    cv::Mat description;
    cv::Ptr<cv::ORB> orb_detector = cv::ORB::create();
    key_points.push_back(P_Mid);
    orb_detector->detectAndCompute(img, cv::Mat(), key_points, description, true);
    //    orb_detector->detect(img, key_points);
    //    orb_detector->compute(img, key_points, description);
    param.fs << "{" <<
    "orb" << description <<
    "key_stamp" << cloud_msg->header.stamp.toSec() <<
    "}";
    // 霍夫变换 line
    cv::Mat res;
    detectLines(img, res);
    // 保存点云
    pcl::io::savePCDFileASCII(param.frame_save_path + std::to_string(cloud_msg->header.stamp.toNSec()) + ".pcd", *cloud);
    cv::imwrite(param.frame_save_path + std::to_string(cloud_msg->header.stamp.toNSec()) + ".jpg", img);
    cv::imwrite(param.frame_save_path + std::to_string(cloud_msg->header.stamp.toNSec()) + "_res" + ".jpg", res);

    return;
}


/**
 * @brief 获取文件路径
 * @param const char * path 文件夹路径
 * @param const char * filename 文件名
 * @param char * filepath 文件路径
 */
void getFilePath(const char *path, const char *filename, char *filepath){
    strcpy(filepath, path);
    if(filepath[strlen(path) - 1] != '/')
        strcat(filepath, "/");
    strcat(filepath, filename);
    //printf("path is = %s\n",filepath);

    return;
}


/**
 * @brief 删除文件
 * @param const char * path 文件夹路径
 * @return true or false
 */
bool deleteFile(const char *path){
    DIR *dir;
    struct dirent *dir_info;
    struct stat stat_buf;
    char filepath[256] = {0};
    lstat(path, &stat_buf);

    if (S_ISREG(stat_buf.st_mode))//判断是否是常规文件
    {
        remove(path);
    }
    else if (S_ISDIR(stat_buf.st_mode))//判断是否是目录
    {
        if ((dir = opendir(path)) == nullptr)
            return true;
        while ((dir_info = readdir(dir)) != nullptr)
        {
            getFilePath(path, dir_info->d_name, filepath);
            if (strcmp(dir_info->d_name, ".") == 0 || strcmp(dir_info->d_name, "..") == 0)//判断是否是特殊目录
                continue;
            deleteFile(filepath);
            rmdir(filepath);
        }
        closedir(dir);
    }

    return false;
}


/**
 * @brief 检测图像中的直线
 * @param cv::Mat & img 输入的图像
 * @param cv::Mat & res 结果保存
 * @return true
 */
bool detectLines(cv::Mat &img, cv::Mat &res){
    cv::cvtColor(img, res, CV_GRAY2BGR);//将二值图转换为RGB图颜色空间，这里重新创建一张空Mat也行
    // 霍夫变换检测
    std::vector<cv::Vec4f> plines;//保存霍夫变换检测到的直线
    HoughLinesP(img, plines, 1, CV_PI / 180, 10, 0, 10);//提取边缘时，会造成有些点不连续，所以maxLineGap设大点
    // 显示检测到的直线
    cv::Scalar color = cv::Scalar(255, 0, 0);//设置颜色 BGR
    for (auto hline : plines){
        line(res, cv::Point(hline[0], hline[1]), cv::Point(hline[2], hline[3]), color, 3, cv::LINE_AA);//绘制直线
    }

    return true;
}
