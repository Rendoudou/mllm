/**
 * @brief 发布里程计，构建点云地图，发布同步节点数据。
 * @author rjy
 * @version 0.5
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


//typedef
typedef pcl::PointXYZ PointT;
#define PI 3.14
//static params
ros::Publisher odom_pub;
ros::Publisher pose_re_pub;
ros::Publisher pc_re_pub;

struct localParam {
    std::string pose_topic = "pose_stamped";
    std::string scan_2_pc_topic = "scan_2_pc";
    std::string odom_topic = "odom";
    std::string child_frame_id = "base_link";
    std::string odom_frame_id = "odom";

    std::string key_pose_topic = "key_pose_re_pub";
    std::string key_pc_topic = "key_pc_re_pub";

    Eigen::Matrix4d last_pose; // 当前位姿
    Eigen::Matrix4d now_pose; // 上次的位姿

    double node_liner_limit = 0.3; // 线性距离
    double node_angular_limit = 1.57; // 弧度
} param;

//static functions
static void initParams(const ros::NodeHandle &nh); // 初始化参数
static void syncPosePCCallback(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr,
                               const sensor_msgs::PointCloud2ConstPtr &scan_2_pc_ptr); // 多消息同步回调函数
static Eigen::Matrix4d
poseStamped2Matrix4d(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr); // pose 转 matrix
static bool isNode(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr); // 筛选节点


/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "syncPosePC"); //create node
    ros::NodeHandle nh; // node handle
    initParams(nh);     // init

    // 多消息接收同步
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, param.pose_topic, 1000,
                                                                     ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::PointCloud2> PC_sub(nh, param.scan_2_pc_topic, 1000,
                                                                 ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), pose_sub, PC_sub);
    sync.registerCallback(boost::bind(&syncPosePCCallback, _1, _2));

    // publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>(param.odom_topic, 5); // 发送里程计结果
    pose_re_pub = nh.advertise<geometry_msgs::PoseStamped>(param.key_pose_topic, 5); // 同步后节点位姿
    pc_re_pub = nh.advertise<sensor_msgs::PointCloud2>(param.key_pc_topic, 5); // 同步后节点点云

    // ros执行
    while(ros::ok()) {
        ros::spin();
    }

    return 0;
}


/**
 * @brief 参数初始化
 * @param const ros::NodeHandle & nh 节点句柄
 */
static void initParams(const ros::NodeHandle &nh) {
    nh.getParam("scan_2_pc_topic", param.scan_2_pc_topic);
    nh.getParam("node_liner_limit", param.node_liner_limit);
    nh.getParam("node_angular_limit", param.node_angular_limit);
    nh.getParam("odom_child_frame_id", param.child_frame_id);

    param.last_pose.setIdentity();
    param.now_pose.setIdentity();

    return;
}


/**
 * @brief pose & pc2的同步回调函数,发布里程计与判断节点
 * @param const geometry_msgs::PoseStampedConstPtr & pose_stamped_ptr
 * @param const sensor_msgs::PointCloud2ConstPtr & scan_2_pc_ptr
 */
static void syncPosePCCallback(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr,
                               const sensor_msgs::PointCloud2ConstPtr &scan_2_pc_ptr) {
    //发布里程计
    static nav_msgs::Odometry odom;                         //里程计信息
    // publish the transform 换个数据格式转发odometry,publish the transform
    odom.header.frame_id = param.odom_frame_id;             //节点id odom
    odom.header.stamp = pose_stamped_ptr->header.stamp;     //设置时间戳
    odom.pose.pose.position = pose_stamped_ptr->pose.position; //提取设置位姿
    odom.pose.pose.orientation = pose_stamped_ptr->pose.orientation;
    odom.child_frame_id = param.child_frame_id;             //laser_frame
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    odom_pub.publish(odom);

    //判断节点
    if (isNode(pose_stamped_ptr)) {
        pose_re_pub.publish(*pose_stamped_ptr);
        pc_re_pub.publish(*scan_2_pc_ptr);
    }
    /*
    ROS_INFO("同步成功, pose stamp: %lf, key_frame_stamp: %lf", param.curr_pose_stamp.toSec(),
             param.key_frame_stamp.toSec());
    */
    return;
}


/**
 * @brief 将geometry_msgs::PoseStamped类型转换为 Eigen::Matrix4d
 * @param const geometry_msgs::PoseStampedConstPtr & pose_stamped_ptr
 * @return Eigen::Matrix4f 转换结果
 */
static Eigen::Matrix4d poseStamped2Matrix4d(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr) {
    Eigen::Matrix4d pose_matrix;
    pose_matrix.setIdentity();
    double qw = pose_stamped_ptr->pose.orientation.w,
            qx = pose_stamped_ptr->pose.orientation.x,
            qy = pose_stamped_ptr->pose.orientation.y,
            qz = pose_stamped_ptr->pose.orientation.z;
    double x = pose_stamped_ptr->pose.position.x,
            y = pose_stamped_ptr->pose.position.y,
            z = pose_stamped_ptr->pose.position.z;
    Eigen::Quaterniond quaternion(qw, qx, qy, qz);
    pose_matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    pose_matrix(0, 3) = x;
    pose_matrix(1, 3) = y;
    pose_matrix(2, 3) = z;

    return pose_matrix;
}



/**
 * @brief 判断是否为合适的定位节点
 * @param const geometry_msgs::PoseStampedConstPtr & pose_stamped_ptr
 * @return true or false 判断结果
 */
static bool isNode(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr) {
    static bool first_node = true;
    if (first_node) {
        param.last_pose = poseStamped2Matrix4d(pose_stamped_ptr);
        param.now_pose = param.last_pose;
        first_node = false;
        return true;
    }

    static double liner_change, angular_change;
    static Eigen::Matrix4d pose_change;

    liner_change = angular_change = 0.0f;
    pose_change.setIdentity();

    param.now_pose = poseStamped2Matrix4d(pose_stamped_ptr);
    pose_change = param.last_pose.inverse() * param.now_pose;
    liner_change = sqrt(pow(pose_change(0, 3), 2) + pow(pose_change(1, 3), 2));
    /*
        static Eigen::Vector3d eulerAngle; // 列向量
        eulerAngle.setIdentity();
        eulerAngle = pose_change.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
        angular_change = fabs(eulerAngle(0,0)); // 弧度 取z轴
    */
    Eigen::Quaterniond pose_change_q(pose_change.block<3, 3>(0, 0));
    tf2::Quaternion tf2_pose_change_q(pose_change_q.x(), pose_change_q.y(), pose_change_q.z(), pose_change_q.w());
    angular_change = fabs(tf2::getYaw(tf2_pose_change_q));

    if (liner_change >= param.node_liner_limit || (angular_change >= param.node_angular_limit && angular_change <= 3)) { // || (angular_change >= param.node_angular_limit && angular_change <= 3)
        param.last_pose = param.now_pose; // 更新pose
        /*
        std::cout << "stamp" << pose_stamped_ptr->header.stamp.toSec() << std::endl
                  << "=====liner change===== :" << liner_change << std::endl
                  << "=====angular change===== :" << angular_change << std::endl;
        */
        return true;
    }

    return false;
}
