/**
 * @brief 发布里程计，构建点云地图，发布同步节点数据。
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
//static params
ros::Publisher map_pub;

struct localParam {
    std::string pose_topic = "pose_stamped";
    std::string scan_2_pc_topic = "scan_2_pc";

    std::string map_pub_topic = "map_pc";
    std::string map_frame_id = "map";
}param;


//static functions
static void syncPosePCCallback(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr,
                               const sensor_msgs::PointCloud2ConstPtr &scan_2_pc_ptr); // 多消息同步回调函数
static Eigen::Matrix4d
poseStamped2Matrix4d(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr); // pose 转 matrix


/**
 * @brief main function
 * @param argc nums of params
 * @param argv params
 * @return none
 */
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "createPCMap"); //create node
    ros::NodeHandle nh; // node handle

    // 多消息接收同步
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, param.pose_topic, 1000,
                                                                     ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::PointCloud2> PC_sub(nh, param.scan_2_pc_topic, 1000,
                                                                 ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), pose_sub, PC_sub);
    sync.registerCallback(boost::bind(&syncPosePCCallback, _1, _2));

    // publisher
    map_pub = nh.advertise<sensor_msgs::PointCloud2>(param.map_pub_topic, 5); // 构建地图点云

    // ros执行
    ros::spin();
    return 0;
}

/**
 * @brief
 * @param pose_stamped_ptr
 * @param scan_2_pc_ptr
 */
static void syncPosePCCallback(const geometry_msgs::PoseStampedConstPtr &pose_stamped_ptr,
                               const sensor_msgs::PointCloud2ConstPtr &scan_2_pc_ptr){
    // 转换点云消息格式
    pcl::PointCloud<PointT>::Ptr cloud_in(new pcl::PointCloud<PointT>); //
    pcl::PointCloud<PointT>::Ptr cloud_in_transfer(new pcl::PointCloud<PointT>);
    static pcl::PointCloud<PointT>::Ptr cloud_map(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*scan_2_pc_ptr, *cloud_in); //转换消息格式

    //构建、发布地图
    sensor_msgs::PointCloud2 map_msgs;
    pcl::transformPointCloud(*cloud_in, *cloud_in_transfer, poseStamped2Matrix4d(pose_stamped_ptr));
    *cloud_map = *cloud_map + *cloud_in_transfer;
    pcl::toROSMsg(*cloud_map, map_msgs);
    map_msgs.header.stamp = pose_stamped_ptr->header.stamp;
    map_msgs.header.frame_id = param.map_frame_id;
    map_pub.publish(map_msgs);

    return;
}


/**
 * @brief
 * @param pose_stamped_ptr
 * @return Eigen::Matrix4f
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
