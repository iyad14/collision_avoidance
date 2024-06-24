#ifndef COLLISION_AVOIDANCE_HPP
#define COLLISION_AVOIDANCE_HPP

#include "cudaCluster/cudaCluster.h"
#include "cudart_platform.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class CollisionAvoidance : public rclcpp::Node {
public:
    CollisionAvoidance(const rclcpp::NodeOptions& options);

private:
    // Parameters
    double fx_, fy_, cx_, cy_;
    bool enable_;
    bool clear_;
    bool enable_auto_clearing_;
    float fov_vertical_;
    double cropbox_min_x_, cropbox_min_y_, cropbox_min_z_;
    double cropbox_max_x_, cropbox_max_y_, cropbox_max_z_;
    float angle_min_deg_, angle_max_deg_;
    float range_min_, range_max_;
    float angle_increment_;
    float angle_min_merger_deg_, angle_max_merger_deg_;
    float range_min_merger_, range_max_merger_;
    float angle_increment_merger_;
    float voxel_x_, voxel_y_, voxel_z_;
    int min_cluster_size_, max_cluster_size_;
    int minimum_points_per_voxel_;
    float shield_distortion_;
    double dynamic_threshold_;
    double grounded_threshold_width_, grounded_threshold_height_;
    double lidar_threshold_;
    double static_obstacle_timeout_;
    double camera_out_vis_;
    float avg_elapsed_time_;
    float init_cov_;
    float params_cov_;
    float speed_threshold_;
    int frames_skipped_;
    float association_threshold_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
      
    // TF listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /// Name of the topic with the depth image data
    std::string depth_image_topic_ = "/depth_image";
    /// Name of the topic with the depth camera info
    std::string depth_camera_info_topic_ = "/depth_intrinsics";
    /// Frame id assigned to the base link
    std::string base_link_ref_frame_ = "base_link";
    /// Frame id assigned to the depth reference
    std::string depth_camera_ref_frame_ = "base_camera";
    // Stores the depth image message
    sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
    // cv object that will hold the depth image
    cv_bridge::CvImagePtr depth_img_ptr_;
    // Camera transformation matrix
    Eigen::Affine3d depth_camera_T_base_link;

    void start();
    void tick();
    void getIntrinsics(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
    void getRobotDescription();
    void getDepthImage(const sensor_msgs::msg::Image::SharedPtr& depth_image_msg, cv_bridge::CvImagePtr& depth_img_ptr);
    // void processImage(const sensor_msgs::msg::Image::ConstSharedPtr depth_image_msg);
    void convertDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void detectClustersWithLables(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif // COLLISION_AVOIDANCE_HPP
