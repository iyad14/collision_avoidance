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

#include "models/FlatScanModels.h"
#include "models/LaserScan.h"
#include "MultiObjectTracker/KalmanFilter.h"
#include "MultiObjectTracker/types.h"

class CollisionAvoidance : public rclcpp::Node {
public:
    CollisionAvoidance(const rclcpp::NodeOptions& options);

private:
    // Parameters
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
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
      
    // TF listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Camera Focal length to be acquired
    Eigen::Vector2f camera_focal_;

    // Camera Center to be acquired
    Eigen::Vector2f camera_center_;

    // Camera image dimensions
    std::pair<int, int> camera_dimensions_;

    /// Name of the topic with the depth image data
    std::string depth_image_topic_ = "/depth_image";
    /// Name of the topic with the depth camera info
    std::string depth_camera_info_topic_ = "/depth_intrinsics";
    /// Frame id assigned to the base link
    std::string base_link_ref_frame_ = "base_link";
    /// Frame id assigned to the depth reference
    std::string depth_camera_ref_frame_ = "base_camera";
    // Frame id assigned to the lidar
    std::string lidar_ref_frame_ = "base_laser_link";
    // Odom frame id
    std::string odom_ref_frame_ = "odom_combined";
    // Stores the depth image message
    sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
    // cv object that will hold the depth image
    cv_bridge::CvImagePtr depth_img_ptr_;
    // Camera transformation matrix
    Eigen::Affine3f depth_camera_T_base_link_;
    // Lidar transformation matrix
    Eigen::Affine3f lidar_T_base_link_;
    // Robot to odom transformation matrix
    Eigen::Affine3f odom_T_base_link;
    // Laser scan message
    // sensor_msgs::msg::LaserScan::SharedPtr current_laser;
    LaserScan current_laser_scan_;
    LaserScan current_laser_scan_shield_;

    void start();
    void tick();
    void getIntrinsics(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
    void getRobotDescription();
    void get_odom_enc_T_robot();
    void convertDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    int detectClustersWithLables(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, kf::Matrix<Eigen::Dynamic, 2> obs, std::vector<bool> camera_only_detections, std::vector<std::vector<int>> clusters_scan);
    inline float read_pointxyz_buffer(float* buffer, size_t point_idx, size_t dim_idx) {
        return buffer[point_idx * sizeof(pcl::PointXYZ) / sizeof(float) + dim_idx];
    }   
    double RadToDeg(double angle_rad) {
        return angle_rad * 180.0 / M_PI;
    }
    double DegToRad(double angle_deg) {
        return angle_deg * M_PI / 180.0;
    }

    inline Eigen::Affine3f toEigenAffine(const geometry_msgs::msg::TransformStamped &tf)
    {
      const auto &tf_q = tf.transform.rotation;
      const auto &tf_t = tf.transform.translation;
      return Eigen::Affine3f(Eigen::Translation3f(tf_t.x, tf_t.y, tf_t.z) *
                             Eigen::Quaternionf(tf_q.w, tf_q.x, tf_q.y, tf_q.z));
    }
    flatscanMetadata buildScanMeta(bool is_merger);
    bool checkPointInsideFOV(double x, double z);
    void initializeLaserScan(LaserScan& scan);
    void publishLaserScan(LaserScan& laserScan);
};

#endif // COLLISION_AVOIDANCE_HPP
