#ifndef COLLISION_AVOIDANCE_HPP
#define COLLISION_AVOIDANCE_HPP

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp> 

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include "models/FlatScanModels.h"
#include "models/LaserScan.h"
#include "MultiObjectTracker/MultiObjectTracker.h"

#include "cudaCluster/cudaCluster.h"
#include "cudart_platform.h"

class CollisionAvoidance : public rclcpp::Node {
public: 
    CollisionAvoidance(const rclcpp::NodeOptions& options);

private:
    // Parameters
    /*************************************************************************************************************************************************************/
    bool enable_;                                           // Enable the component
    bool clear_;                                            // Clear the laserscan in case of blocking
    bool enable_auto_clearing_;                             // Enable auto clearing in case the robot is stuck

    // Camera & Image
    float fov_vertical_;                                    // Camera fov vertical
    double cropbox_min_x_, cropbox_min_y_, cropbox_min_z_;  // Cropping minimum on x, y, and z axis.
    double cropbox_max_x_, cropbox_max_y_, cropbox_max_z_;  // Cropping maximum on x, y, and z axis.
    
    // Laser Scan
    float angle_min_deg_, angle_max_deg_;                   // Angle minimum & maximum of the fictional scan around the robot (for saving purposes).
    float range_min_, range_max_;                           // Range minimum & maximum of the fictional scan around the robot (0 range being undefined).
    float angle_increment_;                                 // Increment angle of the fictional scan around the robot.
    float angle_min_merger_deg_, angle_max_merger_deg_;     // Lidar Angle minimum & maximum for the merger to match lidar.
    float range_min_merger_, range_max_merger_;             // Lidar range minimum & maximum for the merger to match lidar.
    float angle_increment_merger_;                          // Lidar angle increment for the merger to match lidar.
    int beam_count_;                                        // Number of beams in each laser scan
    
    // Clusterer
    float voxel_x_, voxel_y_, voxel_z_;                     // Clusterer voxel width, height & depth.
    int min_cluster_size_, max_cluster_size_;               // Clusterer minimum & maximum cluster size.
    int minimum_points_per_voxel_;                          // Clusterer minimum points per cluster.
    
    // Tracker
    float shield_distortion_;                               // Distortion Parameters
    double dynamic_threshold_;                              // Dynamic obstacle mouvement threshold Parameters
    
    // Grounded obstacle threshold Parameters
    double grounded_threshold_width_, grounded_threshold_height_;   // Width & height difference threshold
    double lidar_threshold_;                                        // Lidar detected obstacle threshold Parameters
    double static_obstacle_timeout_;                                // Static obstacle timeout
    double camera_out_vis_;                                         // Camera output visibilities value
    
    // Kalman Filter params
    float avg_elapsed_time_;            // Rough estimate on the dt
    float init_cov_;                    // Initial Covariance
    float params_cov_;                  // Parameters Covariance

    // Tracker params
    float speed_threshold_;         // Velocity threshold for static decision
    int max_frame_skipped_;         // Non Associated clusters timeout
    float centroid_threshold_;      // Minimum distance to associate clusters
    /*************************************************************************************************************************************************************/


    // Subscriptions
    /*************************************************************************************************************************************************************/
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    /*************************************************************************************************************************************************************/

    // Publishers
    /*************************************************************************************************************************************************************/
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    /*************************************************************************************************************************************************************/

    // TF listener
    /*************************************************************************************************************************************************************/
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /*************************************************************************************************************************************************************/

    Eigen::Vector2f camera_focal_;      // Camera Focal length to be acquired
    Eigen::Vector2f camera_center_;     // Camera Center to be acquired

    // Camera image dimensions
    std::pair<int, int> camera_dimensions_;

    // Frames
    /*************************************************************************************************************************************************************/
    std::string depth_image_topic_ = "/depth_image";                // Name of the topic with the depth image data
    std::string depth_camera_info_topic_ = "/depth_intrinsics";     // Name of the topic with the depth camera info
    std::string base_link_ref_frame_ = "base_link";                 // Frame id assigned to the base link
    std::string depth_camera_ref_frame_ = "base_camera";            // Frame id assigned to the depth reference
    std::string lidar_ref_frame_ = "base_laser_link";               // Frame id assigned to the lidar
    std::string odom_ref_frame_ = "odom_combined";                  // Odom frame id
    /*************************************************************************************************************************************************************/

    // Variables
    /*************************************************************************************************************************************************************/
    sensor_msgs::msg::Image::SharedPtr depth_image_msg_;    // Stores the depth image message
    cv_bridge::CvImagePtr depth_img_ptr_;                   // cv object that will hold the depth image
    Eigen::Affine3f depth_camera_T_base_link_;              // Camera transformation matrix
    Eigen::Affine3f lidar_T_base_link_;                     // Lidar transformation matrix
    Eigen::Affine3f odom_T_base_link_;                      // Robot to odom transformation matrix
    LaserScan current_laser_scan_;          // Current laser scan message
    LaserScan shifted_laser_scan_;          // Shifted laser scan message
    // LaserScan current_laser_scan_shield_;   //Current shield laser scan message
    std::vector<std::vector<int>> clusters_scan_;       // Clusters with associated beams index breakdown matrix
    kf::Matrix<Eigen::Dynamic, 2> obs_;     // X,Y of clusters' centroids at tick time (observation time)
    kf::MOKFTracker mokfTracker;            // Multi Object Tracker
    /*************************************************************************************************************************************************************/

    // Functions
    /*************************************************************************************************************************************************************/
    void initializeLaserScan(LaserScan& scan);
    void getIntrinsics(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);
    void getRobotDescription();
    void get_odom_enc_T_robot();
    void convertDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    bool checkPointInsideFOV(double x, double z);
    void detectClustersWithLables(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void labelHangingClusterBeams(pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidar_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster_cloud, const std::vector<int>& cluster_scan_indices); 
    void shiftHistory() 
    void publishLaserScan(LaserScan& laserScan);
    void start();
    void tick();


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
    /*************************************************************************************************************************************************************/
};

#endif // COLLISION_AVOIDANCE_HPP
