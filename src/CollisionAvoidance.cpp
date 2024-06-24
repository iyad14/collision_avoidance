#include "collision_avoidance/CollisionAvoidance.hpp"

CollisionAvoidance::CollisionAvoidance(const rclcpp::NodeOptions& options)
: Node("collision_avoidance", options) {
    // Declare parameters
    this->declare_parameter("enable", false);
    this->declare_parameter("clear", false);
    this->declare_parameter("enable_auto_clearing", true);
    this->declare_parameter("fov_vertical", 0.52f);

    // Cropbox Parameters
    this->declare_parameter("cropbox_min_x", -2.0);
    this->declare_parameter("cropbox_min_y", 0.0);
    this->declare_parameter("cropbox_min_z", 0.03);
    this->declare_parameter("cropbox_max_x", 2.0);
    this->declare_parameter("cropbox_max_y", 6.0);
    this->declare_parameter("cropbox_max_z", 1.5);

    // FlatScan Parameters
    this->declare_parameter("angle_min_deg", -29.0f);
    this->declare_parameter("angle_max_deg", 27.0f);
    this->declare_parameter("range_min", 0.01f);
    this->declare_parameter("range_max", 20.0f);
    this->declare_parameter("angle_increment", 0.5f);

    // Lidar Shield Parameters
    this->declare_parameter("angle_min_merger_deg", 30.0f);
    this->declare_parameter("angle_max_merger_deg", 90.0f);
    this->declare_parameter("range_min_merger", 0.3f);
    this->declare_parameter("range_max_merger", 20.0f);
    this->declare_parameter("angle_increment_merger", 0.5f);

    // Clusterer Parameters
    this->declare_parameter("voxel_x", 0.3f);
    this->declare_parameter("voxel_y", 0.3f);
    this->declare_parameter("voxel_z", 0.05f);
    this->declare_parameter("min_cluster_size", 1000);
    this->declare_parameter("max_cluster_size", 250000);
    this->declare_parameter("minimum_points_per_voxel", 20);

    this->declare_parameter("shield_distortion", 0.2f);
    this->declare_parameter("dynamic_threshold", 0.1);

    // Grounded obstacle threshold Parameters
    this->declare_parameter("grounded_threshold_width", 1.0);
    this->declare_parameter("grounded_threshold_height", 3.0);

    this->declare_parameter("lidar_threshold", 0.5);
    this->declare_parameter("static_obstacle_timeout", 30.0);
    this->declare_parameter("camera_out_vis", 100.0);

    // Kalman Filter params
    this->declare_parameter("avg_elapsed_time", 1.0f);
    this->declare_parameter("init_cov", 1000.0f);
    this->declare_parameter("params_cov", 1.0f);

    // Tracker params
    this->declare_parameter("speed_threshold", 0.05f);
    this->declare_parameter("frames_skipped", 0);
    this->declare_parameter("association_threshold", 0.3f);

    // Retrieve parameters
    this->get_parameter("enable", enable_);
    this->get_parameter("clear", clear_);
    this->get_parameter("enable_auto_clearing", enable_auto_clearing_);
    this->get_parameter("fov_vertical", fov_vertical_);
    this->get_parameter("cropbox_min_x", cropbox_min_x_);
    this->get_parameter("cropbox_min_y", cropbox_min_y_);
    this->get_parameter("cropbox_min_z", cropbox_min_z_);
    this->get_parameter("cropbox_max_x", cropbox_max_x_);
    this->get_parameter("cropbox_max_y", cropbox_max_y_);
    this->get_parameter("cropbox_max_z", cropbox_max_z_);
    this->get_parameter("angle_min_deg", angle_min_deg_);
    this->get_parameter("angle_max_deg", angle_max_deg_);
    this->get_parameter("range_min", range_min_);
    this->get_parameter("range_max", range_max_);
    this->get_parameter("angle_increment", angle_increment_);
    this->get_parameter("angle_min_merger_deg", angle_min_merger_deg_);
    this->get_parameter("angle_max_merger_deg", angle_max_merger_deg_);
    this->get_parameter("range_min_merger", range_min_merger_);
    this->get_parameter("range_max_merger", range_max_merger_);
    this->get_parameter("angle_increment_merger", angle_increment_merger_);
    this->get_parameter("voxel_x", voxel_x_);
    this->get_parameter("voxel_y", voxel_y_);
    this->get_parameter("voxel_z", voxel_z_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("minimum_points_per_voxel", minimum_points_per_voxel_);
    this->get_parameter("shield_distortion", shield_distortion_);
    this->get_parameter("dynamic_threshold", dynamic_threshold_);
    this->get_parameter("grounded_threshold_width", grounded_threshold_width_);
    this->get_parameter("grounded_threshold_height", grounded_threshold_height_);
    this->get_parameter("lidar_threshold", lidar_threshold_);
    this->get_parameter("static_obstacle_timeout", static_obstacle_timeout_);
    this->get_parameter("camera_out_vis", camera_out_vis_);
    this->get_parameter("avg_elapsed_time", avg_elapsed_time_);
    this->get_parameter("init_cov", init_cov_);
    this->get_parameter("params_cov", params_cov_);
    this->get_parameter("speed_threshold", speed_threshold_);
    this->get_parameter("frames_skipped", frames_skipped_);
    this->get_parameter("association_threshold", association_threshold_);

    start();
}

void CollisionAvoidance::start() {
    // Initialize TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize publishers
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
     
    // Initialize subscriptions
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_camera_info_topic_, 1,
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
            getIntrinsics(msg);
            // Callback scope will end here, allowing the subscriber to be destroyed
            camera_info_sub_.reset();

            getRobotDescription();

            depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                depth_image_topic_, 10, 
                [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                    depth_image_msg_ = msg;
                    tick();
            });
        });
}
void CollisionAvoidance::tick() {
    getDepthImage(depth_image_msg_, depth_img_ptr_);
    if (depth_img_ptr_) {
        // Convert depth image to point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        convertDepthImageToPointCloud(depth_img_ptr_->image, cloud);

        detectClustersWithLables(cloud);
        // Publish point cloud
        // sensor_msgs::msg::PointCloud2 output;
        // pcl::toROSMsg(*cloud, output);
        // output.header = depth_image_msg_->header;
        // output.header.frame_id = depth_camera_ref_frame_;
        // point_cloud_pub_->publish(output);
    }
}
void CollisionAvoidance::getIntrinsics(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg) {
    // Handle camera info
    // depth_camera_ref_frame_ = msg->header.frame_id;

    fx_ = camera_info_msg->k[0];
    fy_ = camera_info_msg->k[4];
    cx_ = camera_info_msg->k[2];
    cy_ = camera_info_msg->k[5];

    // RCLCPP_INFO(this->get_logger(), "Frame ID: %s", depth_camera_ref_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Received depth intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
}
void CollisionAvoidance::getRobotDescription() {
    while (true) {
        try {
            geometry_msgs::msg::TransformStamped depth_transform_stamped = tf_buffer_->lookupTransform(base_link_ref_frame_, depth_camera_ref_frame_, tf2::TimePointZero);
                    
            depth_camera_T_base_link = tf2::transformToEigen(depth_transform_stamped.transform).matrix();
            
            // Print depth_camera_T_base_link matrix
            RCLCPP_INFO(this->get_logger(), "Depth Camera to Base Link Transform Matrix:");
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f %.3f",
                        depth_camera_T_base_link(0, 0), depth_camera_T_base_link(0, 1),
                        depth_camera_T_base_link(0, 2), depth_camera_T_base_link(0, 3));
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f %.3f",
                        depth_camera_T_base_link(1, 0), depth_camera_T_base_link(1, 1),
                        depth_camera_T_base_link(1, 2), depth_camera_T_base_link(1, 3));
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f %.3f",
                        depth_camera_T_base_link(2, 0), depth_camera_T_base_link(2, 1),
                        depth_camera_T_base_link(2, 2), depth_camera_T_base_link(2, 3));
            RCLCPP_INFO(this->get_logger(), "%.3f %.3f %.3f %.3f",
                        depth_camera_T_base_link(3, 0), depth_camera_T_base_link(3, 1),
                        depth_camera_T_base_link(3, 2), depth_camera_T_base_link(3, 3));
                
            break; // Exit the loop once the transform is found
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup robot description from tf service: %s", ex.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep before retrying
        }
    } 
}
void CollisionAvoidance::getDepthImage(const sensor_msgs::msg::Image::SharedPtr& depth_image_msg, cv_bridge::CvImagePtr& depth_img_ptr_) {
    // Handle depth image
    try {
        // Convert ROS Image message to OpenCV image
        depth_img_ptr_ = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);

        // Check if image is valid
        if (depth_img_ptr_->image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty depth image");
            return;
        }
    } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
} 
void CollisionAvoidance::convertDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    double roll = M_PI/2.0 + std::atan2(depth_camera_T_base_link(2, 1), depth_camera_T_base_link(2, 2));

    for (int row = 0; row < depth_image.rows; row += 2) {
        for (int col = 0; col < depth_image.cols; col += 2) {
            float depth_value = depth_image.at<float>(row, col);

            // Calculate x, y, z coordinates from depth image
            double u = col - cx_;
            double v = row - cy_;

            // Convert pixel (r, c) to 3D point in camera coordinates
            double x = (u * depth_value) / fx_;
            double y = depth_value;
            double z = (-v * depth_value) / fy_; 

            // Apply rotation
            double temp_y = y * cos(roll) + z * sin(roll);
            z = -y * sin(roll) + z * cos(roll);
            y = temp_y;

            // Filter out points above and below ground level
            if (x > cropbox_min_x_ && y > cropbox_min_y_ && z > cropbox_min_z_ &&
                x < cropbox_max_x_ && y < cropbox_max_y_ && z < cropbox_max_z_) {
                // Add point to point cloud
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
                cloud->points.push_back(point);
            }
        }
    }
}
void CollisionAvoidance::detectClustersWithLables(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // CUDA memory allocations for clusterer.
    cudaStream_t stream = NULL;
    cudaStreamCreate(&stream);

    float* inputEC = NULL;
    unsigned int sizeEC = cloud->size();
    size_t pointSize = sizeof(pcl::PointXYZ);
    cudaMallocManaged(&inputEC, pointSize * sizeEC, cudaMemAttachGlobal);
    cudaStreamAttachMemAsync(stream, inputEC);
    cudaMemcpyAsync(
        inputEC, cloud->points.data(), pointSize * sizeEC, cudaMemcpyHostToDevice, stream);
    cudaStreamSynchronize(stream);

    float* outputEC = NULL;
    cudaMallocManaged(&outputEC, pointSize * sizeEC, cudaMemAttachGlobal);
    cudaStreamAttachMemAsync(stream, outputEC);
    cudaMemcpyAsync(
        outputEC, cloud->points.data(), pointSize * sizeEC, cudaMemcpyHostToDevice, stream);
    cudaStreamSynchronize(stream);

    unsigned int* indexEC = NULL;
    cudaMallocManaged(&indexEC, pointSize * sizeEC, cudaMemAttachGlobal);
    cudaStreamAttachMemAsync(stream, indexEC);
    cudaMemsetAsync(indexEC, 0, pointSize * sizeEC, stream);
    cudaStreamSynchronize(stream);

    // CUDA Clusterer Initialization.
    extractClusterParam_t ecp;
    ecp.minClusterSize = min_cluster_size_;
    ecp.maxClusterSize = max_cluster_size_;
    ecp.voxelX = voxel_x_;
    ecp.voxelY = voxel_y_;
    ecp.voxelZ = voxel_z_;
    ecp.countThreshold = minimum_points_per_voxel_;
    cudaExtractCluster cudaec(stream);
    cudaec.set(ecp);

    std::cout.setstate(std::ios_base::failbit);
    cudaec.extract(inputEC, sizeEC, outputEC, indexEC);
    std::cout.clear();
    cudaStreamSynchronize(stream);

    // world_T_robot = get_world_T_robot(getTickTime());
    // odom_enc_T_robot_current = get_odom_enc_T_robot(getTickTime());
    // isaac::Pose3d world_T_robot = robot_T_odom_enc * odom_enc_T_robot;

    int number_clusters = indexEC[0];
    // obs = kf::Matrix<Eigen::Dynamic, 2>(number_clusters, 2);
    // camera_only_detections.assign(beam_count, false);
}

inline float read_pointxyz_buffer(float* buffer, size_t point_idx, size_t dim_idx) {
  return buffer[point_idx * sizeof(pcl::PointXYZ) / sizeof(float) + dim_idx];
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto collision_avoidance_node = std::make_shared<CollisionAvoidance>(rclcpp::NodeOptions());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(collision_avoidance_node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}