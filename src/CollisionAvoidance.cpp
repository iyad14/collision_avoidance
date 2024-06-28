#include "collision_avoidance/CollisionAvoidance.hpp"

CollisionAvoidance::CollisionAvoidance(const rclcpp::NodeOptions& options)
: Node("collision_avoidance", options) {
    // Declare parameters
    this->declare_parameter("enable", false);
    this->declare_parameter("clear", false);
    this->declare_parameter("enable_auto_clearing", true);
    this->declare_parameter("fov_vertical", 0.52f);

    // Cropbox Parameters
    this->declare_parameter("cropbox_min_x", 0.0);
    this->declare_parameter("cropbox_min_y", -2.0);
    this->declare_parameter("cropbox_min_z", 0.03);
    this->declare_parameter("cropbox_max_x", 7.0);
    this->declare_parameter("cropbox_max_y", 2.0);
    this->declare_parameter("cropbox_max_z", 2.5);

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
    this->declare_parameter("avg_elapsed_time", 0.1f);
    this->declare_parameter("init_cov", 100.0f);
    this->declare_parameter("params_cov", 0.1f);

    // Tracker params
    this->declare_parameter("speed_threshold", 0.5f);
    this->declare_parameter("max_frame_skipped", 1);
    this->declare_parameter("centroid_threshold", 0.2f);

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
    this->get_parameter("max_frame_skipped", max_frame_skipped_);
    this->get_parameter("centroid_threshold", centroid_threshold_);

    start();
}
void CollisionAvoidance::initializeLaserScan(LaserScan& scan) {
    // Set laser scan parameters
    scan.min_angle = DegToRad(angle_min_deg_);
    scan.max_angle = DegToRad(angle_max_deg_);
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.increment_angle = DegToRad(angle_increment_);

    // Initialize ranges and intensities with default values
    scan.ranges.resize(beam_count_, std::numeric_limits<float>::infinity());
    scan.intensities.resize(beam_count_, 0.0f);
    scan.heights.resize(beam_count_, 0.0f);
    scan.hanging.resize(beam_count_, false);
    scan.states.resize(beam_count_, true);
    scan.in_fov.resize(beam_count_, false);
    scan.camera_only_detections.resize(beam_count_, false);
}
void CollisionAvoidance::getIntrinsics(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg) {
    // Handle camera info
    // depth_camera_ref_frame_ = msg->header.frame_id;

    /// Extract camera intrinsics
    camera_focal_ = Eigen::Vector2f(camera_info_msg->k.at(4), camera_info_msg->k.at(0));
    camera_center_ = Eigen::Vector2f(camera_info_msg->k.at(5),
                                    camera_info_msg->k.at(2)); // 2, 5
    camera_dimensions_ = std::make_pair(camera_info_msg->height, camera_info_msg->width);
}
void CollisionAvoidance::getRobotDescription() {
    while (true) {
        try {
            geometry_msgs::msg::TransformStamped depth_transform_stamped = tf_buffer_->lookupTransform(base_link_ref_frame_, depth_camera_ref_frame_, tf2::TimePointZero);        
            depth_camera_T_base_link_ = toEigenAffine(depth_transform_stamped);
            std::cout << "depth_camera_T_base_link_\n" << depth_camera_T_base_link_.matrix() << std::endl;
            
            geometry_msgs::msg::TransformStamped lidar_transform_stamped = tf_buffer_->lookupTransform(lidar_ref_frame_, base_link_ref_frame_,tf2::TimePointZero);    
            lidar_T_base_link_ = toEigenAffine(lidar_transform_stamped);
            
            break; // Exit the loop once the transform is found
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup robot description from tf service: %s", ex.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep before retrying
        }
    } 
}
void CollisionAvoidance::get_odom_enc_T_robot() {
    try {
        geometry_msgs::msg::TransformStamped odom_transform_stamped = tf_buffer_->lookupTransform(odom_ref_frame_, base_link_ref_frame_,tf2::TimePointZero);         
        odom_T_base_link_ = toEigenAffine(odom_transform_stamped);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup robot description from tf service: %s", ex.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep before retrying
    }
}
void CollisionAvoidance::convertDepthImageToPointCloud(const cv::Mat &depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    float angle_radians = 14.0 * M_PI / 180.0f;
    Eigen::AngleAxisf rotation(angle_radians, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f rotation_matrix = rotation.toRotationMatrix();

    for (int row = 0; row < depth_image.rows; row += 2) {
        for (int col = 0; col < depth_image.cols; col += 2) {
            // Compute the 3D location of the pixel and write to pointcloud
            const float depth = depth_image.at<float>(row, col);
            const Eigen::Vector2f pixel = {row, col};
            const Eigen::Vector2f a = ((pixel - camera_center_).array() / camera_focal_.array()) * depth;
            const Eigen::Vector3f p_camera = {a[1], a[0], depth};

            Eigen::Vector3f p_robot = depth_camera_T_base_link_ * p_camera - depth_camera_T_base_link_.translation();
            p_robot = rotation_matrix * p_robot;
            // Filter out points based on the bounding box criteria
            if (p_robot[0] > cropbox_min_x_ && p_robot[1] > cropbox_min_y_ && p_robot[2] > cropbox_min_z_ &&
                p_robot[0] < cropbox_max_x_ && p_robot[1] < cropbox_max_y_ && p_robot[2] < cropbox_max_z_) {

                // Add point to point cloud
                cloud->points.push_back(pcl::PointXYZ(p_robot[0], p_robot[1], p_robot[2]));
            }
        }
    }
}
bool CollisionAvoidance::checkPointInsideFOV(double x, double z) {
    // if (z >= ((x)*get_fov_vertical()))
    //   std::cout << "Detected out of fov on z:  " << z << " x:  " << x << std::endl;
    double roll = std::atan2(depth_camera_T_base_link_(2, 1), depth_camera_T_base_link_(2, 2));
    double pitch = std::asin(-depth_camera_T_base_link_(2, 0));
    double yaw = std::atan2(depth_camera_T_base_link_(1, 0), depth_camera_T_base_link_(0, 0));
    
    std::cout << "Roll " << RadToDeg(roll) << " pitch " << RadToDeg(pitch) << " yaw " << RadToDeg(yaw) << std::endl;
    double pitch_fov = 90 - RadToDeg(yaw) + 20;
    std::cout << pitch_fov << std::endl;
    double fov_vertical = tan(DegToRad(pitch_fov));
    std::cout << "FOV angle:   " << fov_vertical << std::endl;
    return (!(z >= ((x)*fov_vertical)));
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

    // Supress output
    std::cout.setstate(std::ios_base::failbit);
    cudaec.extract(inputEC, sizeEC, outputEC, indexEC);
    std::cout.clear();
    cudaStreamSynchronize(stream);

    // Stores transformation in odom_T_base_link_
    get_odom_enc_T_robot();

    int number_clusters = indexEC[0];
    obs_ = kf::Matrix<Eigen::Dynamic, 2>(number_clusters, 2);

    for (size_t i = 1, memoffset = 0; i <= indexEC[0]; memoffset += indexEC[i], ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        float centroid_x = 0.0, centroid_y = 0.0;
        std::vector<int> cluster_scan_indices;

        // Iterate over the points in a cluster
        for (std::size_t k = 0; k < indexEC[i]; ++k) {  
            size_t point_idx = memoffset + k;
            pcl::PointXYZ point(read_pointxyz_buffer(outputEC, point_idx, 0),
                read_pointxyz_buffer(outputEC, point_idx, 1),
                read_pointxyz_buffer(outputEC, point_idx, 2));
            
            cluster_cloud->push_back(point);
            centroid_x += point.x;
            centroid_y += point.y;

///////////// Shouldn't it be == ?
            if (point.z < lidar_threshold_) {
                lidar_cloud->push_back(point);
            }

            // Convert to laserscan for navigation.
            double range = hypot(point.x, point.y);
            double angle = RadToDeg(atan2(point.y, point.x));
            if (angle < angle_min_deg_ || angle > angle_max_deg_) {
                continue;
            }

            // if (!checkPointInsideFOV(point.x, point.z)) continue;
            int index = (angle - angle_min_deg_) / angle_increment_;
            if(current_laser_scan_.ranges[index] == 0 || range < current_laser_scan_.ranges[index]) {
                current_laser_scan_.ranges[index] = range;
                current_laser_scan_.intensities[index] = 50.0;
                current_laser_scan_.heights[index] = point.z;
                current_laser_scan_.in_fov[index] = true;
                cluster_scan_indices.push_back(index);
            }


            // Convert to laserscan for flatscan merger for the shield.
            // const Eigen::Vector3f p_robot = {point.x, point.y, point.z};
            // const Eigen::Vector3f p_lidar = lidar_T_base_link_ * p_robot;
            // double range_shield = hypot(p_lidar[0], p_lidar[1]);
            //       if (range_shield < range_min_merger_ || range_shield > range_max_merger_) {
            //     continue;
            // }
            // double angle_shield = RadToDeg(atan2(p_lidar[1], p_lidar[0]));
            // std::cout << "Angle: " << angle_shield << std::endl;
            // if (angle_shield < angle_min_merger_deg_ || angle_shield > angle_max_merger_deg_) {
            //     continue;
            // }
            
            // int index_shield = (angle_shield - angle_min_merger_deg_) / angle_increment_merger_;
            // std::cout << "Index " << index_shield << std::endl;
            // if(current_laser_scan_shield_.ranges[index_shield] == 0 || range_shield < current_laser_scan_shield_.ranges[index_shield]) {
            //     current_laser_scan_shield_.ranges[index_shield] = range_shield - shield_distortion_;
            //     current_laser_scan_shield_.intensities[index_shield] = 50.0;
            // }
        }

        centroid_x /= indexEC[i];
        centroid_y /= indexEC[i];

        const Eigen::Vector3f centroid = {centroid_x, centroid_y, 0.0};
        Eigen::Vector3f centroid_world = odom_T_base_link_ * centroid;
        obs_(i - 1, 0) = centroid_world[0];
        obs_(i - 1, 1) = centroid_world[1];
        
        labelHangingClusterBeams(lidar_cloud, cluster_cloud, cluster_scan_indices);

        clusters_scan_.push_back(cluster_scan_indices);
    }

    publishLaserScan(current_laser_scan_);

    cudaFree(inputEC);
    cudaFree(outputEC);
    cudaFree(indexEC);
    cudaStreamDestroy(stream);
}
void CollisionAvoidance::labelHangingClusterBeams( pcl::PointCloud<pcl::PointXYZ>::ConstPtr lidar_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster_cloud, const std::vector<int>& cluster_scan_indices) {

  bool camera_only_detected = false;

  pcl::PointXYZ minPt;
  pcl::PointXYZ maxPt;
  pcl::PointXYZ minPtLidar;
  pcl::PointXYZ maxPtLidar;

  pcl::getMinMax3D(*cluster_cloud, minPt, maxPt);

  float height, width;
  width = maxPt.y - minPt.y;
  height = maxPt.x - minPt.x;

  pcl::getMinMax3D(*lidar_cloud, minPtLidar, maxPtLidar);

  float height_lidar, width_lidar;
  width_lidar = maxPtLidar.y - minPtLidar.y;
  height_lidar = maxPtLidar.x - minPtLidar.x;

  camera_only_detected = (lidar_cloud->empty()) ||
                         (fabs(width_lidar - width) >= grounded_threshold_width_) ||
                         (fabs(height_lidar - height) >= grounded_threshold_height_);

  for (std::size_t j = 0; j < cluster_scan_indices.size(); ++j) {
    if (current_laser_scan_.camera_only_detections[cluster_scan_indices[j]]) continue;
    current_laser_scan_.camera_only_detections[cluster_scan_indices[j]] = camera_only_detected;
  }
}
void CollisionAvoidance::shiftHistory() {
    initializeLaserScan(shifted_laser_scan_);
}
void CollisionAvoidance::publishLaserScan(LaserScan& laserScan) {
    sensor_msgs::msg::LaserScan msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    msg.angle_min = laserScan.min_angle;
    msg.angle_max = laserScan.max_angle;
    msg.angle_increment = laserScan.increment_angle;
    msg.time_increment = 0.0; // Set as appropriate
    msg.scan_time = 0.0; // Set as appropriate
    msg.range_min = laserScan.range_min;
    msg.range_max = laserScan.range_max;

    msg.ranges = laserScan.ranges;
    msg.intensities = laserScan.intensities;

    laser_scan_pub_->publish(msg);
}
void CollisionAvoidance::start() {
    // Checks for CUDA capable devices
    int gpu_count = 0;
    cudaGetDeviceCount(&gpu_count);
    // Checks if a CUDA capable GPU is available.
    if (!gpu_count) {
        RCLCPP_INFO(this->get_logger(), "3D Collision Avoidance is enabled but couldn't find appropriate GPU");
        return;
    }

    beam_count_ = std::ceil((angle_max_deg_ - angle_min_deg_) / angle_increment_);

    kf::Vector<2> paramsCov{kf::Vector<2>::Ones() * params_cov_};
    mokfTracker = kf::MOKFTracker(avg_elapsed_time_, max_frame_skipped_, centroid_threshold_, init_cov_, paramsCov, speed_threshold_);

    // Initialize TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize publishers
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);

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
                    // Handle depth image
                    try {
                        // Convert ROS Image message to OpenCV image
                        depth_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

                        // Check if image is valid
                        if (depth_img_ptr_->image.empty()) {
                            RCLCPP_WARN(this->get_logger(), "Received empty depth image");
                            return;
                        }
                    } catch (cv_bridge::Exception & e) {
                        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                        return;
                    }
                    tick();
            });
        });
}
void CollisionAvoidance::tick() {
    if (depth_img_ptr_) {
        // Convert depth image to point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        convertDepthImageToPointCloud(depth_img_ptr_->image, cloud);

        // Publish point cloud
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header = depth_image_msg_->header;
        output.header.frame_id = "base_link";
        point_cloud_pub_->publish(output);

        // Checks if cloud is empty.
        if (cloud->size() == 0) {
            return;
        }
        
        // Initialize the output laser for navigation.
        initializeLaserScan(current_laser_scan_);
        // initializeLaserScan(current_laser_scan_shield_);

        // Initialize beam original heights vector.
        // std::vector<float> flatscan_heights(laser_metadata.beam_count, 0.0);

        // Initialize the output laser for flatscan fusion for the shield.
        // auto laser_shield = std::make_shared<sensor_msgs::msg::LaserScan>();
        // flatscanMetadata laser_shield_metadata = buildScanMeta(true);
        // initializeLaserScan(*laser_shield, laser_shield_metadata.min_angle,
        //     laser_shield_metadata.max_angle, laser_shield_metadata.range_min,
        //     laser_shield_metadata.range_max, laser_shield_metadata.beam_count);
        
        // Extract the clusters with their metadata
        detectClustersWithLables(cloud);
        
        // Update the state of each cluster
        mokfTracker.update(obs_, current_laser_scan_.states);

        // Build the array of states for the beam.
        for (std::size_t i = 0; i < clusters_scan_.size(); ++i)
        {
            if (!current_laser_scan_.states[i])
            {
                for (std::size_t j = 0; j < clusters_scan_[i].size(); ++j)
                {
                    current_laser_scan_.states[clusters_scan_[i][j]] = false;
                }
            }
        }

        shiftHistory();
    }
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