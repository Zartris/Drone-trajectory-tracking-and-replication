#include "Node.h"

Node::Node(ORB_SLAM2::System *pSLAM, ros::NodeHandle &node_handle,
           image_transport::ImageTransport &image_transport) {
    name_of_node_ = ros::this_node::getName();
    orb_slam_ = pSLAM;
    node_handle_ = node_handle;
    min_observations_per_point_ = 2;

    //static parameters
    node_handle_.param(name_of_node_ + "/publish_pointcloud",
                       publish_pointcloud_param_, true);
    node_handle_.param<std::string>(name_of_node_ + "/pointcloud_frame_id",
                                    map_frame_id_param_, "map");
    node_handle_.param<std::string>(name_of_node_ + "/camera_frame_id",
                                    camera_frame_id_param_, "camera_link");
    node_handle_.param<std::string>(name_of_node_ + "/bebop_body_frame_id",
                                    bebop_body_frame_id_param_, "bebop_body");
    node_handle_.param<bool>(name_of_node_ + "/publish_scale", publish_scale,
                             false);

    //Setup dynamic reconfigure
    dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_param_callback;
    dynamic_param_callback = boost::bind(&Node::ParamsChangedCallback, this, _1,
                                         _2);
    dynamic_param_server_.setCallback(dynamic_param_callback);

    // Custom subscribers
    record1_subscriber = node_handle_.subscribe("/record_path_toggle1", 1,
                                                &Node::ToggleRecording1, this);
    record2_subscriber = node_handle_.subscribe("/record_path_toggle2", 1,
                                                &Node::ToggleRecording2, this);
    scaler_subscriber = node_handle_.subscribe("/scale_estimate", 1,
                                               &Node::FixScale, this);

    rendered_image_publisher_ = image_transport.advertise(
            "/orb_slam2/debug_image", 1);
    if (publish_pointcloud_param_) {
        map_points_publisher_ =
                node_handle_.advertise<sensor_msgs::PointCloud2>(
                        "/orb_slam2/map_points", 1);
    }

    path1_publisher_ = node_handle_.advertise<nav_msgs::Path>(
            "/orb_slam2/path1", 1);
    path2_publisher_ = node_handle_.advertise<nav_msgs::Path>(
            "/orb_slam2/path2", 1);

    camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
            "/orb_slam2/camera_pose", 100);
    bebop_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
            "/orb_slam2/bebop_pose", 100);

    // Scaled versions if the parameter dictating so is set
    if (publish_scale) {
        camera_pose_scaled_publisher_ = node_handle_.advertise<
                geometry_msgs::PoseStamped>("/orb_slam2/scaled/camera_pose",
                                            100);
        bebop_pose_scaled_publisher_ = node_handle_.advertise<
                geometry_msgs::PoseStamped>("/orb_slam2/scaled/bebop_pose",
                                            100);

        path1_scaled_publisher_ = node_handle_.advertise<nav_msgs::Path>(
                "/orb_slam2/scaled/path1", 1);
        path2_scaled_publisher_ = node_handle_.advertise<nav_msgs::Path>(
                "/orb_slam2/scaled/path2", 1);

        if (publish_pointcloud_param_) {
            map_points_scaled_publisher_ = node_handle_.advertise<
                    sensor_msgs::PointCloud2>("/orb_slam2/scaled/map_points",
                                              1);
        }
    }

}

Node::~Node() {

}

void Node::Update() {
    cv::Mat position = orb_slam_->GetCurrentPosition();

    nav_msgs::Path path1 = orb_slam_->getGoodKeyFrames(1);
    nav_msgs::Path path2 = orb_slam_->getGoodKeyFrames(2);

    if (!position.empty()) {
        PublishPositionAsTransform(position);
        PublishPositionAsPose(position);

    }

    PublishRenderedImage(orb_slam_->DrawCurrentFrame());

    if (publish_pointcloud_param_) {
        PublishMapPoints(orb_slam_->GetAllMapPoints());
    }
    path1.header.frame_id = map_frame_id_param_;
    path2.header.frame_id = map_frame_id_param_;
    path1_publisher_.publish(path1);
    path2_publisher_.publish(path2);
    // Paths are published and we may reuse the variables for scaled versions if we please
    if (publish_scale) {

        // TODO: All of this is quick 'n dirty. Might wanna clean it up later.
        for (int i = 0; i < path1.poses.size(); i++) {
            path1.poses[i].pose.position.x *= scale_estimate;
            path1.poses[i].pose.position.y *= scale_estimate;
            path1.poses[i].pose.position.z *= scale_estimate;
        }
        for (int i = 0; i < path2.poses.size(); i++) {
            path2.poses[i].pose.position.x *= scale_estimate;
            path2.poses[i].pose.position.y *= scale_estimate;
            path2.poses[i].pose.position.z *= scale_estimate;
        }
        path1_scaled_publisher_.publish(path1);
        path2_scaled_publisher_.publish(path2);
    }
}

void Node::FixScale(const std_msgs::Float32 &estimate) {
    // Probably call something on the system
    scale_estimate = estimate.data;
    ROS_INFO("New scale estimate: %6.4lf", scale_estimate);
}

void Node::ToggleRecording1(const std_msgs::Empty &empty) {
    orb_slam_->ToggleRecording1();
    if (orb_slam_->getRecordingState()) {
        ROS_INFO("STARTED RECORDING PATH 1");
    } else {
        ROS_INFO("END RECORDING PATH 1");
    }
}

void Node::ToggleRecording2(const std_msgs::Empty &empty) {
    orb_slam_->ToggleRecording2();
    if (orb_slam_->getRecordingState()) {
        ROS_INFO("STARTED RECORDING PATH 2");
    } else {
        ROS_INFO("END RECORDING PATH 2");
    }
}

void Node::PublishMapPoints(std::vector<ORB_SLAM2::MapPoint *> map_points) {
    sensor_msgs::PointCloud2 cloud = MapPointsToPointCloud(map_points);
    map_points_publisher_.publish(cloud);

    if (publish_scale) {
        sensor_msgs::PointCloud2 cloud_scaled = MapPointsToPointCloud_scaled(map_points);
        map_points_scaled_publisher_.publish(cloud_scaled);
    }
}

void Node::PublishPositionAsTransform(cv::Mat position) {
    tf::Transform transform = TransformFromMat(position);
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(
            tf::StampedTransform(transform, current_frame_time_,
                                 map_frame_id_param_, camera_frame_id_param_));

    tf::Transform transform_to_bebop_body(tf::Quaternion(0, 0, 0, 1),
                                          tf::Vector3(-0.1, 0, 0));

    tf_broadcaster.sendTransform(
            tf::StampedTransform(transform_to_bebop_body, current_frame_time_,
                                 camera_frame_id_param_, bebop_body_frame_id_param_));

    if (publish_scale) {
        tf::Transform transform_scaled = TransformFromMat(position, scale_estimate);
        tf_broadcaster.sendTransform(
                tf::StampedTransform(transform_scaled, current_frame_time_,
                                     map_frame_id_param_, camera_frame_id_param_ + "_scaled"));

        // Create inverse tf (from cam to world)
        tf::Transform transform_to_bebop_body(tf::Quaternion(0, 0, 0, 1),
                                              tf::Vector3(-0.1, 0, 0));

        tf_broadcaster.sendTransform(
                tf::StampedTransform(transform_to_bebop_body, current_frame_time_,
                                     camera_frame_id_param_ + "_scaled", bebop_body_frame_id_param_ + "_scaled"));

    }
}

void Node::PublishPositionAsPose(cv::Mat position) {
    geometry_msgs::Pose pose = PoseFromMat(position);
    position.at<float>(2, 3) += 0.1;
    geometry_msgs::Pose pose2 = PoseFromMat(position);

    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    auto ps = geometry_msgs::PoseStamped();
    ps.header = header;
    ps.pose = pose;
    camera_pose_publisher_.publish(ps);
    auto ps_body = geometry_msgs::PoseStamped();
    ps_body.header.stamp = current_frame_time_;
    ps_body.header.frame_id = map_frame_id_param_;
    ps_body.pose = pose2;
    bebop_pose_publisher_.publish(ps_body);
    if (publish_scale) {
        ps.pose.position.x *= scale_estimate;
        ps.pose.position.y *= scale_estimate;
        ps.pose.position.z *= scale_estimate;
        ps_body.pose.position.x *= scale_estimate;
        ps_body.pose.position.y *= scale_estimate;
        ps_body.pose.position.z *= scale_estimate;
        camera_pose_scaled_publisher_.publish(ps);
        bebop_pose_scaled_publisher_.publish(ps_body);
    }
}

void Node::PublishRenderedImage(cv::Mat image) {
    std_msgs::Header header;
    header.stamp = current_frame_time_;
    header.frame_id = map_frame_id_param_;
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header,
                                                                        "bgr8", image).toImageMsg();
    rendered_image_publisher_.publish(rendered_image_msg);
}

geometry_msgs::Pose Node::PoseFromMat(cv::Mat position_mat) {
    cv::Mat rotation(3, 3, CV_32F);
    cv::Mat translation(3, 1, CV_32F);

    rotation = position_mat.rowRange(0, 3).colRange(0, 3);
    translation = position_mat.rowRange(0, 3).col(3);

    tf::Matrix3x3 tf_camera_rotation(rotation.at<float>(0, 0),
                                     rotation.at<float>(0, 1), rotation.at<float>(0, 2),
                                     rotation.at<float>(1, 0), rotation.at<float>(1, 1),
                                     rotation.at<float>(1, 2), rotation.at<float>(2, 0),
                                     rotation.at<float>(2, 1), rotation.at<float>(2, 2));

    tf::Vector3 tf_camera_translation(translation.at<float>(0),
                                      translation.at<float>(1), translation.at<float>(2));

    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf::Matrix3x3 tf_orb_to_ros(0, 0, 1, -1, 0, 0, 0, -1, 0);

    //Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    //Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    //Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    // Make it a pose
    auto pose = geometry_msgs::Pose();
    pose.position.x = tf_camera_translation.x();
    pose.position.y = tf_camera_translation.y();
    pose.position.z = tf_camera_translation.z();
    auto q = tf::Quaternion();
    tf_camera_rotation.getRotation(q);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

tf::Transform Node::TransformFromMat(cv::Mat position_mat, double scale_factor) {
    cv::Mat rotation(3, 3, CV_32F);
    cv::Mat translation(3, 1, CV_32F);

    rotation = position_mat.rowRange(0, 3).colRange(0, 3);
    translation = position_mat.rowRange(0, 3).col(3) * scale_factor;

    tf::Matrix3x3 tf_camera_rotation(rotation.at<float>(0, 0),
                                     rotation.at<float>(0, 1), rotation.at<float>(0, 2),
                                     rotation.at<float>(1, 0), rotation.at<float>(1, 1),
                                     rotation.at<float>(1, 2), rotation.at<float>(2, 0),
                                     rotation.at<float>(2, 1), rotation.at<float>(2, 2));

    tf::Vector3 tf_camera_translation(translation.at<float>(0),
                                      translation.at<float>(1), translation.at<float>(2));

    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf::Matrix3x3 tf_orb_to_ros(0, 0, 1, -1, 0, 0, 0, -1, 0);

    //Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    //Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    //Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

    return tf::Transform(tf_camera_rotation, tf_camera_translation);
}

sensor_msgs::PointCloud2 Node::MapPointsToPointCloud(
        std::vector<ORB_SLAM2::MapPoint *> map_points) {
    if (map_points.size() == 0) {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = current_frame_time_;
    cloud.header.frame_id = map_frame_id_param_;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[3];
    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points.at(i)->nObs >= min_observations_per_point_) { //nObs isBad()
            data_array[0] = map_points.at(i)->GetWorldPos().at<float>(
                    2); //x. Do the transformation by just reading at the position of z instead of x
            data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(
                    0); //y. Do the transformation by just reading at the position of x instead of y
            data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(
                    1); //z. Do the transformation by just reading at the position of y instead of z
            //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
                   3 * sizeof(float));
        }
    }

    return cloud;
}

sensor_msgs::PointCloud2 Node::MapPointsToPointCloud_scaled(
        std::vector<ORB_SLAM2::MapPoint *> map_points) {
    if (map_points.size() == 0) {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    const int num_channels = 3; // x y z

    cloud.header.stamp = current_frame_time_;
    cloud.header.frame_id = map_frame_id_param_;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    float data_array[3];
    for (unsigned int i = 0; i < cloud.width; i++) {
        if (map_points.at(i)->nObs >= min_observations_per_point_) { //nObs isBad()
            data_array[0] = map_points.at(i)->GetWorldPos().at<float>(2)
                            *
                            scale_estimate; //x. Do the transformation by just reading at the position of z instead of x
            data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(0)
                            *
                            scale_estimate; //y. Do the transformation by just reading at the position of x instead of y
            data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(1)
                            *
                            scale_estimate; //z. Do the transformation by just reading at the position of y instead of z
            //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

            memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
                   3 * sizeof(float));
        }
    }

    return cloud;
}

void Node::ParamsChangedCallback(
        orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level) {
    orb_slam_->EnableLocalizationOnly(config.localize_only);
    min_observations_per_point_ = config.min_observations_for_ros_map;

    if (config.reset_map) {
        orb_slam_->Reset();
        config.reset_map = false;
    }

    orb_slam_->SetMinimumKeyFrames(config.min_num_kf_in_map);
}
