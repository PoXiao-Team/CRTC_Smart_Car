#ifndef ARMOR_DETECTOR_NODE_HPP
#define ARMOR_DETECTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "aim_interfaces/msg/aim_info.hpp" 

class ArmorDetector : public rclcpp::Node
{
public:
    ArmorDetector();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // ROS 2 成员
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<aim_interfaces::msg::AimInfo>::SharedPtr aim_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat R_cam2robot_; // 相机到机器人的旋转矩阵
    cv::Mat t_cam2robot_; // 相机到机器人的平移矩阵

    // YOLO 模型
    cv::dnn::Net net_;
    const int INPUT_WIDTH = 640;
    const int INPUT_HEIGHT = 640;
    const float CONF_THRESHOLD = 0.5;
    const float NMS_THRESHOLD = 0.45;

    // 物理 3D 坐标 (顺序：左上，右上，右下，左下)
    std::vector<cv::Point3f> obj_points_;
};

#endif // ARMOR_DETECTOR_NODE_HPP
