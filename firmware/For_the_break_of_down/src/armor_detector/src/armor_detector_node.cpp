#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

class ArmorDetector : public rclcpp::Node {
public:
    ArmorDetector() : Node("armor_detector") {
        // 订阅宿主机发来的像素坐标 [x1, y1, x2, y2]
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/target_coords", 10, std::bind(&ArmorDetector::coordCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "立体立方体 PnP 节点已启动，等待坐标数据...");

        // ================= 物理尺寸定义 (单位：米) =================
        float length = 0.04; 
        float width  = 0.04;
        // 立方体中心距离前表面的 Z 偏移量
        float half_depth = length / 2.0;

        // 定义“以立方体几何中心为原点”的 3D 坐标点（仅使用前表面 4 个点）
        // 顺序必须严格对应：左上、右上、右下、左下
        // 这里的 Z 轴正方向是远离相机（即深入纸面），所以前表面 Z = +half_depth
        obj_points_.push_back(cv::Point3f(-width/2, -length/2,  half_depth)); // 左上
        obj_points_.push_back(cv::Point3f( width/2, -length/2,  half_depth)); // 右上
        obj_points_.push_back(cv::Point3f( width/2,  length/2,  half_depth)); // 右下
        obj_points_.push_back(cv::Point3f(-width/2,  length/2,  half_depth)); // 左下

        // ================= 相机内参 =================
        // 是占位符,摄像头重新标定
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            1000.0, 0.0, 640.0,   // fx, 0, cx
            0.0, 1000.0, 360.0,   // 0, fy, cy
            0.0, 0.0, 1.0);
        dist_coeffs_ = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0); // 畸变系数暂设为0
    }

private:
    void coordCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;

        // 1. 提取 2D 像素坐标
        float x1 = msg->data[0];
        float y1 = msg->data[1];
        float x2 = msg->data[2];
        float y2 = msg->data[3];

        // 2. 构建图像坐标系下的四个角点（顺序：左上->右上->右下->左下）
        std::vector<cv::Point2f> img_points;
        img_points.push_back(cv::Point2f(x1, y1)); // 左上
        img_points.push_back(cv::Point2f(x2, y1)); // 右上
        img_points.push_back(cv::Point2f(x2, y2)); // 右下
        img_points.push_back(cv::Point2f(x1, y2)); // 左下

        // 3. 执行 PnP 解算
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(obj_points_, img_points, camera_matrix_, dist_coeffs_, rvec, tvec);

        if (success) {
            // 4. 提取 3D 坐标
            double x = tvec.at<double>(0, 0);
            double y = tvec.at<double>(1, 0);
            double z = tvec.at<double>(2, 0); // 立方体几何中心的 Z 距离

            // 5. 计算偏航角(Yaw)和俯仰角(Pitch) -- 如果后续需要控制云台
            cv::Mat R;
            cv::Rodrigues(rvec, R); // 旋转向量转旋转矩阵
            double yaw = std::atan2(-R.at<double>(0, 2), R.at<double>(2, 2)) * 180.0 / CV_PI;
            double pitch = std::asin(R.at<double>(1, 2)) * 180.0 / CV_PI;

            RCLCPP_INFO(this->get_logger(), 
                "📦 立方体中心: X=%.2f, Y=%.2f, Z=%.2f 米 | Yaw=%.1f°, Pitch=%.1f°", 
                x, y, z, yaw, pitch);

            // 在这里将 x, y, z, yaw, pitch 发送
            // sendToSTM32(x, y, z, yaw, pitch);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    std::vector<cv::Point3f> obj_points_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetector>());
    rclcpp::shutdown();
    return 0;
}
