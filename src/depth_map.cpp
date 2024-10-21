#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class PointCloudToDepthMap : public rclcpp::Node
{
public:
    PointCloudToDepthMap()
        : Node("point_cloud_to_depth_map"), width_(1200), height_(600), scale_(100)
    {
        // Subscriber for PointCloud2 messages
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan/points", 10, std::bind(&PointCloudToDepthMap::point_cloud_callback, this, std::placeholders::_1));
        
        // Publisher for depth map images
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/depth_map", 10);
        
        RCLCPP_INFO(this->get_logger(), "PointCloud to Depth Map Node has been started.");
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Create an empty depth map
        cv::Mat depth_map = cv::Mat::zeros(height_, width_, CV_8UC1);

        // Define the center of the depth map 
        int center_y = width_ / 2;
        int center_z = height_ / 2;

        // Iterate over the point cloud
        for (const auto& point : pcl_cloud.points)
        {
            float x = point.x;
            float y = point.y;
            float z = point.z;

            // Convert to pixel coordinates (flip the z-axis to correct the image orientation)
            int pixel_y = static_cast<int>(center_y + y * scale_);
            int pixel_z = static_cast<int>(center_z - z * scale_);  // Flip z-axis to correct orientation

            // Check if the pixel is within image bounds
            if (pixel_y >= 0 && pixel_y < width_ && pixel_z >= 0 && pixel_z < height_)
            {
                // Normalize depth value to 0-255
                int depth_value = std::clamp(static_cast<int>(x * 255 / 100), 0, 255);
                depth_map.at<uint8_t>(pixel_z, pixel_y) = 255 - depth_value;  // Invert so closer points are brighter
            }
        }

        // Convert the depth map to a ROS Image message
        sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", depth_map).toImageMsg();

        // Publish the depth map
        publisher_->publish(*image_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    int width_;
    int height_;
    float scale_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToDepthMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
