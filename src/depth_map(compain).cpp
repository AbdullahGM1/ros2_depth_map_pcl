#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Create two empty depth maps
        cv::Mat depth_map_x = cv::Mat::zeros(height_, width_, CV_8UC1); // For y-z depth (x as depth)
        cv::Mat depth_map_z = cv::Mat::zeros(height_, width_, CV_8UC1); // For x-y depth (z as depth)

        // Apply filtering to remove far-away points for x-based depth
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(pcl_cloud);
        pass_x.setFilterFieldName("x");  // Set filter for the x-axis
        pass_x.setFilterLimits(0.3, 5.0);  // Filter points with x in the range [0, 50] meters
        pass_x.filter(*filtered_cloud_x);  // Apply filter

        // Apply filtering to remove far-away points for z-based depth
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(pcl_cloud);
        pass_z.setFilterFieldName("z");  // Set filter for the z-axis
        pass_z.setFilterLimits(0.3, 20.0);  // Filter points with z in the range [0, 50] meters
        pass_z.filter(*filtered_cloud_z);  // Apply filter

        // Define the center of both depth maps
        int center_y_x = width_ / 2;
        int center_z_x = height_ / 2;
        
        int center_x_z = width_ / 2;
        int center_y_z = height_ / 2;

        // ---- First image: (y, z) coordinates with X-based depth map ----
        for (const auto& point : filtered_cloud_x->points)
        {
            float x = point.x;
            float y = point.y;
            float z = point.z;

            int pixel_y_x = static_cast<int>(center_y_x + y * scale_);
            int pixel_z_x = static_cast<int>(center_z_x - z * scale_);  // Flip z-axis to correct orientation

            // Check if the pixel is within image bounds
            if (pixel_y_x >= 0 && pixel_y_x < width_ && pixel_z_x >= 0 && pixel_z_x < height_)
            {
                // Normalize depth value to 0-255 (depth using x)
                int depth_value_x = std::clamp(static_cast<int>(x * 255 / 50), 0, 255);  // Assuming max x depth is 50
                depth_map_x.at<uint8_t>(pixel_z_x, pixel_y_x) = 255 - depth_value_x;  // Invert so closer points are brighter
            }
        }

        // ---- Second image: (x, y) coordinates with Z-based depth map ----
        for (const auto& point : filtered_cloud_z->points)
        {
            float x = point.x;
            float y = point.y;
            float z = point.z;

            int pixel_x_z = static_cast<int>(center_x_z + x * scale_);
            int pixel_y_z = static_cast<int>(center_y_z + y * scale_);

            // Check if the pixel is within image bounds
            if (pixel_x_z >= 0 && pixel_x_z < width_ && pixel_y_z >= 0 && pixel_y_z < height_)
            {
                // Normalize depth value to 0-255 (depth using z)
                int depth_value_z = std::clamp(static_cast<int>(z * 255 / 50), 0, 255);  // Assuming max z depth is 50
                depth_map_z.at<uint8_t>(pixel_y_z, pixel_x_z) = 255 - depth_value_z;  // Invert so closer points are brighter
            }
        }

        // Combine both depth maps into one image (side-by-side)
        cv::Mat combined_depth_map(height_, 2 * width_, CV_8UC1);
        depth_map_x.copyTo(combined_depth_map(cv::Rect(0, 0, width_, height_)));  // Copy depth_map_x to the left
        depth_map_z.copyTo(combined_depth_map(cv::Rect(width_, 0, width_, height_)));  // Copy depth_map_z to the right

        // Convert the combined depth map to a ROS Image message
        sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", combined_depth_map).toImageMsg();

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