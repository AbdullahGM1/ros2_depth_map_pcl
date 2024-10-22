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
        : Node("point_cloud_to_depth_map"), width_(600), height_(600), scale_(50), MinDepth(0.0f), MaxDepth(30.0f)
    {
        // Subscriber for PointCloud2 messages
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/scan/points", 10, std::bind(&PointCloudToDepthMap::point_cloud_callback, this, std::placeholders::_1));
        
        // Publisher for combined depth map images
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/depth_map", 10);
        
        RCLCPP_INFO(this->get_logger(), "PointCloud to Depth Map Node has been started.");
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Create two empty depth maps for x and z depth representations
        cv::Mat depth_map_x = cv::Mat::zeros(height_, width_, CV_8UC1);
        cv::Mat depth_map_z = cv::Mat::zeros(height_, width_, CV_8UC1);

        // Filter cloud and create depth map for the X axis as depth
        create_depth_map(pcl_cloud, depth_map_x, "x");

        // Filter cloud and create depth map for the Z axis as depth
        create_depth_map(pcl_cloud, depth_map_z, "z");

        // Combine the two depth maps horizontally into a single image
        cv::Mat combined_depth_map;
        cv::hconcat(depth_map_x, depth_map_z, combined_depth_map);  // Concatenate images side by side

        // Convert the combined depth map to a ROS Image message
        sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", combined_depth_map).toImageMsg();

        // Publish the combined depth map
        publisher_->publish(*image_msg);
    }

    void create_depth_map(pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud, cv::Mat& depth_map, const std::string& axis)
    {
        // Apply filtering to remove far-away points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        
        if (axis == "x")
        {
            pass.setFilterFieldName("x");  // Set filter for the x-axis (depth)
        }
        else if (axis == "z")
        {
            pass.setFilterFieldName("z");  // Set filter for the z-axis (depth)
        }

        pass.setFilterLimits(MinDepth, MaxDepth);  // Filter points with x or z in the range [MinDepth, MaxDepth]
        pass.filter(*filtered_cloud);  // Apply filter

        // Define the center of the depth map
        int center_x = width_ / 2;
        int center_y = height_ / 2;

        // Iterate through the filtered point cloud and map points to the depth map
        for (const auto& point : filtered_cloud->points)
        {
            float x = point.x;  
            float y = point.y;
            float z = point.z;

            // For x-axis depth map
            if (axis == "x")
            {
                int pixel_y = static_cast<int>(center_x + y * scale_);
                int pixel_z = static_cast<int>(center_y - z * scale_);

                if (pixel_y >= 0 && pixel_y < width_ && pixel_z >= 0 && pixel_z < height_)
                {
                    int depth_value = std::clamp(static_cast<int>(x * 255 / MaxDepth), 0, 255);  
                    depth_map.at<uint8_t>(pixel_z, pixel_y) = 255 - depth_value;  // Invert so closer points are brighter
                }
            }
            // For z-axis depth map
            else if (axis == "z")
            {
                int pixel_x = static_cast<int>(center_x + x * scale_);
                int pixel_y = static_cast<int>(center_y - y * scale_);

                if (pixel_x >= 0 && pixel_x < width_ && pixel_y >= 0 && pixel_y < height_)
                {
                    int depth_value = std::clamp(static_cast<int>(z * 255 / MaxDepth), 0, 255);  
                    depth_map.at<uint8_t>(pixel_y, pixel_x) = 255 - depth_value;  // Invert so closer points are brighter
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    int width_;
    int height_;
    float scale_;
    float MinDepth;
    float MaxDepth;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToDepthMap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
