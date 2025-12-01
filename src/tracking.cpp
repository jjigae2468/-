#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>

class TrackingNode : public rclcpp::Node
{
public:
  TrackingNode()
  : Node("tracking_cpp")
  {
    obs_sub_ = this->create_subscription<f110_msgs::msg::ObstacleArray>(
      "/perception/detection/raw_obstacles_cpp", 10,
      std::bind(&TrackingNode::obsCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_points", 10);

    RCLCPP_INFO(get_logger(), "tracking_cpp started.");
  }

private:
  void obsCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;          // frame_id = "base_link" 이어야 costmap이 해석 쉬움
    cloud.height = 1;
    cloud.width  = msg->obstacles.size();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(msg->obstacles.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    for (const auto & obs : msg->obstacles)
    {
      double x = obs.s_center;   // simple_perception에서 x로 채웠다고 가정
      double y = obs.d_center;   // y
      *out_x = static_cast<float>(x);
      *out_y = static_cast<float>(y);
      *out_z = 0.0f;

      ++out_x; ++out_y; ++out_z;
    }

    cloud_pub_->publish(cloud);
  }

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};
