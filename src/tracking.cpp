#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <f110_msgs/msg/obstacle_array.hpp>

#include <unordered_map>
#include <utility>
#include <cmath>

class TrackingNode : public rclcpp::Node
{
public:
  TrackingNode()
  : Node("tracking_cpp")
  {
    // 구독: Perception 결과
    obstacle_sub_ = this->create_subscription<f110_msgs::msg::ObstacleArray>(
      "/perception/detection/raw_obstacles_cpp",
      10,
      std::bind(&TrackingNode::obstacleCallback, this, std::placeholders::_1));

    // 발행: Costmap에 들어갈 PointCloud2
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/obstacle_points", 10);

    RCLCPP_INFO(this->get_logger(), "TrackingNode started.");
  }

private:
  // 간단한 smoothing을 위한 이전 위치 저장
  std::unordered_map<int, std::pair<double, double>> prev_center_;

  void obstacleCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    // PointCloud2 메시지 생성
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = "base_link";  // costmap 기준
    cloud_msg.height = 1;

    // 장애물 개수만큼 width
    cloud_msg.width = msg->obstacles.size();

    // PointCloud2 필드 생성 (x,y,z)
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
        3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32);

    modifier.resize(msg->obstacles.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (const auto & obs : msg->obstacles)
    {
      double cx = obs.s_center * std::cos(0.0) - 0.0; // 간단하게 사용
      double cy = obs.d_center;

      // smoothing (이전 값 있으면 평균)
      if (prev_center_.count(obs.id))
      {
        cx = 0.7 * prev_center_[obs.id].first + 0.3 * cx;
        cy = 0.7 * prev_center_[obs.id].second + 0.3 * cy;
      }

      prev_center_[obs.id] = {cx, cy};

      // PointCloud2에 채우기
      *iter_x = static_cast<float>(cx);
      *iter_y = static_cast<float>(cy);
      *iter_z = 0.0f;   // Costmap은 z=0 권장

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud_pub_->publish(cloud_msg);
  }

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
