#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>

#include <cmath>
#include <vector>
#include <utility>
#include <limits>
//일단... /scan을 구독하고 발행을 raw_obstacles_cpp로 예를 들자.
class DetectCppNode : public rclcpp::Node
{
public:
  DetectCppNode()
  : Node("detect_cpp")
  {
    // 파라미터
    cluster_angle_deg_ = this->declare_parameter<double>("cluster_angle_deg", 10.0);
    max_range_         = this->declare_parameter<double>("max_range", 10.0);
    min_cluster_size_  = this->declare_parameter<int>("min_cluster_size", 5);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, //구독
      std::bind(&DetectCppNode::scanCallback, this, std::placeholders::_1));

    obs_pub_ = this->create_publisher<f110_msgs::msg::ObstacleArray>(
      "/perception/detection/raw_obstacles_cpp", 10); //발행

    RCLCPP_INFO(this->get_logger(), "DetectCppNode started.");
  }

private:
  using Point2D = std::pair<double,double>;
//토픽을 받아 좌표들을 묶어버리는 것을 클러스터링이라고 하는데
// 클러스터링을 통해 장애물을 인지할거임 
//Callback함수 : clusterscan()으로 클러스터 만들고 publishobstacles를 호출해서
//메시지로 만들어 publish
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 1.스캔을 간단히 클러스터링
    auto clusters = clusterScan(*msg);

    // 2.클러스터를 ObstacleArray 메시지로 변환해서 publish
    publishObstacles(clusters, msg->header.stamp);
  }

  std::vector<std::vector<Point2D>>
  clusterScan(const sensor_msgs::msg::LaserScan &scan)
  {
    std::vector<std::vector<Point2D>> clusters;

    const auto & ranges = scan.ranges;
    const size_t N = ranges.size();
    if (N == 0) return clusters;

    // 각도 간격
    double angle = scan.angle_min;
    double angle_inc = scan.angle_increment;

    // 클러스터에 넣을 현재 그룹
    std::vector<Point2D> current_cluster;
    current_cluster.reserve(50);

    // 거리 차이가 이 값보다 크면 새 클러스터 시작
    double angle_window_rad = cluster_angle_deg_ * M_PI / 180.0;
    int max_idx_step = std::max(1, (int)std::round(angle_window_rad / angle_inc));

    // 첫 점 처리
    double prev_x = 0.0, prev_y = 0.0;
    bool first_valid = false;

    for (size_t i = 0; i < N; ++i)
    {
      double r = ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < scan.range_min || r > scan.range_max) continue;
      if (r > max_range_) continue;

      double theta = scan.angle_min + i * angle_inc;
      double x = r * std::cos(theta);
      double y = r * std::sin(theta);

      if (!first_valid)
      {
        current_cluster.clear();
        current_cluster.push_back({x,y});
        prev_x = x;
        prev_y = y;
        first_valid = true;
        continue;
      }

      // 이전 점과 거리 비교
      double dx = x - prev_x;
      double dy = y - prev_y;
      double dist = std::sqrt(dx*dx + dy*dy);

      // 간단하게: 인접한 점 사이 거리 < 0.3m 이면 같은 클러스터
      // (이 값은 필요에 따라 조정)
      double dist_threshold = 0.3;

      if (dist < dist_threshold)
      {
        current_cluster.push_back({x,y});
      }
      else
      {
        // 이전 클러스터 저장
        if ((int)current_cluster.size() >= min_cluster_size_)
          clusters.push_back(current_cluster);

        // 새 클러스터 시작
        current_cluster.clear();
        current_cluster.push_back({x,y});
      }

      prev_x = x;
      prev_y = y;
    }

    // 마지막 클러스터도 저장
    if ((int)current_cluster.size() >= min_cluster_size_)
      clusters.push_back(current_cluster);

    return clusters;
  }
//클러스터를 obstaclearray 메시지로 변환해야하니까
// frame을 라이다 프레임 기준으로 base_link라고 하고,
//각 클러스터에 대해 중심을 구하고
//클러스터에서 가장 먼 점까지의 거리 *2를 이용해 장애물 한 변 길이처럼 사용
//r은 차량 기준 장애물까지의 거리
//obs_pub_으로 publish
//이렇게하면 frenet은 안써서 간단해지니까 좋네
  void publishObstacles(const std::vector<std::vector<Point2D>> &clusters,
                        const rclcpp::Time & stamp)
  {
    f110_msgs::msg::ObstacleArray array_msg;
    array_msg.header.stamp = stamp;
    array_msg.header.frame_id = "base_link";  // 라이다 프레임 기준이라고 가정

    int id = 0;
    for (const auto & cluster : clusters)
    {
      if (cluster.empty()) continue;

      // 중심 계산
            // 중심 계산
      double sum_x = 0.0, sum_y = 0.0;
      for (const auto & p : cluster)
      {
        sum_x += p.first;
        sum_y += p.second;
      }
      double cx = sum_x / cluster.size(); // 라이다 기준 x
      double cy = sum_y / cluster.size(); // 라이다 기준 y

      ...

      f110_msgs::msg::Obstacle obs;
      obs.id = id++;
      obs.size = size;

      // 여기서는 Frenet 안 쓰고 base_link 좌표라고 생각
      double x = cx;
      double y = cy;

      obs.s_center = x;                // "전방 거리" 느낌
      obs.d_center = y;                // "좌우 거리"
      obs.s_start  = x - size/2.0;
      obs.s_end    = x + size/2.0;
      obs.d_right  = y - size/2.0;
      obs.d_left   = y + size/2.0;

      obs.vs = 0.0;
      obs.vd = 0.0;
      obs.is_static = true;
      obs.is_actually_a_gap = false;
      obs.is_visible = true;

      array_msg.obstacles.push_back(obs);
    }

    obs_pub_->publish(array_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obs_pub_;

  double cluster_angle_deg_;
  double max_range_;
  int    min_cluster_size_;
};
//rclcpp::init -> DetectCppNode 생성하고 spin으로 콜백을 계속 돌림

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
