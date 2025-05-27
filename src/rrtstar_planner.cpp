#include "rrtstar_planner/rrtstar_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

PLUGINLIB_EXPORT_CLASS(rrtstar_planner::RRTStarPlanner, nav2_core::GlobalPlanner)

namespace rrtstar_planner {
  RRTStarPlanner::RRTStarPlanner() {}
  RRTStarPlanner::~RRTStarPlanner() {}

using nav2_costmap_2d::LETHAL_OBSTACLE;

void RRTStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  node_->get_parameter_or(name + ".max_iterations", max_iterations_, 10000);
  node_->get_parameter_or(name + ".step_length", step_length_, 1.0);
  node_->get_parameter_or(name + ".search_radius", search_radius_, 5.0);
  node_->get_parameter_or(name + ".goal_tolerance", goal_tolerance_, 5.0);
  node_->get_parameter_or(name + ".lethal_cost", lethal_cost_, 254);
  node_->get_parameter_or(name + ".obstacle_threshold", obstacle_threshold_, 254);
}

void RRTStarPlanner::cleanup() {}
void RRTStarPlanner::activate() {}
void RRTStarPlanner::deactivate() {}

nav_msgs::msg::Path RRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  //// How many times does this createPlan being called
  // static int call_count = 0;
  // call_count++;
  // RCLCPP_INFO(node_->get_logger(), "[RRT*] createPlan() call #%d", call_count);

  // Debug
  RCLCPP_INFO(node_->get_logger(), "[RRT*] Received planning request.");

  nav_msgs::msg::Path path;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = node_->now();

  ////  Deal with Map ////
  auto costmap = costmap_ros_->getCostmap();
  int W = costmap->getSizeInCellsX();
  int H = costmap->getSizeInCellsY();
  RCLCPP_INFO(node_->get_logger(), "[RRT*] Grid size: %d x %d", W, H);

  // Deal with Costmap : Turn it into vector
  std::vector<std::vector<int>> grid(H, std::vector<int>(W, 0));
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      unsigned char cost = costmap->getCost(x, y);
      // RCLCPP_INFO(node_->get_logger(), "Point (%d,%d) cost=%d", x, y, cost);
      if (cost >= obstacle_threshold_) {
          grid[y][x] = 1;
          // Debug print to help tune threshold
          // if (x % 10 == 0 && y % 10 == 0)
              // RCLCPP_INFO(node_->get_logger(), "Obstacle at (%d, %d), cost = %d", x, y, cost);
      } else {
          grid[y][x] = 0;
      }
    }
  }
  // Debug the costmap
  int free_count = 0;
  int total_cells = H * W;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      if (grid[y][x] == 0)
        free_count++;
    }
  }

  double free_ratio = static_cast<double>(free_count) / total_cells;
  // RCLCPP_INFO(node_->get_logger(), "Free space ratio = %.2f%% (%d / %d)", free_ratio * 100.0, free_count, total_cells);


  //// Goal and Start Point  //// 
  RCLCPP_INFO(node_->get_logger(), "[RRT*] Start: (%.2f, %.2f), Goal: (%.2f, %.2f)",
            start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  int sx, sy, gx, gy;
  worldToMap(costmap, start.pose.position.x, start.pose.position.y, sx, sy);
  worldToMap(costmap, goal.pose.position.x, goal.pose.position.y, gx, gy);
  RCLCPP_INFO(node_->get_logger(), "[RRT*] Start map index: (%d, %d), Goal map index: (%d, %d)", sx, sy, gx, gy);


  // Goal Detection : Make sure the goal is not in obstacle
  if (costmap->getCost(gx, gy) >= obstacle_threshold_) {
    RCLCPP_WARN(node_->get_logger(), "[RRT*] Goal is in obstacle. Planning aborted.");
    return path;
  }

  // Get the heading and create the start and goal tree node
  double sheading = tf2::getYaw(start.pose.orientation);
  double gheading = tf2::getYaw(goal.pose.orientation);
  auto startNode = std::make_shared<TreeNode>((double)sx, (double)sy, sheading);
  auto goalNode  = std::make_shared<TreeNode>((double)gx, (double)gy, gheading);

  // RRTStarAlgorithm Start! Return back the path
  RRTStarAlgorithm planner(
    startNode, goalNode, max_iterations_, grid,
    step_length_, search_radius_, goal_tolerance_);
  planner.run();

  auto rawPath = planner.getPath();
  if (rawPath.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[RRT*] Planning failed: no valid path found.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "[RRT*] Planning succeeded. Path length: %lu", rawPath.size());
  }

  // Turn the rawPath(from my rrtstar) into the ROS Format
  for (auto it = rawPath.begin(); it != rawPath.end(); ++it) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    // grid → world 座標
    costmap->mapToWorld((int)it->first, (int)it->second,
                        pose.pose.position.x, pose.pose.position.y);

    // 計算 heading（方向）
    double yaw = 0.0;
    if (std::next(it) != rawPath.end()) {
      auto next = std::next(it);
      yaw = std::atan2(next->second - it->second, next->first - it->first);
    } else if (it != rawPath.begin()) {
      auto prev = std::prev(it);
      yaw = std::atan2(it->second - prev->second, it->first - prev->first);
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  } 
  return path;
}

void RRTStarPlanner::worldToMap(
  const nav2_costmap_2d::Costmap2D * map,
  double wx, double wy,
  int & mx, int & my) const
{
  map->worldToMapNoBounds(wx, wy, mx, my);
}

}  // namespace rrtstar_planner



