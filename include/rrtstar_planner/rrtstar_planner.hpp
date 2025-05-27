#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rrtstar_planner/rrtstar_algorithm.hpp"

namespace rrtstar_planner
{
class RRTStarPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTStarPlanner();
  ~RRTStarPlanner();

void configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  double step_length_;
  double search_radius_;
  double goal_tolerance_;
  int max_iterations_;
  int lethal_cost_;
  int obstacle_threshold_;

  void worldToMap(
    const nav2_costmap_2d::Costmap2D * map,
    double wx, double wy,
    int & mx, int & my) const;
};

}  // namespace rrtstar_planner


