#include "rrtstar_planner/rrtstar_algorithm.hpp"
#include <algorithm>  // for std::clamp, std::reverse
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace rrtstar_planner {

RRTStarAlgorithm::RRTStarAlgorithm(
  std::shared_ptr<TreeNode> start,
  std::shared_ptr<TreeNode> goal,
  int iterations,
  const std::vector<std::vector<int>>& grid,
  double step_len,
  double search_radius,
  double goal_tol)
: root_(start), goal_(goal), grid_(grid), iterations_(iterations),
  step_length_(step_len), search_radius_(search_radius), goal_tolerance_(goal_tol),
  rng_(std::random_device{}())
{
  x_dist_ = std::uniform_int_distribution<int>(0, static_cast<int>(grid_[0].size()) - 1);
  y_dist_ = std::uniform_int_distribution<int>(0, static_cast<int>(grid_.size()) - 1);
  // std::cout << "[RRT*] Initialized RRTStarAlgorithm with " << iterations_ << " iterations\n";
}

std::pair<int, int> RRTStarAlgorithm::samplePoint() {
  auto pt = std::make_pair(x_dist_(rng_), y_dist_(rng_));
  // std::cout << "[RRT*] Sampled point: (" << pt.first << ", " << pt.second << ")\n";
  return pt;
}

double RRTStarAlgorithm::distance(
  const std::shared_ptr<TreeNode>& node,
  const std::pair<int, int>& point) const
{
  double dx = node->x - point.first;
  double dy = node->y - point.second;
  return std::hypot(dx, dy);
}

std::shared_ptr<TreeNode> RRTStarAlgorithm::findNearest(
  const std::shared_ptr<TreeNode>& current,
  const std::pair<int, int>& point)
{
  if (!current) return nullptr;

  std::shared_ptr<TreeNode> best = current;
  double minDist = distance(current, point);

  for (const auto& child : current->children) {
    auto cand = findNearest(child, point);
    double d = distance(cand, point);
    if (d < minDist) {
      best = cand;
      minDist = d;
    }
  }
  return best;
}

std::pair<double, double> RRTStarAlgorithm::steerToPoint(
  const std::shared_ptr<TreeNode>& from,
  const std::pair<int, int>& to)
{
  double dx = to.first - from->x;
  double dy = to.second - from->y;
  double norm = std::hypot(dx, dy); //unit vector

  double nx = from->x + (dx / norm) * step_length_;
  double ny = from->y + (dy / norm) * step_length_;

  nx = std::clamp(nx, 0.0, (double)grid_[0].size() - 1);
  ny = std::clamp(ny, 0.0, (double)grid_.size() - 1);

  // std::cout << "[RRT*] Steered to: (" << nx << ", " << ny << ")\n";
  return {nx, ny};
}

bool RRTStarAlgorithm::isInObstacle(const std::pair<double, double>& pt) const {
  int ix = std::round(pt.first);
  int iy = std::round(pt.second);

  if (ix < 0 || iy < 0 || iy >= (int)grid_.size() || ix >= (int)grid_[0].size()) return true; // out of map's range
  
  bool obstacle = grid_[iy][ix] != 0; // is obstacle
  // if (obstacle) {
  //   // std::cout << "[RRT*] Point (" << ix << ", " << iy << ") is in obstacle.\n";
  // }
  return obstacle;
}

void RRTStarAlgorithm::findNeighbouringNodes(
  const std::shared_ptr<TreeNode>& current,
  const std::pair<int, int>& point)
{
  if (!current) return;
  double d = distance(current, point);
  if (d <= search_radius_) neighbouringNodes_.push_back(current);
  for (auto& child : current->children) {
    findNeighbouringNodes(child, point);
  }
}

double RRTStarAlgorithm::findPathDistance(const std::shared_ptr<TreeNode>& node) const {
  double total = 0.0;
  auto current = node;
  while (auto p = current->parent.lock()) {
    total += current->parentDistance;
    current = p;
  }
  return total;
}

void RRTStarAlgorithm::addChild(const std::shared_ptr<TreeNode>& newNode) {
  if (auto p = newNode->parent.lock()) {
    p->children.push_back(newNode);
  }
}

void RRTStarAlgorithm::retracePath() {
  finalPath_.clear();
  auto node = goal_;
  while (node) {
    finalPath_.push_back({node->x, node->y});
    if (node == root_) break;
    node = node->parent.lock();
  }
  std::reverse(finalPath_.begin(), finalPath_.end());
  // std::cout << "[RRT*] Path retraced. Path size: " << finalPath_.size() << "\n";
}

void RRTStarAlgorithm::run() {
  double min_goal_distance = std::numeric_limits<double>::max();
  std::cout << "[RRT*] Starting RRT* algorithm...\n";
  for (int i = 0; i < iterations_; ++i) {
    auto pt = samplePoint();
    auto nearest = findNearest(root_, pt);
    auto newPos = steerToPoint(nearest, pt);

    if (isInObstacle(newPos)) {
      // std::cout << "[RRT*] Skipped due to obstacle.\n";
      continue;
    }

    //// Having a new point -> start assigning heading and treeNode
    double heading = std::atan2(pt.second - newPos.second, pt.first - newPos.first);
    auto newNode = std::make_shared<TreeNode>(newPos.first, newPos.second, heading);
    newNode->parent = nearest;
    newNode->parentDistance = step_length_;
    //// Growing Tree -> Define the relationship of parent and child
    addChild(newNode);
    // std::cout << "[RRT*] Added new node at (" << newNode->x << ", " << newNode->y << ")\n";

    //// Debug : Making Sure that the min distance to goal converges
    double goalDist = distance(newNode, {goal_->x, goal_->y});
    if (goalDist < min_goal_distance) {
      min_goal_distance = goalDist;
      std::cout << "[RRT*] Iteration " << i
                << " | New minimum dist to goal: " << goalDist << "\n";
    }
    

    ///// Rewiring Part
    neighbouringNodes_.clear();
    findNeighbouringNodes(root_, {static_cast<int>(newPos.first), static_cast<int>(newPos.second)});
    // std::cout << "[RRT*] Found " << neighbouringNodes_.size() << " neighbours for rewiring\n";

    for (auto& n : neighbouringNodes_) {
      double oldCost = findPathDistance(n);
      double newCost = findPathDistance(newNode) + distance(newNode, {static_cast<int>(n->x), static_cast<int>(n->y)});
      if (newCost < oldCost && !isInObstacle({n->x, n->y})) {
        n->parent = newNode;
        n->parentDistance = distance(newNode, {static_cast<int>(n->x), static_cast<int>(n->y)});
        // std::cout << "[RRT*] Rewired node at (" << n->x << ", " << n->y << ")\n";
      }
    }

    // double goalDist = distance(newNode, {static_cast<int>(goal_->x), static_cast<int>(goal_->y)});
    // RCLCPP_INFO(rclcpp::get_logger("RRTStar"), "[RRT*] Distance to goal: %.2f", goalDist);

    if (distance(newNode, {static_cast<int>(goal_->x), static_cast<int>(goal_->y)}) <= goal_tolerance_) {
      goal_->parent = newNode;
      newNode->children.push_back(goal_);
      std::cout << "[RRT*] Goal reached! Retracing path...\n";
      retracePath();
      break;
    }
  }
  std::cout << "[RRT*] Planning complete.\n";
}

std::vector<std::pair<double, double>> RRTStarAlgorithm::getPath() const {
  return finalPath_;
}

} // namespace rrtstar_planner


