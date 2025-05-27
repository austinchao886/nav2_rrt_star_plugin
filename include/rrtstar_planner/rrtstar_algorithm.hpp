// rrtstar_algorithm.hpp
#pragma once

#include <vector>
#include <memory>
#include <random>
#include <utility>
#include "rrtstar_planner/tree_node.hpp"

namespace rrtstar_planner {

class RRTStarAlgorithm {
public:
  RRTStarAlgorithm(
    std::shared_ptr<TreeNode> start,
    std::shared_ptr<TreeNode> goal,
    int iterations,
    const std::vector<std::vector<int>>& grid,
    double step_length,
    double search_radius,
    double goal_tolerance);

  void run();
  std::vector<std::pair<double, double>> getPath() const;

private:
  std::shared_ptr<TreeNode> root_;
  std::shared_ptr<TreeNode> goal_;
  std::vector<std::vector<int>> grid_;
  int iterations_;
  double step_length_;
  double search_radius_;
  double goal_tolerance_;

  std::mt19937 rng_;
  std::uniform_int_distribution<int> x_dist_;
  std::uniform_int_distribution<int> y_dist_;

  std::vector<std::shared_ptr<TreeNode>> neighbouringNodes_;
  std::vector<std::pair<double, double>> finalPath_;

  std::pair<int, int> samplePoint();
  double distance(const std::shared_ptr<TreeNode>& node, const std::pair<int, int>& point) const;
  std::pair<double, double> steerToPoint(const std::shared_ptr<TreeNode>& from, const std::pair<int, int>& to);
  bool isInObstacle(const std::pair<double, double>& pt) const;
  std::shared_ptr<TreeNode> findNearest(const std::shared_ptr<TreeNode>& current, const std::pair<int, int>& point);
  void addChild(const std::shared_ptr<TreeNode>& newNode);
  void findNeighbouringNodes(const std::shared_ptr<TreeNode>& current, const std::pair<int, int>& point);
  void retracePath();
  double findPathDistance(const std::shared_ptr<TreeNode>& node) const;
};

} // namespace rrtstar_planner





