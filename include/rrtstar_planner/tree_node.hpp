#pragma once

#include <vector>
#include <memory>
#include <cmath>
namespace rrtstar_planner {

struct TreeNode
{
  double x;
  double y;
  double heading;

  std::vector<std::shared_ptr<TreeNode>> children;
  std::weak_ptr<TreeNode> parent;

  double parentDistance = 0.0;

  TreeNode(double x_, double y_, double heading_)
  : x(x_), y(y_), heading(heading_) {}
};

} // namespace rrtstar_planner


// #pragma once
// #include <vector>
// #include <memory>

// namespace rrtstar_planner {

// struct TreeNode
// {
//   double x;
//   double y;
//   double heading;

//   std::vector<std::shared_ptr<TreeNode>> children;
//   std::weak_ptr<TreeNode> parent;

//   double parentDistance = 0.0; //distance from now to parent
//   std::vector<double> xTP;  // Dubin's Path relay point
//   std::vector<double> yTP;

//   TreeNode(double x_, double y_, double heading_)
//     : x(x_), y(y_), heading(heading_) {}
// };

// } // namespace rrtstar_planner


// shared_ptr 是為了安全的記憶體管理：多個物件可以共同持有一個節點。
// weak_ptr 是一種「不擁有資源」的指標，只能觀察，不會延長物件生命週期。

/*
  In this file, I've create a data structure which represent a single tree node
  of our project. 
  It contains position, heading, parents and child node, and dubins path sequence.
  
  How to execute?
  std::shared_ptr<rrtstar_planner::TreeNode> node =
  std::make_shared<rrtstar_planner::TreeNode>(x, y, heading);

  using namespace rrtstar_planner;
  // 然後就可以直接用 TreeNode
  TreeNode node(x, y, heading);
*/
