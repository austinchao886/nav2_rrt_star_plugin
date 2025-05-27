// File: include/rrtstar_planner/dubins_path.hpp
#pragma once

#include <vector>
#include <cmath>
#include <tuple>

namespace rrtstar_planner {
namespace dubins {

// === Types & Helpers ===

// Turn types for Dubins paths
enum class TurnType { RSR, RSL, LSR, LSL, RLR, LRL };

// Normalize angle into (-π, π]
inline double normalizeAngle(double a) {
  double r = std::fmod(a + M_PI, 2.0 * M_PI);
  if (r < 0) r += 2.0 * M_PI;
  return r - M_PI;
}

// Distance between two points
inline double dist(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

// Compute centre of the turning circle
inline std::pair<double,double> circleCenter(
  double x, double y, double psi,
  int sigma, double R)
{
  return {
    x + R * std::cos(psi + sigma * M_PI/2.0),
    y + R * std::sin(psi + sigma * M_PI/2.0)
  };
}

// Decide CSC type (RSL, RSR, LSR, LSL)
inline std::tuple<int,int,TurnType> findCSCTurn(double cp1, double cp2) {
  if (cp1 <= 0 && cp2 >  0) return {-1,  1, TurnType::RSL};
  if (cp1 <= 0 && cp2 <  0) return {-1, -1, TurnType::RSR};
  if (cp1 >  0 && cp2 <  0) return { 1, -1, TurnType::LSR};
  /* cp1>0 && cp2>0 */       return { 1,  1, TurnType::LSL};
}

// Decide CCC type (RLR, LRL) if end turns match
inline std::tuple<int,int,TurnType,bool> findCCCTurn(TurnType old) {
  bool proceed = false;
  int s1=0, s2=0;
  TurnType t = old;
  if (old==TurnType::RSR || old==TurnType::RSL) {
    s1 = s2 = -1; t = TurnType::RLR; proceed = true;
  }
  if (old==TurnType::LSL || old==TurnType::LSR) {
    s1 = s2 =  1; t = TurnType::LRL; proceed = true;
  }
  return {s1, s2, t, proceed};
}

// === Core Calculators ===

// Generate a CSC trajectory segment
// - (x1,y1,psi1) → circle1 → straight → circle2 → (x2,y2,psi2)
// - out_x/out_y are appended with the full trajectory
// - pathLength accumulates total length
void calcCSCTraj(
  double x1, double y1, double psi1,
  double xc1, double yc1,
  double x2, double y2, double psi2,
  double xc2, double yc2,
  double R, TurnType type,
  int sigma1, int sigma2,
  std::vector<double>& out_x,
  std::vector<double>& out_y,
  double& pathLength);

// Generate a CCC trajectory segment
void calcCCCTraj(
  double x1, double y1,
  double xc1, double yc1,
  double xc2, double yc2,
  double x2, double y2,
  double R, TurnType type,
  int sigma1, int sigma2,
  std::vector<double>& out_x,
  std::vector<double>& out_y,
  double& pathLength);

// === Main Entry ===
// Fills out_x/out_y with the Dubins trajectory, returns its length
inline double returnDubinsPath(
    double x1, double y1, double psi1,
    double x2, double y2, double psi2,
    double R,
    std::vector<double>& out_x,
    std::vector<double>& out_y)
{
  // Unit headings
  double ux1 = std::cos(psi1), uy1 = std::sin(psi1);
  double dx = x2 - x1,       dy = y2 - y1;
  double dnorm = std::hypot(dx, dy);
  double ux2 = dx/dnorm,     uy2 = dy/dnorm;
  double ux3 = std::cos(psi2), uy3 = std::sin(psi2);

  // Cross products
  double cp1 = ux1*uy2 - uy1*ux2;
  double cp2 = ux2*uy3 - uy2*ux3;

  // CSC: find type & circle centers
  int s1, s2; TurnType cscType;
  std::tie(s1, s2, cscType) = findCSCTurn(cp1, cp2);
  auto [xc1_c, yc1_c] = circleCenter(x1, y1, psi1, s1, R);
  auto [xc2_c, yc2_c] = circleCenter(x2, y2, psi2, s2, R);

  // Decide which pattern
  double centerDist = dist(xc1_c, yc1_c, xc2_c, yc2_c);
  double totalLen = -1.0;
  out_x.clear(); out_y.clear();

  if (centerDist >= 3*R) {
    totalLen = 0.0;
    calcCSCTraj(x1,y1,psi1,
                xc1_c,yc1_c,
                x2,y2,psi2,
                xc2_c,yc2_c,
                R, cscType, s1, s2,
                out_x, out_y, totalLen);
  } else if (centerDist >= 2*R) {
    int c1, c2; TurnType cccType; bool ok;
    std::tie(c1,c2,cccType,ok) = findCCCTurn(cscType);
    if (ok) {
      totalLen = 0.0;
      calcCCCTraj(x1,y1,
                  xc1_c,yc1_c,
                  xc2_c,yc2_c,
                  x2,y2,
                  R, cccType, c1, c2,
                  out_x, out_y, totalLen);
    }
  }
  return totalLen;
}

} // namespace dubins
} // namespace rrtstar_planner
