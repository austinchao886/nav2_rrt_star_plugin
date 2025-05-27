// File: src/dubins_path.cpp
#include "rrtstar_planner/dubins_path.hpp"
#include <cmath>
#include <algorithm>


namespace rrtstar_planner {
namespace dubins {

static constexpr double STEP = 0.05;  // angular increment per sample

void calcCSCTraj(
  double x1, double y1, double psi1,
  double xc1, double yc1,
  double x2, double y2, double psi2,
  double xc2, double yc2,
  double R, TurnType /*type*/,
  int sigma1, int sigma2,
  std::vector<double>& out_x,
  std::vector<double>& out_y,
  double& pathLength)
{
  // 1. Determine psi_D
  double psiL = std::atan2(yc2 - yc1, xc2 - xc1);
  double Stan = dist(xc1, yc1, xc2, yc2);
  double psiD = psiL;
  // Adjust for LSR / RSL if needed
  if (sigma1 != sigma2) {
    double ang = std::atan(2*R/Stan);
    if (sigma1 < sigma2)      psiD = psiL + ang;  // LSR
    else if (sigma1 > sigma2) psiD = psiL - ang;  // RSL
    psiD = normalizeAngle(psiD);
  }

  // 2. Turn-out arc from psi1 → psiD around center1
  double psiT = psi1;
  out_x.push_back(x1);  out_y.push_back(y1);
  for (int i = 1; std::fabs(normalizeAngle(psiT - psiD)) > 0.1; ++i) {
    double ang = STEP * i;
    double cosA = std::cos(ang), sinA = std::sin(ang);
    // rotate point (x1-xc1, y1-yc1)
    double xr = x1 - xc1, yr = y1 - yc1;
    double xnew =  cosA * xr - sigma1 * sinA * yr + xc1;
    double ynew =  sigma1 * sinA * xr + cosA * yr + yc1;
    out_x.push_back(xnew);
    out_y.push_back(ynew);
    pathLength += STEP * R;
    psiT += sigma1 * STEP;
    psiT = normalizeAngle(psiT);
  }

  // 3. Straight segment between circles
  // last turn-out point:
  double xco = out_x.back(), yco = out_y.back();
  // first turn-in point:
  double psiI = psi2;
  // compute cut-in coords:
  // rotate from psi2 backwards until align
  double psiTin = psi2;
  double xci = x2, yci = y2; // initial
  // similar loop backwards
  for (int i = 1; std::fabs(normalizeAngle(psiTin - psiD)) > 0.1; ++i) {
    double ang = STEP * i;
    double cosA = std::cos(ang), sinA = std::sin(ang);
    double xr = x2 - xc2, yr = y2 - yc2;
    double xi =  cosA * xr + sigma2 * sinA * yr + xc2;
    double yi = -sigma2 * sinA * xr + cosA * yr + yc2;
    xci = xi; yci = yi;
    psiTin -= sigma2 * STEP;
    psiTin = normalizeAngle(psiTin);
  }
  // straight from (xco,yco) → (xci,yci)
  double vx = xci - xco, vy = yci - yco;
  double d = dist(xco, yco, xci, yci);
  int nseg = static_cast<int>(d / 3.0);
  double ux = vx / d, uy = vy / d;
  for (int i = 0; i < nseg; ++i) {
    double nx = out_x.back() + 3.0 * ux;
    double ny = out_y.back() + 3.0 * uy;
    out_x.push_back(nx);
    out_y.push_back(ny);
  }
  pathLength += d;

  // 4. Turn-in arc
  for (int i = 1; std::fabs(normalizeAngle(psiTin - psiD)) > 0.1; ++i) {
    double ang = STEP * i;
    double cosA = std::cos(ang), sinA = std::sin(ang);
    double xr = x2 - xc2, yr = y2 - yc2;
    double xnew =  cosA * xr + sigma2 * sinA * yr + xc2;
    double ynew = -sigma2 * sinA * xr + cosA * yr + yc2;
    out_x.push_back(xnew);
    out_y.push_back(ynew);
    pathLength += STEP * R;
    psiTin -= sigma2 * STEP;
    psiTin = normalizeAngle(psiTin);
  }
  // finally append the exact goal
  out_x.push_back(x2);
  out_y.push_back(y2);
}

void calcCCCTraj(
  double x1, double y1,
  double xc1, double yc1,
  double xc2, double yc2,
  double x2, double y2,
  double R, TurnType /*type*/,
  int sigma1, int sigma2,
  std::vector<double>& out_x,
  std::vector<double>& out_y,
  double& pathLength)
{
  // 1. Distance & angle between centers
  double D = dist(xc1, yc1, xc2, yc2);
  double theta = std::acos(D / (4.0 * R));
  // center of middle circle:
  double ux = (xc2 - xc1)/D, uy = (yc2 - yc1)/D;
  // rotation for mid-center
  double cosT = std::cos(theta), sinT = std::sin(theta);
  double mx = xc1 + 2*R*( cosT*ux - sigma1*sinT*uy );
  double my = yc1 + 2*R*( sigma1*sinT*ux + cosT*uy );
  // cut-out and cut-in
  double psiTin = 0; // we skip angle math details for brevity

  // Start at (x1,y1)
  out_x.push_back(x1); out_y.push_back(y1);
  // First arc: turn from start → mid arc
  double psiT = 0.0;
  for (int i = 1; i*STEP*R < dist(x1,y1,mx,my); ++i) {
    double ang = STEP * i;
    double cosA = std::cos(ang), sinA = std::sin(ang);
    double xr = x1 - xc1, yr = y1 - yc1;
    double xnew =  cosA * xr - sigma1 * sinA * yr + xc1;
    double ynew =  sigma1 * sinA * xr + cosA * yr + yc1;
    out_x.push_back(xnew);
    out_y.push_back(ynew);
    pathLength += STEP * R;
  }
  // Middle arc
  for (int i = 1; i*STEP*R < dist(mx,my,xc2,yc2); ++i) {
    double ang = STEP * i;
    double cosA = std::cos(ang), sinA = std::sin(ang);
    double xr = out_x.back() - mx, yr = out_y.back() - my;
    double xnew =  cosA * xr + sigma2 * sinA * yr + mx;
    double ynew = -sigma2 * sinA * xr + cosA * yr + my;
    out_x.push_back(xnew);
    out_y.push_back(ynew);
    pathLength += STEP * R;
  }
  // Last arc: mid circle → goal
  for (int i = 1; i*STEP*R < dist(xc2,yc2,x2,y2); ++i) {
    double ang = STEP * i;
    double cosA = std::cos(ang), sinA = std::sin(ang);
    double xr = out_x.back() - xc2, yr = out_y.back() - yc2;
    double xnew =  cosA * xr - sigma1 * sinA * yr + xc2;
    double ynew =  sigma1 * sinA * xr + cosA * yr + yc2;
    out_x.push_back(xnew);
    out_y.push_back(ynew);
    pathLength += STEP * R;
  }
  out_x.push_back(x2);
  out_y.push_back(y2);
}

}  // namespace dubins
}  // namespace rrtstar_planner
