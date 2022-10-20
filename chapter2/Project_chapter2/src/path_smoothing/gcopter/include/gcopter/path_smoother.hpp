#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

#include "cubic_spline.hpp"
#include "lbfgs.hpp"

#include <Eigen/Eigen>

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

namespace path_smoother {

class PathSmoother {
private:
  // 三次样条曲线
  cubic_spline::CubicSpline cubSpline;

  int pieceN;
  Eigen::Matrix3Xd diskObstacles; //圆形障碍物，坐标＋半径
  double penaltyWeight;
  Eigen::Vector2d headP;
  Eigen::Vector2d tailP;
  Eigen::Matrix2Xd points;
  Eigen::Matrix2Xd gradByPoints;

  lbfgs::lbfgs_parameter_t lbfgs_params;

private:
  static inline double costFunction(void *ptr, const Eigen::VectorXd &x,
                                    Eigen::VectorXd &g) {
    // TODO
    double cost = 0;
    return cost;
  }

public:
  // 测试使用，检查样条曲线的情况

  inline bool setup(const Eigen::Vector2d &initialP,
                    const Eigen::Vector2d &terminalP, const int &pieceNum,
                    const Eigen::Matrix3Xd &diskObs, const double penaWeight) {
    pieceN = pieceNum;
    diskObstacles = diskObs;
    penaltyWeight = penaWeight;
    headP = initialP;
    tailP = terminalP;
    // 初始化样条曲线
    cubSpline.setConditions(headP, tailP, pieceN);

    points.resize(2, pieceN - 1);
    gradByPoints.resize(2, pieceN - 1);

    return true;
  }

  inline double optimize(CubicCurve &curve, const Eigen::Matrix2Xd &iniInPs,
                         const double &relCostTol) {
    std::cout << "in optimize " << std::endl;
    cubSpline.setInnerPoints(iniInPs);
    cubSpline.getCurve(curve);
    lbfgs::lbfgs_parameter_t param;
    // param.


    auto flag = lbfgs::lbfgs_optimize();

    // TODO
    double minCost = 0;
    return minCost;
  }
};

} // namespace path_smoother

#endif