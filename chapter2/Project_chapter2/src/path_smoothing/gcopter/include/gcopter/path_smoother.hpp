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

    // cost由两部分组成
    // 曲线的Energy +障碍物检测 Potential
    // double energy;
    // cubSpline.getStretchEnergy(energy);

    return cost;
  }

  double getPotential(const Eigen::VectorXd &x, Eigen::VectorXd &g);

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
    // 优化的点，
    points.resize(2, pieceN - 1);
    // 优化点的梯度
    gradByPoints.resize(2, pieceN - 1);

    return true;
  }

  inline double optimize(CubicCurve &curve, const Eigen::Matrix2Xd &iniInPs,
                         const double &relCostTol) {

    // TODO
    double minCost = 0;

    std::cout << "in optimize " << std::endl;
    cubSpline.setInnerPoints(iniInPs);
    cubSpline.getCurve(curve);
    lbfgs::lbfgs_parameter_t param;
    // 参考 汪博开源gcopter中的用法
    param.mem_size = 18;
    param.g_epsilon = 0.0;
    param.min_step = 1.0e-32;
    param.past = 3;
    param.delta = 1.0e-7;
    // 优化的参数是中间节点
    // auto flag = lbfgs::lbfgs_optimize(

    // );
    return minCost;
  }
};

} // namespace path_smoother

#endif
