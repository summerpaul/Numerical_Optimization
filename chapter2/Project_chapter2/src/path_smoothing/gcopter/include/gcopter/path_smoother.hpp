#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

#include "cubic_spline.hpp"
#include "lbfgs.hpp"

#include <Eigen/Eigen>

#include <cfloat>
#include <cmath>
#include <iostream>
#include <sys/time.h>
#include <vector>
namespace path_smoother {
static double getTimeNow() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec / 1000000.0;
}
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
  // 把二维的点变成一维
  static inline double costFunction(void *ptr, const Eigen::VectorXd &x,
                                    Eigen::VectorXd &g) {
    // TODO
    // std::cout << "in costFunction " << std::endl;
    // std::cout << "x is " << std::endl;
    // PRINT_MATRIX(x);
    double cost{0}, cost_engrgy{0}, cost_obstacle{0};

    // 静态成员函数 访问非静态成员需要传递对象指针变量，*ptr应该是用this指针
    auto ins = reinterpret_cast<path_smoother::PathSmoother *>(ptr);
    // std::cout << "get instance " << std::endl;

    // 更新 points 与 gradByPoints
    int in_ps_size = x.size() / 2;
    ins->points.row(0) = x.head(in_ps_size).transpose();
    ins->points.row(1) = x.tail(in_ps_size).transpose();
    // PRINT_MATRIX(ins->points);

    // 更新三次样条曲线中的中间节点
    ins->cubSpline.setInnerPoints(ins->points);
    // std::cout << "setInnerPoints " << std::endl;
    // 更新三次样条曲线的能量
    ins->cubSpline.getStretchEnergy(cost_engrgy);
    ins->gradByPoints.setZero();

    // std::cout << "cost_engrgy is " << cost_engrgy << std::endl;
    // 更新样条曲线的梯度
    ins->cubSpline.getGrad(ins->gradByPoints);
    // 遍历节点与障碍物获取障碍物代价

    // PRINT_MATRIX(ins->gradByPoints);

    for (int i = 0; i < ins->points.cols(); i++) {
      // 找到每个节点的最大障碍物
      for (int j = 0; j < ins->diskObstacles.cols(); j++) {
        // point_circle_center_vec
        // 是点与圆心的距离的向量，也是需要原理的梯度gxng 方向
        auto point_circle_center_vec =
            ins->points.col(i) - ins->diskObstacles.col(j).head(2);
        // 点到圆心的距离
        double point_dis_to_circle_center = point_circle_center_vec.norm();

        // 点越在圆的内部，代价越大
        double point_cost =
            ins->diskObstacles.col(j).z() - point_dis_to_circle_center;

        // 点的圆的内部，有代价，在圆的外部代价为0
        // max{r(j) - ||x(i) - o(j)||, 0}
        if (point_cost > 0) {
          // 增加惩罚系数
          cost_obstacle += ins->penaltyWeight * point_cost;
          // 与障碍物的梯度计算，
          ins->gradByPoints.col(i) +=
              ins->penaltyWeight *
              (-point_circle_center_vec / point_dis_to_circle_center);
        }
      }
    }
    // PRINT_MATRIX(ins->gradByPoints);

    cost = cost_engrgy + cost_obstacle;
    // 将二维的梯度转为一维
    // if (g.size() != x.size()) {
    //   g.resize(x.size());
    // }
    g.setZero();
    g.head(in_ps_size) = ins->gradByPoints.row(0).transpose();
    g.tail(in_ps_size) = ins->gradByPoints.row(1).transpose();
    // for (int i = 0; i < ins->gradByPoints.cols(); i++) {
    //   g(2 * i) = ins->gradByPoints.col(i)[0];
    //   g(2 * i + 1) = ins->gradByPoints.col(i)[1];
    // }

    // static int count = 0;
    // std::cout << "count is" << count++ << std::endl;
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
    // cubSpline.setInnerPoints(iniInPs);
    // cubSpline.getCurve(curve);
    // 参考 汪博开源gcopter中的用法
    lbfgs_params.mem_size = 64;
    lbfgs_params.g_epsilon = 1.0e-3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.past = 3;
    lbfgs_params.delta = relCostTol;

    // 求解参数的大小 点的数量 *2
    int in_ps_size = iniInPs.cols();
    Eigen::VectorXd x(in_ps_size * 2);
    // for (int i = 0; i < iniInPs.cols(); i++) {
    //   x(2 * i) = iniInPs.col(i)[0];
    //   x(2 * i + 1) = iniInPs.col(i)[1];
    // }
    x.head(in_ps_size) = iniInPs.row(0).transpose();
    x.tail(in_ps_size) = iniInPs.row(1).transpose();

    // PRINT_MATRIX(iniInPs);
    // PRINT_MATRIX(x);

    // 优化的参数是中间节点
    double last_t = getTimeNow();
    // int flag = 1;
    auto flag = lbfgs::lbfgs_optimize(x, minCost, &PathSmoother::costFunction,
                                      nullptr, this, lbfgs_params);
    double current_t = getTimeNow();
    std::cout << "dt is " << (current_t - last_t) << std::endl;
    std::cout << "flag is " << flag << std::endl;
    if (flag >= 0) {
      cubSpline.getCurve(curve);
    } else {
      curve.clear();
      minCost = INFINITY;
      std::cout << "Optimization Failed: " << std::endl;
    }
    return minCost;
  }
};

} // namespace path_smoother

#endif
