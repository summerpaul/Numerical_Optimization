/**
 * @Author: YunKai Xia
 * @Date:   2022-10-06 12:27:37
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-10-06 18:31:41
 */
#include <cmath>
#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <vector>

#include "common/common.h"
#include "common/matplotlibcpp.h"
#include "gradient_descent/gradient_descent.h"
using namespace std;
using namespace numerical_optimization::common;
using namespace numerical_optimization::gradient_descent;
namespace plt = matplotlibcpp;
void test1() {
  CostFunction fun = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient,
               const bool& plot) {
    double fval = (x(0) - 2) * (x(0) - 2) + (x(1) - 2) * (x(1) - 2);
    gradient.setZero();
    gradient(0) = 2 * (x(0) - 2);
    gradient(1) = 2 * (x(1) - 2);
    return fval;
  };
  Eigen::VectorXd init_x(2);
  init_x(0) = 53;
  init_x(1) = -68;
  GradientDecent gd(fun, init_x);
  auto result = gd.solve();
  std::cout << "result x is " << result.res_x << "result grad is "
            << result.res_gradient << "result iter time is " << result.iter_time
            << std::endl;
}
std::vector<double> x0_list;
std::vector<double> x1_list;
void test2() {
  CostFunction fun = [](const Eigen::VectorXd& x, Eigen::VectorXd& gradient,
               const bool& plot = false) {
    if (plot) {
      x0_list.push_back(x(0));
      x1_list.push_back(x(1));
    }

    double val = 0;
    gradient.resize(x.size());
    gradient.setZero();
    for (size_t i = 0; i < x.size() / 2; ++i) {
      double p1 = x[2 * i] * x[2 * i] - x[2 * i + 1];
      double p2 = x[2 * i] - 1;
      val += 100 * pow(p1, 2) + pow(p2, 2);

      gradient[2 * i] += 400 * (pow(x[2 * i], 3) - x[2 * i + 1] * x[2 * i]) +
                         2 * (x[2 * i] - 1);
      gradient[2 * i + 1] += -200 * (x[2 * i] * x[2 * i] - x[2 * i + 1]);
    }
    return val;
  };

  Eigen::VectorXd init_x(2);
  init_x(0) = 0;
  init_x(1) = 0;
  std::cout << "init_x is " << init_x << std::endl;
  GradientDecent gd(fun, init_x, 1.0);
  double start_t = getTimeNow();
  auto result = gd.solve();
  std::cout << "cost time is " << getTimeNow() - start_t << "s" << std::endl;
  std::cout << "result x is " << result.res_x << "\n "
            << "result grad is " << result.res_gradient << "\n"
            << "result iter num is " << result.iter_time << std::endl;
}

void plot() {
  plt::figure_size(1280, 720);
  plt::named_plot("iter", x0_list, x1_list, "r--");
  plt::legend();
  plt::grid(true);
  plt::show();
}
int main(int argc, char const* argv[]) {
  
  test2();
  
  plot();
  return 0;
}
