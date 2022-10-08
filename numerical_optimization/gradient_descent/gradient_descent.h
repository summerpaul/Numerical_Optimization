/**
 * @Author: YunKai Xia
 * @Date:   2022-10-06 12:48:42
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-10-06 17:59:44
 */
#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <vector>
namespace numerical_optimization::gradient_descent {

struct SolveResult {
  Eigen::VectorXd res_x;
  Eigen::VectorXd res_gradient;
  size_t iter_time;
};

using CostFunction = std::function<double(
    const Eigen::VectorXd& x, Eigen::VectorXd& gradient, const bool& plot)>;

class GradientDecent {
 public:
  GradientDecent(const CostFunction& cost_function,
                 const Eigen::VectorXd& init_x, const double& init_tau = 0.01,
                 const double& c = 1e-4, const double& epsilon = 1e-7,
                 const size_t& max_iters = 1e5)
      : tau_(init_tau),
        c_(c),
        epsilon_(epsilon),
        max_iters_(max_iters),
        cost_function_(cost_function),
        x_(init_x) {
    int size = init_x.size();
    result_.res_x = Eigen::VectorXd::Zero(size);
    result_.res_gradient = Eigen::VectorXd::Zero(size);
    result_.iter_time = 0;
    std::cout << "epsilon is " << epsilon_ << std::endl;
    std::cout << "max iters is " << max_iters_ << std::endl;
  }
  SolveResult solve();

 private:
  double tau_ = 1.0;
  Eigen::VectorXd x_;
  CostFunction cost_function_;
  SolveResult result_;
  double c_ = 1e-4;
  double epsilon_ = 1e-6;
  size_t max_iters_ = 1e5;
};
}  // namespace numerical_optimization::gradient_descent