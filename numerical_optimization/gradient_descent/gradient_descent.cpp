/**
 * @Author: YunKai Xia
 * @Date:   2022-10-06 12:57:19
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-10-06 18:01:32
 */
#include "gradient_descent.h"

#include <iostream>
using namespace std;

namespace numerical_optimization::gradient_descent {
SolveResult GradientDecent::solve() {
  // 全局的梯度
  Eigen::VectorXd grad = Eigen::VectorXd::Zero(x_.size());
  //   临时用的梯度，没啥用
  Eigen::VectorXd temp_grad = Eigen::VectorXd::Zero(x_.size());
  //   计算函数值与梯度值
  double val = cost_function_(x_, grad, false);
  double val2 = 0;
  while (grad.norm() > epsilon_) {
    Eigen::VectorXd d = -grad;
    val = cost_function_(x_, grad, false);
    val2 = cost_function_(x_ + tau_ * d, temp_grad, false);
    // Armijo condition
    while (val2 > val + c_ * tau_ * d.transpose() * grad) {
      tau_ *= 0.5;
      val2 = cost_function_(x_ + tau_ * d, temp_grad, false);
      std::cout << "meet  Armijo condition tau is " << tau_ << std::endl;
    }
    x_ += tau_ * d;
    cost_function_(x_, grad, true);
    // 超过迭代次数跳过
    if (result_.iter_time > max_iters_) {
      break;
    }
    result_.iter_time++;
  }
  //   返回结果
  std::cout << "last tau is " << tau_ << std::endl;
  result_.res_x = x_;
  result_.res_gradient = grad;

  return result_;
}

}  // namespace numerical_optimization::gradient_descent
