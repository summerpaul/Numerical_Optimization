/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-21 19:38:45
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-26 11:15:35
 */
#include <iostream>

#include "sdqp/sdqp.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  int m = 7;
  Eigen::Matrix<double, 3, 3> Q;
  Eigen::Matrix<double, 3, 1> c;
  Eigen::Matrix<double, 3, 1> x;        // decision variables
  Eigen::Matrix<double, -1, 3> A(m, 3); // constraint matrix
  Eigen::VectorXd b(m);                 // constraint bound

  /**
   * Q = [2.0, 1.0, 1.0]
   *     [1.0, 2.0, 1.0]
   *     [1.0, 1.0, 2.0]
   */
  Q << 2.0, 1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 2.0;
  /**
   * C = [1.2]
   *     [2.5]
   *     [-10.0]
   */
  c << 1.2, 2.5, -10.0;
  /**
   * A = [1.0, 0.0, 0.0]
   *     [1.0, 0.0, 0.0]
   *     [0.0, 0.0, 1.0]
   *     [-0.7, 0.5, 0.0]
   *     [0.5, -1.0, 0.0]
   *     [0.0, 0.13, -1.0]
   *     [0.1, -3.0, -1.3]
   *
   */

  A << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -0.7, 0.5, 0.0, 0.5, -1.0,
      0.0, 0.0, 0.13, -1.0, 0.1, -3.0, -1.3;
  /**
   * b = [10.0]
   *     [10.0]
   *     [10.0]
   *     [1.7]
   *     [-7.1]
   *     [-3.31]
   *     [2.59]
   *
   */
  b << 10.0, 10.0, 10.0, 1.7, -7.1, -3.31, 2.59;

  /**
   * minimize     0.5 x' Q x + c' x
   * subject to       A x <= b
   * Q must be positive definite
   **/

  double minobj = sdqp::sdqp<3>(Q, c, A, b, x);

  std::cout << "optimal sol: " << x.transpose() << std::endl;
  std::cout << "optimal obj: " << minobj << std::endl;
  std::cout << "cons precision: " << (A * x - b).maxCoeff() << std::endl;

  return 0;
}
