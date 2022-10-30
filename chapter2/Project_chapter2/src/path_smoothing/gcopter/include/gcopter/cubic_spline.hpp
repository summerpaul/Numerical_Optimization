#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include "cubic_curve.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <iostream>
#include <vector>
#define PRINT_MATRIX(x) std::cout << #x "=\n" << x << std::endl
namespace cubic_spline {

// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
// 带状稀疏矩阵求解
// 参考https://baike.baidu.com/item/%E5%B8%A6%E7%8A%B6%E7%9F%A9%E9%98%B5/1441572
// 带状矩阵即在矩阵A中，所有的非零元素都集中在以主对角线为中心的带状区域中。
// 对于n*n的方阵，若它的全部非零元素落在一个以主对角线为中心的带状区域中，这个带状区域包含主对角线，
// 以及主对角线下面及上面各b条对角线上的元素，那么称该方阵为半带宽为b的带状矩阵

class BandedSystem {
public:
  // The size of A, as well as the lower/upper
  // banded width p/q are needed
  inline void create(const int &n, const int &p, const int &q) {
    // In case of re-creating before destroying
    destroy();
    N = n;
    lowerBw = p;
    upperBw = q;
    // 实际元素的数量
    int actualSize = N * (lowerBw + upperBw + 1);
    ptrData = new double[actualSize];
    std::fill_n(ptrData, actualSize, 0.0);
    return;
  }

  inline void destroy() {
    if (ptrData != nullptr) {
      delete[] ptrData;
      ptrData = nullptr;
    }
    return;
  }

private:
  int N;
  int lowerBw;
  int upperBw;
  // Compulsory nullptr initialization here
  double *ptrData = nullptr;

public:
  // Reset the matrix to zero
  inline void reset(void) {
    std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
    return;
  }

  // The band matrix is stored as suggested in "Matrix Computation"
  inline const double &operator()(const int &i, const int &j) const {
    return ptrData[(i - j + upperBw) * N + j];
  }

  inline double &operator()(const int &i, const int &j) {
    return ptrData[(i - j + upperBw) * N + j];
  }

  // This function conducts banded LU factorization in place
  // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
  // LU分解
  inline void factorizeLU() {
    int iM, jM;
    double cVl;
    for (int k = 0; k <= N - 2; ++k) {
      iM = std::min(k + lowerBw, N - 1);
      cVl = operator()(k, k);
      for (int i = k + 1; i <= iM; ++i) {
        if (operator()(i, k) != 0.0) {
          operator()(i, k) /= cVl;
        }
      }
      jM = std::min(k + upperBw, N - 1);
      for (int j = k + 1; j <= jM; ++j) {
        cVl = operator()(k, j);
        if (cVl != 0.0) {
          for (int i = k + 1; i <= iM; ++i) {
            if (operator()(i, k) != 0.0) {
              operator()(i, j) -= operator()(i, k) * cVl;
            }
          }
        }
      }
    }
    return;
  }

  // This function solves Ax=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  template <typename EIGENMAT> inline void solve(EIGENMAT &b) const {
    int iM;
    for (int j = 0; j <= N - 1; ++j) {
      iM = std::min(j + lowerBw, N - 1);
      for (int i = j + 1; i <= iM; ++i) {
        if (operator()(i, j) != 0.0) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; --j) {
      b.row(j) /= operator()(j, j);
      iM = std::max(0, j - upperBw);
      for (int i = iM; i <= j - 1; ++i) {
        if (operator()(i, j) != 0.0) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
    return;
  }

  // This function solves ATx=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  template <typename EIGENMAT> inline void solveAdj(EIGENMAT &b) const {
    int iM;
    for (int j = 0; j <= N - 1; ++j) {
      b.row(j) /= operator()(j, j);
      iM = std::min(j + upperBw, N - 1);
      for (int i = j + 1; i <= iM; ++i) {
        if (operator()(j, i) != 0.0) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; --j) {
      iM = std::max(0, j - lowerBw);
      for (int i = iM; i <= j - 1; ++i) {
        if (operator()(j, i) != 0.0) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
  }
};

class CubicSpline {
public:
  CubicSpline() = default;
  ~CubicSpline() { A.destroy(); }

private:
  int N;
  Eigen::Vector2d headP;
  Eigen::Vector2d tailP;
  BandedSystem A;
  Eigen::MatrixX2d b;

  Eigen::MatrixXd partial_D;
  Eigen::MatrixXd partial_diff_x;

public:
  // 设置边界条件,起点与终点 + 曲线数量
  inline void setConditions(const Eigen::Vector2d &headPos,
                            const Eigen::Vector2d &tailPos,
                            const int &pieceNum) {
    headP = headPos;
    tailP = tailPos;
    N = pieceNum;
    // TODO
    // 确定各参数的数量
    // 所有点的数量N+1
    // 曲线数量N
    // 中间节点数量N-1
    // 求解的多项式矩阵维度 (N-1)*(N-1)
    // b矩阵的维度N-1
    // D的数量N+1

    // 设置B矩阵的数量与维度
    b.resize(N - 1, 2);

    // 设置A矩阵的维度与带状矩阵，上下带宽为1
    A.create(N - 1, 1, 1);

    Eigen::MatrixXd A_tmp;
    A_tmp.resize(N - 1, N - 1);
    Eigen::MatrixXd partial_B;
    partial_B.resize(N - 1, N - 1);
    A_tmp.setZero();
    partial_B.setZero();
    for (int i = 0; i < N - 1; ++i) {
      if (i == 0) {
        A_tmp(0, 0) = 4;
        A_tmp(0, 1) = 1;
        partial_B(0, 1) = 3;
      } else if (i == N - 2) {
        A_tmp(N - 2, N - 3) = 1;
        A_tmp(N - 2, N - 2) = 4;
        partial_B(N - 2, N - 3) = -3;
      } else {
        A_tmp(i, i - 1) = 1;
        A_tmp(i, i) = 4;
        A_tmp(i, i + 1) = 1;
        partial_B(i, i - 1) = -3;
        partial_B(i, i + 1) = 3;
      }
    }
    Eigen::MatrixXd A_inv = A_tmp.inverse();
    // dD/dx = A^(-1)*dB/dx
    partial_D = A_inv * partial_B;
    // PRINT_MATRIX(partial_D);

    // d(x(i)- x(i+1))/dx
    partial_diff_x.resize(N, N - 1);
    partial_diff_x.setZero();
    partial_diff_x(0, 0) = -1;
    partial_diff_x(N - 1, N - 2) = 1;
    for (int i = 1; i < N - 1; ++i) {
      partial_diff_x(i, i) = -1;
      partial_diff_x(i, i - 1) = 1;
    }

    return;
  }
  // 三次样条曲线，中间节点插值,2 * (N-1) 的矩阵
  inline void setInnerPoints(const Eigen::Ref<const Eigen::Matrix2Xd> &inPs) {

    // TODO

    // 带状矩阵A的赋值
    // 矩阵的维度是 n-1维
    // 中间节点的存储形式的2 * (N-1)的矩阵
    // b矩阵的存储形式是(N-1) *2 的矩阵

    // 同时给A矩阵与b矩阵赋值

    // x(0) = headP
    // x(N) = tailP
    // x(i+1) = inPs(i) (0<i<N)
    Eigen::Matrix2Xd X; //将起点，中间点，进行整合
    X.resize(2, N + 1);
    X.col(0) = headP;
    X.col(N) = tailP;
    for (int i = 1; i < N; i++) {
      X.col(i) = inPs.col(i - 1);
    }
    // 补充A矩阵
    for (int i = 0; i < N - 1; ++i) {
      A(i, i) = 4;  // A矩阵每个对象线都是4
      if (i == 0) { // 第一行数据
        A(i, i + 1) = 1;
      } else if (i == N - 2) { //最后一行数据
        A(i, i - 1) = 1;
      } else { //中间的矩阵块
        A(i, i - 1) = 1;
        A(i, i + 1) = 1;
      }
      // b(i) = 3 *(x(i+2) - x(i))
      b.row(i) = 3 * (X.col(i + 2) - X.col(i)).transpose();
    }

    A.factorizeLU(); // A矩阵进行LU分解
    A.solve(b);      //求解D
    // std::cout << "solve b " << std::endl;
    // 存储D
    Eigen::MatrixX2d D = Eigen::MatrixX2d::Zero(N + 1, 2);
    for (int i = 1; i < N; i++) {
      D.row(i) = b.row(i - 1);
    }
    // 修改b,用于存储系数
    b.resize(N * 4, 2);

    Eigen::Matrix<double, 2, 4> coeffMat; //单个系数矩阵

    for (int i = 0; i < N; ++i) {
      // a(i) = x(i)
      coeffMat.col(3) = X.col(i);
      // b(i) = D(i)
      coeffMat.col(2) = D.row(i).transpose();
      // c(i) = 3(x(i+1) - x(i)) - 2D(i) - D(i+1)
      coeffMat.col(1) = 3 * (X.col(i + 1) - X.col(i)) -
                        2 * D.row(i).transpose() - D.row(i + 1).transpose();
      // d(i) = 2(x(i) - x(i+1)) + D(i) + D(i+1)
      coeffMat.col(0) = 2 * (X.col(i) - X.col(i + 1)) + D.row(i).transpose() +
                        D.row(i + 1).transpose();
      b.block(i * 4, 0, 4, 2) = coeffMat.transpose();
    }
    // std::cout << "calc coeffMat " << std::endl;

    return;
  }

  inline void getCurve(CubicCurve &curve) const {
    // TODO
    curve.clear();
    for (int i = 0; i < N; ++i) {
      curve.emplace_back(
          CubicPolynomial(1.0, b.block(i * 4, 0, 4, 2).transpose()));
    }
    return;
  }

  inline void getStretchEnergy(double &energy) const {
    // TODO
    // p(s) = ai + bi*s + ci * s^2 + di * s^3
    // p''(s) = 2 * ci  +  6 * di * s
    // (p''(s))^2 = 4 * ci ^2 + 36 * di ^2 * s ^ 2 + 24 * ci * di * s

    // 定积分的值 = 4 * ci ^2 + 12 * di ^2 + 12 * * ci * di
    energy = 0;
    Eigen::Vector2d c, d;
    Eigen::Matrix<double, 2, 4> coeffMat;
    for (int i = 0; i < N; i++) {
      coeffMat = b.block(i * 4, 0, 4, 2).transpose();
      d = coeffMat.col(0);
      c = coeffMat.col(1);
      energy +=
          4.0 * c.squaredNorm() + 12.0 * d.squaredNorm() + 12.0 * d.dot(c);
      // std::cout << "d is " << d << " c is " << c << std::endl;
    }

    return;
  }

  // 获取梯度，梯度信息是每个节点的
  inline void getGrad(Eigen::Ref<Eigen::Matrix2Xd> gradByPoints) const {
    // TODO
    // grad = 8 * ci * ci' + 24 * di * di' + 12 * di * ci' + 12 * di' * ci
    // grad = (8 * ci + 12 * di) * ci' + (24 * di + 12 * ci)* di'
    // c(i) = 3(x(i+1) - x(i)) - 2D(i) - D(i+1)
    // dc(i)/dx = 3d(d(x(i+1) - x(i)))/dx -2dD(i)/dx - dD(i-1)/dx
    // d(i) = 2(x(i) - x(i+1)) + D(i) + D(i+1)
    // dd(i)/dx = 2 d(x(i) - x(i-1))/dx + dD(i)/dx + dD(i+1)/dx
    Eigen::MatrixXd partial_c = Eigen::MatrixXd::Zero(N, N - 1);
    Eigen::MatrixXd partial_d = Eigen::MatrixXd::Zero(N, N - 1);
    //
    partial_c.row(0) = -3 * partial_diff_x.row(0) - partial_D.row(0);
    partial_d.row(0) = 2 * partial_diff_x.row(0) + partial_D.row(0);
    for (int i = 1; i < N - 1; ++i) {
      partial_c.row(i) = -3 * partial_diff_x.row(i) - 2 * partial_D.row(i - 1) -
                         partial_D.row(i);
      partial_d.row(i) =
          2 * partial_diff_x.row(i) + partial_D.row(i - 1) + partial_D.row(i);
    }
    partial_c.row(N - 1) =
        -3 * partial_diff_x.row(N - 1) - 2 * partial_D.row(N - 2);
    partial_d.row(N - 1) = 2 * partial_diff_x.row(N - 1) + partial_D.row(N - 2);

    gradByPoints.setZero();
    Eigen::Vector2d c, d;
    Eigen::Matrix<double, 2, 4> coeffMat;

    for (int i = 0; i < N; ++i) {
      coeffMat = b.block(i * 4, 0, 4, 2).transpose();
      d = coeffMat.col(0);
      c = coeffMat.col(1);
      gradByPoints += (24 * d + 12 * c) * partial_d.row(i) +
                      (12 * d + 8 * c) * partial_c.row(i);
    }
  }
};
} // namespace cubic_spline

#endif
