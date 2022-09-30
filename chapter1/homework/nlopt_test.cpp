/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-30 13:51:24
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-30 14:10:00
 */
#include <iostream>

using namespace std;
#include <cmath>
#include <nlopt.hpp>

// 目标函数
double myfunc(const std::vector<double> &x, std::vector<double> &grad,
              void *func_data) {
  double val = 0;
  for (int i = 1; i < x.size() * 0.5 + 1e-6; i += 2) {
    val += 100 * pow(pow(x[2 * i - 1], 2) - x[2 * i], 2) +
           pow(x[2 * i - 1] - 1, 2);
  }

  return val;
}

int main(int argc, char const *argv[]) {
  /* code */
  return 0;
}
