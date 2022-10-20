/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-17 08:57:59
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-20 18:19:25
 */
/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-17 08:57:59
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-17 09:02:08
 */
#include "gcopter/cubic_curve.hpp"
#include "gcopter/cubic_spline.hpp"
#include "gcopter/path_smoother.hpp"
#include "misc/visualizer.hpp"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

struct Config {
  std::string targetTopic;
  double penaltyWeight;//100
  Eigen::Matrix3Xd circleObs;
  double pieceLength;//0.5m
  double relCostTol;// 1e-6

  Config(const ros::NodeHandle &nh_priv) {
    std::vector<double> circleObsVec;

    nh_priv.getParam("TargetTopic", targetTopic);
    nh_priv.getParam("PenaltyWeight", penaltyWeight);
    nh_priv.getParam("CircleObs", circleObsVec);
    nh_priv.getParam("PieceLength", pieceLength);
    nh_priv.getParam("RelCostTol", relCostTol);

    circleObs = Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>>(
        circleObsVec.data(), 3, circleObsVec.size() / 3);
  }
};

// 曲线生成类
class CurveGen {
private:
  Config config;
  ros::NodeHandle nh;
  ros::Subscriber targetSub;

  Visualizer visualizer;
  std::vector<Eigen::Vector2d> startGoal;

  CubicCurve curve;

public:
  CurveGen(ros::NodeHandle &nh_)
      : config(ros::NodeHandle("~")), nh(nh_), visualizer(nh) {
    // 目标点的订阅
    targetSub = nh.subscribe(config.targetTopic, 1, &CurveGen::targetCallBack,
                             this, ros::TransportHints().tcpNoDelay());
  }

  // 障碍物的显示，只显示一次
  inline void vizObs() { visualizer.visualizeDisks(config.circleObs); }

  inline void plan() {
    // 在目标点有两个的时候进行计算法
    if (startGoal.size() == 2) {
      // 根据起点与终点距离，得到分段曲线的数量
      // 公用N段曲线，共N+1个节点， N-1个中间节点
      const int N =
          (startGoal.back() - startGoal.front()).norm() / config.pieceLength;
      // 根据线段长度进行差值
      // 使用2行n-1列的矩阵表示中间点
      Eigen::Matrix2Xd innerPoints(2, N - 1);
      for (int i = 0; i < N - 1; ++i) {
        innerPoints.col(i) =
            (startGoal.back() - startGoal.front()) * (i + 1.0) / N +
            startGoal.front();
      }

      // 轨迹平滑
      path_smoother::PathSmoother pathSmoother;
      // 初始化
      std::cout << "pathSmoother set up " << std::endl;
      // 使用路径平滑器
      pathSmoother.setup(startGoal.front(), startGoal.back(), N,
                         config.circleObs, config.penaltyWeight);
      CubicCurve curve;

      if (std::isinf(
              pathSmoother.optimize(curve, innerPoints, config.relCostTol))) {
                std::cout << "fail to optimize " << std::endl;
        return;
      }
      if (curve.getPieceNum() > 0) {
        visualizer.visualize(curve);
      }
    }
  }

  // 订阅目标点
  inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    if (startGoal.size() >= 2) {
      startGoal.clear();
    }
    // C++11中添加元素，效率更高，自己代码替换测试
    startGoal.emplace_back(msg->pose.position.x, msg->pose.position.y);

    plan();

    return;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "curve_gen_node");
  ros::NodeHandle nh_;

  CurveGen curveGen(nh_);

  ros::Duration(2.0).sleep();

  // 障碍物的显示
  curveGen.vizObs();

  ros::spin();

  return 0;
}
