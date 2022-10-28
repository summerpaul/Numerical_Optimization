/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-25 14:28:18
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-25 15:01:38
 */
#include <iostream>

#include "cubic_spline_test.h"

CubicSplineTest::CubicSplineTest() : visualizer(nh_) {}

bool CubicSplineTest::init() {
  targetSub_ = nh_.subscribe("/move_base_simple/goal", 1,
                             &CubicSplineTest::targetCallBack, this,
                             ros::TransportHints().tcpNoDelay());
  return true;
}

void CubicSplineTest::targetCallBack(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {

  Eigen::Vector2d new_point(msg->pose.position.x, msg->pose.position.y);

  all_points_.push_back(new_point);
  if (all_points_.size() < 3) {
    return;
  }
  Eigen::Vector2d headP = all_points_.front();
  Eigen::Vector2d tailP = all_points_.back();
  Eigen::Matrix2Xd inPos;
  inPos.resize(2, all_points_.size() - 2);
  for (size_t i = 1; i < all_points_.size() - 1; i++) {
    inPos.col(i - 1) = all_points_[i];
  }

  cubic_spline::CubicSpline spline;
  spline.setConditions(headP, tailP, all_points_.size()-1);
  ROS_INFO("setConditions");

  spline.setInnerPoints(inPos);
  ROS_INFO("setInnerPoints");
  CubicCurve curve;
  spline.getCurve(curve);
  ROS_INFO("getCurve");
  visualizer.visualize(curve);
  double stretchEnergy{0};
  spline.getStretchEnergy(stretchEnergy);
  ROS_INFO("getStretchEnergy %f", stretchEnergy);
}