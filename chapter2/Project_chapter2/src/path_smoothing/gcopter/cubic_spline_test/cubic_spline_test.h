/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-25 14:28:13
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-25 14:49:39
 */
#include <stdint.h>

#ifndef __CUBIC_SPLINE_TEST_H__
#define __CUBIC_SPLINE_TEST_H__

#include "gcopter/cubic_spline.hpp"
#include "misc/visualizer.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
class CubicSplineTest {

public:
  CubicSplineTest();
  bool init();

private:
  void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber targetSub_;
  std::vector<Eigen::Vector2d> all_points_;
  Visualizer visualizer;
};

#endif /* __CUBIC_SPLINE_TEST_H__ */
