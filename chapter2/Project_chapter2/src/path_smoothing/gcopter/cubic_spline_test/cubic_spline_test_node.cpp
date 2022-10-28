/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-25 14:28:27
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-25 14:52:10
 */
#include <iostream>
#include "cubic_spline_test.h"
using namespace std;


int main(int argc, char  **argv)
{
    ros::init(argc, argv, "cubic_spline_test");
    CubicSplineTest demo;
    if(!demo.init()){
        return 1;
    }
    ros::spin();
    return 0;
}
