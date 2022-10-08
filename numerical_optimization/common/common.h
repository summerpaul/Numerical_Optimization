/**
 * @Author: YunKai Xia
 * @Date:   2022-10-06 12:34:00
 * @Last Modified by:   YunKai Xia
 * @Last Modified time: 2022-10-06 12:44:38
 */

#ifndef __COMMON_H__
#define __COMMON_H__
#include <time.h>
#include <sys/time.h>
namespace numerical_optimization::common {
// 获取时间戳
static double getTimeNow() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec / 1000000.0;
}

}  // namespace numerical_optimization::common

#endif /* __COMMON_H__ */
