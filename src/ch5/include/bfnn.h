#ifndef __BFNN_H__
#define __BFNN_H__

#include "common/eigen_types.h"
#include "common/point_types.h"

#include <thread>

namespace sad
{
/**
 * Brute-force Nearest Neighbour
 * @param cloud 点云
 * @param point 待查找点
 * @return 找到的最近点索引
 */
int bfnn_point(CloudPtr cloud, const Vec3f& point);

/**
 * 对点云进行BF最近邻 多线程版本
 * @param cloud1
 * @param cloud2
 * @param matches
 */
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches);
    
} // namespace sad


#endif