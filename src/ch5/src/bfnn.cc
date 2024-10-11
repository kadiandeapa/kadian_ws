#include "bfnn.h"
#include <execution>

namespace sad {

int bfnn_point(CloudPtr cloud, const Vec3f& point) {
    return std::min_element(cloud->points.begin(), cloud->points.end(),
                            [&point](const PointType& pt1, const PointType& pt2) -> bool {
                                return (pt1.getVector3fMap() - point).squaredNorm() <
                                       (pt2.getVector3fMap() - point).squaredNorm();
                            }) -
           cloud->points.begin();
}
    /**
 * 对点云进行BF最近邻
 * @param cloud1  目标点云
 * @param cloud2  被查找点云
 * @param matches 两个点云内的匹配关系
 * @return
 */
void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches){
    //　单线程版本
    std::vector<size_t> index(cloud2->size());

    //std::for_each 是 C++ 标准库中的一个算法，用于对给定范围中的每个元素执行某个操作。
    //mutable 关键字表示允许修改捕获列表中的值（即可以修改 idx 的值）
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    matches.resize(index.size());
    
    //std::execution::seq（顺序执行）c++17
    std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto idx){
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
    
}

void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2, std::vector<std::pair<size_t, size_t>>& matches){
    // 先生成索引
    std::vector<size_t> index(cloud2->size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

    // 并行化for_each
    matches.resize(index.size());
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
    });
}

}