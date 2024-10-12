#ifndef SLAM_IN_AUTO_DRIVING_KDTREE_H
#define SLAM_IN_AUTO_DRIVING_KDTREE_H

#include "common/eigen_types.h"
#include "common/point_types.h"

#include <glog/logging.h>

namespace sad {

struct KdTreeNode
{
    int id_ = -1;
    int point_idx_ = 0;           // 点的索引
    int axis_index_ = 0;          // 分割轴
    float split_thresh_ = 0.0;    // 分割位置
    KdTreeNode* left_ = nullptr;  // 左子树
    KdTreeNode* right_ = nullptr; // 右子树

    bool IsLeaf() const { return left_ == nullptr && right_ == nullptr; }  // 是否为叶子
};

/**
 * 手写kd树
 * 测试这个kd树的召回!
 */
class KdTree {
    public:


    private:
        int k_ = 5;                                      // knn最近邻数量
        std::shared_ptr<KdTreeNode> root_ = nullptr;     // 根节点
        std::vector<Vec3f> cloud_;                       // 输入点云
        std::unordered_map<int, KdTreeNode*> nodes_;     // for bookkeeping

        size_t size_ = 0;       // 叶子节点数量
        int tree_node_id_ = 0;  // 为kdtree node 分配id

        // 近似最近邻
        bool approximate_ = true;
        float alpha_ = 0.1;

};


}
#endif
