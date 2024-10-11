#ifndef SLAM_IN_AUTO_DRIVING_GRID2D_HPP
#define SLAM_IN_AUTO_DRIVING_GRID2D_HPP

#include "common/eigen_types.h"
#include "common/point_types.h"
#include <glog/logging.h>
#include <execution>

namespace sad
{
/**
 * @brief 栅格法最近邻
 * @tparam dim 模板参数，使用2D或3D栅格
 * 
 */
template <int dim>
class GridNN {
    public:
        using KeyType = Eigen::Matrix<int, dim, 1>;
        using PtType = Eigen::Matrix<float, dim, 1>;

        enum class NearbyType {
            CENTER,  // 只考虑中心

            //for 2D
            NEARBY4,  // 上下左右
            NEARBY8,  // 上下左右+四角

            //for 3D
            NEARBY6,  // 上下左右前后

        };

        /**
        * 构造函数
        * @param resolution 分辨率
        * @param nearby_type 近邻判定方法
        */
        // explicit 防止隐式构造(可能产生隐式转换)，要求只能通过显示构造函数构造对象
        explicit GridNN(float resolution = 0.1, NearbyType nearby_type = NearbyType::NEARBY4)
            : resolution_(resolution), nearby_type_(nearby_type){
            inv_resolution_ = 1.0 / resolution_;

            //check dim and nearby
            if (dim == 2 && nearby_type_ == NearbyType::NEARBY6) {
                LOG(INFO) << "2D grid does not support nearby6, using nearby4 instead.";
                nearby_type_ = NearbyType::NEARBY4;
            } else if (dim == 3 && (nearby_type_ != NearbyType::NEARBY6 && nearby_type_ != NearbyType::CENTER)) {
                LOG(INFO) << "3D grid does not support nearby4/8, using nearby6 instead.";
                nearby_type_ = NearbyType::NEARBY6;
            }

            GenerateNearbyGrids();
        }

        // 获取最近邻
        bool GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx);

    
    private:
        float resolution_ = 0.1;    //分辨率
        float inv_resolution = 10;  //分辨率倒数

        NearbyType nearby_type_ = NearbyType::NEARBY4;

        std::unordered_map<KeyType, std::vector<size_t>, hash_vec<dim>> grids_; //栅格数据
        CloudPtr cloud_;

        std::vector<KeyType> nearby_grids_; //附近的栅格

        //　根据最近邻的类型，生成附近网络
        void GenerateNearbyGrids();

        /// 空间坐标转到grid
        KeyType Pos2Grid(const PtType& pt);

};

template <int dim>
Eigen::Matrix<int, dim, 1> GridNN::Pos2Grid(const Eigen::Matrix<float, dim, 1>& pt){
    return pt.array().template round().template cast<int>();
    // Eigen::Matrix<int, dim, 1> ret;
    // for (int i = 0; i < dim; ++i) {
    //     ret(i, 0) = round(pt[i] * inv_resolution_);
    // }
    // return ret;
}

template <>
void GridNN<2>::GenerateNearbyGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY4) {
        nearby_grids_ = {Vec2i(0, 0), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1)};
    } else if (nearby_type_ == NearbyType::NEARBY8) {
        nearby_grids_ = {
            Vec2i(0, 0), Vec2i(-1, 0), Vec2i(1, 0), Vec2i(0, 1), Vec2i(0, -1),
            Vec2i(-1, -1), Vec2i(-1, 1), Vec2i(1, -1), Vec2i(1, 1)
        };
    }
}

template <>
void GridNN<3>::GenerateNearbyGrids() {
    if (nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    }
}

template <int dim>
bool GetClosestPoint(const PointType& pt, PointType& closest_pt, size_t& idx){
    // 在pt栅格周边寻找最近邻
    std::vector<size_t> idx_to_check;
    auto key = Pos2Grid()
}
    
} // namespace sad




#endif