#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gflags/gflags.h> //处理命令行参数解析
#include <glog/logging.h>  //日志记录
#include "bfnn.h"
#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "common/sys_utils.h"
#include <pcl/io/pcd_io.h>
#include "gridnn.hpp"

DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

/**
 * 评测最近邻的正确性
 * @param truth 真值
 * @param esti  估计
 */
void EvaluateMatches(const std::vector<std::pair<size_t, size_t>>& truth,
                     const std::vector<std::pair<size_t, size_t>>& esti) {
    int fp = 0;  // false-positive，esti存在但truth中不存在
    int fn = 0;  // false-negative, truth存在但esti不存在

    LOG(INFO) << "truth: " << truth.size() << ", esti: " << esti.size();

    /// 检查某个匹配在另一个容器中存不存在
    auto exist = [](const std::pair<size_t, size_t>& data, const std::vector<std::pair<size_t, size_t>>& vec) -> bool {
        return std::find(vec.begin(), vec.end(), data) != vec.end();
    };

    int effective_esti = 0;
    for (const auto& d : esti) {
        if (d.first != sad::math::kINVALID_ID && d.second != sad::math::kINVALID_ID) {
            effective_esti++;

            if (!exist(d, truth)) {
                fp++;
            }
        }
    }

    for (const auto& d : truth) {
        if (!exist(d, esti)) {
            fn++;
        }
    }

    float precision = 1.0 - float(fp) / effective_esti;
    float recall = 1.0 - float(fn) / truth.size();
    LOG(INFO) << "precision: " << precision << ", recall: " << recall << ", fp: " << fp << ", fn: " << fn;
}


TEST(CH5_TEST, BFNN)
{
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);

    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty())
    {
        LOG(ERROR) << "CANNOT LOAD CLOUD";
        FAIL();
    }
    else
    {
        LOG(INFO) << "load cloud sucessfully"; 
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first, 0.5);
    sad::VoxelGrid(second, 0.5);

    LOG(INFO) << "points: " << first->size() << ", " << second->size() << "\n";

    sad::evaluate_and_call(
        //　lambda表达式
        // [capture](parameters) -> return_type {
        //     // function body
        // };
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            sad::bfnn_cloud(first, second, matches);
        },
        "暴力匹配（单线程）", 5
    );

    sad::evaluate_and_call(
        [&first, &second]() {
            std::vector<std::pair<size_t, size_t>> matches;
            sad::bfnn_cloud_mt(first, second, matches);
        },
        "暴力匹配（多线程）", 5
    ); 

    SUCCEED();
}

TEST(CH5_TEST, GRID_NN) {
    sad::CloudPtr first(new sad::PointCloudType), second(new sad::PointCloudType);
    pcl::io::loadPCDFile(FLAGS_first_scan_path, *first);
    pcl::io::loadPCDFile(FLAGS_second_scan_path, *second);

    if (first->empty() || second->empty()) {
        LOG(ERROR) << "cannot load cloud";
        FAIL();
    }

    // voxel grid 至 0.05
    sad::VoxelGrid(first, 0.5);
    sad::VoxelGrid(second, 0.5);

    LOG(INFO) << "points: " << first->size() << ", " << second->size();

    std::vector<std::pair<size_t, size_t>> truth_matches;
    sad::bfnn_cloud(first, second, truth_matches);

    // 对比不同种类的grid
    sad::GridNN<2> grid0(0.1, sad::GridNN<2>::NearbyType::CENTER), grid4(0.1, sad::GridNN<2>::NearbyType::NEARBY4),
        grid8(0.1, sad::GridNN<2>::NearbyType::NEARBY8);
    sad::GridNN<3> grid3(0.1, sad::GridNN<3>::NearbyType::NEARBY6);

    grid0.SetPointCloud(first);
    grid4.SetPointCloud(first);
    grid8.SetPointCloud(first);
    grid3.SetPointCloud(first);

    // 评价各种版本的Grid NN
    // sorry没有C17的template lambda... 下面必须写的啰嗦一些
    LOG(INFO) << "===================";
    std::vector<std::pair<size_t, size_t>> matches;
    sad::evaluate_and_call(
        [&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloud(first, second, matches); },
        "Grid0 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid0, &matches]() { grid0.GetClosestPointForCloudMT(first, second, matches); },
        "Grid0 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloud(first, second, matches); },
        "Grid4 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid4, &matches]() { grid4.GetClosestPointForCloudMT(first, second, matches); },
        "Grid4 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloud(first, second, matches); },
        "Grid8 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid8, &matches]() { grid8.GetClosestPointForCloudMT(first, second, matches); },
        "Grid8 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloud(first, second, matches); },
        "Grid 3D 单线程", 10);
    EvaluateMatches(truth_matches, matches);

    LOG(INFO) << "===================";
    sad::evaluate_and_call(
        [&first, &second, &grid3, &matches]() { grid3.GetClosestPointForCloudMT(first, second, matches); },
        "Grid 3D 多线程", 10);
    EvaluateMatches(truth_matches, matches);

    SUCCEED();
}

int main(int argc, char** argv)
{
    //初始化ros
    ros::init(argc, argv, "ch5_test_nn_node");  //将节点名称设为test_nn_node
    ros::NodeHandle private_nh("~");

    // 从参数服务器中获取参数并存储在普通字符串变量中
    std::string param_first_scan_path;
    std::string param_second_scan_path;

    private_nh.param<std::string>("first_pcd_path", param_first_scan_path, "first.pcd");
    private_nh.param<std::string>("second_pcd_path", param_second_scan_path, "second.pcd");

    // 如果从参数服务器中获取的参数和 gflags 的参数不同，则进行更新
    if (param_first_scan_path != FLAGS_first_scan_path) {
        FLAGS_first_scan_path = param_first_scan_path;
        ROS_INFO("Updated first_scan_path to: %s", FLAGS_first_scan_path.c_str());
    }
    if (param_second_scan_path != FLAGS_second_scan_path) {
        FLAGS_second_scan_path = param_second_scan_path;
        ROS_INFO("Updated first_scan_path to: %s", FLAGS_second_scan_path.c_str());
    }

    // 初始化 Google Logging 库
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    // 初始化 Google Test 框架
    testing::InitGoogleTest(&argc, argv);
    google::ParseCommandLineFlags(&argc, &argv, true);

    // 运行所有测试用例
    return RUN_ALL_TESTS();
    
}