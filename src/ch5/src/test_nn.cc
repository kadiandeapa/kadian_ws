#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gflags/gflags.h> //处理命令行参数解析
#include <glog/logging.h>  //日志记录
#include "bfnn.h"
#include "common/point_cloud_utils.h"
#include "common/point_types.h"
#include "common/sys_utils.h"
#include <pcl/io/pcd_io.h>

DEFINE_string(first_scan_path, "./data/ch5/first.pcd", "第一个点云路径");
DEFINE_string(second_scan_path, "./data/ch5/second.pcd", "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

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