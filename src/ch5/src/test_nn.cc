#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gflags/gflags.h> //处理命令行参数解析
#include <glog/logging.h>  //日志记录
#include "bfnn.h"

int main(int argc, char **argv)
{
    //初始化ros
    ros::init(argc, argv, "ch5_test_nn_node");  //将节点名称设为test_nn_node
    ros::NodeHandle nh;

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