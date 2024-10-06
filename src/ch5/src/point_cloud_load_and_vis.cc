#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "ch5_pcl_cloud_load_and_vis_node");
    ros::NodeHandle pricate_nh("~");  // 使用私有命名空间的 NodeHandle

    // 定义一个用于存储点云文件路径的字符串，并从参数服务器获取
    std::string pcd_path;
    pricate_nh.param<std::string>("pcd_path", pcd_path, "./data/ch5/map_example.pcd");  // 从参数服务器读取 pcd_path，默认为 "./data/ch5/map_example.pcd"

    // 检查文件路径是否为空
    if (pcd_path.empty()) {
        ROS_ERROR("The PCD file path is empty.");
        return -1;
    }

    // 读取点云文件
    PointCloudType::Ptr cloud(new PointCloudType);
    if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
        ROS_ERROR("Cannot load the PCD file: %s", pcd_path.c_str());
        return -1;
    }

    // 检查点云是否为空
    if (cloud->empty()) {
        ROS_ERROR("Loaded cloud is empty.");
        return -1;
    }

    ROS_INFO("Loaded cloud points: %zu", cloud->size());

    // 创建 PCL 可视化窗口
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> handle(cloud, "z");  // 使用高度（z 轴）来着色
    viewer.addPointCloud<PointType>(cloud, handle);
    viewer.setBackgroundColor(0.05, 0.05, 0.05);  // 设置背景颜色为深灰色

    // ROS 运行循环
    while (!viewer.wasStopped() && ros::ok()) {
        viewer.spinOnce();
        ros::spinOnce();
    }

    return 0;
}
