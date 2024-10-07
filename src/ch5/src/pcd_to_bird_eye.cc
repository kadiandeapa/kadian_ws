#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

#define image_resolution 0.1  //俯视图分辨率
#define min_z 0.2            //俯视图最低高度
#define max_z 2.5             //俯视图最高高度

void GenerateBEVImage(PointCloudType::Ptr cloud);

void GenerateBEVImage(PointCloudType::Ptr cloud)
{
    auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) {return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) {return p1.y < p2.y; });

    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;

    const double inv_r = 1.0 / image_resolution;

    const int image_rows = int((max_y - min_y) * inv_r);
    const int image_cols = int((max_x - min_x) * inv_r);

    float x_center = 0.5 * (max_x + min_x);
    float y_center = 0.5 * (max_y + min_y);
    float x_center_image = image_cols / 2;
    float y_center_image = image_rows / 2;

    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& pt : cloud->points)
    {
        int x = int((pt.x - x_center) * inv_r + x_center_image);
        int y = int((pt.y - y_center) * inv_r + y_center_image);
        if (x < 0 || x >= image_cols || y < 0 || y >= image_rows || pt.z < min_z || pt.z > max_z) {
            continue;
        }

        image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
    }

    // cv::imshow("bev Image", image);
    // cv::waitKey(0);
    if (!cv::imwrite("/home/zy/kadian_ws/src/ch5/launch/bev.png", image)) 
    {
        ROS_ERROR("Failed to save the BEV image.");
    }
    else
    {
        ROS_INFO("Save the BEV image sucessfully.");
    }
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "ch5_pcd_to_bird_eye_node");
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

    GenerateBEVImage(cloud);

    ros::spinOnce();

    return 0;
}
