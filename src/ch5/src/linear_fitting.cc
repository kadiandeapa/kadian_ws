#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "common/eigen_types.h"
#include "common/math_utils.h"

DEFINE_int32(num_tested_points_plane, 10, "number of tested points in plane fitting");
DEFINE_int32(num_tested_points_line, 100, "number of tested points in line fitting");
DEFINE_double(noise_sigma, 0.01, "noise of generated samples");

void PlaneFittingTest();

int main(int argc, char** argv){
    //初始化ros
    ros::init(argc, argv, "ch5_linear_fitting_node");  //将节点名称设为test_nn_node
    ros::NodeHandle private_nh("~");

    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    LOG(INFO) << "testing plane fitting";
    PlaneFittingTest();

    return 0;

}

void PlaneFittingTest() {
    Vec4d true_plane_coeffs(0.1, 0.2, 0.3, 0.4);
    true_plane_coeffs.normalize();

    std::vector<Vec3d> points;

    // 随机生成仿真平面点
    cv::RNG rng;   //生成随机数的类
    for (int i = 0; i < FLAGS_num_tested_points_plane; ++i) {
        // 先生成一个随机点，计算第四维，增加噪声，再归一化
        Vec3d p(rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0));
        
        // 计算p第四维,n4代表一个缩放因子,用于确保把p正确投影到平面true_plane_coeffs上
        double n4 = -p.dot(true_plane_coeffs.head<3>()) / true_plane_coeffs[3];
        p = p / (n4 + std::numeric_limits<double>::min());  // 防止除零
        p += Vec3d(rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma), rng.gaussian(FLAGS_noise_sigma));

        points.emplace_back(p);

        // 验证在平面上
        LOG(INFO) << "res of p: " << p.dot(true_plane_coeffs.head<3>()) + true_plane_coeffs[3];
    }

    Vec4d estimated_plane_coeffs;
    if (sad::math::FitPlane(points, estimated_plane_coeffs)) {
        LOG(INFO) << "estimated coeffs: " << estimated_plane_coeffs.transpose()
                  << ", true: " << true_plane_coeffs.transpose();
    } else {
        LOG(INFO) << "plane fitting failed";
    }
}











