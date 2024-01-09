#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


// pts1 = [sR|t] * pts2
void pose_estimation_3d3d(const std::vector<cv::Point3f>& pts2,
                          const std::vector<cv::Point3f>& pts1, float& scale, cv::Mat& R,
                          cv::Mat& t) {
    // estimate scale
    // TODO: scale计算可以取均值, 或者作为优化问题
    // cv::Point3f point1_sub = pts1[0] - pts1[1];
    // cv::Point3f point2_sub = pts2[0] - pts2[1];
    // float sum1 = point1_sub.x*point1_sub.x + point1_sub.y*point1_sub.y +
    // point1_sub.z*point1_sub.z; float sum2 = point2_sub.x*point2_sub.x +
    // point2_sub.y*point2_sub.y + point2_sub.z*point2_sub.z; scale =
    // sqrt(sum1/sum2); printf("scale_estimated:%f \n", scale);

    float sum_scale = 0;
    int sum_id = 0;
    for (int i = 0; i < pts2.size() - 1; i++) {
        for (int j = i + 1; j < pts2.size(); j++) {
            cv::Point3f point1_sub = pts1[i] - pts1[j];
            cv::Point3f point2_sub = pts2[i] - pts2[j];
            float sum1 = point1_sub.x * point1_sub.x + point1_sub.y * point1_sub.y +
                         point1_sub.z * point1_sub.z;
            float sum2 = point2_sub.x * point2_sub.x + point2_sub.y * point2_sub.y +
                         point2_sub.z * point2_sub.z;
            sum_scale += sqrt(sum1 / sum2);
            sum_id += 1;
            // printf("scale_estimated:%f \n", scale);
        }
    }
    scale = sum_scale / sum_id;

    std::vector<cv::Point3f> pts2_scaled(pts2.size());
    for (int32_t i = 0; i < pts2.size(); ++i) {
        pts2_scaled[i] = pts2[i] * scale;
    }

    cv::Point3f p1, p2;  // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2_scaled[i];
    }
    p1 = cv::Point3f(cv::Vec3f(p1) / N);
    p2 = cv::Point3f(cv::Vec3f(p2) / N);
    std::vector<cv::Point3f> q1(N), q2(N);  // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2_scaled[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) *
             Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    // cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0) {
        for (int x = 0; x < 3; ++x) {
            U(x, 2) *= -1;
        }
    }

    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = (cv::Mat_<float>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2), R_(1, 0), R_(1, 1), R_(1, 2),
         R_(2, 0), R_(2, 1), R_(2, 2));
    t = (cv::Mat_<float>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

int main(int argc, char** argv) {
    int cam_num = atoi(argv[1]);
    string xml_gt_path = argv[2]; // 真值（world）
    string xml_noise_colmap_path = argv[3]; // 优化值（colmap）
    string xml_noise_world_path = argv[4]; // 优化值（world），结果输出

    std::vector<cv::Point3f> pts1; // 世界点：真值的相机位置
    std::vector<cv::Point3f> pts2; // colmap：优化值的相机位置

    vector<Mat> vec_RT_noise, vec_K_noise;

    FileStorage fs_gt, fs_noise_colmap;
    for (int cam_id = 0; cam_id < cam_num; cam_id++) {
        boost::format fmt_gt(xml_gt_path);
        boost::format fmt_noise_colmap(xml_noise_colmap_path);
        string xml_gt = (fmt_gt % cam_id).str();
        string xml_noise_colmap = (fmt_noise_colmap % cam_id).str();

        // 读取真值数据
        fs_gt.open(xml_gt, FileStorage::READ);
        if (!fs_gt.isOpened()) {
            cerr << "did not find gt xml file!" << xml_gt << endl;
        }

        Mat RT_gt;
        read(fs_gt["RT"], RT_gt);
        fs_gt.release();
        RT_gt = RT_gt.inv(); // 取逆结果即为相机在世界系下的坐标
        pts1.push_back({RT_gt.at<float>(0, 3), RT_gt.at<float>(1, 3), RT_gt.at<float>(2, 3)});

        // 读取噪声值的 colmap
        fs_noise_colmap.open(xml_noise_colmap, FileStorage::READ);
        if (!fs_noise_colmap.isOpened()) {
            cerr << "did not find noise xml file!" << xml_noise_colmap << endl;
        }

        Mat RT_noise_colmap, K_noise_colmap;
        read(fs_noise_colmap["RT"], RT_noise_colmap);
        read(fs_noise_colmap["IntrinsicCam"], K_noise_colmap);
        vec_K_noise.push_back(K_noise_colmap);

        fs_noise_colmap.release();
        RT_noise_colmap = RT_noise_colmap.inv(); // 相机在 COLMAP 坐标系下的结果
        pts2.push_back({RT_noise_colmap.at<float>(0, 3), RT_noise_colmap.at<float>(1, 3),
                        RT_noise_colmap.at<float>(2, 3)});

        vec_RT_noise.push_back(RT_noise_colmap);
    }

    // 计算坐标系之间的 3D-3D 刚体变换 sRT
    float scale;
    Mat R;
    Mat t;
    Mat sRt_34;  // [sR|t]
    Mat sRt;     // [sR|t] 4x4
    Mat last_row = (Mat_<float>(1, 4) << 0, 0, 0, 1);
    // pts1 = [sR|t] * pts2
    pose_estimation_3d3d(pts2, pts1, scale, R, t);

    hconcat(scale * R, t, sRt_34);   // 3x4
    vconcat(sRt_34, last_row, sRt);  // 3x4

    cout << "sRt " << sRt << "scale: " << scale << endl;

    // 根据 sRt 将 COLMAP 坐标系下的 xml 转换到世界坐标系下
    for (int cam_id = 0; cam_id < cam_num; ++cam_id) {
        boost::format fmt_noise_world(xml_noise_world_path);
        string xml_noise_world = (fmt_noise_world % cam_id).str();
        FileStorage fs_noise_world(xml_noise_world, FileStorage::WRITE);
        if (!fs_noise_world.isOpened()) {
            cerr << "did not find " << xml_noise_world << endl;
            return -1;
        }

        // 读取噪声值的colmap
        boost::format fmt_noise_colmap(xml_noise_colmap_path);
        string xml_noise_colmap = (fmt_noise_colmap % cam_id).str();
        fs_noise_colmap.open(xml_noise_colmap, FileStorage::READ);
        if (!fs_noise_colmap.isOpened()) {
            cerr << "did not find cameraParams.xml file!" << xml_noise_colmap << endl;
        }

        Mat RT_noise_colmap, K_noise_colmap;
        read(fs_noise_colmap["RT"], RT_noise_colmap);

        Mat final_RT = RT_noise_colmap * sRt.inv() * scale;  // 4x4

        Mat pt2 = (Mat_<float>(4, 1) << pts2[cam_id].x, pts2[cam_id].y, pts2[cam_id].z, 1);
        Mat pt1 = sRt * pt2;

        Mat final_RT_test =
            (cv::Mat_<float>(3, 4) << final_RT.at<float>(0, 0), final_RT.at<float>(0, 1),
             final_RT.at<float>(0, 2), final_RT.at<float>(0, 3), final_RT.at<float>(1, 0),
             final_RT.at<float>(1, 1), final_RT.at<float>(1, 2), final_RT.at<float>(1, 3),
             final_RT.at<float>(2, 0), final_RT.at<float>(2, 1), final_RT.at<float>(2, 2),
             final_RT.at<float>(2, 3));  // 3x4

        Mat final_P_test = vec_K_noise[cam_id] * final_RT_test;
        Mat last_row = (Mat_<float>(1, 4) << 0.0f, 0.0f, 0.0f, 1.0f);
        Mat final_P;
        vconcat(final_P_test, last_row, final_P);  // 4x4
        Mat temp;
        vconcat(final_RT_test, last_row, temp);  // 4x4

        fs_noise_world << "P" << final_P;
        fs_noise_world << "RT" << final_RT_test;
        fs_noise_world << "R" << final_RT_test.colRange(0, 3).rowRange(0, 3);
        fs_noise_world << "t" << final_RT_test.col(3).rowRange(0, 3);
        fs_noise_world << "Intrinsics" << vec_K_noise[cam_id];
    }
}
