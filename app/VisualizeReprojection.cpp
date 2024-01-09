#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

vector<Mat> P;
vector<Mat> P_gt;
vector<Mat> rvec;
vector<Mat> tvec;
vector<Mat> intrinsics;
vector<Mat> distCoeffs;
vector<Mat> extrinsics;
vector<vector<Point2d>> picture_point_set;

Vec3f rotationMatrixToEulerAngles(Mat& R) {
    float sy =
        sqrtf(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;  // If
    float x, y, z;
    if (!singular) {
        x = atan2f(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = atan2f(R.at<double>(1, 0), R.at<double>(0, 0));
    } else {
        x = atan2f(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = 0;
    }
#if 1
    x = x * 180.0f / 3.141592653589793f;
    y = y * 180.0f / 3.141592653589793f;
    z = z * 180.0f / 3.141592653589793f;
#endif
    return Vec3f(x, y, z);
}

void drawCross(Mat& img, Point point, Scalar color, int size, int thickness) {
    //绘制横线
    line(img, Point(point.x - size / 2, point.y - size / 2),
         Point(point.x + size / 2, point.y + size / 2), color, thickness, 8, 0);
    //绘制竖线
    line(img, Point(point.x + size / 2, point.y - size / 2),
         Point(point.x - size / 2, point.y + size / 2), color, thickness, 8, 0);
}

int main(int argc, char** argv) {
    int CAMERA_NUMBER = atoi(argv[1]);
    string xml_gt = argv[2];
    string xml_noise = argv[3];
    string res_path = argv[4];

    picture_point_set.resize(CAMERA_NUMBER);
    rvec.resize(CAMERA_NUMBER);
    tvec.resize(CAMERA_NUMBER);
    P_gt.resize(CAMERA_NUMBER);

    vector<Mat> P_gt_all, P_noise_all;
    vector<Mat> R_gt_all, T_gt_all;
    vector<Mat> R_noise_all, T_noise_all;
    vector<Mat> intrinsics_gt_all, intrinsics_noise_all;

    for (int camera_id = 0; camera_id < CAMERA_NUMBER; camera_id++) {
        boost::format fmt_gt(xml_gt);
        string xml_gt_path = (fmt_gt % camera_id).str();
        boost::format fmt_noise(xml_noise);
        string xml_noise_path = (fmt_noise % camera_id).str();

        FileStorage fs_gt(xml_gt_path, FileStorage::READ);
        FileStorage fs_noise(xml_noise_path, FileStorage::READ);

        Mat P_gt, P_noise;

        fs_noise["P"] >> P_noise;
        P_noise = P_noise.rowRange(0, 3).clone();

        P_noise.convertTo(P_noise, CV_64F);
        P_noise_all.emplace_back(P_noise);

        fs_gt["P"] >> P_gt;
        P_gt.convertTo(P_gt, CV_64F);
        P_gt = P_gt.rowRange(0, 3).clone();
        P_gt_all.emplace_back(P_gt);
    }

    Mat sum_tvec = Mat::zeros(3, 1, CV_64F);
    Vec3f sum_euler;

    ofstream fs_euler_tvec(res_path + "/euler_tvec_error.txt");
    double sumx1, sumy1, sumz1 = 0.0;
    // 求 gt 和 noise 之间的差距
    for (int camera_id = 0; camera_id < CAMERA_NUMBER; camera_id++) {
        Mat tvec_error = (cv::Mat_<double>(3, 1) << 0, 0, 0);

        Mat cameraMatrix, cameraMatrix_noise; // 内参
        Mat rotMatrix, rotMatrix_noise; // 旋转
        Mat tvec, tvec_noise, rotMat_inv; // 平移
        cv::decomposeProjectionMatrix(P_noise_all[camera_id], cameraMatrix, rotMatrix, tvec);
        cameraMatrix = cameraMatrix / cameraMatrix.ptr<double>(2)[2];  //内参最后一个元素为1

        Mat rvec;
        Rodrigues(rotMatrix, rvec); // 罗德里格斯变换
        rotMat_inv = rotMatrix.inv();
        Vec3f euler_noise = rotationMatrixToEulerAngles(rotMat_inv); // 转为欧拉角

        Mat tvec_31_noise =
            (cv::Mat_<double>(3, 1) << tvec.ptr<double>(0)[0] / tvec.ptr<double>(3)[0],
             tvec.ptr<double>(1)[0] / tvec.ptr<double>(3)[0],
             tvec.ptr<double>(2)[0] / tvec.ptr<double>(3)[0]);
        tvec_31_noise.convertTo(tvec_31_noise, CV_64F);

        cv::decomposeProjectionMatrix(P_gt_all[camera_id], cameraMatrix, rotMatrix, tvec);
        cameraMatrix = cameraMatrix / cameraMatrix.ptr<double>(2)[2];  //内参最后一个元素为1

        Rodrigues(rotMatrix, rvec);
        rotMat_inv = rotMatrix.inv();
        Vec3f euler_gt = rotationMatrixToEulerAngles(rotMat_inv);

        Mat tvec_31_gt = (cv::Mat_<double>(3, 1) << tvec.ptr<double>(0)[0] / tvec.ptr<double>(3)[0],
                          tvec.ptr<double>(1)[0] / tvec.ptr<double>(3)[0],
                          tvec.ptr<double>(2)[0] / tvec.ptr<double>(3)[0]);
        tvec_31_gt.convertTo(tvec_31_gt, CV_64F);

        // 一范数
        sumx1 += abs(euler_gt[0] - euler_noise[0]);
        sumy1 += abs(euler_gt[1] - euler_noise[1]);
        sumz1 += abs(euler_gt[2] - euler_noise[2]);

        sum_tvec += abs(tvec_31_gt - tvec_31_noise);
    }

    fs_euler_tvec << "euler error: [" << sumx1 / (double)CAMERA_NUMBER << ", "
                  << sumy1 / (double)CAMERA_NUMBER << ", " << sumz1 / (double)CAMERA_NUMBER << "]"
                  << endl;
    fs_euler_tvec << "tvec error: " << sum_tvec / (double)CAMERA_NUMBER << endl;

    // 反投影结果展示
    Mat img = imread("./input/image/0000.jpg");
    Mat drawImg = img.clone();

    ofstream fs;
    fs.open(res_path + "/error.txt", ios::out);
    float sum = 0;
    int loopCount = 0;

    // 选取指定范围内、固定间隔的空间点，利用 gt 和 noise 进行反投影
    for (int x = -22000; x < 22000; x += 1000) {
        for (int y = -22000; y < 22000; y += 1000) {
            for (int z = 0; z < 2500; z += 500) {
                int id = 0;
                float sum_id = 0;
                for (int camera_id = 0; camera_id < CAMERA_NUMBER; camera_id++) {
                    vector<Point2d> points_2d;
                    vector<Point3d> points_3d;
                    vector<Point3d> points_3d_BA;
                    // double z = 0;
                    Mat point3dmat = (Mat_<double>(4, 1) << x, y, z, 1);
                    Mat point2d_noise = P_noise_all[camera_id] * point3dmat;

                    Point2d pointdraw =
                        Point2d(point2d_noise.at<double>(0) / point2d_noise.at<double>(2),
                                point2d_noise.at<double>(1) / point2d_noise.at<double>(2));

                    // 最后一个判断条件是深度值，这个和半径相关
                    if (pointdraw.x >= 1920 || pointdraw.x < 0 || pointdraw.y >= 1080 ||
                        pointdraw.y < 0 || (point2d_noise.at<double>(2)) < 5000) {
                        continue;
                    }

                    Mat point2d_gt = P_gt_all[camera_id] * point3dmat;
                    Point2d pointgt = Point2d(point2d_gt.at<double>(0) / point2d_gt.at<double>(2),
                                              point2d_gt.at<double>(1) / point2d_gt.at<double>(2));
                    
                    if (pointgt.x >= 1920 || pointgt.x < 0 || pointgt.y >= 1080 || pointgt.y < 0 ||
                        (point2d_gt.at<double>(2)) < 5000) {
                        continue;
                    }
                    
                    drawCross(drawImg, pointdraw, Scalar(0, 255, 0), 10, 1);
                    circle(drawImg, pointgt, 10, Scalar(0, 0, 255), 1);
                    float rmse =
                        sqrtf(pow(pointdraw.x - pointgt.x, 2) + pow(pointdraw.y - pointgt.y, 2) +
                              pow(pointdraw.y - pointgt.y, 2));

                    if (rmse > 10) {
                        cout << "xyz " << x << " " << y << " " << z << " "
                             << point2d_gt.at<double>(2, 0) << endl;
                        cout << "gt " << pointgt << endl;
                        cout << "ours " << pointdraw << endl;
                    }

                    id++;
                    sum += rmse;
                    sum_id += rmse;
                    loopCount++;
                }
                fs << x << " " << y << " " << z << " " << sum_id / id << "\n";
            }
        }
    }
    fs_euler_tvec << "mean_rmse error: " << sum / (double)loopCount << endl;
    cv::imwrite(res_path + "/drawImg.png", drawImg);

    std::cout << "calibration finished!" << endl;

    fs_euler_tvec.close();
    fs.close();

    return 0;
}
