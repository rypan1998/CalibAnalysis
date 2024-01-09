/*
 * @Author: FengLY
 * @Date: 2021-11-11 20:40:35
 * @LastEditTime: 2021-11-30 17:19:11
 * @LastEditors: Please set LastEditors
 * @Description: 从txt中读取出对应的xml
 * @FilePath: /convertXml/app/getXmlFormColmap.cpp
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
#include <string>
#include <unordered_map>

using namespace std;
using namespace cv;

// [COLMAP简易教程(命令行模式)](https://www.jianshu.com/p/95cf3a63b6bb)

struct CameraRT {
    cv::Mat quaternion;
    cv::Mat translation_matrix;
};

std::vector<pair<int, CameraRT>> cam_ext_parameters_arr;

int main(int argc, char **argv) {
    // TomlConfig::Ptr m_config = TomlConfig::GetInstance("../data/config.toml");
    // int camera_num = m_config->cameraNum;
    int camera_num = atoi(argv[1]);

    set<int> s_camera_skip;
    // for(int i=0;i<m_config->pathMap.size();++i){
    //     s_camera_skip.insert((int)m_config->pathMap[i]);
    // }

    // std::string inputDir_path = m_config->savePath + argv[1]; //txt的位置
    // std::string outputDir_path = m_config->savePath;
    std::string inputDir_path = argv[2];  // txt的位置
    std::string outputDir_path = argv[3];

    std::string images_path = inputDir_path + "/images.txt";
    cout << images_path << endl;
    ifstream in(images_path);

    string line;
    FileStorage fs;
    unordered_map<int, cv::Mat> camera_poistion_arr;
    vector<int> not_lose_camera_id;

    if (in) {
        while (getline(in, line)) {
            if (line.find("jpg") != string::npos) {
                std::cout << line << std::endl;
                int camera_index = 0;
                std::vector<float> temp_quaternion;
                std::vector<float> temp_translation_matrix;
                int find_count = 0;
                char *cstr = new char[line.length() + 1];
                std::strcpy(cstr, line.c_str());
                char *p = std::strtok(cstr, " ");
                while (p != NULL) {
                    find_count++;
                    if (find_count == 9) {
                        camera_index = atoi(p);
                    } else if (find_count > 1 && find_count <= 5) {
                        temp_quaternion.push_back(atof(p));
                    } else if (find_count > 5 && find_count <= 8) {
                        temp_translation_matrix.push_back(atof(p));
                    }

                    if (find_count >= 9) break;
                    p = std::strtok(NULL, " ");
                }
                cv::Mat quaternion = (cv::Mat_<float>(4, 1) << temp_quaternion[0],
                                      temp_quaternion[1], temp_quaternion[2], temp_quaternion[3]);
                cv::Mat translation_matrix =
                    (cv::Mat_<float>(3, 1) << temp_translation_matrix[0],
                     temp_translation_matrix[1], temp_translation_matrix[2]);

                std::cout << "cameraindex = " << camera_index << std::endl;
                std::cout << "quaternion = " << quaternion << std::endl;
                std::cout << "translation_matrix = " << translation_matrix << std::endl;

                not_lose_camera_id.push_back(camera_index);

                Mat R = (cv::Mat_<float>(3, 3) << (1 - 2 * pow(temp_quaternion[2], 2) -
                                                   2 * pow(temp_quaternion[3], 2)),
                         (2 * temp_quaternion[1] * temp_quaternion[2] -
                          2 * temp_quaternion[0] * temp_quaternion[3]),
                         (2 * temp_quaternion[1] * temp_quaternion[3] +
                          2 * temp_quaternion[0] * temp_quaternion[2]),
                         (2 * temp_quaternion[1] * temp_quaternion[2] +
                          2 * temp_quaternion[0] * temp_quaternion[3]),
                         (1 - 2 * pow(temp_quaternion[1], 2) - 2 * pow(temp_quaternion[3], 2)),
                         (2 * temp_quaternion[2] * temp_quaternion[3] -
                          2 * temp_quaternion[0] * temp_quaternion[1]),
                         (2 * temp_quaternion[1] * temp_quaternion[3] -
                          2 * temp_quaternion[0] * temp_quaternion[2]),
                         (2 * temp_quaternion[2] * temp_quaternion[3] +
                          2 * temp_quaternion[0] * temp_quaternion[1]),
                         (1 - 2 * pow(temp_quaternion[1], 2) - 2 * pow(temp_quaternion[2], 2)));

                std::cout << "R = " << R << std::endl;
                std::string output_xml_path = outputDir_path + to_string(camera_index - 1) + ".xml";
                fs.open(output_xml_path, FileStorage::WRITE);
                fs << "cameraindex" << camera_index - 1;
                fs << "R" << R;  //世界到相机的
                fs << "t" << translation_matrix;

                cv::Mat camera_poistion = -R.t() * translation_matrix;
                camera_poistion_arr[camera_index] = camera_poistion;

                Mat Rt_34;  // [sR|t]
                Mat Rt;     // [sR|t] 4x4
                Mat last_row = (Mat_<float>(1, 4) << 0, 0, 0, 1);
                hconcat(R, translation_matrix, Rt_34);  // 3x4
                vconcat(Rt_34, last_row, Rt);           // 4x4
                fs << "RT" << Rt;
                fs.release();
            }
        }
    } else {
        cout << "no images file" << endl;
    }

    //如果丢失相机，可以在这里进行保存
    vector<int> lose_camera;
    std::string xml_folder_path = argv[3];
    ofstream lose_calibration(xml_folder_path + "lose_calibration.txt");
    for (int i = 0; i < camera_num; i++) {
        vector<int>::iterator it_not_lose =
            find(not_lose_camera_id.begin(), not_lose_camera_id.end(), i + 1);
        if (it_not_lose == not_lose_camera_id.end()) {
            lose_camera.push_back(i + 1);
            lose_calibration << i + 1 << ",";
        }
    }
    vector<int> false_camera;
    uchar out;
    ifstream false_calibration(xml_folder_path + "false_calibration.txt");

    string buff;
    vector<double> nums;

    while (getline(false_calibration, buff)) {
        char *s_input = (char *)buff.c_str();
        const char *split = ",";
        char *p = strtok(s_input, split);
        double a;
        while (p != NULL) {
            a = atof(p);
            nums.push_back(a);
            p = strtok(NULL, split);
        }  // end while
    }      // end while
    false_calibration.close();

    for (int i = 0; i < nums.size(); i++) {
        cout << nums[i] << " ";
    }
    std::vector<cv::Point3f> save_camera_position_vec;
    for (int camera_id = 1; camera_id <= camera_num; camera_id++) {
        vector<int>::iterator it = find(lose_camera.begin(), lose_camera.end(), camera_id);
        if (it != lose_camera.end()) {
            continue;
        }

        it = find(false_camera.begin(), false_camera.end(), camera_id);
        if (it != false_camera.end()) {
            continue;
        }

        if (s_camera_skip.count(camera_id)) {
            continue;
        }

        // if (camera_id > camera_skip_start && camera_id < camera_skip_end)
        // {
        // 	continue;
        // }
        cv::Point3f temp_point = Point3f(camera_poistion_arr[camera_id].at<float>(0, 0),
                                         camera_poistion_arr[camera_id].at<float>(1, 0),
                                         camera_poistion_arr[camera_id].at<float>(2, 0));
        save_camera_position_vec.push_back(temp_point);
    }
    FileStorage fs_camera_poistion_arr;
    std::string camera_position_xml = inputDir_path + "/points.xml";  //这个其实不太需要
    fs_camera_poistion_arr.open(camera_position_xml, FileStorage::WRITE);
    fs_camera_poistion_arr << "points" << save_camera_position_vec;
    fs_camera_poistion_arr.release();

    std::string cameras_path = inputDir_path + "/cameras.txt";
    ifstream in_camera(cameras_path);
    if (in_camera) {
        while (getline(in_camera, line)) {
            int camera_index;
            std::vector<float> IntrinsicCamVec;
            std::vector<float> DistortionCamVec;
            // XXX: 如果用针孔相机模式，这里就会有问题
            if (line.find("OPENCV") != string::npos) {
                std::cout << line << std::endl;
                int find_count = 0;  // ???????????? find_count = 5 - 8 ?????? ????fx, fy, cx,
                                     // cy???? 9 - 12 ?????? (k1 k2 p1 p2)
                char *cstr = new char[line.length() + 1];
                std::strcpy(cstr, line.c_str());
                char *p = std::strtok(cstr, " ");
                while (p != NULL) {
                    find_count++;
                    if (find_count == 1) {
                        camera_index = atoi(p);
                    } else if (find_count >= 5 && find_count <= 8) {
                        IntrinsicCamVec.push_back(atof(p));
                    } else if (find_count >= 9) {
                        DistortionCamVec.push_back(atof(p));
                    }
                    p = std::strtok(NULL, " ");
                }
                Mat IntrinsicCam =
                    (cv::Mat_<float>(3, 3) << IntrinsicCamVec[0], 0.0f, IntrinsicCamVec[2], 0.0f,
                     IntrinsicCamVec[1], IntrinsicCamVec[3], 0.0f, 0.0f, 1.0f);
                Mat DistortionCam =
                    (cv::Mat_<float>(1, 5) << DistortionCamVec[0], DistortionCamVec[1],
                     DistortionCamVec[2], DistortionCamVec[3], 0.0f);

                std::cout << "cameraindex : " << camera_index << std::endl;
                std::cout << "IntrinsicCam : " << IntrinsicCam << std::endl;
                std::cout << "DistortionCam : " << DistortionCam << std::endl;

                std::string output_xml_path = outputDir_path + to_string(camera_index - 1) + ".xml";
                // std::string file_name = "C:/Users/HSK/Desktop/Project1/data/color_demo/xml/" +
                // to_string(camera_index - 1) + ".xml";
                fs.open(output_xml_path, FileStorage::APPEND);

                fs << "IntrinsicCam" << IntrinsicCam;
                fs << "DistortionCam" << DistortionCam;
                fs.release();
            } else if (line.find("SIMPLE_RADIAL") != string::npos) {
                std::cout << line << std::endl;
                int find_count = 0;
                char *cstr = new char[line.length() + 1];
                std::strcpy(cstr, line.c_str());
                char *p = std::strtok(cstr, " ");
                while (p != NULL) {
                    find_count++;
                    if (find_count == 1) {
                        camera_index = atoi(p);
                    } else if (find_count >= 5 && find_count <= 7) {
                        IntrinsicCamVec.push_back(atof(p));
                    } else if (find_count >= 8) {
                        DistortionCamVec.push_back(atof(p));
                    }
                    p = std::strtok(NULL, " ");
                }
                Mat IntrinsicCam =
                    (cv::Mat_<float>(3, 3) << IntrinsicCamVec[0], 0.0f, IntrinsicCamVec[1], 0.0f,
                     IntrinsicCamVec[0], IntrinsicCamVec[2], 0.0f, 0.0f, 1.0f);
                Mat DistortionCam =
                    (cv::Mat_<float>(1, 5) << DistortionCamVec[0], 0.0f, 0.0f, 0.0f, 0.0f);

                std::cout << "cameraindex : " << camera_index << std::endl;
                std::cout << "IntrinsicCam : " << IntrinsicCam << std::endl;
                std::cout << "DistortionCam : " << DistortionCam << std::endl;

                std::string output_xml_path = outputDir_path + to_string(camera_index - 1) + ".xml";
                // std::string file_name = "C:/Users/HSK/Desktop/Project1/data/color_demo/xml/" +
                // to_string(camera_index - 1) + ".xml";
                fs.open(output_xml_path, FileStorage::APPEND);

                fs << "IntrinsicCam" << IntrinsicCam;
                fs << "DistortionCam" << DistortionCam;
                fs.release();
            } else if (line.find("PINHOLE") != string::npos) {
                std::cout << line << std::endl;
                int find_count = 0;
                char *cstr = new char[line.length() + 1];
                std::strcpy(cstr, line.c_str());
                char *p = std::strtok(cstr, " ");
                while (p != NULL) {
                    find_count++;
                    if (find_count == 1) {
                        camera_index = atoi(p);
                    } else if (find_count >= 5 && find_count <= 8) {
                        IntrinsicCamVec.push_back(atof(p));
                    }

                    p = std::strtok(NULL, " ");
                }
                // Mat IntrinsicCam = (cv::Mat_<float>(3, 3) << IntrinsicCamVec[0], 0.0f,
                // IntrinsicCamVec[2], 					0.0f, IntrinsicCamVec[1], IntrinsicCamVec[3], 					0.0f,
                // 0.0f, 1.0f);
                Mat IntrinsicCam =
                    (cv::Mat_<float>(3, 3) << IntrinsicCamVec[0], 0.0f, IntrinsicCamVec[1], 0.0f,
                     IntrinsicCamVec[0], IntrinsicCamVec[2], 0.0f, 0.0f, 1.0f);
                Mat DistortionCam = (cv::Mat_<float>(1, 5) << 0.0, 0.0f, 0.0f, 0.0f, 0.0f);

                std::cout << "cameraindex : " << camera_index << std::endl;
                std::cout << "IntrinsicCam : " << IntrinsicCam << std::endl;
                std::cout << "DistortionCam : " << DistortionCam << std::endl;

                std::string output_xml_path = outputDir_path + to_string(camera_index - 1) + ".xml";
                // std::string file_name = "C:/Users/HSK/Desktop/Project1/data/color_demo/xml/" +
                // to_string(camera_index - 1) + ".xml";
                fs.open(output_xml_path, FileStorage::APPEND);

                fs << "IntrinsicCam" << IntrinsicCam;
                fs << "DistortionCam" << DistortionCam;
                fs.release();
            }
        }
    } else {
        cout << "no cameras file" << endl;
    }

    std::cout << "From txt extract RT , IntrinsicCam, DistortionCam finished!!!!" << std::endl;
    return 0;
}
