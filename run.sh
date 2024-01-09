#!/bin/bash
camera_num=60
code_path=/home/fhy/Desktop/workspace/rypan/repos/重投影误差可视化
root_dir=/home/fhy/Desktop/workspace/rypan/repos/序列标定/template/圆形单层/circleDist

inputDir_path=$root_dir/r_20/0 # txt 路径
outputDir_path=$code_path/input/xml_noise_colmap/ # 保存转换得到的 xml 文件，需要以 / 结尾
gt_xml_path=$root_dir/xml_gt/%d.xml # 真值 xml
noise_colmap_xml_path=$code_path/input/xml_noise_colmap/%d.xml # 优化得到的 xml 路径（colmap坐标系）
noise_world_xml_path=$code_path/input/xml_noise_world/%d.xml # 优化得到的 xml 路径（世界坐标系）


# 1. colmap txt->xml
# 输入 inputDir_path，输出转换得到的 COLMAP 坐标系下的 xml 文件，保存在 outputDir_path 中
echo "----- step 1: txt to xml +++++"
$code_path/build/getXmlFormColmap $camera_num \
    $inputDir_path \
    $outputDir_path

# 2. 坐标系转换
# 输入真值 gt_xml_path 和 COLMAP 坐标系下的优化值 noise_colmap_xml_path，输出世界坐标系下的优化值 noise_world_xml_path
$code_path/build/cal_srt $camera_num $gt_xml_path $noise_colmap_xml_path $noise_world_xml_path

# 3. 可视化重投影误差
# 输入真值 gt_xml_path 和世界坐标系下的优化值 noise_world_xml_path，以及图像所在路径，输出误差值
$code_path/build/visualize_reprojection $camera_num $gt_xml_path $noise_world_xml_path $inputDir_path
