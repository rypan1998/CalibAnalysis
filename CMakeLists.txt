cmake_minimum_required(VERSION 3.1)

project(visualize_reprojection)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OpenCV_DIR "/usr/local/opencv420/lib/cmake/opencv4")
# set(OpenCV_DIR "/home/rypan/repos/opencv/install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

include_directories("/usr/local/include/eigen3")
# include_directories("/usr/include/eigen3")

add_executable(getXmlFormColmap ./app/getXmlFormColmap.cpp)
target_link_libraries(getXmlFormColmap ${OpenCV_LIBS})

add_executable(cal_srt ./app/CalsRT.cpp)
target_link_libraries(cal_srt ${OpenCV_LIBS})

add_executable(${PROJECT_NAME} ./app/VisualizeReprojection.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
