cmake_minimum_required(VERSION 3.22)
project(mission4redo)
find_package( OpenCV REQUIRED ) # 找到需求的库或者组件
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_STANDARD 20)

add_executable(mission4redo main.cpp
        Armor/Armor.cpp
        Armor/Armor.h
        Defines.h
        PreProcess/PreProcess.cpp
        PreProcess/PreProcess.h
        Detect/Detect.cpp
        Detect/Detect.h
        camera/Camera.h
        Kalman/Kalman.cpp
        Kalman/Kalman.h
        camera/Camera.cpp
)


include_directories( ${OpenCV_INCLUDE_DIRS} ) # 添加一个用于包含的文件去构建的文件夹
target_link_libraries(mission4redo ${OpenCV_LIBS}  openvino::runtime) # 链接的库
target_link_libraries(mission4redo Eigen3::Eigen)