# VSLAM_ws
借助colcon构建一些需要手动编译的SLAM软件包, 同时构建一些视觉SLAM算法

## 此分支环境
| 软件      | 版本 |
| ----------- | ----------- |
| ubuntu      | 24.04       |
| ros2   | jazzy        |

⚠ 编译需要 RAM >= 32G, 如果不够则添加虚拟内存
## 主要内容
| 软件      | 修改说明 |
| ----------- | ----------- |
| ORB-SLAM2      | 24.04       |
| ov2slam   | 适配ubuntu24.04和ROS2; 增加直接读取数据集代码   |

引用 
[ORB_SLAM2_detailed_comments](https://github.com/electech6/ORB_SLAM2_detailed_comments)
[VINS-MONO-ROS2](https://github.com/dongbo19/VINS-MONO-ROS2)
[ov2slam](https://github.com/ov2slam/ov2slam)
## 1. 下载依赖
```
vcs import < dependencies.yaml --shallow

# 安装ROS2
sudo apt install ros-jazzy-desktop

# pangolin依赖
sudo apt install libepoxy-dev 
```

## 编译
```
colcon build
```