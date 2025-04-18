# VSLAM_ws
借助colcon构建一些需要手动编译的SLAM软件包, 同时构建一些视觉SLAM算法

## 此分支环境
| 软件   | 版本  |
| ------ | ----- |
| ubuntu | 24.04 |
| ros2   | jazzy |

⚠ 编译需要 RAM >= 32G, 如果不够则添加虚拟内存
## 主要内容
| 依赖     | 版本         |
| -------- | ------------ |
| g2o      | 20230223_git |
| sophus   | 1.24.6       |
| pangolin | v0.9.2       |
| DBoW2    | master       |
| ncnn     | master       |

| 原仓库                                                                                  | 修改说明                                    |
| --------------------------------------------------------------------------------------- | ------------------------------------------- |
| [obindex2](https://github.com/emiliofidalgo/obindex2)                                   | 删除 boost 依赖                             |
| [ibow_lcd](https://github.com/emiliofidalgo/ibow-lcd)                                   | 只保留库文件                                |
| [ORB_SLAM2_detailed_comments](https://github.com/electech6/ORB_SLAM2_detailed_comments) | 适配新版g2o和opencv                         |
| [ov2slam](https://github.com/ov2slam/ov2slam)                                           | 适配新版ceres和ROS2; 增加直接读取数据集代码 |
| [VINS-MONO-ROS2](https://github.com/dongbo19/VINS-MONO-ROS2)                            | 适配新版ceres和ROS2; 增加直接读取数据集代码 |

## 依赖
```sh
# 安装ROS2
sudo apt install ros-jazzy-desktop
sudo apt install python3-vcstool libceres-dev

# pangolin依赖
sudo apt install libepoxy-dev 

# g2o依赖
sudo apt install libsuitesparse-dev

# 其他依赖
sudo apt install libpoco-dev

vcs import --input dependencies.yaml --shallow

```

## 编译
```sh
colcon build

# 如果内存不够，可以使用单线程编译，但是内存可能还是会爆，建议加大虚拟内存
colcon build --executor sequential

# 如果显示找不到工作空间的某个包, 可以先source工作空间, 注意区分.zsh和.bash
source ./install/setup.zsh
colcon build

# 解压orbvoc
cd ./src/config_pkg/voc
tar -xzvf ./ORBvoc.txt.tar.gz

# 7z解压
7z x zip.7z
```
💡 build完成后也可以把 ```source (path)/install/setup.zsh(bash)``` 添加到环境变量中, 这样可以在对应包目录中单独利用cmake编译包, 运行和调试

## SLAM运行
⚠ 若提示缺失动态库，需要先source工作空间
⚠ 在WSL2运行时记得打开 ROS2 daemon：```ros2 daemon start```，否则相关可视化功能无法使用
1. 在build目录中运行
```sh
# ov2slam
./build/ov2slam/ov2slam_node config_file_path

# orbslam2
./build/ORB_SLAM2/stereo_kitti  ./src/config_pkg/voc/ORBvoc.txt ./src/config_pkg/config/ORB_SLAM2/KITTI00-02.yaml ~/datasets/KITTI/00

# ssvio 
./build/ssvio/test/test_ssvio --config_yaml_path=/home/fu/VSLAM_ws/src/config_pkg/config/ssvio/kitti_04-12.yaml --kitti_dataset_path=/home/fu/datasets/KITTI/09
```

2. 通过ros2 run运行
```sh
# ov2slam
ros2 run ov2slam ov2slam_node config_file_path
ros2 run ov2slam kitti 00 ~/VSLAM_ws/src/config_pkg/config/ov2slam/accurate/kitti/kitti_00-02.yaml
```

3. 通过ros2 launch运行
```sh
# ov2slam
ros2 launch ov2slam kitti.py

# vins_mono 跑uma数据集
ros2 launch vins_mono uma_vi.py    
```
## colcon配置

1. 在目录下放置名为COLCON_IGNORE的空文件, 则该目录不会被索引
2. 使用colcon.meta自定义每个包的命令行参数
