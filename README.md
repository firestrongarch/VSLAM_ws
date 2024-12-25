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
## 依赖
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

# 如果显示找不到某个包, 可以先source工作空间, 注意区分.zsh和.bash
source ./install/setup.zsh
colcon build

```
💡 build完成后也可以把 ```source (path)/install/setup.zsh(bash)``` 添加到环境变量中, 这样可以在对应包目录中单独利用cmake编译包, 运行和调试

## 运行