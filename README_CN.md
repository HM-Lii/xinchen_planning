# Planning Offline Task

本项目基于游戏模拟器和C++编程语言，涉及自动驾驶中的规划控制。您可以选择任何操作系统和IDE来完成此项目。


## 代码和报告提交要求

### 代码风格

请尽力遵循[Google C++代码风格指南](https://google.github.io/styleguide/cppguide.html)。


您可以通过创建zip/tar压缩文件直接发送给我们来提交作品，或者将仓库推送到GitHub或GitLab等平台并分享链接。**但请注意，请勿公开分享此项目。** 您可以在大多数网站上分享私有仓库，例如在GitHub上创建私有仓库并[邀请协作者](https://docs.github.com/zh/account-and-profile/setting-up-and-managing-your-personal-account-on-github/managing-access-to-your-personal-repositories/inviting-collaborators-to-a-personal-repository)。

### 评分标准

评审将基于以下几个方面：

* 您的研究报告，详细解释每一步的思考过程
* 代码质量及问题回答质量
* 代码和环境的可复现性

研究报告最好以PDF格式提交，但如果您偏好其他格式（如Markdown或Jupyter Notebook）也可以。您提供的C++代码应至少通过初始化Git仓库并进行一次提交来进行版本控制。

如果您有具体问题或感到卡住无法进展，请直接联系我们。

## 任务目标
本项目需要您实现一个算法在虚拟高速公路上安全导航，其他车辆将以限速50英里/小时的±10英里/小时速度行驶。您将获得车辆定位数据和传感器融合数据，以及高速公路的稀疏地图航点列表。车辆应尽可能接近50英里/小时的限速行驶，这意味着需要适时超越慢车（请注意其他车辆也会变换车道）。车辆必须不惜一切代价避免碰撞，并且始终在标记车道内行驶（变道期间除外）。车辆需要完整绕行6946米长的高速公路一圈。以50英里/小时速度行驶时，单圈耗时约5分多钟。同时车辆总加速度不得超过10 m/s²，加加速度（jerk）不得超过10 m/s³。

### 高速公路地图位于data/highway_map.csv
每个航点包含[x,y,s,dx,dy]值：x和y是航点的地图坐标位置，s值是沿道路到该航点的距离（米），dx和dy定义指向高速公路环线外侧的单位法向量。

高速公路航点呈环状分布，因此frenet坐标的s值（沿道路距离）从0到6945.554循环。

### 模拟器向C++程序提供的数据

#### 主车定位数据（无噪声）

["x"] 车辆在地图坐标系中的x坐标  
["y"] 车辆在地图坐标系中的y坐标  
["s"] 车辆在frenet坐标系中的s坐标  
["d"] 车辆在frenet坐标系中的d坐标  
["yaw"] 车辆在地图中的偏航角  
["speed"] 车辆速度（英里/小时）

#### 提供给规划器的历史路径数据

//注意：需返回已处理点被移除后的历史点列表，这可有效显示自上次处理后路径的进展程度

["previous_path_x"] 之前提供给模拟器的x点列表  
["previous_path_y"] 之前提供给模拟器的y点列表

#### 历史路径的终点s和d值

["end_path_s"] 历史路径最后一个点的frenet s值  
["end_path_d"] 历史路径最后一个点的frenet d值

#### 传感器融合数据：同侧道路上所有其他车辆的属性列表（无噪声）

["sensor_fusion"] 二维车辆向量，包含[车辆唯一ID, 车辆在地图坐标系中的x坐标, 车辆在地图坐标系中的y坐标, 车辆x方向速度(m/s), 车辆y方向速度(m/s), 车辆在frenet坐标系中的s坐标, 车辆在frenet坐标系中的d坐标]


### 模拟器
如果遇到模拟器卡顿的情况，请使用全屏模式。

在Mac/Linux系统上运行模拟器时，请先通过以下命令赋予二进制文件可执行权限：
```shell
sudo chmod u+x {模拟器文件名}
```

### 环境依赖

* cmake >= 3.5  
  * 所有操作系统：[安装说明](https://cmake.org/install/)
* make >= 4.1  
  * Linux：大多数发行版默认安装
  * Mac：[安装Xcode命令行工具](https://developer.apple.com/xcode/features/)
  * Windows：[安装说明](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4  
  * Linux：大多数发行版默认安装
  * Mac：与make相同-[安装Xcode命令行工具](https://developer.apple.com/xcode/features/)
  * Windows：推荐使用[MinGW](http://www.mingw.org/)
* Boost >= 1.66（需要Boost.System和Boost.Beast）
  * Ubuntu：运行`bash ./install-ubuntu.sh`
  * macOS：运行`bash ./install-mac.sh`

### 构建与运行

```shell
cmake -S . -B build
cmake --build build --parallel
cd build
./path_planning
```

程序会从构建目录中的`data/highway_map.csv`读取地图，并监听本机`4567`端口。随后启动模拟器即可连接规划程序。

## 声明

本项目基于Udacity的无人驾驶汽车纳米学位项目，并遵循其版权声明。
