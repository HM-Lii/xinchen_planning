# 自动驾驶路径规划离线任务

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
* OSQP 1.x（需要提供CMake包和`osqp::osqp`目标）
  * Ubuntu：运行`bash ./install-ubuntu.sh`
  * macOS：运行`bash ./install-mac.sh`
  * 安装脚本会从官方`v1.0.0`源码构建到项目内的`.deps/osqp`
  * CMake会优先搜索该本地目录，也支持调用者提供的系统级OSQP
  * Eigen 3.3头文件已包含在仓库中，QP封装直接使用OSQP C接口

### 构建与运行

```shell
cmake -S . -B build
cmake --build build --parallel
cd build
./path_planning
```

项目未显式指定其他配置时默认使用Release构建，因此只需要保留`build`目录。程序会从构建目录中的`data/highway_map.csv`读取地图，并监听本机`4567`端口。随后启动模拟器即可连接规划程序。

### 规划器接口契约

通信层会将模拟器遥测转换为`PlannerInput`，规划器返回`PlannerOutput`。每次输出必须满足：

* `next_x`和`next_y`长度相同且固定为50个点
* 相邻点的执行时间间隔为0.02秒
* 尚未执行历史路径的前15点最多作为不可修改前缀原样保留；更远点允许依据新观测重新规划
* 缺少字段、字段类型错误、历史路径长度不一致或非有限数值会进入手动模式

### 当前规划器

当前在线入口明确选择 `BehaviorActive`。规划器先生成并硬校验同周期 `LaneCruiseOnly` 备份，再同步评估全车道交通、稳定 `GapId`、空间路径、ST走廊和候选QP；只有连续稳定且完整笛卡尔轨迹通过速度、动力学、道路边界、预测覆盖和扫掠OBB碰撞硬门禁的变道候选才能替换备份并进入控制。库配置默认仍是 `LaneCruiseOnly`，便于独立比较和显式回退。

控制碰撞门显式屏蔽当前帧同车道且车身已完全位于自车后方的车辆，避免后车追尾预测反向触发最大制动；屏蔽数量和ID仍写入周期与候选CSV。当前已重叠后车、前车、相邻侵入对象不屏蔽，主动 tracker/预测/Gap仍完整处理后车风险。

正常策略不允许时距松弛；降级策略可以牺牲运行时距，但始终保留车辆本体物理边界。只有普通候选全部失败且证据证明碰撞不可避免时，才由独立最小风险QP显式使用碰撞违反变量并通过 `MinimumRiskDispatch` 下发；输入、地图、求解或证据不足则分类为 `InfrastructureFailure`。失败候选不会污染横向拼接、QP warm start 或下一周期控制状态。

全车道 tracker、保守预测、Gap粗准入、继承三阶边界的七次空间路径、固定通行拓扑、双车道ST约束和曲率速度预算由主动规划器在控制事务内同步执行。源车道前车门逐条评估预测假设，并只处理变道起点位于前方的车辆；当前观测 GapId 的历史稳定性不被未来拓扑变化清零。目标 Gap 粗门从同一个初始位置/速度传播有界状态集，只在合流完成时要求前时距、后时距和TTC同时成立。全链路不使用横向不确定度扩张：车道归属、Gap、双车道ST/QP及最终校验只使用车辆预测物理轮廓和实测横向速度产生的横向延续轨迹；无实际横向侵入的邻车不会触发减速或放弃变道。前车保守减速度默认 `-1.0 m/s²`、硬下限 `-2.0 m/s²`，后车保守加速默认 `0.5 m/s²`、硬上限 `1.0 m/s²`，纵向预测不确定度以 `0.25 m/s` 纯线性增长且不含二次项。一般未来重排序和宽可达区间侵入仅保留诊断；只有预期前后边界真正反转才产生 `GapTopologyChanged`，非边界目标车则逐假设过滤同一组相干状态，只有安全状态清零才产生 `MergeCorridorBlocked`。空间路径与纵向QP合成为完整8秒笛卡尔轨迹，复用统一 `TrajectoryValidator`，仅允许一次确定性的动力学限速收紧重求，并按硬余量优先、代价次之进行确定性排序。行为层使用 `KeepLane -> PrepareCandidate -> Committed -> Settling` 状态机：候选必须连续多周期保持同一目标车道、Gap和路径身份；提交后不重新选择目标，不重建变道几何，直到目标车道中心和横向速度连续稳定才完成。

`PathPlanner::SetOperatingMode` 保留 `kBehaviorActive` 与 `kLaneCruiseOnly` 的运行时切换接口。准备阶段切回巡航立即生效；已经提交的变道先由主动执行器安全完成并稳定到目标车道，再把有效模式交还巡航，防止中途强制回原车道。`operating_mode()` 返回请求模式，`effective_operating_mode()` 返回实际控制模式，`behavior_diagnostics()` 提供阶段、承诺身份、稳定周期、振荡、取消和完成计数。在线监控写入周期、点、QP、控制候选、行为候选和碰撞事件CSV；`planner_cycle.csv` 记录主动Evaluate/事务/异常及各级候选计数，`planner_behavior_candidates.csv` 记录Gap、粗准入、空间路径、ST/QP和完整轨迹硬门的逐候选证据，其中源车道前车门禁包含限制车辆、预测假设、首次风险时刻和有符号净余量，目标粗可达性包含首次失败时刻、传播/可行状态数和三项最佳余量，Gap拓扑区分一般重排序、真实边界反转、宽区间侵入和相干状态全部受阻。

设计思路见[高速领航整体原理](doc/highway_navigation_principles.md)，安全、坐标、状态和模式语义见[高速领航架构](doc/highway_navigation_architecture.md)。车道巡航的参数、数学模型、状态管理、异常处理及测试方法见[车道巡航实现细节](doc/lane_cruising_implementation.md)。

### 测试

```shell
ctest --test-dir build --output-on-failure
```

## 声明

本项目基于Udacity的无人驾驶汽车纳米学位项目，并遵循其版权声明。
