# 高速领航架构

> 状态：当前架构基线
>
> 目的：锁定安全、坐标、时间、状态和模式语义
>
> 适用范围：车道巡航与主动行为规划

本文定义规划器必须保持一致的架构约束，不规定需要通过仿真标定的权重、阈值和候选数量。当前实现细节见[车道巡航实现细节](lane_cruising_implementation.md)。

当其他方案文档与本文在坐标、安全、候选提交或运行模式上冲突时，以本文为准。

## 1. 目标与非目标

目标：

* 保留当前车道巡航，作为可独立运行的基线模式和所有正常周期都存在的保守候选。
* 在同一套预测、优化、验证和状态提交语义上支持行为搜索与变道。
* 正常控制输出只能来自通过完整硬校验的候选；不存在普通可行候选时，只允许通过显式 MinimumRiskDispatch 下发最小风险输出。
* 变道只在保守预测下具有明显容错空间时获准。
* 求解失败、行为规划失败和日志失败不得污染控制状态。
* 对无法物理避免的碰撞使用明确状态表达，不伪造安全结论。

非目标：

* 不保证应对周车无限加速度、瞬时横移或违反配置运动边界的行为。
* 不追求混合整数或非线性规划的全局最优性。
* 不允许未经稳定性门禁和完整硬校验的变道候选影响实际输出。
* 不长期维护一套带有已知安全缺陷的旧实现作为生产回退。

## 2. 固定架构决策

| 主题 | 决策 |
| --- | --- |
| 运行模式 | 车道巡航与行为主动两种模式 |
| 保守基线 | 车道巡航始终是一等候选，不是异常分支 |
| 交通坐标 | 使用展开的参考线 RoadSUnwrapped 做全局排序和预测 |
| 候选坐标 | 使用候选路径真实物理弧长 PathProgress 做纵向优化 |
| 横向候选 | 使用固定空间路径，时间由纵向 QP 决定 |
| QP 状态 | p、v、a、j 分别为路径进度及其一至三阶时间导数 |
| 安全层次 | 物理碰撞硬边界、运行裕量、舒适目标三层分离 |
| 历史前缀 | 只允许保留有界时长，并纳入当前周期安全校验 |
| 候选评估 | 基于不可变快照、无副作用评估、只提交胜者 |
| 最终判定 | 完整时域 Cartesian 硬校验，而不是监控后继续输出 |
| 变道策略 | 保守准入，条件不满足时继续车道巡航 |
| 回退语义 | 回退是按可行性排序的候选，不承诺任意状态都安全 |
| 输出分类 | ValidatedCandidate、MinimumRiskDispatch 和 InfrastructureFailure 明确分离 |
| 模式回滚 | 先禁止新行为，已承诺行为按状态安全结束后再完成模式切换 |

## 3. 运行模式与行为状态

运行模式控制哪些规划结果可以影响输出：

~~~cpp
enum class PlannerOperatingMode {
  kLaneCruiseOnly,
  kBehaviorActive
};
~~~

模式语义：

| 模式 | 实际输出 | 行为候选 | 用途 |
| --- | --- | --- | --- |
| LaneCruiseOnly | 车道巡航或当前车道制动 | 不参与控制 | 基线、回归、禁用变道 |
| BehaviorActive | 分层排序后的已验证候选 | 可参与控制 | 保守主动变道 |

运行模式与车辆行为状态是两套不同概念。行为状态至少包括：

~~~cpp
enum class ManeuverPhase {
  kKeepLane,
  kPrepareLaneChange,
  kLaneChangeCommitted,
  kLaneChangeSettling,
  kEmergencyBraking,
  kCollisionUnavoidable
};
~~~

改变运行模式不得隐式改变当前已承诺轨迹。模式切换必须经过显式初始化或安全过渡。

### 3.1 模式切换契约

请求的运行模式和当前生效模式必须分开保存。收到降级请求时，立即禁止发起新的主动变道，但生效模式按行为状态转换：

| 当前行为状态 | 降级到 LaneCruiseOnly |
| --- | --- |
| KeepLane | 本周期完成状态校验后切换 |
| PrepareLaneChange，车身仍完全位于原车道 | 选择已验证的保持车道或制动候选后切换 |
| LaneChangeCommitted | 保持主动执行能力，只允许已承诺继续、减速完成或独立验证的中止候选 |
| LaneChangeSettling | 完成目标车道稳定判定后切换 |
| EmergencyBraking / CollisionUnavoidable | 先保持紧急状态；恢复到可判定的车道内状态后再切换 |

因此，配置开关关闭 BehaviorActive 的直接效果是 `allow_new_maneuver=false`。只有 `effective_mode` 完成安全过渡后，才能报告模式切换完成。强行清除承诺状态、重置目标车道或无条件生成返回原车道轨迹均属于契约违规。

## 4. 坐标与单位语义

### 4.1 道路坐标

| 名称 | 单位 | 语义 |
| --- | --- | --- |
| RoadSWrapped | m | 模拟器和地图提供的环路参考线参数，范围为一个环长 |
| RoadSUnwrapped | m | 连续展开的参考线坐标，用于排序、相对位置和跨零点跟踪 |
| FrenetD | m | 相对参考线的横向坐标 |
| LaneIndex | 无 | 从左到右或从内到外的离散车道编号，项目内只保留一种约定 |

RoadSUnwrapped 是交通拓扑坐标，不代表偏置车道或变道曲线的真实物理弧长。任何从车辆速度到 RoadS 一阶导数的转换，都必须使用道路切线和局部几何，而不能直接把 Cartesian 速度模长相加到 RoadS。

### 4.2 候选路径坐标

每条候选先生成一条固定空间曲线：

$$
P(p)=\operatorname{Cartesian}\bigl(RoadS(p),d(p)\bigr)
$$

其中 PathProgress \(p\) 是从规划前沿开始、沿候选最终 Cartesian 曲线测量的真实物理弧长，单位为米。

纵向 QP 的状态固定为：

$$
x_k=[p_k,v_k,a_k],\qquad u_k=j_k
$$

$$
v=\frac{dp}{dt},\qquad
a=\frac{d^2p}{dt^2},\qquad
j=\frac{d^3p}{dt^3}
$$

因此 QP 速度与输出点列的实际沿路径速度使用同一物理语义。RoadS 与 PathProgress 之间只能通过候选路径的弧长表、投影或显式映射转换。

### 4.3 横向路径决策

横向候选采用空间域 \(d(p)\)，而不是把固定 \(d(t)\) 与未知纵向速度直接叠加。理由：

* 当前车道巡航已经以真实路径弧长为纵向语义。
* 固定空间路径允许预计算曲率、占道区间和 RoadS/PathProgress 映射。
* 纵向 QP 可以直接决定车辆何时到达路径上的不同位置。
* 总横向动力学仍由曲率速度上限和最终 Cartesian 校验约束。

车道巡航路径是当前回中路径；变道路径是连接当前横向状态与目标车道中心的候选路径。变道候选采样过渡长度或空间形状，不直接承诺固定完成时间。

### 4.4 强类型要求

接口不得用语义不明的单个 double 同时表示 RoadS、相对距离和 PathProgress。字段名必须包含语义，例如：

* road_s_unwrapped_m
* relative_road_s_m
* path_progress_m
* path_speed_mps
* prediction_time_s

## 5. 时间语义

| 名称 | 默认值 | 语义 |
| --- | ---: | --- |
| SimulatorDt | 0.02 s | 模拟器执行点间隔 |
| OutputPoints | 50 | 每次发送的总点数 |
| QpDt | 0.1 s | 纵向 QP 节点间隔 |
| PlanningHorizon | 8 s | 内部候选评估时域 |
| MaximumRetainedPrefix | 0.3 s | 可配置的保留前缀上限 |
| PostManeuverObservation | 2 s | 变道完成后的最低观察时域，最终值由生产配置确定 |

统一时间原点：

* TelemetryTime 是本次遥测时刻。
* RetainedPrefixDuration 是本次仍决定保留的历史前缀时长。
* PlanningFrontierTime 等于 TelemetryTime 加 RetainedPrefixDuration。
* QP 和候选路径从 PlanningFrontierTime 开始。
* 周车预测必须覆盖遥测时刻到完整规划时域结束。

完整候选必须包含保留前缀和规划前沿之后的 8 秒内部轨迹。输出仍只发送 50 个点，但安全判定不得只检查这 50 个点。

变道候选还必须满足以下时域覆盖契约之一：

1. 车辆完全进入目标车道的预计时刻不晚于 `PlanningHorizon - PostManeuverObservation`，并在候选时域内验证完成后观察区间。
2. 显式延长候选轨迹、ST 走廊和预测时域，直到变道完成时刻加 PostManeuverObservation。

不得只延长周车预测，却用超出主车候选时域的恒速外推证明完成后安全。未在时域内完成的空间变道路径必须标记 HorizonInsufficient，而不是普通 valid。

## 6. 交通状态与预测语义

跟踪状态至少包含：

~~~cpp
struct TrackedVehicleState {
  int id;
  double road_s_unwrapped_m;
  double d_m;
  double road_s_rate_mps;
  double d_rate_mps;
  double longitudinal_acceleration_mps2;
  double length_m;
  double width_m;
  double age_s;
};
~~~

要求：

* 车辆 ID 是跟踪主键。
* 环路跨零点通过上一帧状态展开。
* road_s_rate 由速度向量投影到道路切向后得到。
* 后车保留为负的相对 RoadS，不映射成接近一圈的正距离。
* 每个预测值携带时间和不确定性包络。
* 所有候选共享同一份只读预测快照。

预测安全只在配置的运动边界内成立。至少需要配置并记录：

* 前车最大保守减速度。
* 后车最大保守加速度。
* 最大横向速度或横向到达包络。
* 随预测时长线性增长的纵向不确定性，以及由实测横向速度驱动的横向延续轨迹；横向占用不使用不确定度扩张。
* 轨迹陈旧时间上限。

## 7. 安全语义

### 7.1 三层约束

物理硬边界：

* 车辆实体和硬膨胀包络不得重叠。
* 车辆不得越出道路可行边界。
* 最终速度、总加速度和总 jerk 不得超过评价硬上限。
* 该层不得为获得数值解而放松。

上述规则定义普通候选可行域。MinimumRiskDispatch 可以在已经证明该可行域为空后量化不可避免的碰撞违反，但该量化不改变硬边界定义，也不能把结果重新分类为 valid。

运行裕量：

* 正常跟车时间头距。
* 变道前后附加距离。
* 目标后车 TTC。
* 预测不确定性和历史前缀响应裕量。
* 正常候选和变道准入必须满足；紧急制动可以违反部分运行裕量，但不能违反物理硬边界。

舒适目标：

* 期望速度。
* 舒适加速度和 jerk。
* 变道收益、频率和一致性。
* 只参与可行候选之间的排序。

### 7.2 安全结论

安全结论必须写成条件结论：

> 在输入有效、周车运动位于配置预测边界、保留前缀仍可执行且完整候选通过硬校验的条件下，当前输出对模型包络安全。

不得把以下状态记录为普通安全成功：

* 物理碰撞约束在不可修改前缀内已不可行。
* 最大制动仍无法避免预测碰撞。
* 预测数据陈旧或完整校验未完成。

这些情况必须进入显式降级、InfrastructureFailure 或 CollisionUnavoidable 状态。

### 7.3 正常提交与最小风险下发

规划输出分类固定为：

~~~cpp
enum class PlanDisposition {
  kValidatedCandidate,
  kMinimumRiskDispatch,
  kInfrastructureFailure
};
~~~

ValidatedCandidate 必须通过完整硬校验，才能执行普通 Commit。MinimumRiskDispatch 只允许在所有正常、降级和最大制动候选都不能成为 ValidatedCandidate 后使用，并遵守：

* 仍然运行完整 validator，不跳过、不删除障碍物，也不伪造 `validation.valid=true`。
* 对可满足的非碰撞物理边界继续强制执行；不可满足项逐项记录。
* 风险采用固定分级规则比较：先避免新增碰撞对象，再推迟首个碰撞时刻，再降低预计相对碰撞速度和重叠程度，最后比较道路、动力学与舒适性损失。
* 风险评估模型、权重和并列规则必须版本化；不能用普通舒适总代价选择最小风险输出。
* 只提交紧急行为状态和该输出继续执行所需的状态，不得提交失败普通候选的目标车道或 warm start。
* MinimumRiskDispatch 永远不计为普通规划成功，也不能用于通过安全门禁。

InfrastructureFailure 表示输入、地图、时钟或执行环境不足以形成上述安全判定。它必须与 CollisionUnavoidable 分开，避免把软件或数据故障误报为物理不可避免。此时规划器不得宣称存在 ValidatedCandidate 或物理不可避免结论，而是把失败原因和最后可判定状态交给系统级安全监督器；监督器如何维持或停止执行属于独立接口契约。

### 7.4 变道保守准入

变道准入是硬门，不是软代价。至少同时满足：

* 条件连续多个周期稳定成立。
* 保留前缀期间无冲突。
* 主车完全离开原车道前，原车道前车约束持续有效。
* 目标车道前车在保守制动包络下仍有裕量。
* 目标车道后车在保守加速包络下仍有裕量。
* 完整变道和完成后的观察时域均安全。
* 跨线前同时存在继续变道与保持原车道制动两条可行方案。
* 跨线后继续完成变道本身仍对保守包络安全。

条件不满足时，唯一正常结果是继续车道巡航或跟车。

## 8. 候选生命周期与状态所有权

每周期先构造不可变 PlanningSnapshot。所有候选基于同一快照评估：

~~~text
PlanningSnapshot
  -> Generate candidates
  -> Build fixed path
  -> Build ST corridor
  -> Solve longitudinal QP
  -> Compose full trajectory
  -> Hard validate
  -> If valid candidates exist: rank and commit exactly one winner
  -> Else if valid evidence proves collision unavoidable: evaluate explicit minimum-risk dispatch
  -> Else: report InfrastructureFailure to the system safety supervisor
~~~

评估阶段不得修改：

* 目标车道和行为阶段。
* 历史纵向或横向状态队列。
* PathStitcher 已承诺状态。
* 控制规划器的 QP warm start。
* 上一条已提交轨迹。

CandidateEvaluation 必须同时返回候选结果和假定胜出后的 NextPlannerState。只有胜者通过 Commit 后，NextPlannerState 才成为下一周期状态。

MinimumRiskDispatch 不进入普通候选排名，也不调用普通 Commit。它使用独立 DispatchEmergency 接口，只更新紧急状态和被实际下发轨迹的继承信息。

## 9. 车道巡航的架构位置

车道巡航不是旧规划器回调，而是始终存在的候选生成器：

* LaneCruiseOnly 模式只评估车道巡航与当前车道制动候选。
* BehaviorActive 模式把车道巡航与变道候选一起比较。
* 变道收益不足或准入失败时，车道巡航自然胜出。

保留的是固定车道巡航、跟车、回中和纵向平滑行为。以下旧语义不构成可保留基线：

* 清除物理障碍约束以获得 QP 解。
* 发现 Cartesian 硬违规后仍返回该轨迹。
* 把所有历史路径无界地视为不可修改前缀。

## 10. 选择、承诺与回退

候选选择采用分层规则：

1. 输入和预测有效。
2. 物理硬约束全部满足。
3. 模式和行为状态允许该候选。
4. 变道候选通过保守准入。
5. 在剩余候选中比较运行裕量和总代价。

正常回退顺序：

1. 重新验证后仍安全的上一条已提交轨迹。
2. 当前车道巡航或跟车。
3. 当前车道保守制动。
4. 最大物理制动。
5. 输入、预测和 validator 证据完整且碰撞物理不可避免时，进入 CollisionUnavoidable 并选择 MinimumRiskDispatch；证据不足时进入 InfrastructureFailure。

变道已经承诺并跨线后，不得把“回到原车道”当作无条件回退。此时优先：

1. 继续已承诺变道。
2. 在目标间隙内减速完成。
3. 只有返回轨迹独立通过硬校验时才允许中止。
4. 最大物理制动；证据完整但仍无普通可行结果时进入 CollisionUnavoidable 和 MinimumRiskDispatch，证据不足时进入 InfrastructureFailure。

回滚配置不得撤销 hard-gate、有界历史前缀或物理碰撞边界。生产回滚必须切换到上一份经过验证的二进制与配置。

## 11. 最终硬校验

TrajectoryValidator 是候选提交前的门禁，至少检查：

* 保留前缀与新轨迹的完整时间连续性。
* 0.02 秒 Cartesian 速度、总加速度和总 jerk。
* 车辆矩形或更保守包络的碰撞。
* 相邻采样时刻之间的扫掠碰撞或等效膨胀。
* 车身角点是否位于道路边界内。
* 环路接缝。
* 预测覆盖时域和数据新鲜度。

ValidationResult 必须分别表达“是否完成验证”“是否通过全部硬约束”“哪些约束已不可满足”。MinimumRiskDispatch 可以拥有完成的 ValidationResult，但不得因此被标记为 valid。

监控器消费 ValidationResult 并记录结果，不再自行定义另一套安全真值。

## 12. 实时与可观测性

控制规划必须具备：

* 固定候选上限。
* 固定 QP 维度和可复用稀疏结构。
* 每次 QP 有迭代或墙钟上限。
* 控制路径不等待日志 I/O。
* 每候选淘汰原因、求解状态、验证结果和耗时。
* 模式、行为阶段、目标间隙和状态重置原因。

性能采集用于定位退化和容量规划，报告实际执行环境、样本数、平均值、P95、P99和最大值，但不作为硬件或绝对时延门槛。主动模式变更必须保持本文规定的安全语义、状态事务和模式隔离硬门。

## 13. 参数与开放问题

以下内容由后续回放和仿真标定，不属于架构决策：

* 前车保守减速度和后车保守加速度。
* 不确定性增长率。
* 准入稳定周期数。
* 前后时间头距和 TTC 阈值。
* 横向过渡长度样本。
* 行为代价权重。
* Top-K 和密集验证候选数量。

参数可以调整，但不得改变本文定义的单位、坐标、约束层次、候选事务和模式隔离语义。
