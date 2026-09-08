# 独立 Ackermann 仿真模型改造记录

## User Intent

用户要求开始 Ackermann 构型开发。此项先提供可独立验证的物理资产，不开放正式导航车型。模型参数是独立仿真原型，**不是真车标定值**：轴距 0.44 m、前后轮距 0.44 m、轮半径 0.076 m、中心转角上限 0.4 rad、中心转角速率上限目标 0.3 rad/s。后轴中心为 base 的 XY 原点，保持既有传感器坐标、测量接口及全向资产。

## Scope

simulation 子模块分支 `feat/alioth-minco-ackermann-simulation`，基线 `83357eb`。新增静态 SDF/model.config、独立静态测试和本记录；只在 `/tmp` 创建平地世界、临时 bridge 与物理探针。允许作者 alioth 在本地提交，不推送。

## Out of Scope

未修改现有 omni/diff 模型、CMake、正式 launch、bridge YAML、adapter、导航 YAML、planner/controller、地图或主仓库文档。未启动 MINCO/Nav2，不宣称 Ackermann 导航通过。没有实时生成 SDF 的生产脚本、新命令消息、额外控制节点或 Manager。

## Explorer Findings

### Files inspected

- `sentry_simulation/resource/models/sentry_omni/model.sdf`、`model.config`；既有传感器和 GT 来源。
- `sentry_simulation/sentry_simulation/cmd_vel_adapter.py`、`config/ros_gz_bridge.yaml`、`launch/simulation.launch.py`；现有 body-frame Twist 链路及车型拒绝边界。
- 安装的 Gazebo 6.18 AckermannSteering、JointStatePublisher、JointController、JointPositionController 与 DART 6.12.1 / ign-physics 5.4.0。
- 官方精确版本源码和本机 bridge 转换；上一轮完整可行性材料 `/tmp/minco_ackermann_actuation_feasibility.md`。
- 根 AGENTS.md/CONTRIBUTING.md；simulation 无嵌套同名规则。

### Active logic path

当前正式 launch 仍拒绝 Ackermann。新资产由临时平地世界直接 include。官方 AckermannSteering 独占两个后轮 rolling joint 和两个前轮 steering joint；前轮 rolling joint 被动滚动。JointStatePublisher 仅读取转向状态，没有其他执行系统写相同 joint。

### Data flow

临时 probe 的 `/sim/cmd_vel`（ROS Twist）经临时 bridge 到 `/sentry/cmd_vel`（Gazebo Twist）。linear.x 是后轴前进速度，angular.z 是 yaw rate，保持 `ω=v·tanδ/L` 语义。真实关节反馈 `/sentry/steering_joint_state` → `/sim/steering_joint_states`（JointState），明确包含 `front_left_steering_joint` 和 `front_right_steering_joint`。原生 OdometryPublisher 的 `/sentry/ground_truth_odometry` → `/sim/ground_truth/odom`；不是 Ack 插件积分里程计。

所有已有 LiDAR/IMU link（含 pose、sensor 配置）与 omni 一致，共同测量 XY 保持 `[0,-0.2]`。本物理试验未加载 GPU Sensors 系统，传感器数据流仅完成资产静态一致性验证，未做点云/定位验收。

### Risk notes

- Gazebo 6.18 的最小转弯半径为 `L/sin(steering_limit)`，故采用 `asin(tan(0.4))=0.436525365377011`；不能把插件参数直接写 0.4。
- 固定 P=1 转向跟踪存在滞后；全零 Twist 请求回正，不能保持任意零速转角。`|ω|<0.001 rad/s` 会被插件视作直行。
- 原型保持后轴原点，body 惯性、视觉、碰撞中心移到 x=0.22；传感器仍保留原 model 坐标。因此外观传感器位置并非实车结构设计结论。
- 官方 JointStatePublisher 精确 6.18 不读取 update_rate；只选两个 steering joint，物理试验记录约 1 kHz 反馈。后续集成需要评估该频率成本。
- 控制速度限幅与实际物理反馈分别验证，不以命令限值冒充实测速率。

### Recommended modification boundary

先固定新模型几何、单一执行器所有权和反馈接口。未来 planner/controller 按同一几何推导可跟踪速率并考虑 P=1 滞后；正式车型路由由后续成对集成任务另行完成。

## Modifier Changes

### Files changed

1. `sentry_simulation/resource/models/sentry_ackermann/model.sdf`
2. `sentry_simulation/resource/models/sentry_ackermann/model.config`
3. `sentry_simulation/test/test_ackermann_model.py`
4. 本记录。
5. [临时物理探针知识文档](../knowledge/20260908_ackermann_probe_typed_zero.md)。

### Key changes

四个普通圆柱轮，rear x=0/front x=0.44，y=±0.22，轮中心 z=0.076。两个独立前转向 knuckle，滚动轴随 knuckle 转动。驱动为官方 AckermannSteering；加入两个真实转向关节的 JointStatePublisher，保留原生 GT。

机械角度与速率没有直接照抄中心 0.4/0.3：

| 量 | 数值及来源 |
|---|---|
| 中心命令角上限 | 0.4 rad |
| 最大内轮转角 | 0.4921314267 rad，Ackermann 几何 |
| 每关节机械角度 | ±0.52 rad，较最大内轮角留 0.0278685733 rad |
| 转向关节 effort | 有限 50 N·m，供 DART SERVO 限制生效 |
| 中心角至内轮角最大 Jacobian | 1.4722475105，中心角域 ±0.4 |
| 保守最低 Jacobian | 0.6698262005，将机械余量角域扩至 ±0.52 计算 |
| 对应关节速率上界 | 0.3×J_min=0.2009478601 rad/s |
| 实际 SDF 每关节 velocity | 0.198 rad/s，较上述上界留约 1.47% 数值余量 |

这会降低可跟踪的中心速率：中心角 0 附近关节速度允许约 0.198 rad/s；中心角 0.4 处，按最大内轮 Jacobian，约 0.1344889 rad/s。**0.3 是本轮中心速率目标上限，不是全角域瞬时可实现的跟踪速率。** 后续 controller 不得仍把 0.3 当作无执行器滞后的恒定可用速率。本轮物理探针用 4 s 将中心目标从 0 斜坡到 ±0.4（目标 rate=0.1），再保持、全零停车回正。

### Behavior preserved

原正式入口仍只开放既有车型。omni/diff 和现有 topic/frame/传感器外参、watchdog、launch、bridge 参数均未修改。前向速度与 yaw rate 的 Twist 语义不变。GT 仍由原生真值系统提供。

### Behavior intentionally adjusted

仅新增独立 Ackermann 资产与新转向状态话题。新模型不具备全向侧移或原地旋转；全零命令会使轮速停下并渐进回正。上述边界尚未进入正式导航路由。

### Notes

性能：复用已装官方系统，增加一个仅选两关节的状态发布；高频状态成本留待集成测量。结构：静态模型与独立测试，无自定义执行框架。功能：后轮驱动、前轮独立转向。扩展性：明确几何和真实反馈供后续模型复用。接口：保留 Twist 与传感器/GT；只新增所需 JointState。正确性：几何、控制归属、限制、实测停车与曲率均有证据。

## 验证命令与实际结果

### 静态与 TDD

均在 simulation 子模块运行，未执行 colcon：本次仅新增 XML 资产和 Python 静态测试，不涉及编译包；实际物理试验直接加载该源 SDF。

```bash
python3 -m unittest discover -s sentry_simulation/test -p test_ackermann_model.py -v
ign sdf -k sentry_simulation/resource/models/sentry_ackermann/model.sdf
python3 -m unittest discover -s sentry_simulation/test -p test_kinematics.py -v
git diff --check
```

- 缺资产时 6 项失败，原因均为预期的 missing model，日志 `/tmp/minco_ackermann_asset_red.log`。
- 新资产后 6 项通过，日志 `/tmp/minco_ackermann_asset_green.log`。
- 按父任务更严格的停车回正速率要求先更新测试，旧 0.46 rad/s 机械 rate 出现 1 项预期失败，`/tmp/minco_ackermann_rate_red.log`；改为推导后的 0.198，6 项通过，`/tmp/minco_ackermann_rate_green.log`。
- SDF 官方校验 `Valid.`，日志 `/tmp/minco_ackermann_sdf_check.log`。
- 既有 kinematics 7 项通过。
- `.gitignore` 的 `test/`、`docs/` 会忽略新增测试与记录，因此只对明确授权的测试、本记录及知识文档使用 scoped `git add -f`。CMake 没有修改，本项测试用上述命令执行，尚未新增 ament 注册。

### 隔离与原始材料

运行命令 `python3 /tmp/minco_ackermann_physics/run.py`。每次创建独立目录保存 SDF 快照/hash、世界、bridge YAML、probe/runner 源码快照、进程命令、所有原始事件和结果。没有生成脚本进入 Git。

两轮均使用 `ROS_DOMAIN_ID=228`、`ROS_LOCALHOST_ONLY=1`、唯一 IGN_PARTITION、Fast DDS，未设置 LD_PRELOAD；启动前 `/proc` 环境扫描未发现该域进程，64400..64649 UDP 端口无占用。成功轮实际 ROS graph 只有 probe 与临时 bridge。所有仿真/bridge/probe 是新 session/PGID 的子进程，清理只针对该 PGID，未触碰其他节点或构建进程。

1. `/tmp/minco_ackermann_physics/run_20260908T211643_98a19ac4`：临时 probe 的整数 0 赋 ROS 浮点字段触发类型断言，尚未发起行驶。原始失败保持不变；runner overall false，仅 SIGINT 清理 PGID 452935，最终无残留。见独立知识文档，**不是资产物理故障**。
2. `/tmp/minco_ackermann_physics/run_20260908T211720_360536c0`：修正临时 probe 类型和 finally 结果保存后全新启动，SDF 完全不变。probe exit 0、runner exit 0、9 类物理断言全部通过。PGID 453064 仅 SIGINT、leader exit 0、最终进程列表为空。

成功轮 SDF SHA256：`28ce3d26a257e148d0a411735c37badea2e3230dd3911cc2ebb38813c3988a37`。原始材料见该目录的 `events.jsonl`、`probe_result.json`、`manifest.json`、`gazebo.log`、`bridge.log`、`asset.sdf`。Gazebo 6.18，临时世界明确选择 DART，平地无导航/定位。

### 成功轮实测摘要

阶段：静止 2 s → 0.3 m/s 直行 6 s → 零停 3 s → 0.2 m/s 左转斜坡 4 s、保持 12 s → 零停回正 8 s → 右侧同样一组。稳态指标取各阶段最后 2 s。

| 检查 | 实测 |
|---|---|
| 直行位移 | 1.787080 m，稳态速度 0.29999996 m/s，yaw rate 约 3e−10 rad/s |
| 左弯中心角 | 0.399997997 rad |
| 右弯中心角 | −0.399998018 rad |
| 左弯真值曲率 | 0.930974277 m⁻¹，理论 0.960893679，约 3.11% 偏差 |
| 右弯真值曲率 | −0.960795247 m⁻¹ |
| 最大实测关节角 | 0.492130577 rad，低于机械 0.52 |
| 最大实测关节速度 | 0.1979999993 rad/s，未靠提高限值过验收 |
| 最大重建中心转角速度 | 0.205485124 rad/s，包含全零回正阶段，低于 0.3 |
| 转向原始样本 | 58,778 条（不含 ready 阶段） |
| 左停终段最大 GT 速度 / yaw rate | 1.2374e−5 m/s / 2.1207e−4 rad/s |
| 左停终段最大回正误差 | 0.001624174 rad |
| 右停终段 GT 速度 / yaw rate | 0 / 0 |
| 右停终段最大回正误差 | 0.001587246 rad |

每个物理断言显式检查实测数据：直行位移/低 yaw rate，左右曲率符号与误差、角度目标、停车速度与回正、机械角度和实测速率。最大 joint rate 判据允许 0.01 rad/s 测量余量，但实际峰值未超过 SDF 0.198；中心 rate 判据为 ≤0.3。反馈全程有更新，未出现持续超过 1 s 的接收缺口。

## Auditor Review

### Checks performed

Modifier 自检已执行，独立审计由父 Agent 安排：

- [x] 关键路径与单一关节所有权检查
- [x] scoped diff / git diff --check
- [x] 参数与 topic / joint 引用检查
- [x] XML 与几何/限制/传感器契约检查
- [x] 6 项新静态测试、7 项既有纯 kinematics 测试
- [x] 记录无 colcon 的原因；记录隔离物理进程、命令与原始失败/成功日志
- [ ] 父 Agent 独立审阅最终提交

### Issues found

原型不存在已观察到的资产物理故障。临时探针首次类型错误已定位并留知识材料，未改写首轮失败。普通轮胎左弯约 3.11% 的稳态曲率偏差在本次 25% 验收界内；未为缩小偏差调摩擦或改资产。该偏差及 P=1 滞后需供后续 controller 使用实际反馈处理。

### Final result

**PASS（Modifier 限域验证）；独立 Auditor 尚待父 Agent 审阅。**

未验收：MINCO Ack 规划/控制、正式 launch/YAML 路由、传感器点云/定位、障碍物环境、倒车/泊车、任意零速持角或真实车辆。没有推送、合并或创建 PR。
