# Ackermann 资产探针首次退出：ROS 浮点字段的零命令类型

## 现象与触发条件

2026-09-08 独立 Ackermann 资产首次平地检查中，Gazebo 已加载模型，临时 probe 在发布初始零命令时退出，尚未进入直行或圆弧阶段。原始记录：`/tmp/minco_ackermann_physics/run_20260908T211643_98a19ac4`。runner 正确报告失败并仅用 SIGINT 清理自己 PGID 452935，最终无残留。

## 由来与根因

临时 Python probe 的 `publish(v,d)` 直接把参数 v 赋给 `Twist.linear.x`。初始调用用了整数 `0`；Humble 生成的 Python ROS 消息 setter 要求该字段为 float，整数零也会触发类型断言。日志明确为 `The 'x' field must be of type 'float'`。

同一个 publish 函数又在 finally 的停车路径调用，第二次异常掩盖了首次捕获的异常结果，使首轮没有 `probe_result.json`。runner 根据实际 probe exit 1 仍判失败。这里的问题是**临时测试工具类型/收尾错误，不是 Ack 模型不能行驶或 Gazebo 停车失败**。资产 SDF 在前后两轮的 SHA256 一致。

## 修复作用点与证据

只修改 `/tmp/minco_ackermann_physics/probe.py`：写 ROS 消息前使用 `float(v)` 和浮点计算结果；finally 中独立捕获停车发送异常，再保存原始结果。没有修改正式 adapter、车型路由或任何资产参数。用已安装的 `geometry_msgs.msg.Twist` 验证 `float(0)` 赋值通过，然后全新启动相同资产的隔离试验。

第二轮目录：`/tmp/minco_ackermann_physics/run_20260908T211720_360536c0`。probe 与 runner 均 exit 0，9 类物理检查通过，真实最大关节速度 0.198 rad/s、中心速率 0.205485 rad/s；左右弯后全零停车回正通过。只 SIGINT 清理本轮 PGID 453064，最终无进程残留。首轮失败目录未重写或改成通过。

## 验证边界与后续排查

此次没有修复生产节点。若类似工具在发出第一条命令前退出，优先看 probe traceback 和 ROS 字段类型，再看消息是否真的到达 bridge，不能直接归因物理模型。finally 应始终保存结果，清理异常不能掩盖原始原因。临时脚本每轮快照与日志保留，可回看旧脚本重现；无需回滚正式资产。

这份材料只解释工具异常，Ack 资产几何、实测阈值及仍未完成的导航验收见[改造记录](../ai_refactor_records/20260908_ackermann_simulation_model.md)。
