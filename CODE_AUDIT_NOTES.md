# Code Audit Notes

本文记录招聘整理期间发现、但因涉及飞行行为、协议、安全或工程配置而没有自动修复的问题。严重度表示“在下一次继续硬件/飞行前的处理优先级”，不是对历史飞行结果的推断。

## P0 — 下一次硬件/飞行前应处理

### 1. V307 frame parser payload 长度不一致

- 位置：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c::V307_AlarmPoll()`；调用点位于 `main.c` 主循环。
- 事实：协议注释描述 `0xBB + 4-byte payload`，当前状态机只跳过 3 个 payload byte；第 4 个 byte 可能被解释为 `0xCC` / `0xDD` 告警 tag。
- 风险：可能错误触发与停机相关的 V307 告警语义。
- 本轮动作：只报告，不修改。修复前需要协议样例、parser 单元测试和台架验证。

### 2. 没有通用 IMU freshness armed guard

- 位置：V5F shared publication 与 V3F `main.c` armed/hold 条件。
- 事实：generic `update_tick` 是主循环 heartbeat，不是 IMU frame sequence；V3F armed 保持条件未以独立 IMU freshness 为门槛。
- 风险：传感器接收停滞时，现有代码不能被描述为有通用 sensor-stale failsafe。
- 本轮动作：只报告，不修改 arming/failsafe/control behavior。

### 3. RC 字段命名与实际 stick 宏存在历史错位风险

- 位置：两侧 `shared_data.h` 字段与 V3F `STICK_*` 宏。
- 事实：`pitch` / `throttle` 的字段命名与当前 V3F 宏映射不完全直观。
- 风险：仅做“看起来正确”的重命名可能改变真实遥控通道。
- 本轮动作：不改。必须使用真实遥控器逐通道记录后再整理。

## P1 — 工程可靠性与可维护性

### 4. Shared SRAM 不是结构体级原子 snapshot

- 位置：两侧 `shared_data.h` 与 V5F/V3F producer/consumer。
- 事实：TOF 路径有 commit marker、fence 与读侧 double-check；其他数据通道没有完全相同的协议。
- 影响：消费者可能组合到来自相邻更新时刻的字段。
- 本轮动作：文档准确限定 TOF 路径，不改共享内存逻辑。

### 5. 在线参数命令校验不统一

- 位置：`V3F/User/main.c::CMD_Parse()`。
- 事实：部分 `strtof()` 结果没有统一拒绝 NaN/Inf、非法尾部或越界增益。
- 影响：调试命令可能把不合理值写入运行时控制参数。
- 本轮动作：不修改 PID/参数行为；建议先建立命令 parser 测试和各参数允许范围。

### 6. 没有 WCET / overrun 监测

- 位置：V3F TIM2 control tick、V5F TIM3 estimator tick。
- 事实：源码有固定周期配置，但没有 deadline miss、最大执行时间或 ISR 占用率统计。
- 影响：README 只能写 configured rate，不能写 timing guarantee。
- 本轮动作：只约束文档 claim。

### 7. 实验性 `XYKF` 命名容易引发误解

- 位置：`V5F/User/main.c`。
- 事实：当前实现是每轴 predictor/corrector 与启发式有效性/衰减逻辑，不具备可从代码确认的完整标准 EKF covariance propagation/update。
- 附带 warning：`XYKF_AxisCorrectVel()` 当前未使用。
- 本轮动作：文档称“experimental XY predictor/corrector”；未修改 sensor-fusion code。

## P2 — 无效/遗留代码与仓库卫生

### 8. V5F legacy PWM module — 已清理

- 原位置：`V5F/User/bsp_pwm.c`、`bsp_pwm.h`，以及 `bsp.h` include。
- 证据：object 可生成 PWM symbols，但最终 V5F ELF 中没有这些 symbols，linker garbage collection 移除了该模块。
- 结论：当前固件路径不使用它；最终电机输出属于 V3F。
- 最终动作：作者明确授权后删除该模块、`bsp.h` include 和 V5F `MOTOR_*` legacy macros；V3F 有效 PWM 实现未修改。删除后重新完整链接两核。

### 9. V5F legacy standalone TOF driver — 已清理

- 原位置：`V5F/User/bsp_tof.c`、`bsp_tof.h`。
- 证据：最终 V5F ELF 中没有 `TOF_*` symbols；当前高度 range 数据来自 `bsp_lf.c` 的 RANGE frame。
- 结论：该 standalone VL53-400 text driver 是当前入口未使用的遗留实现。
- 最终动作：作者明确授权后删除该模块和 V5F `TOF_*` legacy board macros。当前 `bsp_lf.c` RANGE frame、shared `tof_*` 字段与 V3F consumer 保持不变；删除后重新完整链接两核。

### 10. 当前入口未引用的公共 driver API

- 例子：部分 COMM、PWM、VOFA debug 与 NRF helper API。
- 事实：部分函数未进入最终 ELF。
- 判断：公共 driver API 可能是预留接口；“本次未调用”不足以证明“永远无用”。
- 本轮动作：不做批量删除。若要收窄 API，应先建立调用图和接口保留清单。

### 11. IDE metadata 已被 Git 跟踪 — 已清理

- 位置：根目录与工程目录的 `.mrs`、`.mrs-workspace`、`.snapshot`。
- 影响：增加噪声并可能携带机器状态。
- 最终动作：作者明确授权后删除 4 个 tracked workspace/snapshot 文件；`.gitignore` 继续覆盖未来生成内容。V3F/V5F 工程描述、linker/startup 配置未删除。

## 本轮唯一代码删除

- 删除 `V3F/User/main.c` 未引用宏 `VOFA_PERIOD_MS`。
- 静态验证：全仓无引用；宏删除不改变预处理后的有效控制路径。
- 运行行为：不改变。
- 硬件/飞行验证：未执行，也不需要据此声称任何飞行效果。
