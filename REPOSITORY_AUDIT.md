# Repository Audit

审计日期：2026-09-16  
审计对象：当前工作区中的 CH32H417 V3F/V5F 裸机工程  
审计目标：恢复源码事实、识别招聘展示障碍，不把“代码存在”“编译通过”写成硬件或飞行验证。

> 本文是代码审计记录，不是性能证明。当前工作区还包含赛后工程化改动；比赛版本、当前源码和真实飞行证据之间的差异见 `INTERVIEW_CODE_DELTA.md`。

## 1. 工程结构与真实入口

| 领域 | 核心 | 入口 / 关键函数 | 文件 |
| --- | --- | --- | --- |
| 启动与初始化 | V3F | `main()`、`PID_Timer_Init()` | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` |
| 控制定时中断 | V3F | `TIM2_IRQHandler()` → `PID_Tick()` | `V3F/User/ch32h417_it.c`、`V3F/User/main.c` |
| 解锁、停机与控制 | V3F | `main()` 状态机、`PID_Tick()` | `V3F/User/main.c` |
| PID / 控制器状态 | V3F | PID 初始化、更新、复位 | `V3F/User/bsp_pid.c`、`bsp_pid.h` |
| 高度链路 | V3F | `HeightControl_Update()` 及保护/诊断 | `V3F/User/bsp_height.c`、`bsp_height.h` |
| Mixer 与电机目标 | V3F | `PID_Tick()` 内 X 型混控、slew、clamp | `V3F/User/main.c` |
| PWM / ESC | V3F | `PWM_Init()`、`PWM_Arm()`、`PWM_SetAllPulseUs()`、`PWM_Lock()` | `V3F/User/bsp_pwm.c`、`bsp_pwm.h` |
| VOFA / 调参 | V3F | `VOFA_Telemetry_Send()`、`USART3_IRQHandler()`、`CMD_Parse()` | `V3F/User/bsp_vofa.c`、`main.c` |
| V307 通信 | V3F | `COMM_Init()`、`USART5_IRQHandler()`、主循环解析 | `V3F/User/bsp_comunicate.c`、`main.c` |
| 启动与传感器编排 | V5F | `main()`、`Bringup_Run()`、`Timebase_Init()` | `V5F/User/main.c` |
| IMU | V5F | `IMU_IRQHandler()`、JY61P frame parser | `V5F/User/bsp_imu.c`、`bsp_imu.h` |
| 光流 / 测距 | V5F | `LF_IRQHandler()`、`LF_DecodeFrame()` | `V5F/User/bsp_lf.c`、`bsp_lf.h` |
| RC radio | V5F | `Bringup_LinkPollRC()`、NRF driver | `V5F/User/main.c`、`bsp_nrf.c`、`bsp_nrf.h` |
| 实验性 XY 估计 | V5F | `TIM3_IRQHandler()` → `XYKF_TickISR()` | `V5F/User/ch32h417_it.c`、`main.c` |
| Shared SRAM | 双核 | `SharedSensorData_t`、`g_shared_sensor` | 两侧 `User/shared_data.h` |
| 芯片支持与链接 | 双核 | startup、WCH peripheral library、linker scripts | `EXAM/SRC/` |
| IDE 工程 | 双核 | MounRiver workspace / projects | `EXAM/GPIO/GPIO_Toggle/GPIO_Toggle.wvsln`、两侧 `.wvproj` |

路径表中的 `V3F/User` 和 `V5F/User` 均相对于 `EXAM/GPIO/GPIO_Toggle/`。

## 2. Architecture Audit

### 2.1 执行模型

这是 **bare-metal / 裸机** 工程，没有 FreeRTOS、RT-Thread、任务线程或 RTOS scheduler。

- V5F 主循环进行无线链路、传感器数据搬运和共享数据发布；USART2/USART4 中断接收光流/测距和 IMU 数据；TIM3 以代码配置的 200 Hz 运行实验性 XY predictor/corrector。
- V3F 主循环处理状态机、命令和遥测；TIM2 以 6667 us 周期触发 `PID_Tick()`，最终控制计算和电机目标更新发生在该固定节拍中。
- 中断与主循环通过静态状态、环形缓冲区和固定 Shared SRAM 区域交换数据，没有 RTOS 同步原语。

### 2.2 双核职责

| 核心 | 当前源码职责 | 明确边界 |
| --- | --- | --- |
| V5F | JY61P IMU、匿名光流/测距帧、NRF24L01+ 遥控与 ACK payload、诊断字段、共享数据发布、实验性 XY 状态估计 | 不拥有最终 armed 决策；最终 ELF 不包含 V5F 旧 PWM 模块 |
| V3F | 解锁/停机、RC 与 V307 告警路径、姿态/角速度控制、光流与高度控制接入、X 型混控、4 路 PWM、VOFA | 不直接解析主要 IMU/光流/NRF 底层协议 |

### 2.3 数据与执行链

```text
JY61P / optical-flow+range / NRF RC
                 │
                 ▼
V5F IRQ + main-loop parsing/publication
                 │
                 ▼
SharedSensorData_t @ 0x20140000
                 │
                 ▼
V3F safety/state machine + multi-rate control
                 │
                 ▼
X mixer → per-motor slew/clamp → TIM4 PWM → ESC/motors
```

旁路包括：V307 → V3F 告警/视觉通信；V3F → USART3 VOFA telemetry/Commander；NRF ACK payload 由 V5F 周期更新。

### 2.4 多速率调度

| 功能 | 代码配置频率 | 实现方式 |
| --- | ---: | --- |
| V3F rate loop | 约 150 Hz | TIM2 每 6667 us 调用 `PID_Tick()` |
| Roll/Pitch attitude loop | 约 75 Hz | `PID_Tick()` 内每 2 tick 分频 |
| flow velocity-to-angle | 约 50 Hz | 每 3 tick 分频 |
| flow position | 约 25 Hz | 每 6 tick 分频 |
| height velocity | 约 50 Hz | `HeightControl_Update()` 分频 |
| height position | 约 25 Hz | `HeightControl_Update()` 分频 |
| V5F XY predictor/corrector | 200 Hz | TIM3 ISR；实验性、未飞行验证 |
| V5F timebase | 1 kHz | SysTick1；赛后修复、未硬件/飞行验证 |

这些数字是代码配置值，不是 WCET、jitter 或 deadline 测量结果。

### 2.5 Shared SRAM 一致性

- 两核把 `SharedSensorData_t` 映射到固定地址 `0x20140000`；当前两侧声明由静态 ABI 脚本核对。
- 当前 TOF 字段实际由匿名光流/测距模块的 RANGE frame 发布。该路径使用 commit marker、memory fence 和读侧 begin/end double-check。
- 不能把 TOF 的一致性协议推广为整个结构体的原子 snapshot。IMU、RC、FLOW 和其他字段没有完全相同的提交协议。
- generic `update_tick` 是 V5F 主循环 heartbeat，不等于 IMU frame sequence，也不是 armed 状态的通用 sensor-freshness guard。

### 2.6 Safety 路径

源码中可确认：上电锁定；Fly 模式、RC link、低油门、无 V307 over-current 条件下尝试解锁；500 ms RC 超时；高角速度连续计数保护；强制停机清理；PWM armed gate；操作油门上限、每电机硬限幅和 slew limit。

源码不能证明：通用 IMU stale failsafe、sensor freshness arming check、WCET/overrun monitoring、生产级 fault tolerance。详见 `CODE_AUDIT_NOTES.md`。

## 3. 招聘展示审计

| 招聘方问题 | 审计前状态 | 本轮处理 |
| --- | --- | --- |
| 30 秒内能否定位项目价值 | 有信息但首屏偏长，裸机属性不突出 | README 首屏改为一句话、架构图、5 个 highlights 和验证边界 |
| 双核职责是否直观 | 有角色表，但路径导航不足 | 增加职责、执行模型和“想深挖看哪里” |
| Shared SRAM 是否容易找到 | 有提及，无独立章节 | 增加专节并说明 TOF-only 一致性边界 |
| 控制频率是否明确 | 已列频率 | 明确标注为 code-configured，而非时序测量 |
| safety 是否明确 | 分散在长文和限制中 | 汇总实际存在的门控、超时、限幅和诊断 |
| 如何编译是否明确 | helper script 描述过度接近 clean build | 改为 IDE-first；脚本仅在生成 `obj/Makefile` 后可用 |
| hardware mapping 是否明确 | 主要散落在驱动和 board config | 新增 `docs/hardware-map.md`，区分 active 与 legacy 声明 |
| 临时文件是否影响阅读 | tracked `.mrs`、本地输出/权属包存在 | 本地目录加入 ignore；经作者授权删除 tracked IDE metadata |

## 4. Security / Hygiene

### 未发现

- 未发现私钥、Wi-Fi 密码、API key、GitHub/OpenAI/AWS token 形态。
- 未发现邮箱、手机号、身份证号或私网 IP 明文。
- `git ls-files` 未发现需要公开的固件二进制、巨型日志或构建产物。

这只是模式扫描，不等同于完整秘密检测或历史提交扫描。

### 已处理

- `CLAUDE.md` 中两个本机绝对路径改为仓库相对/外部仓库描述。
- `.gitignore` 增加 `/output/`、`/software_copyright_package/`、`/tmp/`；文件未删除。
- 构建 helper 在缺少 MounRiver 生成 Makefile 时给出明确错误，避免把当前机器的生成目录误当成 clean-clone 能力。

### 当前处理结论

- 已跟踪的 `.mrs` / `.mrs-workspace` / `.snapshot` IDE 状态文件经作者授权删除；未来生成文件继续由 `.gitignore` 排除。
- `software_copyright_package/` 含权属与申请工作材料及重复源码，不应进入公开仓库；当前已 ignore。
- `tools/output/` 和 `output/` 中有 VOFA 数据、接触表和源码图，但缺少清晰 provenance，不应直接作为性能证据。
- `tools/vofa_attitude_advisor.py` 的作者与来源需要确认后再决定是否公开展示。

## 5. Unused Code Audit

通过最终 ELF symbol 检查确认：V5F 的旧 `bsp_pwm.c` 与 standalone `bsp_tof.c` 整个模块均未进入最终 V5F ELF。当前测距来自 `bsp_lf.c` 的 RANGE frame，最终电机 PWM 由 V3F 实现。

在作者后续明确授权后，已删除这两个遗留模块、`bsp.h` 的旧 include 和对应 V5F board macros，并重新完整链接 V5F。当前有效的 LF RANGE → shared `tof_*` 路径没有删除。其他被 linker garbage collection 删除的公共辅助 API 没有批量清理，因为仅凭当前入口未调用，不能证明它们不是有意保留的可复用接口。

本轮只删除了 V3F `main.c` 中无任何引用的 `VOFA_PERIOD_MS` 宏；该删除不改变生成代码或运行行为。

## 6. 最值得补充的 3 张图

1. **真实四旋翼实机照片**：当前仓库没有；应由作者提供，不使用生成图或网图替代。
2. **双核软件架构图**：README 的 Mermaid 已可用，后续可导出一张适合简历/社交平台预览的 SVG/PNG。
3. **有 provenance 的 telemetry / control 截图**：只有以后找到来源明确的数据时再补；不为了展示使用来源不清的曲线。比赛演示视频为私有材料，不在公开仓库链接。

现有 contact sheet 主要显示光流模块配置界面，不适合作为“飞控控制效果”证据；现有源码图行号已可能随赛后改动失效。

## 7. 作者确认与主动省略

- 比赛演示视频不公开，仅作为作者保留的定性证据；不补公开链接。
- 作者负责遥控器与飞机整体控制。
- IMU、光流与 NRF driver 基于厂家例程修改实现，不 claim 为从零原创。
- 机架、电机、桨、电池和 ESC 精确型号无法可靠回忆，公开文档主动省略。
- 不再筛选来源不明的 VOFA 数据作为展示素材。
- 作者已授权删除 tracked `.mrs` metadata 和未链接的 V5F PWM/standalone TOF 模块。

## 8. P0 / P1 / P2 整理计划

- **P0：事实边界**——明确 bare-metal、双核职责、TOF-only consistency、构建前提、真实验证与非验证内容。
- **P1：招聘阅读路径**——README 重构、源码入口索引、hardware map、审计 notes、local-output ignore。
- **P2：后续可选项**——修 V307 parser、增强 IMU freshness、在线参数校验；只有取得真实素材时才补实机图或可追溯曲线。V5F 旧 PWM/TOF 与 tracked IDE metadata 已清理。
