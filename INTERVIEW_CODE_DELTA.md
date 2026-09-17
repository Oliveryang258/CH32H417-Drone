# Interview Code Delta

这份文件回答两个问题：哪些是比赛期间真实做过的，哪些是赛后为了提高工程质量补做的；面试中如何说得有含金量，但不把未验证功能包装成飞行成果。

## 一句话定位

> 这是一个真实搭建并飞行测试过、但没有通过线上初赛且没有完成完整任务闭环的本科双核无人机项目。我负责的核心价值是把传感器/无线链路和实时飞控拆到 CH32H417 双核上，建立固定周期级联控制、共享数据接口、四电机输出和调试链路；赛后又用源码审计补齐了构建、诊断和事实文档，并保留了尚未关闭的安全问题。

## 1. 比赛期间真实实现的主体

作者确认本人负责遥控器与飞机整体控制。以下内容可以作为个人项目主线；其中 IMU、光流和 NRF driver 是基于厂家例程修改和集成，不应表述为从零编写：

| 能力 | 源码证据 | 面试可用表述 | 验证边界 |
| --- | --- | --- | --- |
| CH32H417 双核职责拆分 | V3F/V5F 两套工程；两侧 `shared_data.h` | “我把确定性控制与异步传感器/通信拆到两个核心，并定义固定 SRAM 接口。” | 架构真实存在；当前 HEAD 的全部字段不等于全部飞过 |
| 固定周期级联控制 | `V3F/User/main.c::PID_Timer_Init/PID_Tick` | “用 TIM2 建立 150 Hz 角速度内环，Roll/Pitch 外环按 75 Hz 分频，避免 I/O 决定 PID dt。” | 频率可静态证明；没有 WCET/jitter 测量 |
| 四旋翼 X mixer 与 PWM 输出 | `PID_Tick()`、`bsp_pwm.c` | “把总距和三轴修正映射到四个电机，并在最终 PWM 层保留 armed gate 和 pulse clamp。” | 不主动展示/背诵当前参数作为最优参数 |
| IMU、光流/测距接入 | `V5F/User/bsp_imu.c`、`bsp_lf.c` | “处理不同串口协议、校验、坐标/单位、质量和更新时间，而不只是在变量里收到数。” | 光流/高度的当前闭环效果不能 claim 为已充分飞行验证 |
| NRF 遥控与数据回传 | `bsp_nrf.c`、`Bringup_LinkPoll/RefreshAckPayload` | “飞机作为 PRX 接收固定包，校验 magic/checksum，并用 ACK payload 回传传感器状态。” | 端到端时延和丢包率没有量化 |
| VOFA 在线调参与遥测 | `bsp_vofa.c`、`CMD_Parse()` | “建立 8 通道 JustFloat 遥测和多视图切换，缩短观察—修改—复测循环。” | 目前没有理想 VOFA 曲线，不能给出虚构响应指标 |
| 安全/状态路径 | `main()` armed state、`PWM_Arm/Lock`、overspeed/V307 checks | “实现过上电锁定、低油门解锁、RC 状态维持和若干紧急停机路径。” | 只能说逻辑已实现；没有完整 safety validation |

项目级事实可以说：**整机做过飞行测试，并有比赛演示视频。** 更精确的说法是“比赛时期的一版软硬件完成过飞行”，而不是“当前仓库 HEAD 的每个功能都飞过”。

## 2. 当前源码中存在，但不能自动归入‘比赛飞行验证’的功能

| 功能 | 当前状态 | 面试准确说法 |
| --- | --- | --- |
| 高度保持/高度保护 | 代码实现完整度较高，包含有效性、filter、进入/退出和 fallback | “做到了代码实现和软件联调层面；现有仓库证据不足以证明当前版本完成了稳定飞行验证。” |
| 光流速度/位置闭环 | 有质量、超时、source、bias、target 等逻辑 | “做过控制链尝试和问题排查；不要说已经实现稳定自主悬停。” |
| V5F XY-KF | TIM3 200 Hz 实验性 estimator | “这是当前源码里的实验性状态估计实现，不作为比赛飞行成果 claim。” |
| TOF commit-marker snapshot | V5F clear/fence/payload/fence/commit，V3F 双读确认 | “代码里已有针对 packed shared memory 撕裂风险的局部改进；是否属于实际飞行版本需要 commit/试验记录佐证。” |
| NRF ACK payload telemetry | 当前代码可见 | “协议和实现存在；没有测过完整时延分布和 RF 可靠性。” |

## 3. 本轮赛后工程化改进

这些内容应明确说成“后来复盘时改的”，反而能体现你会面对遗留代码，而不是削弱项目真实性。

| 改进 | 这次做了什么 | 证据 | 可 claim 的验证 |
| --- | --- | --- | --- |
| V5F build/timebase repair | 用实际 `SysTick1` 寄存器和 startup ISR 名替代不存在的 `SysTick_Config` | `Timebase_Init()`、`SysTick1_Handler()` | 当前源码 compile/link；**未上板、未复飞** |
| 构建入口 | 新增 PowerShell 脚本，在 MounRiver 已生成 Makefile 后显式完整构建两个核心 | `tools/build_firmware.ps1` | 本机双核 full rebuild PASS；不是独立 clean-clone build system |
| Shared ABI guard | 自动比较两侧 72 个字段和共享常量 | `tools/check_shared_abi.py` | source contract check PASS；不证明 runtime atomicity |
| UART/VOFA observability | 增加 RX overflow、TX drop 等计数；NULL boundary | `bsp_comunicate.*`、`bsp_vofa.*` | 编译/静态路径检查；未做硬件压力测试 |
| API/注释清理 | 修正 8-channel/36-byte、USART4/5、RC mode、heartbeat 语义，删除死函数 | 多个 V3F/V5F 文件 | 引用一致性和完整构建 |
| 审计文档 | 建立 source-of-truth、变更账本和 claim 边界 | 根目录 4 份文档 | 文档—源码交叉核对 |

统一标签：

`Post-competition engineering improvement; not flight-validated.`

## 4. 面试时怎么讲

### 60 秒版本

> 我做过一个 CH32H417 双核四旋翼。V5F 处理 JY61P、光流/测距和 NRF，V3F 用 TIM2 跑 150 Hz 角速度环、75 Hz 姿态环，再经过 X 型混控和 PWM 安全门输出。整机在比赛开发期间飞过，但线上初赛没有通过，我们也没有把每次飞行的 commit、参数和 VOFA 日志保存好，所以我不会给出没有证据的悬停精度。后来我重新审计了仓库，发现 V5F 当前分支时基不能链接、共享 heartbeat 注释错误、V307 parser 有潜在误报警等问题。我只直接修了低风险构建、接口、错误计数和文档；涉及 failsafe、PID 和遥控映射的问题单独列出，等待硬件测试后再改。

### 3 分钟技术版本

按照“约束—决策—问题—验证—反思”讲：

1. **约束**：同一 MCU 上既有固定周期飞控，又有多串口、光流、NRF 和打印等不确定耗时任务。
2. **决策**：V3F 保留最终控制权，V5F 负责 I/O；共享 SRAM 传递值、质量、marker 和诊断。
3. **控制链**：75 Hz angle P 生成 rate setpoint，150 Hz rate loop 生成三轴修正，最后 mixer/PWM gate 输出。
4. **真实坑**：控制周期不能被 I/O 决定；坐标/通道命名不能靠猜；`volatile` 不是跨核原子快照；遥测队列满时需要可观测。
5. **验证态度**：飞行视频证明项目确实飞过；compile 证明当前代码能链接；两者不能互相替代。没有保存的性能曲线就不报数。

### STAR 示例：控制周期重构

- **S**：传感器、串口和控制在同一执行路径，PID 周期受到 I/O 抖动影响。
- **T**：让控制频率可由硬件定时器定义，并保持传感器链路继续工作。
- **A**：将 V3F 控制迁移到 TIM2 150 Hz，中间用 2:1 分频运行姿态外环；把传感器/NRF 放在 V5F，遥测改为中断队列。
- **R**：形成了明确的 150/75 Hz 代码结构并参加整机开发；但没有保存 WCET、jitter 或量化飞行响应，所以结果只描述为“结构确定、完成飞行联调”，不描述为“达到某毫秒实时性”。

### STAR 示例：赛后源码审计

- **S**：仓库更像提交备忘录，README 与源码、当前分支与实际飞行版本混在一起。
- **T**：让招聘方可以构建、追源码并分清真实验证状态。
- **A**：做双核 full build、共享 ABI 对比、安全路径审计；修复 V5F link blocker、stale API 和 diagnostics；对行为改变项只立项。
- **R**：两个核心可完整链接，72-field ABI check 可重复执行，并形成 source-of-truth；赛后改动仍明确标记未上板/未复飞。

## 5. 私有演示视频的表述

原比赛演示视频保留为私有材料，不公开上传或在 README 放链接。面试时如需说明，可以准确表述“保留比赛时期演示视频，但因个人选择不公开；不以此声称量化性能”。

1. 不上传视频，不提供公开链接。
2. 不从视频推导悬停精度、稳定定高或其他量化指标。
3. 明确视频对应的 exact firmware commit 和参数未保存。
4. 当前仓库含赛后工程化修改，未对该 delta 复飞。

推荐 caption：

> Competition-era flight demonstration. This video provides qualitative evidence that the prototype was flown. The exact firmware commit and parameter snapshot were not preserved; post-competition engineering changes in this repository have not been flight-validated.

该 caption 只用于作者私下展示视频时，不放入公开 README 链接。

## 6. 不能 claim 的内容

面试和简历中不要写：

- “实现稳定自主悬停”（当前证据不足）；
- “当前代码完成飞行验证”（没有 HEAD 对应飞行记录）；
- “具备工业级/产品级可靠性”；
- “所有 failsafe 均经过故障注入”；
- “共享内存无竞争/完全原子”；
- “控制精度 ±X cm/deg、响应 X ms、抗风 X 级”（没有原始数据）；
- “V307 过流保护已经可靠验证”（当前 parser 还有 P0 长度问题）；
- “所有 PID 调参命令都有安全边界”（事实相反）；
- “V5F 赛后 SysTick 修复已经上板或飞行验证”。

## 7. 简历表述建议

### 保守版

> 基于 CH32H417 V3F/V5F 双核实现四旋翼飞控原型，拆分实时控制与 IMU/光流/NRF 任务；以 TIM2 构建 150 Hz 角速度环和 75 Hz 姿态环，完成 X 型混控、4 路 PWM、在线遥测与整机飞行联调。赛后补充 IDE 生成后的双核完整构建入口、共享 ABI 一致性检查和通信错误统计，并对未复飞改动进行显式标记。

### 面向嵌入式岗位

> 设计 CH32H417 双核固件边界和固定 SRAM 数据契约，接入多路 UART/SPI/NRF 中断链路；将控制周期从 I/O 路径中解耦，建立 150/75 Hz 定时控制。通过 source-level ABI check、ring-buffer overflow counters 和完整双核构建改善遗留工程可诊断性；保留 failsafe/共享快照等未验证风险清单。

## 8. 被追问时最有价值的回答

如果面试官问“比赛没过，为什么这个项目还有价值”，可以答：

> 比赛结果说明任务交付没有成功，我不会回避。但工程本身真实飞过，也留下了能追到源码的双核、控制、通信和安全问题。更重要的是，我后来没有把仓库只整理得好看，而是重新 full build、核对 startup vector 和共享 ABI，区分了能静态修的错误与必须重新上硬件验证的控制问题。这更接近真实嵌入式工作：不仅写新功能，也要知道证据在哪里、哪些修改不能靠编译成功就放行。
