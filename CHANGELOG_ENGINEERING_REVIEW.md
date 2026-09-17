# Engineering Review Changelog

审计日期：2026-08-29  
审计分支：`150HZ-inner-loop`  
审计起点：`e459503`  
目的：把真实比赛工程整理为可审阅、可构建、可解释的求职展示版本，同时保持飞行验证边界真实。

## 状态约定

| 标记 | 含义 |
| --- | --- |
| Competition / flight-era | 比赛开发与整机飞行时期形成的代码框架 |
| Post-competition | 比赛结束后的源码审计或工程化修改 |
| Software-verified | 仅完成编译、链接、静态检查或引用一致性检查 |
| Hardware-validated | 已在对应硬件上验证该具体改动 |
| Flight-validated | 已用对应源码/参数完成飞行验证 |

除非单项明确写明，否则本轮代码修改均属于：

`Post-competition engineering improvement; not flight-validated.`

## 审计基线

审计前执行 generated Makefile 的显式 `all -B -j1`：

- V3F：链接成功；无 project warning。
- V5F：链接失败；`SysTick_Config` 隐式声明且最终 undefined reference；`SysTick_Handler` 也没有对应 V5F 启动向量中的 `SysTick1_Handler`。
- 两侧 `SharedSensorData_t` 去除注释后字段声明一致，共 72 个字段。
- generated Makefile 的默认目标不是可靠的完整工程构建；必须显式调用 `all`。

## 已实施修改

### E-001 — 修复 V5F 1 ms 时基的链接和中断入口

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | `main()` 调用 SDK 不存在的 `SysTick_Config()`，完整链接失败；ISR 名为 `SysTick_Handler`，但 V5F startup vector 使用 `SysTick1_Handler`。 |
| 修改位置 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c`：`Timebase_Init()` / `main()`；`V5F/User/ch32h417_it.c`：`SysTick1_Handler()` |
| 修改内容 | 使用 CH32H417 的 `SysTick1` 寄存器配置 1 kHz 自动重装与中断；将 ISR 改为 startup vector 实际引用的符号；在所有会调用 WCH blocking delay 的外设初始化完成后才启动 timebase。 |
| 为什么修改 | 当前分支否则不能生成 V5F ELF，所有基于 `g_tick` 的超时/调度也没有可工作的中断入口。 |
| 是否改变运行行为 | **是。** 当前不可链接源码被恢复为具有 1 ms 时基的实现。它影响 V5F 主循环调度、RC timeout 和 ACK refresh。 |
| 验证方式 | WCH GCC12 完整编译/链接 PASS；startup symbol reference consistency check PASS；静态确认 IMU/NRF 初始化的 `Delay_Ms` 发生在 `Timebase_Init()` 之前。 |
| 硬件/飞行验证 | **未完成。** 必须上板测量 1 ms tick，并复核与 WCH `Delay_Us/Delay_Ms` 共用 SysTick1 的约束。 |

### E-002 — 清理 V307 UART 的死 heartbeat 接口

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | 头文件声明 `COMM_HeartbeatTick(uint32_t)`，实现却提供未声明、未调用的 `COMM_Tick(void)`；注释宣称自动每秒发送 `B`，实际主线没有调用。 |
| 修改位置 | `V3F/User/bsp_comunicate.h`、`V3F/User/bsp_comunicate.c` |
| 修改内容 | 删除未使用的 heartbeat 常量、声明、状态变量和 `COMM_Tick()`；把模块职责改为显式发送与接收。 |
| 为什么修改 | 消除 header/source/README 事实漂移，避免面试时把不存在的运行功能当成已实现。 |
| 是否改变运行行为 | 否；被删除函数和状态在当前工程中没有引用。 |
| 验证方式 | `rg` 引用检查；V3F 完整链接 PASS。 |
| 硬件/飞行验证 | 不需要声称；未做硬件复验。 |

### E-003 — V307 RX ring buffer 可见性、空指针边界和溢出统计

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | ISR 写、主循环读的 head/tail 未标 `volatile`；`COMM_RxRead(NULL)` 会解引用；缓冲区满时静默丢字节。 |
| 修改位置 | `V3F/User/bsp_comunicate.h/.c`：ring state、`COMM_RxRead()`、`COMM_GetDebugInfo()`、`USART5_IRQHandler()` |
| 修改内容 | head/tail 改为 `volatile`；增加 NULL guard；新增累计 RX byte/overflow counters 和只读查询接口。 |
| 为什么修改 | 提高 ISR/main 接口可读性和故障可观测性；溢出不再不可见。 |
| 是否改变运行行为 | 正常输入路径不变；非法 NULL 调用现在安全返回；ISR 多执行计数器自增。 |
| 验证方式 | V3F `-Wall` 完整编译/链接 PASS；接口引用检查。 |
| 硬件/飞行验证 | **未完成。** 尚未用串口突发流量触发 overflow。 |

### E-004 — VOFA 协议事实、const correctness 和死 API

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | 头文件写“4 float/20 bytes”，实际发送 8 float/36 bytes；`BSP_VOFA_Send()` 未使用 const；未引用的 4 通道 helper 会发送残留的后 4 通道。 |
| 修改位置 | `V3F/User/bsp_vofa.h/.c` |
| 修改内容 | 文档改为 8 通道 36 字节；参数改为 `const float *`；增加 NULL guard；删除未引用且语义危险的 `BSP_VOFA_SendJustFloat()`。 |
| 为什么修改 | 让协议说明与线上的实际帧完全一致，收紧接口语义，移除误导性死代码。 |
| 是否改变运行行为 | 当前所有有效调用路径不变；仅非法 NULL/未使用 API 的行为改变。 |
| 验证方式 | 全仓引用检查；V3F 完整链接 PASS。 |
| 硬件/飞行验证 | 未做蓝牙/VOFA 端到端复验。 |

### E-005 — VOFA RX/TX 错误统计

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | RX ring 满和 TX queue 空间不足时会静默丢数据/整帧，无法从调试信息判断遥测是否跟不上。 |
| 修改位置 | `V3F/User/bsp_vofa.h/.c`：`VOFA_DebugInfo_t`、queue/ISR counters、`BSP_VOFA_GetDebugInfo()` |
| 修改内容 | 增加 RX bytes、RX overflow、TX frames、TX drops 累计计数。 |
| 为什么修改 | 把“曲线断点究竟是控制问题还是传输丢帧”变成可观测问题。 |
| 是否改变运行行为 | 队列策略仍是“不阻塞控制、空间不足丢整帧”；只增加计数开销。 |
| 验证方式 | V3F 完整编译/链接 PASS；静态检查计数分支。 |
| 硬件/飞行验证 | **未完成。** 尚未用 VOFA 高负载验证计数准确性。 |

### E-006 — 共享内存字段语义修正

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | 两侧注释称 `update_tick` 为“每次传感器写入后递增/用于 freshness”，实际在 V5F hot loop 每轮递增；`rc_sw` 注释把 1 写成 Wait，实际常量为 1=Hover。 |
| 修改位置 | V3F/V5F `User/shared_data.h` |
| 修改内容 | 只修正注释：`update_tick` 是 V5F main-loop heartbeat；RC mode 是 0=Wait, 1=Hover, 2=Fly。 |
| 为什么修改 | 防止将 generic heartbeat 错当成传感器提交序号或安全门条件。 |
| 是否改变运行行为 | 否。 |
| 验证方式 | 两侧字段声明静态 ABI 对比 PASS。 |
| 硬件/飞行验证 | 不适用；未改 ABI。 |

### E-007 — 清理明确的编译 warning 和重复定义

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | V3F/V5F `bsp_pwm.c` 各有一个从未调用的阻塞 `PWM_RampTo()`；V5F `bsp_lf.h` 重复定义同一 invalid sentinel；V3F header 声称 `PWM_Arm` 有 3 s delay，实际没有。 |
| 修改位置 | 两侧 `bsp_pwm.c`、V3F `bsp_pwm.h`、V5F `bsp_lf.h` |
| 修改内容 | 删除未引用 ramp helper 和 stale delay constant；删除重复 macro。 |
| 为什么修改 | 清除项目 warning、死代码和虚假的解锁等待描述。 |
| 是否改变运行行为 | 否；删除项均无引用，没有修改 `PWM_Arm()`、限幅或 armed gate。 |
| 验证方式 | V3F warning 从 1 条降为 0；V5F PWM warning 消失；双核完整链接 PASS。 |
| 硬件/飞行验证 | 未做；控制和 PWM 运行路径未变。 |

### E-008 — stale comments 与接口说明

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | V3F TIM2 priority 注释写 USART1，实际安全告警 UART 是 USART5；overspeed 的 10 个 150 Hz tick 写成约 50 ms；V5F 文件头把 IMU 写成 USART1，实际驱动为 USART4。 |
| 修改位置 | V3F/V5F `User/main.c` |
| 修改内容 | 分别改为 USART5、约 66.7 ms、USART4。 |
| 为什么修改 | 让设计说明可从源码静态验证。 |
| 是否改变运行行为 | 否。 |
| 验证方式 | `rg` reference consistency check。 |
| 硬件/飞行验证 | 不适用。 |

### E-009 — IDE 生成后的双核命令行构建入口

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | generated Makefile 默认 target 可能只处理 startup object，且工具链依赖本机 PATH；容易误把局部构建当成全工程成功。 |
| 修改位置 | `tools/build_firmware.ps1` |
| 修改内容 | 新增 `-Core All/V3F/V5F`、`-Rebuild`，定位 GNU Make/WCH GCC 后显式调用各子工程 `all`；若缺少 MounRiver 生成的 `obj/Makefile`，给出明确错误。 |
| 为什么修改 | 在 IDE 已生成工程文件的前提下，用一个入口执行完整 compile/link，避免把默认 target 的局部构建当成全工程成功。该脚本不是独立 clean-clone build system。 |
| 是否改变运行行为 | 否；只影响构建流程。 |
| 验证方式 | `tools/build_firmware.ps1 -Core All -Rebuild` PASS。 |
| 硬件/飞行验证 | 不适用。 |

### E-010 — 双核共享 ABI source check

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | V3F/V5F 各复制一份 packed struct，任何单侧字段改动都会静默破坏后续字段解释。 |
| 修改位置 | `tools/check_shared_abi.py` |
| 修改内容 | 去除注释后比较 72 个字段声明以及 base/alarm constants，输出 contract digest 和明确免责声明。 |
| 为什么修改 | 把人工检查变成可重复的静态契约检查。 |
| 是否改变运行行为 | 否。 |
| 验证方式 | 当前输出 `PASS: shared ABI declarations match (72 fields, contract 9eb522c89215ad16)`。 |
| 硬件/飞行验证 | 不适用；该脚本不证明跨核快照原子性。 |

### E-011 — 求职展示和事实文档

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | README 把真实飞过的项目写成“未完成自由飞行”，同时又对 freshness/safety 作了超过源码证据的概括；缺少比赛代码与赛后改动的清晰边界。 |
| 修改位置 | `README.md`、`SOURCE_OF_TRUTH.md`、`INTERVIEW_CODE_DELTA.md`、本文件及 `docs/` 相关说明 |
| 修改内容 | 重建架构、职责、频率、验证证据、known limitations、构建、面试表述和逐源码索引。 |
| 为什么修改 | 让仓库从提交备忘录变成可快速评审、可追问、不过度 claim 的工程案例。 |
| 是否改变运行行为 | 否。 |
| 验证方式 | Markdown link/reference grep、事实与函数名对照、`git diff --check`。 |
| 硬件/飞行验证 | 不适用。 |

### E-012 — 招聘版 repository audit 与展示卫生

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | 首屏裸机属性不突出，Shared SRAM/safety/hardware/build 前提缺少独立导航；本地导出包和绝对路径降低公开仓库专业度。 |
| 修改位置 | `README.md`、`REPOSITORY_AUDIT.md`、`CODE_AUDIT_NOTES.md`、`docs/hardware-map.md`、`.gitignore`、`CLAUDE.md`、`tools/build_firmware.ps1`、`V3F/User/main.c` |
| 修改内容 | 按 11 节招聘阅读路径重构 README；新增审计、问题清单和 active hardware map；忽略本地输出/权属包；移除本机绝对路径；构建脚本增加 generated-Makefile guard；删除未引用 `VOFA_PERIOD_MS` 宏。 |
| 为什么修改 | 让招聘方先理解架构和证据边界，同时避免把本地材料、legacy code 或构建前提误当成公开项目能力。 |
| 是否改变运行行为 | 文档、ignore、构建 guard 均不改变固件；未引用宏删除不改变生成代码。 |
| 验证方式 | 双核 full rebuild、Shared ABI check、PowerShell syntax、Markdown links、secret/path scan、`git diff --check`。 |
| 硬件/飞行验证 | 未执行；本项不支持任何新增飞行性能 claim。 |

### E-013 — 作者事实确认与 legacy V5F 删除

| 项目 | 说明 |
| --- | --- |
| 修改前问题 | README 尚未确认个人分工/driver 来源；tracked `.mrs` 增加噪声；V5F 保存了两个已证明未进入最终 ELF 的 PWM/standalone TOF 实现。 |
| 修改位置 | `README.md`、审计文档、4 个 tracked `.mrs` 文件、`V5F/User/bsp.h`、`board_config.h`、旧 `bsp_pwm.c/.h`、旧 `bsp_tof.c/.h` |
| 修改内容 | 记录作者负责遥控器和飞机整体控制、driver 基于厂家例程修改、视频不公开；删除 tracked IDE state；删除 V5F legacy PWM/TOF 及其 include/专用宏。 |
| 为什么修改 | 准确表达个人 ownership，并移除会误导招聘方理解双核职责的未使用重复实现。 |
| 是否改变运行行为 | 按删除前 final ELF 证据，这两个模块未链接进固件；当前 LF RANGE 和 V3F PWM 路径未改。 |
| 验证方式 | 全仓引用检查、MounRiver source-discovery 配置检查、双核 full rebuild、ELF symbol check、Shared ABI check。 |
| 硬件/飞行验证 | 未执行；删除的是未链接模块，不据此增加任何 flight claim。 |

## P0 行为改变型问题：本轮只报告，未修改

| ID | 问题与证据 | 风险 | 为什么不直接改 | 后续验证建议 |
| --- | --- | --- | --- | --- |
| P0-01 | `V3F/User/bsp_height.c::V307_AlarmPoll()` 在 `0xBB` 后只消费 3 个 payload 字节，而协议说明为 X/Y 各 2 字节 | 第 4 个 payload byte 若为 `0xCC/0xDD`，可能被当作低压/过流；过流进入强制 disarm | 直接影响 failsafe/arming 行为，用户明确禁止未经批准修改 | 用 host parser test 覆盖连续帧、payload 含 tag、截断和噪声，再上板注入 UART 字节流 |
| P0-02 | V3F 记录 generic `update_tick` 最近变化时间，但它只进入 VOFA snapshot，不参与 armed 保持；PID 每 tick 直接读取共享 IMU | V5F/IMU 冻结时可能继续使用旧但数值正常的数据 | 新增 stale disarm 会改变 failsafe 和飞行状态机 | 定义独立 IMU sequence/timestamp；拆桨故障注入后再决定 freeze/degrade/disarm 策略 |
| P0-03 | V5F 写 `rc_pitch/rc_throttle`，V3F 宏把 pitch/throttle 使用关系做了历史映射 | 仅按字段名“修复”可能交换油门与俯仰 | 直接影响真实遥控输入与解锁条件 | 用遥控器逐通道动作 + telemetry/PWM-off 记录确认线序，再统一命名 |
| P0-04 | `CMD_Parse()` 直接 `strtof(line+2,NULL)`；多项 PID/外环 gain 无范围，未统一拒绝 NaN/Inf/非法尾部 | 串口脏数据或错误命令可能向控制器注入非有限参数 | 属于 PID 参数/运行时控制行为 | 先建立命令 parser host tests、参数 schema、armed 状态权限和 ACK，再实施 |

## P1/P2 保留项

- `V5F/User/main.c::XYKF_AxisCorrectVel()` 未使用。它位于 sensor-fusion 区域，本轮不为追求零 warning 而删除。
- WCH `EXAM/SRC/Debug/debug.c::USART_Printf_Init()` 有 2 条与编译配置有关的 unused warning。第三方 vendor 文件未修改。
- FLOW/XYKF 多字段发布没有和 TOF 等价的 commit-marker + read-retry 完整快照协议。
- V5F `LF_GetRangeSample()` seqlock reader 在 writer 永久停留 odd sequence 时没有有界退出。
- NRF ACK payload 的 3-level FIFO freshness 和实际端到端延迟没有测量。
- 赛后 `SysTick1` 时基与 future runtime `Delay_Us/Delay_Ms` 共用硬件 timer 的限制需要明确；当前主循环使用的 NRF RX/ACK 路径不调用这些 blocking delay，但未来启用 TX helper 前必须复核。

## 最终软件验证记录

| 检查 | 结果 |
| --- | --- |
| V3F clean rebuild + link | PASS；text 54,936 / data 1,260 / bss 3,272 bytes；0 project warning |
| V5F clean rebuild + link | PASS；text 26,008 / data 504 / bss 2,588 bytes |
| V5F residual warnings | 1 project warning（保留的 XYKF dead helper）+ 2 vendor warnings |
| Shared ABI declaration check | PASS；72 fields；contract `9eb522c89215ad16` |
| Hardware bench test for this delta | NOT RUN |
| Flight test for this delta | NOT RUN |

**结论：当前求职展示分支的软件可构建性和事实一致性得到改进；任何赛后运行时修复仍不能表述为硬件或飞行验证完成。**
