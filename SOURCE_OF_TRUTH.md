# Source of Truth

本文件是当前仓库的源码事实索引。它回答“功能在哪里、由谁调用、默认是否启用、能 claim 到什么验证程度”，而不是根据 README 反推代码。

基线：`150HZ-inner-loop` / `e459503` + 2026-08-29 engineering-review working tree。

## 如何读验证状态

| 值 | 含义 |
| --- | --- |
| Project-level flight evidence | 作者确认比赛时期整机飞过并有演示视频，但 exact commit/参数未保存 |
| Software checked | 当前源码已 compile/link、静态检查或引用检查 |
| Implemented, validation unproven | 代码存在，但当前仓库没有对应硬件/飞行证据 |
| Post-competition, not flight-validated | 本轮赛后工程化新增/修改，未上板或未复飞 |
| Open finding | 审计发现，因涉及关键行为而未修改 |

## V3F：飞控与最终输出

| 功能 | File path | Function / symbol | 当前状态 | 默认启用 | 飞行验证 |
| --- | --- | --- | --- | --- | --- |
| V3F 初始化与唤醒 V5F | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` | `main()` / `NVIC_WakeUp_V5F()` | Implemented / software checked | 取决于 `Run_Core` build config；双核工程为是 | 双核项目级有飞行证据；当前 HEAD 未逐 commit 证明 |
| 150 Hz control timer | 同上 | `PID_Timer_Init()` | TIM2 1 MHz counter，ARR=6666 | 是 | 飞行时期框架；当前定时 jitter/WCET 未测 |
| 150 Hz rate loop | 同上 | `PID_Tick()` / `PID_Update()` | Implemented / software checked | armed 后执行 | 项目级有飞行证据；当前参数曲线缺失 |
| 75 Hz Roll/Pitch angle loop | 同上 | `PID_Tick()`，每 2 tick | Implemented / software checked | armed 后执行 | 项目级有飞行证据；没有量化响应数据 |
| Yaw target/rate path | 同上 | `PID_Tick()` yaw sections | Implemented | armed 后执行 | 功能级 exact validation 不可从 repo 证明 |
| Rate feed-forward | 同上 | `g_roll_rate_ff/g_pitch_rate_ff` | Implemented，默认 gain=0 | 否（默认无贡献） | 不 claim 为飞行验证 |
| Yaw throttle feed-forward | 同上 | `g_yaw_ff_gain/g_yaw_ff_limit` | Implemented，默认非零 | 是 | exact current setting 未建立飞行证据 |
| X quad mixer | 同上 | `PID_Tick()` m1–m4 equations | Implemented；本轮未改 | armed 且非 test mode | 比赛飞行主体的一部分；current mapping 仍应结合机架证据 |
| Per-motor slew limit | 同上 | `pwm_slew()` / `g_motor_slew_us` | Implemented；本轮未改 | 正常混控默认是 | exact flight validation 未记录 |
| PWM TIM4 output | `V3F/User/bsp_pwm.c/.h` | `PWM_Init/Arm/Lock/SetAllPulseUs` | Implemented / V3F build PASS | boot locked；arm 后开 | 项目级有飞行证据；本轮 dead-code cleanup 未复飞 |
| Armed/disarmed state | `V3F/User/main.c` | `main()` state switch | Implemented；本轮未改 | boot disarmed | 项目级有飞行证据；未做系统化 fault injection |
| RC link loss disarm | 同上 + V5F shared status | `STATE_ARMED` condition | Implemented；依赖 V5F `rc_link_ok` | 是 | 逻辑存在；最大响应和异常核场景未验证 |
| Angular-rate watchdog | 同上 | `PID_Tick()` overspeed block | 500 deg/s × 10 control ticks（约 66.7 ms） | 是 | 逻辑存在；硬件故障注入证据不足 |
| V307 battery/overcurrent parser | `V3F/User/bsp_height.c` | `V307_AlarmPoll()` | **Open P0 finding：0xBB payload 长度处理不一致** | 轮询默认是；overcurrent 参与 disarm | **不能 claim 可靠飞行验证** |
| V307 UART ring buffer | `V3F/User/bsp_comunicate.c/.h` | `COMM_Init/RxRead/GetDebugInfo` | Implemented；overflow counters 为赛后新增 | 是 | counters 未硬件/飞行验证 |
| VOFA 8-channel telemetry | `V3F/User/bsp_vofa.c/.h` | `BSP_VOFA_Send/VOFA_Telemetry_Send` | 36-byte JustFloat frame；software checked | 是，默认 50 Hz/control view；需先收到 RX 才标 connected | 比赛有调试链路；本轮 counters/const delta 未复验 |
| VOFA TX/RX queue diagnostics | 同上 | `BSP_VOFA_GetDebugInfo()` | Post-competition | counters 自动开启；getter 未接 UI | 未硬件/飞行验证 |
| VOFA Commander parser | `V3F/User/main.c` | `CMD_Parse/CMD_Poll` | Implemented；部分参数有范围、部分无范围 | 是 | **Open P0：NaN/Inf/无界 PID 参数未统一拒绝** |
| Single-motor test | 同上 | `g_test_motor` branch | Implemented | 否，默认 0 | 仅可描述为测试路径；不要 claim 带桨验证 |
| Fixed-throttle override | 同上 | `g_thr_override` | Implemented | 否，默认 0 | 调试功能；不 claim 飞行验证 |
| Height estimator | `V3F/User/bsp_height.c` | `HeightEstimator_Update()` | TOF validation、tilt compensation、filter、commit read | 是（估计器运行） | 当前版本飞行验证不足 |
| Height hold | 同上 | `HeightControl_Update()` | 25 Hz pos / 50 Hz vel，含 entry/fallback | 否，`g_height_hold_enable=0` | Implemented, not flight-validated |
| Height guard | 同上 | height guard path in `PID_Tick()` | Implemented | 否，`g_height_guard_enable=0` | Implemented, not flight-validated |
| Optical-flow velocity assist | `V3F/User/main.c` | `PID_Tick()` flow block | quality/source/marker/timeout gates；默认 gains 为 0 | flag 默认开，但 gain=0 时无角度贡献 | 当前版本不 claim 稳定飞行效果 |
| Optical-flow position mode | 同上 | `g_flow_pos_enable` path | Implemented | 否 | Implemented, not flight-validated |
| Generic V5F heartbeat observation | 同上 | `sensor_seen_update_tick/local_ms` | 只用于 telemetry snapshot | 是 | **不是 armed IMU freshness guard** |

## V5F：传感器、无线和共享发布

| 功能 | File path | Function / symbol | 当前状态 | 默认启用 | 飞行验证 |
| --- | --- | --- | --- | --- | --- |
| V5F 1 ms timebase | `V5F/User/main.c`、`ch32h417_it.c` | `Timebase_Init()` / `SysTick1_Handler()` | Post-competition link/runtime repair | 是 | **未上板、未飞行验证** |
| IMU USART driver | `V5F/User/bsp_imu.c/.h` | `IMU_Init/IMU_IRQHandler/GetData` | USART4，frame parser + UART error counters | 是 | IMU 属于飞行主体；current HEAD exact validation 未证明 |
| LF optical-flow/range driver | `V5F/User/bsp_lf.c/.h` | `LF_Init/LF_IRQHandler/LF_GetData/LF_GetRangeSample` | USART2，checksum/length/UART errors，range seqlock | 是 | current feature-level flight validation 不足 |
| NRF24L01 driver | `V5F/User/bsp_nrf.c/.h` | `NRF_Init/Config/ReadRXPayload/WriteAckPayload` | SPI3 PRX + ACK payload | 是（NRF check 成功时） | 遥控项目级有飞行证据；当前 RF metrics 未测 |
| RC packet validation | `V5F/User/main.c` | `Bringup_LinkPoll()` | magic + XOR checksum，写 shared RC fields | 是 | 真实链路用过；字段名/通道历史映射需确认 |
| RC timeout | 同上 | `Bringup_Run()` timeout block | 500 ms 后 `rc_link_ok=0` | 是 | 逻辑存在；端到端 fault-injection 未记录 |
| ACK sensor payload | 同上 | `Bringup_LinkRefreshAckPayload()` | 50 Hz 填 32-byte payload 到 ACK FIFO | 是 | 实现存在；freshness/latency 未量化 |
| Shared main-loop heartbeat | 同上 | `g_shared_sensor.update_tick++` | 每个 hot-loop iteration 增长 | 是 | 仅 heartbeat，不是 sensor sequence |
| Shared IMU publish | 同上 | `Bringup_Run()` IMU ready blocks | 分字段写 roll/pitch/yaw/gyro/accel | 是 | 没有完整 snapshot/commit；通用 stale failsafe 缺失 |
| Shared FLOW publish | 同上 | `Bringup_Run()` flow block | payload + `flow_update_tick` | 是 | marker 不构成完整 seqlock snapshot |
| Shared TOF publish | 同上 | `Bringup_Run()` range block | clear marker → fence → payload → fence → commit | 是 | 代码级一致性设计存在；exact flight validation 未证明 |
| Shared LF debug mirror | 同上 | `Bringup_Run()` | counters/last-byte copy | 是 | diagnostic only |
| XY-KF 200 Hz | 同上 + `ch32h417_it.c` | `XYKF_TimerInit/XYKF_TickISR` | Experimental sensor-fusion implementation | 是 | **Implemented, not flight-validated** |
| OF2 bias calibration | `V5F/User/main.c` | `OF2_BiasCal*` / XYKF path | Implemented | 条件触发 | 不 claim 飞行验证 |
| Mechanical gripper | `V5F/User/hardware.c` 或相关 BSP；`main.c::MEG_Control` call | 根据 RC link + `rc_meg` 控制 | Implemented | 由 RC 命令触发 | 功能级飞行/任务验证未证明 |
| UART error handling | `bsp_imu.c`、`bsp_lf.c` | respective IRQ/debug counters | ORE/NE/FE/PE accounting 已存在 | 是 | software path exists；未做系统化 injection |

## Shared memory contract

| 功能 | File path | Function / symbol | 当前状态 | 默认启用 | 飞行验证 |
| --- | --- | --- | --- | --- | --- |
| Fixed base address | 两侧 `User/shared_data.h` | `SHARED_DATA_BASE_ADDR` | `0x20140000` | 是 | 比赛架构事实；地址安全仍依赖 linker/memory map |
| Packed contract | 两侧同文件 | `SharedSensorData_t` | 72 field declarations；`#pragma pack(1)` | 是 | source-level agreement only |
| ABI drift check | `tools/check_shared_abi.py` | `main()` | Post-competition；当前 PASS / contract `9eb522c89215ad16` | 手动/CI 调用 | 不涉及飞行验证 |
| TOF coherent snapshot | V5F `main.c` + V3F `bsp_height.c` | commit publish / retry read | 局部 commit-marker 协议 | 是 | 代码审计可证明写读顺序；硬件 stress test 未完成 |
| Generic coherent snapshot | N/A | N/A | **未实现** | 否 | 不可 claim |

## Build, tests and diagnostics

| 功能 | File path | Function / symbol | 当前状态 | 默认启用 | 飞行验证 |
| --- | --- | --- | --- | --- | --- |
| Dual-core build wrapper | `tools/build_firmware.ps1` | script entry | Post-competition；All/V3F/V5F + Rebuild；依赖 MounRiver 已生成 `obj/Makefile` | 手动运行 | build-only |
| V3F full link | `V3F/obj/Makefile` via wrapper | target `all` | PASS；0 project warning | 手动运行 | 不等于硬件验证 |
| V5F full link | `V5F/obj/Makefile` via wrapper | target `all` | PASS；1 project + 2 vendor warning | 手动运行 | 不等于硬件验证 |
| PID simulation | `tools/rate_pid_sim.py` | CLI | Existing analysis tool | 手动 | model/tool only，不替代 flight data |
| VOFA offline analysis | `tools/vofa_pid_advisor.py` | CLI | Existing tool；需真实 CSV | 手动 | 当前仓库没有完成 provenance 审查的最终 VOFA evidence set |

## Open findings that override optimistic README interpretations

1. `V307_AlarmPoll()` 的 `0xBB` payload 长度问题可能让 payload byte 被误当报警；过流报警进入强制停机路径。
2. `update_tick` 不代表 IMU freshness；当前 V3F 没有 generic IMU stale disarm/degrade 条件。
3. RC 实际控制映射必须看 `STICK_*` 宏，不要仅按 `rc_pitch/rc_throttle` 字段名推断。
4. `CMD_Parse()` 对 PID/angle gains 没有统一 finite/range/armed-state schema。
5. FLOW/XYKF/general IMU publish 不是完整 coherent snapshot。
6. V5F 赛后 SysTick1 修复 compile/link 通过，但未上板；未来若启用 runtime `Delay_Us/Delay_Ms` 路径，需要复核同一 SysTick1 被重配置的冲突。

## 更新规则

任何新功能进入 README 前，至少在此表补充：

- 实际 file/function；
- default enable condition；
- implemented / software checked / bench / flight 证据；
- exact commit、参数和硬件版本（若要 claim 飞行验证）；
- 若是赛后运行时改进，保留 `Post-competition engineering improvement; not flight-validated.` 标签，直到完成对应硬件/飞行验证。
