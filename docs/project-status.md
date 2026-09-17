# 项目范围与验证状态

## 背景

本项目源于本科嵌入式竞赛。作者确认整机在比赛开发阶段完成过飞行测试，并保留比赛演示视频；项目未通过线上初赛，之后没有继续完成竞赛任务闭环。

仓库目前同时包含三种时间层次：

1. 比赛/飞行时期形成的双核、控制、传感器、遥控和调试主体；
2. 后续继续写入当前分支、但无法从仓库证明参加过飞行的实验性功能；
3. 2026-08-29 源码审计后的工程化修复。

由于没有保存“演示视频—commit—参数—硬件版本”的精确映射，项目级飞行事实不能自动扩展为当前 HEAD 每项功能的 flight validation。

## 状态定义

| 状态 | 含义 |
| --- | --- |
| Implemented | 代码路径存在于当前工程 |
| Software checked | 完成 compile/link、静态或引用检查 |
| Project-level flight evidence | 比赛时期整机飞过，但 exact source baseline 未保存 |
| Feature flight-validated | 该具体功能、源码、参数和硬件有对应飞行证据 |
| Post-competition / not flight-validated | 赛后工程化修改，尚未上板或复飞 |

## 功能矩阵

| 功能 | 当前状态 | 默认启用 | 证据边界 |
| --- | --- | --- | --- |
| V3F/V5F 双核拆分 | Implemented / software checked | 是 | 属于真实飞行项目主体；current HEAD 未逐 commit 证明 |
| 150 Hz rate / 75 Hz angle loop | Implemented / software checked | armed 后 | 比赛时期控制框架；没有保留量化飞行曲线 |
| X mixer 与 4 路 PWM gate | Implemented / software checked | armed 后 | 项目级有飞行证据；本轮未改关键行为 |
| NRF 遥控与 500 ms timeout | Implemented | 是 | 真实遥控链路的一部分；未保存丢包/时延数据 |
| IMU 接入 | Implemented | 是 | 飞行主体；当前无 generic IMU stale failsafe |
| 光流速度/位置路径 | Implemented | flow flag 默认开、position 默认关；默认 gains 为 0 | 当前实现不 claim 稳定悬停或充分飞行验证 |
| 高度估计/保持 | Implemented | estimator 运行、hold 默认关 | 当前实现不 claim 充分飞行验证 |
| XY-KF 200 Hz | Implemented / experimental | 是 | 不 claim 为已飞行验证 |
| V307 过流停机链路 | Implemented / open P0 finding | parser 默认轮询 | parser payload 长度问题关闭前不能 claim 可靠保护 |
| V5F 1 ms SysTick1 | Post-competition / software checked | 是 | compile/link PASS；未上板、未复飞 |
| Shared ABI check | Post-competition / software checked | 手动运行 | 证明声明一致，不证明 runtime atomicity |
| 自主悬停 | 证据不足 | N/A | 不作为成果声明 |
| 完整竞赛任务 | Not completed | N/A | 线上初赛未通过 |

## 可以公开陈述

> 设计并实现 CH32H417 双核四旋翼飞控工程，将固定周期控制与传感器/无线链路拆分，建立 150/75 Hz 级联控制、共享 SRAM 接口、四电机输出与在线遥测，并完成比赛时期整机飞行测试。项目未完成竞赛任务闭环；赛后工程化修改只完成软件验证，未复飞。

## 不应公开陈述

- 当前 HEAD 全部完成飞行验证；
- 已实现稳定自主悬停；
- 达到某悬停精度、超调量、响应时间或抗风等级；
- 所有 failsafe 已完成故障注入；
- V307 过流链路可靠；
- 达到工业级/产品级可靠性。

## 私有视频证据

原比赛演示视频由作者保留，但不公开上传、不在 README 提供链接。若面试现场由作者自行展示，caption 应明确：

> Competition-era flight demonstration. The exact firmware commit and parameter snapshot were not preserved. Post-competition engineering changes in this repository have not been flight-validated.

视频不需要配一张虚构或来源不明的 VOFA 曲线。没有曲线时，只不报告量化性能即可；不公开视频不影响源码作为主要工程证据。

## 如果恢复硬件验证

1. 建立 `flight-baseline` tag，保存固件 hash、参数 dump、硬件和桨/电机/ESC 配置。
2. 拆桨验证 SysTick1 1 ms、四路 PWM、armed gate 和所有强制停机路径。
3. 为 V307 parser 建 host byte-stream tests，再做 USART 故障注入。
4. 定义 IMU sequence/timestamp 和 stale policy，先在拆桨条件注入 V5F/IMU 停更。
5. 用遥控器逐通道确认 pitch/throttle 历史映射。
6. 按拆桨、固定机架、系留、低风险自由飞行逐级验证并保存 VOFA/示波器日志。
