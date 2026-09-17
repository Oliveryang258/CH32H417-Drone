# CH32H417 源码事实地图

> 审计快照：工作树分支 `150HZ-inner-loop`，`HEAD=e4595032044027098c6d52d6a1212e942fa2302f`。本文只把当前活动工程 `EXAM/GPIO/GPIO_Toggle/{V3F,V5F}/User` 当作实现事实；`software_copyright_package/source_code` 中对应核心文件的 SHA-256 与活动工程相同，视为发布镜像，不重复计数。仓库当前没有 tag，也没有能把某次真实飞行与某个 commit 一一绑定的机器可验证记录。

> **快照说明：** 本文行号和“修改前”问题对应 `e459503` 审计基线。2026-08-29 的工程化 delta 已修正其中部分 stale comments、dead API 和 V5F build blocker；当前状态以根目录 [SOURCE_OF_TRUTH.md](../SOURCE_OF_TRUTH.md) 和 [CHANGELOG_ENGINEERING_REVIEW.md](../CHANGELOG_ENGINEERING_REVIEW.md) 为准。

## 0. 证据规则

- **实现事实**：由当前 `.c/.h` 中的宏、初始化语句、调用点或读写语句直接证明。
- **计算事实**：只使用当前宏和寄存器配置可推导的值，并保留公式。
- **注释声明**：只有注释、没有对应配置语句时，不升级为实现事实。
- **历史事实**：Git 提交信息可以证明“某次提交这样描述”，不能单独证明硬件上真实飞过或当时烧录的二进制。
- 每个源码结论都给出 `路径 — 函数/作用域 — 行号`。行号对应本审计快照。

---

## 1. IMU（JY61P）

### 1.1 一页结论

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| MCU / 核 | V5F 负责 IMU 串口接收与解析 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级驱动 — L1-L35 |
| USART instance | **USART4** | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级宏 — L9-L12 |
| 引脚 / AF | PC6=TX、PC7=RX、AF7 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级宏 — L15-L18；`IMU_Init` — L99-L110 |
| 波特率 | **115200 bit/s**，8N1，无流控 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级宏 — L13；`IMU_Init` — L112-L120 |
| 中断入口 | `USART4_IRQHandler()` 转调 `IMU_IRQHandler()` | `EXAM/GPIO/GPIO_Toggle/V5F/User/ch32h417_it.c` — `USART4_IRQHandler` — L91-L100 |
| 字节/帧入口 | `IMU_IRQHandler()`：RXNE 每字节推进，11 字节成帧 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `IMU_IRQHandler` — L132-L215 |
| 帧校验 | 前 10 字节累加和必须等于第 11 字节 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_CheckSum` — L258-L269 |
| 解析入口 | `JY61P_ParseFrame()` | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `IMU_IRQHandler` 调用 — L194-L209；`JY61P_ParseFrame` — L276-L327 |
| `0x51` | 三轴加速度；换算范围因子为 ±16 g | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.h` — 帧宏 — L10-L15；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_ParseFrame` — L288-L298；`JY61P_ConvertAccel` — L332-L335 |
| `0x52` | 三轴角速度；换算范围因子为 ±2000 °/s | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.h` — 帧宏 — L10-L15；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_ParseFrame` — L300-L310；`JY61P_ConvertGyro` — L340-L343 |
| `0x53` | 三轴欧拉角；换算范围因子为 ±180° | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.h` — 帧宏 — L10-L15；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_ParseFrame` — L312-L322；`JY61P_ConvertAngle` — L348-L351 |
| 输出内容配置 | 上电发送解锁命令和 `FF AA 02 0E 00` 输出内容命令 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_EnableFlightOutputs` — L50-L60；`IMU_Init` — L124 |
| configured output rate | **当前源码没有发送输出频率设置命令，无法由代码证明。** L13 注释声称模块已保存为 200 Hz；这只是外部持久配置假设 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级宏注释 — L13；`JY61P_EnableFlightOutputs` — L50-L60 |
| DataReady 存储 | `static volatile uint8_t s_imu_data_ready` | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级状态 — L25-L29 |
| DataReady 置位 | **任何一帧**校验通过的 `0x51`、`0x52` 或 `0x53` 都会置 1 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_ParseFrame` — L288-L321 |
| DataReady 清除 | V5F 主循环先取内部结构指针，再清 `s_imu_data_ready` 和 `frame_updated` | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `IMU_ClearDataReady` — L231-L235；`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1012-L1016 |
| 共享内存发布 | 任一 DataReady 事件后，把当前缓存里的 angle、gyro、accel 全组复制到共享 SRAM；roll/pitch 先减安装偏置 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1011-L1032；安装偏置宏 — L27-L28 |

### 1.2 DataReady 的精确语义

`IMU_DataReady()==1` 的真实含义是：**自上次清标志后，至少有一个校验通过的 0x51/0x52/0x53 单帧更新过内部缓存**。它不表示三类帧组成的一整组已经全部到齐，也不携带“哪一类帧触发”的原子事件队列。

证据链：

1. 每个 case 都独立更新自己的 3 轴字段，并独立执行 `s_imu_data_ready = 1U`：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_ParseFrame` — L288-L321。
2. `frame_updated` 记录最近触发的帧类型，但发布代码不检查它：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `JY61P_ParseFrame` — L295、L307、L319；`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1012-L1031。
3. 发布时一次复制 angle、gyro、accel 全组，因此共享 SRAM 中的一次“IMU 发布”可能包含来自不同串口帧时刻的三个子组：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1021-L1031。

### 1.3 IMU 链路图（源码事实）

```text
JY61P
  └─ USART4 RX / PC7 / 115200
       └─ USART4_IRQHandler()
            └─ IMU_IRQHandler(): 字节同步 → 11B → checksum
                 └─ JY61P_ParseFrame()
                      ├─ 0x51 更新 accel，DataReady=1
                      ├─ 0x52 更新 gyro， DataReady=1
                      └─ 0x53 更新 angle，DataReady=1
                           └─ V5F Bringup_Run()
                                └─ 全组复制 angle+gyro+accel 到 0x20140000
                                     └─ V3F PID_Tick() 直接读取
```

---

## 2. Control（V3F 控制链）

### 2.1 TIM2 基准频率

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| 周期宏 | `PID_PERIOD_US = 6667` | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — 文件级宏 — L438 |
| 计数时钟 | `SystemCoreClock / (PSC+1) = 1 MHz` | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Timer_Init` — L561-L573 |
| ARR | `PID_PERIOD_US - 1 = 6666` | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Timer_Init` — L572-L576 |
| 实际更新频率 | `1,000,000 / 6,667 = 149.9925 Hz`，工程中按约 150 Hz 使用 | 同上；这是由 L572-L573 的寄存器配置计算所得 |
| ISR 调用 | 每次 TIM2 update 清标志、累计软件毫秒、调用 `PID_Tick()` | `EXAM/GPIO/GPIO_Toggle/V3F/User/ch32h417_it.c` — `TIM2_IRQHandler` — L20-L37 |
| 控制 `dt` | `PID_DT = 0.006667 s` | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.h` — 文件级宏 — L14 |

### 2.2 各控制环真实节拍与实现

| 环路 | 真实调用节拍 | 算法类型 | 实现位置 |
|---|---:|---|---|
| Roll/Pitch 姿态外环 | 每 2 个 TIM2 tick，约 **74.996 Hz** | 角度误差乘 `g_kp_*_angle`，限幅后得到角速度期望；**P** | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` 外环块 — L903-L962，核心公式 L949-L960 |
| Yaw 指令 | 同一外环块更新，约 **74.996 Hz** | 当前实现把摇杆直接映射为 ±30 °/s；计算了 yaw angle error 但未用 `g_kp_yaw_angle` 闭环输出 | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L949-L960；参数定义 L31、L434 |
| Roll/Pitch 角速度内环 | 每个 TIM2 tick，约 **149.993 Hz** | 滤波误差上的 **PD** + rate feed-forward | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `RatePD_Update` — L506-L541；`PID_Tick` 调用 — L964-L992 |
| Yaw 角速度内环 | 每个 TIM2 tick，约 **149.993 Hz** | 完整 **PID**，D-on-measurement，D 一阶滤波 | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pid.c` — `PID_Update` — L23-L71；`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` 调用 — L993 |
| 高度估计器 | 每个 TIM2 tick 调入口，约 **149.993 Hz**；只有确认到新 RANGE 快照才更新滤波状态 | 双读 marker + fence + 连续两个控制 tick 一致确认，再做倾斜补偿、LPF、差分速度 | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L608-L610；`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `Height_ReadTofSnapshot` — L237-L272；`HeightEstimator_Update` — L274-L423 |
| 高度位置环 | Active 状态每 6 tick，约 **24.999 Hz** | 高度误差 **P** → 垂直速度期望，带速度/加速度限幅 | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `HeightControl_PositionLoop` — L590-L636；`HeightControl_Update` 分频 — L768-L770、L838-L852 |
| 高度速度环 | Active 状态每 3 tick，约 **49.998 Hz** | 垂直速度 **PI**，积分冻结与限幅 | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `HeightControl_VelocityLoop` — L638-L685；`HeightControl_Update` 分频 — L768-L770、L838-L852 |
| XY 位置控制 | flow_ok 且位置模式时每 6 tick，约 **24.999 Hz** | X/Y 位置误差 **P** → 速度期望 | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` 光流块 — L775-L778、L848-L862 |
| XY 速度控制 | flow_ok 时每 3 tick，约 **49.998 Hz** | 速度误差 **P** → roll/pitch 角度目标 | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` 光流块 — L864-L900 |
| V5F XY 状态估计 | TIM3 **200 Hz** | XY-KF / OF2 状态发布，不是 V3F 电机控制环 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `XYKF_RATE_HZ`/`XYKF_DT` — L29-L30；`XYKF_TimerInit` — L310-L325；`EXAM/GPIO/GPIO_Toggle/V5F/User/ch32h417_it.c` — `TIM3_IRQHandler` — L84-L88 |

### 2.3 PID / P / PD 实现索引

- 通用 PID 数据结构：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pid.h` — `PID_t` — L4-L14。
- 通用 PID 初始化/复位：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pid.c` — `PID_Init` / `PID_Reset` — L3-L21。
- Yaw 完整 PID：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pid.c` — `PID_Update` — L49-L71。
- Roll/Pitch rate PD：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `RatePD_Update` — L520-L541。
- Roll/Pitch attitude P：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L949-L959。
- XY position P：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L853-L861。
- XY velocity P：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L864-L900。
- Height position P：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `HeightControl_PositionLoop` — L600-L635。
- Height velocity PI：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `HeightControl_VelocityLoop` — L653-L684。

---

## 3. Shared SRAM

### 3.1 地址、ABI 与访问方向

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| 固定地址 | **`0x20140000`** | `EXAM/GPIO/GPIO_Toggle/V3F/User/shared_data.h` — `SHARED_DATA_BASE_ADDR` — L25；`EXAM/GPIO/GPIO_Toggle/V5F/User/shared_data.h` — L15 |
| 映射形式 | `(*(volatile SharedSensorData_t *)SHARED_DATA_BASE_ADDR)` | V3F `shared_data.h` — `g_shared_sensor` — L115；V5F `shared_data.h` — L105 |
| ABI 布局 | 两侧都使用 `#pragma pack(1)`；字段顺序和类型一致，只有注释文本不同 | V3F `shared_data.h` — `SharedSensorData_t` — L28-L109；V5F `shared_data.h` — L18-L99 |
| 主要 writer | V5F 主循环写 IMU、FLOW、RANGE、RC、调试字段；V5F TIM3 ISR 写 XY 状态 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1005-L1123；`XYKF_TickISR` — L328-L662 |
| 主要 reader | V3F `PID_Tick`、高度模块、VOFA 遥测、主循环状态机 | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L608-L1240；`bsp_height.c` — `Height_ReadTofSnapshot`/`HeightEstimator_Update` — L237-L423；`main` — L1283-L1362 |
| 反向 writer | V3F 写 `calib_test_flag`、`flow_source_select`、`alarm_flags`，所以共享区不是严格单向 | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `CMD_Parse` — L292、L345；`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `V307_AlarmPoll` — L76-L120 |

### 3.2 TOF/RANGE commit marker

TOF 四字段是当前共享 ABI 中唯一有明确跨核提交协议、写侧 fence、读侧 fence 和重复确认的业务快照：

1. V5F 先把 `tof_update_tick=0`，执行 `fence rw,rw`：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1100-L1103。
2. V5F 写 `tof_valid/tof_distance_mm/tof_state/tof_valid`：同文件同函数 — L1104-L1107。
3. V5F 再执行 `fence rw,rw`，最后把 `source_mark` 写入 `tof_update_tick`：同文件同函数 — L1108-L1109。
4. V3F 读取 `mark_begin`，fence，读取三个 payload 字段，fence，再读取 `mark_end`；首尾 marker 相同才接受：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `Height_ReadTofSnapshot` — L237-L272。
5. 因 `#pragma pack(1)` 使 `tof_update_tick` 非自然对齐，V3F 还要求连续两个 150 Hz tick 得到完全相同的快照才真正消费：同文件 — `HeightEstimator_Update` — L284-L305。

**TOF commit marker 的精确语义**：非零的 `tof_update_tick` 是 `range_sample.timestamp_ms + 1`，在 payload 写完和写 fence 之后最后发布；V3F 把 marker 变化视为新源帧。来源计算见 `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1071-L1080，发布见 L1100-L1109。

### 3.3 fence 定义

这里没有统一宏或封装函数；代码直接内联 RISC-V 指令：

```c
__asm__ volatile("fence rw, rw" ::: "memory");
```

- 写侧：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1103、L1108。
- 读侧：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `Height_ReadTofSnapshot` — L255、L259。

`"memory"` 是编译器内存 clobber；`fence rw,rw` 是处理器内存顺序屏障。两者同时出现，但只覆盖 TOF 快照路径。

### 3.4 没有跨核一致快照保护的字段组

| 字段组 | writer / commit-like 字段 | reader | 当前保护事实 |
|---|---|---|---|
| IMU：`roll/pitch/yaw/gyro_dps/accel_g` | V5F `Bringup_Run` L1021-L1031；没有 IMU 专用 marker | V3F `PID_Tick` L625-L627、L751-L753、L949-L951；高度估计 L306-L307 | 多字段裸 `volatile` 读写，无 fence、无首尾序号；可能看到跨 0x51/0x52/0x53 时刻的混合缓存 |
| FLOW payload | V5F `Bringup_Run` L1044-L1063；`flow_update_tick` 在 payload 中途靠后写，但无清零/fence/读侧双检 | V3F `PID_Tick` L782-L900 | 没有一致快照协议 |
| XYKF/OF2 payload | V5F `XYKF_TickISR` L634-L661；最后 `ekf_update_tick++` | V3F `PID_Tick` L782-L900 | marker 最后递增，但无写 fence，V3F 只读一次 marker 且随后分散读取字段，不构成一致快照 |
| RC payload | V5F `Bringup_LinkPollRC` L812-L824 | V3F 主循环 L1297-L1305、L1345-L1350，控制宏 L132-L134 | 多字段裸写/读，无 packet sequence 或 fence |
| `lf_range_*` 镜像 | V5F `Bringup_Run` L1111-L1119 | V5F `XYKF_TickISR` L383-L389、L485-L496；V3F VOFA | `lf_range_update_tick` 最后写，但没有 fence/双检；不要与受保护的 `tof_*` 快照混同 |
| heartbeat：`update_tick/calib_time_ms` | V5F 自由运行主循环每次迭代先写 L1005-L1009 | V3F 主循环 L1287-L1291 | `update_tick` 是自由循环心跳，不是“某组传感器写完”的提交序号 |
| LF debug counters | V5F `Bringup_Run` L1033-L1042 | V3F VOFA `bsp_vofa.c` L245-L252 | 多字段裸复制，无快照保护 |
| V3F→V5F 控制字段 | V3F `CMD_Parse` L292、L345；`V307_AlarmPoll` L120 | V5F `XYKF_TickISR` L398-L399/L504；ACK 打包 L718 | 单字段访问为主，但没有跨核协议或 fence |

`volatile` 只强制每次生成内存访问，不保证多字段原子快照，也不替代跨核内存顺序协议。源码依据是两侧宏的 `volatile SharedSensorData_t` 映射（V3F `shared_data.h` L115；V5F L105）与只有 TOF 路径出现 `fence rw,rw` 的事实。

---

## 4. NRF / RC / ACK Payload 链路

### 4.1 无线硬件与空口配置

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| MCU / 核 | V5F 通过 NRF24L01 接收遥控输入，并把传感器数据装入 ACK Payload | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkInitRX` / `Bringup_LinkPollRC` / `Bringup_LinkRefreshAckPayload` — L683-L824 |
| SPI instance | **SPI3**，Mode 0，8 bit，MSB first，软件 NSS，预分频 Mode7 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — 文件级宏 — L24-L26；`NRF_Init` — L95-L158 |
| SPI 引脚 | PC10=SCK、PC11=MISO、PC12=MOSI，AF6 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_Init` — L125-L157 |
| 控制引脚 | PD0=CSN、PD1=CE、PD2=IRQ；当前接收逻辑实际轮询 FIFO，没有配置 EXTI | `EXAM/GPIO/GPIO_Toggle/V5F/User/board_config.h` — NRF 宏 — L51-L60；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_Init` — L106-L143；`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkPollRC` — L786-L825 |
| 空口频道 | `RF_CH=40`，按驱动注释即 2440 MHz | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `BRINGUP_LINK_CHANNEL` — L68；`Bringup_LinkInitRX` — L688-L690；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_Config` — L292-L307 |
| 空中速率 / 功率 | **250 kbps / -6 dBm** | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkInitRX` — L688-L691；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_Config` — L295-L307 |
| 地址 | 5 字节；飞机本机 A1=`34 43 10 10 A1`，遥控器 B1=`34 43 10 10 B1` | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — 文件级地址 — L142-L144；`Bringup_LinkInitRX` — L692-L695 |
| NRF 角色 | 飞机长期 **PRX**；pipe1 接收遥控包，ACK Payload 自动回传，不手动切 TX/RX | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkInitRX` — L677-L706 |
| 自动 ACK / 动态载荷 | pipe0/1 开自动 ACK；随后 FEATURE=`0x06`、DYNPD=`0x03` 开 ACK Payload 与 pipe0/1 动态载荷 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_Config` — L280-L320；`NRF_EnableAckPayload` — L916-L919 |
| 重传寄存器 | 实际写 `SETUP_RETR=0xF3`；源码旁注“ARD=500us”与该字面值不一致，精确时序不能按注释作答 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_Config` — L289-L290 |

### 4.2 RC 上行包与共享内存发布

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| 包长 | `NRF_RC_Packet_t`，packed **16 B** | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — RC 结构体 — L108-L130；`Bringup_LinkInitRX` — L695 |
| 包格式 | magic `0x5A`、8 bit seq、roll/pitch/yaw/throttle 四个 `int8_t`、模式、机械爪、flags、6 B reserved、checksum | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `NRF_RC_Packet_t` — L112-L130 |
| 校验 | 对前 15 B 做 XOR；长度、magic、checksum 任一失败都不发布 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_RCChecksum` — L771-L779；`Bringup_LinkPollRC` — L792-L810 |
| DataReady 精确语义 | `NRF_DataReady()==1` 只表示 **RX FIFO 非空**，不表示包已通过应用层 magic/checksum 校验 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_DataReady` — L528-L548；`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkPollRC` — L792-L810 |
| FIFO 处理 | 主循环每轮用 `while` 排空最多三级 RX FIFO；长度异常会 FlushRX，magic/checksum 异常只丢当前包 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkPollRC` — L782-L810 |
| 通道发布 | 校验后逐字段写 `rc_roll/pitch/yaw/throttle/sw/meg/flags`，再写 `rc_link_ok=1`、计数和本地收包时刻 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkPollRC` — L812-L824 |
| 超时 | 距最后一个有效包 **500 ms** 后把 `rc_link_ok=0`、`rc_meg=0`，lost count 加一 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `RC_LINK_TIMEOUT_MS` — L132；`Bringup_Run` — L1132-L1139 |
| V3F 消费 | `rc_sw` 为 Fly/HeightHold 且 `rc_link_ok=1` 才维持解锁；仅 Fly + 低油门可进入解锁 | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `main` 状态机 — L1296-L1305、L1343-L1350 |

**RC 发布边界**：包内有 `seq`，但 V5F 不检查跳号、重复或乱序，也不把 `seq` 发布到共享 SRAM。共享区 RC 字段也没有 fence 或快照双检。因此，当前源码能证明“校验通过后逐字段复制”，不能证明 V3F 每次读到的四通道和开关一定来自同一个原子包。证据：V5F `main.c` — `NRF_RC_Packet_t` L117-L130、`Bringup_LinkPollRC` L812-L824；V3F `main.c` — 状态机 L1297-L1305、L1345-L1350。

### 4.3 ACK 下行包

- ACK 包是 packed **32 B**：`0xA5`、type `0x01`、递增 seq、alarm、FLOW、IMU raw、TOF、XOR checksum。结构与字段见 `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `BringupLinkPacket_t` — L70-L106。
- `Bringup_LinkBuildPacket` 直接从 IMU/LF 内部缓存和共享 `tof_*` 取“当前最新值”，没有建立跨传感器同一时刻快照：同文件 — L709-L747。
- 主循环名义每 20 ms（50 Hz）构造一包并写 pipe1 ACK FIFO：同文件 — `Bringup_LinkRefreshAckPayload` — L755-L765；`Bringup_Run` — L1125-L1130。
- `NRF_WriteAckPayload` 只发送 W_ACK_PAYLOAD 命令和 1~32 B 数据，不检查 TX_FULL、返回状态或写入成功：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_WriteAckPayload` — L930-L942。
- 因此注释“最多 60 ms 旧”只在 ACK FIFO 按预期持续被对端取走、周期写入均成功时成立；RC 停发或 FIFO 满时，当前源码没有时间戳/满检查来保证这一上界。冲突注释位于 V5F `main.c` — `Bringup_LinkRefreshAckPayload` — L763-L765。

---

## 5. LF 光流 / RANGE / V5F XYKF → V3F XY 控制

### 5.1 串口与帧解析

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| USART | **USART2**，500000 bit/s，8N1，无流控，TX+RX | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — 文件级宏 — L31-L42；`LF_GetDefaultConfig` — L88-L97；`LF_InitEx` — L127-L169 |
| 引脚 / AF | 实现是 **PD5=TX、PD6=RX、AF7** | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — 文件级宏 — L31-L39；`LF_InitEx` — L147-L163 |
| ISR 入口 | `USART2_IRQHandler()` → `LF_IRQHandler()` → `LF_ParseByte()` | `EXAM/GPIO/GPIO_Toggle/V5F/User/ch32h417_it.c` — `USART2_IRQHandler` — L103-L112；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_IRQHandler` — L231-L275 |
| 帧结构 | `AA addr id len payload sumcheck addcheck`，最大 payload 32 B | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.h` — 帧宏 — L27-L44；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_ParseByte` — L516-L595 |
| 双校验 | 对 header 到 payload 累加 `sumcheck`，并对每步 sum 再累加 `addcheck` | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_CheckFrame` — L633-L665 |
| 支持帧 | FLOW `0x51` 长 5/7/15；RANGE `0x34` 长 7；IMU `0x01` 长 13；QUAT `0x04` 长 9 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_IsSupportedFrame` — L704-L738 |
| FLOW 模式 | raw=0、decoupled=1、fusion=2；fusion 才含速度、FIX 速度和积分位移全组 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.h` — 模式宏 — L37-L39；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_DecodeFrame` — L812-L843 |
| RANGE 内容 | direction 1 B、angle 2 B LE、distance_cm 4 B LE；`0xFFFFFFFF` 为无效 | `EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.h` — 无效宏 — L47-L50；`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_DecodeFrame` — L846-L864 |

### 5.2 LF DataReady 的精确语义

`LF_DataReady()==1` 的含义是：**最近至少有一个受支持、双校验通过且成功解码的 FLOW/RANGE/IMU/QUAT 帧把公共标志置位**。`frame_updated` 只保留最近一个帧 ID；它不是事件队列。

1. 四种 frame case 都会覆盖 `frame_updated` 并置 `s_lf_data_ready=1`：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_DecodeFrame` — L806-L890。
2. V5F 主循环只在 `frame_updated==FLOW` 时把 FLOW 发布到共享 SRAM，随后无条件清标志：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1044-L1065。
3. 因而 FLOW 到达后若在主循环消费前又到一个 IMU/QUAT/RANGE 帧，公共 `frame_updated` 可被覆盖，FLOW 通知可丢。`LF_GetData()` 又直接返回 ISR 正在更新的静态结构体指针，没有 FLOW 专用 seqlock：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_GetData` — L330-L333。
4. RANGE 为避免这种覆盖，另建 `s_lf_range_seq/sample_count/timestamp_ms`，写时奇偶序号包围 payload，读时首尾双检：同文件 — 状态 L59-L65；`LF_DecodeFrame` L846-L864；`LF_GetRangeSample` L335-L368。

### 5.3 RANGE 到 TOF 提交路径

- RANGE 解码时用 V5F `MCYCLE` 生成单调毫秒时间；首帧为 1，零毫秒增量强制变为 1：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_CaptureRangeTimestampMs` — L947-L983。
- V5F 主循环只消费 `sample_count` 变化的新 RANGE；要求 distance 5~400 cm，且 direction=0、angle=0 才当下视有效：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1068-L1098。
- 有效时厘米乘 10 转毫米；无效、超范围、非下视分别发布 state 1/2/3：同文件同函数 — L1081-L1109。
- `tof_*` 走完整跨核提交协议；`lf_range_*` 只是给 V5F XYKF 使用的镜像，虽然 marker 最后写，但没有硬件 fence：同文件同函数 — L1100-L1119。完整 TOF 协议见本文 3.2。

### 5.4 V5F 200 Hz XY 状态发布

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| 调度 | TIM3，名义 **200 Hz**，`DT=0.005 s` | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `XYKF_RATE_HZ/XYKF_DT` — L29-L30；`XYKF_TimerInit` — L310-L325；`EXAM/GPIO/GPIO_Toggle/V5F/User/ch32h417_it.c` — `TIM3_IRQHandler` — L84-L88 |
| 默认 source | 初始化把 `flow_source_select=2`、active=2，即默认走 **OF2 直通/偏置校准路径** | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` 初始化 — L882-L888 |
| source 0 | 不等于 2 时才执行 IMU 预测 + flow 速度修正的 `s_xkf/s_ykf` 路径 | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `XYKF_TickISR` — L398-L500、L649-L657 |
| source 2 有效条件 | fusion mode、`flow_valid`、quality≥150、最近 20 tick（100 ms）收到 FLOW | `EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — 常量 — L31、L50；`XYKF_TickISR` — L504-L510 |
| source 2 速度 | `DX_2/DY_2 - 地面静止偏置`；无效时发布 0 | 同文件 — `XYKF_TickISR` — L515-L517、L640-L645 |
| 偏置校准 | 未飞行、flow 有效、三轴最大角速度≤1.5°/s、FIX 速度≤1 cm/s；目标 20 s，最短 5 s，位移超 10 cm 重置 | 同文件 — 常量 — L59-L64；`XYKF_TickISR` — L525-L560；`OF2_BiasCalFinish` — L211-L232 |
| 位置 | disarmed 时跟随原点并发布 0；flying 时把机体系积分增量按 yaw 旋到大地系累加 | 同文件 — `XYKF_TickISR` — L562-L635 |
| 静止抑制 | flying 且有效、角速度<1.5°/s、校正速度<0.5 cm/s 持续 300 ms 后速度归零并不累加位置 | 同文件 — 常量 — L65-L67；`XYKF_TickISR` — L612-L631 |
| 发布 marker | 最后 `ekf_update_tick++`，但无写 fence；V3F 也不做首尾双读 | 同文件 — `XYKF_TickISR` — L640-L661；V3F `main.c` — `PID_Tick` — L782-L804 |

### 5.5 V3F XY 控制入口与坐标系

- V3F 的 flow_ok 还要求：用户开 `g_flow_hold_enable`、非单电机测试、`ekf_update_tick` 年龄≤120 ms、质量达到可调阈值、`ekf_flags` bit5 valid 且 bit7 timeout 未置位：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — 参数 L62-L69；`PID_Tick` — L782-L805。
- source2 下 OF2 速度定义为机体系 X=前/Y=右；位置为大地系。位置模式先把大地系速度目标按 yaw 旋回机体系，再与 OF2 速度比较：同文件 — `PID_Tick` — L833-L839、L869-L885。
- source0 下估计速度视为大地系，速度误差再旋到机体系：同文件 — `PID_Tick` — L886-L899。
- XY 位置 P 约 25 Hz、速度 P→姿态约 50 Hz，详见本文 2.2。

---

## 6. 解锁 / 失控保护 / PWM 输出状态链

### 6.1 PWM 硬件与“锁定”的精确含义

| 项目 | 当前源码事实 | 源码定位 |
|---|---|---|
| Timer / pins | TIM4_CH1~CH4 → PD12~PD15，AF2 | `EXAM/GPIO/GPIO_Toggle/V3F/User/board_config.h` — MOTOR 宏 — L6-L11；`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — `PWM_TIM4_Init` — L190-L237 |
| PWM 基准 | 1 MHz 计数，period=6667 μs，实际约 **149.9925 Hz** | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.h` — 宏 — L29-L40；`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — `PWM_TIM4_Init` — L213-L225 |
| 驱动范围 | 通用驱动 clamp 1000~2000 μs；正常控制 mixer 再限制到 1000~1750 μs；基础油门上限当前 1550 μs | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — `PWM_ClampPulseUs` — L250-L263；`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.h` — L15-L17；V3F `main.c` — `mix_clamp` — L475-L480 |
| 上电状态 | `PWM_Init` 立即启动 TIM4 并输出四路 1000 μs，同时软件 `s_pwm_armed=0` | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `main` — L1257-L1265；`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — `PWM_Init` — L49-L61；`PWM_TIM4_Init` — L246-L247 |
| Lock | 把四个 CCR 写回 1000 μs并清软件 armed；**不关闭 TIM4/引脚输出** | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — `PWM_Lock` — L63-L73 |
| Arm | 只验证驱动已初始化、缓存四路均为 1000 μs，然后把软件 armed 置 1；函数内没有 3 s 等待 | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — `PWM_Arm` — L75-L94 |
| 输出门禁 | 未 armed 时，单路或四路 API 拒绝任何高于 1000 μs 的值 | 同文件 — `PWM_SetPulseUs` — L102-L125；`PWM_SetAllPulseUs` — L128-L151 |

### 6.2 主状态机

```text
启动
  └─ PWM_Init(): TIM4运行，四路=1000us，driver locked
       └─ STATE_DISARMED
            ├─ 仅当 rc_sw=FLY(2)
            ├─ rc_link_ok=1
            ├─ STICK_THROTTLE<=-100（实际别名是 rc_pitch）
            └─ 无 V307 overcurrent
                 └─ PWM_Arm() 成功 → s_armed=1 → STATE_ARMED

STATE_ARMED
  ├─ rc_sw=FLY 或 HEIGHT_HOLD 且 link_ok=1 → 保持
  └─ RC模式不满足 或 overcurrent
       └─ 暂停TIM2 IRQ → PWM_Lock() → 清控制状态 → 恢复IRQ → STATE_DISARMED
```

- 解锁条件实现：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `main` — L1343-L1380。
- 保持/上锁条件实现：同文件 — L1296-L1337。
- `STICK_THROTTLE` 在高度模块实际映射 `rc_pitch`：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.h` — L8-L14；这与字段名不是同一含义，见本文 7.2。

### 6.3 保护源与动作

| 保护源 | 触发条件 | 动作 / 边界 | 源码定位 |
|---|---|---|---|
| RC link loss | V5F 500 ms 未收到有效 RC 包，`rc_link_ok=0` | V3F 主循环紧急 `PWM_Lock`，状态回 DISARMED | V5F `main.c` — L1132-L1139；V3F `main.c` — L1297-L1337 |
| 模式开关 | armed 时不再是 Fly/HeightHold | 同上 | V3F `main.c` — L1297-L1337 |
| V307 overcurrent | 收到 `0xDD` 后保持 500 ms | 阻止解锁；armed 时紧急 Lock；蜂鸣 80 ms 周期内响 35 ms | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.h` — L19-L26；`bsp_height.c` — `V307_AlarmPoll` — L76-L122；`main.c` — L1293-L1306、L1348-L1350 |
| V307 battery low | 收到 `0xCC` 后保持 500 ms | 置 alarm + 持续蜂鸣；**不参与解锁禁止或自动上锁条件** | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.c` — `V307_AlarmPoll` — L76-L120；V3F `main.c` — L1293-L1305、L1348-L1350 |
| Gyro overspeed | 任一轴绝对值>500°/s 连续 10 个 TIM2 tick，实际约 **66.67 ms** | ISR 内 Lock、`s_armed=0`、清控制状态并 return | `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L616-L668；周期 L438 |
| 固定油门 soft-stop | override>1 且油门≤-100 | 从当前四路 PWM 线性降到 1000 μs，2 s 后 Lock | 同文件 — `PID_Tick` — L686-L740；`soft_stop_step` — L543-L559 |
| 高度 guard soft-stop | guard 开启、非测试/非高度环接管，TOF 达阈值并持续 | 启动同一 2 s soft-stop | 同文件 — `PID_Tick` — L1073-L1116 |
| PWM API software gate | driver 未 armed 而请求>1000 μs | 返回 `PWM_LOCKED`，不写 CCR | `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — L119-L125、L141-L151 |

### 6.4 Mixer 与最终电机映射

当前**实现表达式**是：

```text
M1 / PD12 / TIM4_CH1 / FR CCW = T + R - P - Y
M2 / PD13 / TIM4_CH2 / FL CW  = T - R - P + Y
M3 / PD14 / TIM4_CH3 / RL CCW = T - R + P - Y
M4 / PD15 / TIM4_CH4 / RR CW  = T + R + P + Y
```

实现证据：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` — L1198-L1203；物理通道证据：`EXAM/GPIO/GPIO_Toggle/V3F/User/board_config.h` — L6-L11；`bsp_pwm.c` — `PWM_WriteChannelRaw` — L277-L303。

正常模式下四路再经过每 tick 默认 ±17 μs slew 后写 `PWM_SetAllPulseUs`；单电机测试模式 1~4 只抬对应一路、其余 1000 μs，并跳过该 slew：V3F `main.c` — `PID_Tick` — L1205-L1249。

### 6.5 当前安全状态链的源码边界

1. **没有 IMU/V5F heartbeat 自动上锁条件**：主循环会记录 `update_tick` 最近变化时间，但 `sensor_seen_local_ms` 只进入 VOFA 快照，不参与解锁/维持条件。证据：V3F `main.c` — 状态变量 L110-L111；心跳记录 L1287-L1291；状态条件 L1297-L1305、L1345-L1350；VOFA 快照 L1424-L1425。
2. **overspeed/soft-stop 只清 `s_armed`，不直接修改 `main()` 的局部 `state`**：PWM 已锁住，PID 会返回；但状态机仍停在 STATE_ARMED，直到 RC 模式/链路或过流条件使它转回 DISARMED。这形成需要拨档/失联后才能重新走 Arm 的锁存行为。证据：V3F `main.c` — overspeed L632-L664；soft-stop L715-L726；主状态机变量和转换 L1281-L1380。
3. **“3 s 解锁等待”未实现**：`PWM_ARM_DELAY_MS` 和 `PWM_ARM_WAIT_MS` 都是 3000，但没有引用；`PWM_Arm()` 立即置 armed。证据：V3F `bsp_pwm.h` — L42-L43；`bsp_pwm.c` — L23、L79-L93；仓库搜索只有定义。
4. **Mixer 注释和实现的 M3/M4 yaw 符号相反**：注释写 M3 `+Y`、M4 `-Y`，代码实际 M3 `-Y`、M4 `+Y`。仅凭当前仓库不能判断注释 stale 还是电机旋向/实现错误，必须以拆桨电机映射测试确认。证据：V3F `main.c` — 注释 L1189-L1193；实现 L1200-L1203。

---

## 7. stale comment / 命名与实现不一致

### 7.1 已确认 stale comment

1. **IMU USART1**：V5F 文件头写 JY61P 使用 USART1，但驱动实际为 USART4。
   - 冲突注释：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — 文件头 — L2-L7。
   - 实现事实：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级宏 — L9-L12；`ch32h417_it.c` — `USART4_IRQHandler` — L91-L100。

2. **IMU USART2 / 100 Hz**：V5F 主循环说明写 USART2、100 Hz；实际串口是 USART4，源码不配置输出频率。
   - 冲突注释：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` 前说明 — L951-L954。
   - 实现事实：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — 文件级宏 — L9-L13；`JY61P_EnableFlightOutputs` — L50-L60。

3. **“一组可用姿态数据”**：`IMU_DataReady` 注释像是在描述完整帧组，实际任意单个 0x51/0x52/0x53 都置位。
   - 注释：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_imu.c` — `IMU_DataReady` 文档 — L218-L225。
   - 实现事实：同文件 — `JY61P_ParseFrame` — L288-L321。

4. **RatePD alpha**：传感器读取说明写 `alpha=0.2`，实际 `RatePD_Update` 用 `0.45`。
   - 冲突注释：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` 传感器块 — L743-L750。
   - 实现事实：同文件 — `RatePD_Update` — L526-L532。

5. **超速 10 tick ≈50 ms**：在 6667 µs 控制 tick 下，10 tick 约为 **66.67 ms**。
   - 冲突注释：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Tick` 看门狗块 — L616-L621。
   - 周期事实：同文件 — `PID_PERIOD_US` — L438；`PID_Timer_Init` — L567-L573。

6. **TIM2 “150.0 Hz”**：注释按一位小数四舍五入正确，但若面试要求精确值，配置是 149.9925 Hz，不是数学上的 150.0000 Hz。
   - 注释与配置：`EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — `PID_Timer_Init` — L567-L573。

### 7.2 命名与实现不一致

1. `SharedSensorData_t` 的总注释写“V5F 写入，V3F 读取”，但 V3F 会反向写 `calib_test_flag`、`flow_source_select`、`alarm_flags`。
   - 总注释：V3F `shared_data.h` — L27；V5F `shared_data.h` — L17。
   - 反向写证据：V3F `main.c` — `CMD_Parse` — L292、L345；V3F `bsp_height.c` — `V307_AlarmPoll` — L120。

2. RC 字段名与 V3F 控制别名交叉：`STICK_PITCH` 实际读取 `rc_throttle`，而高度模块的 `STICK_THROTTLE` 实际读取 `rc_pitch`。
   - `EXAM/GPIO/GPIO_Toggle/V3F/User/main.c` — 控制别名宏 — L132-L134。
   - `EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_height.h` — 高度别名宏 — L8-L14。
   - V5F 接包仍按 packet 字段名写 `rc_pitch` 与 `rc_throttle`：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkPollRC` — L812-L820。
   - 仅凭当前仓库不能判断这是遥控器物理通道映射、历史兼容，还是命名错误；面试时不要把字段名直接等同于物理摇杆轴。

3. `update_tick` 注释称“每次写入后递增”，实际是在 V5F 自由主循环每次迭代开头递增，之后才按 DataReady 条件写各传感器。
   - 字段注释：V5F `shared_data.h` — L26；V3F `shared_data.h` — L36。
   - 实现：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` — L1005-L1012。

4. `tof_*` 名称保留 TOF，但当前数据源是匿名光流模块的独立 RANGE 帧；未链接的旧 VL53-400 `bsp_tof.c/.h` 已在招聘整理中删除。
   - 字段注释：V3F `shared_data.h` — L51-L54；V5F `shared_data.h` — L41-L44。
   - 当前发布源：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` 的 `LF_GetRangeSample` 路径 — L1068-L1109。

5. V5F `main.c` 文件头仍称“新板 bring-up 测试 main”，但函数已承担运行时传感器发布、XY 状态估计、RC 链路和共享内存初始化。这个名字/说明不能作为“只用于测试、不参与飞控”的依据。
   - 文件头：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — L1-L18。
   - 实际主循环职责：同文件 — `Bringup_Run` — L939-L1003、L1005-L1145。

---

7. **LF 文件头 TX/RX 对调**：文件头写 USART2_TX→PD6、RX→PD5；实现宏和初始化是 TX=PD5、RX=PD6。
   - 冲突注释：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — 文件头 — L13-L15。
   - 实现事实：同文件 — 文件级宏 L31-L39；`LF_InitEx` L147-L163。

8. **LF 公共 DataReady 被描述为“新的光流数据”**：实际上 RANGE/IMU/QUAT 也会置位，且 `frame_updated` 只保留最后帧。
   - 注释：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_lf.c` — `LF_DataReady` 文档 — L278-L291。
   - 实现事实：同文件 — `LF_DecodeFrame` — L806-L890。

9. **NRF RC 主循环说明仍写数组协议**：注释称解析 `ch[0..5]`、`sw[0..1]` 并写 `rc_ch[]/rc_sw[]`；当前包实际是四个命名摇杆、单个 `sw_status`，共享区也是命名字段。
   - 冲突注释：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_Run` 说明 — L980-L985。
   - 实现事实：同文件 — `NRF_RC_Packet_t` — L117-L130；`Bringup_LinkPollRC` — L812-L824。

10. **ACK“最多 60 ms 旧”没有完整实现保证**：写函数不查 TX_FULL/状态，包也无源时间戳；链路中断后 FIFO 中旧包可继续保留。
    - 注释：`EXAM/GPIO/GPIO_Toggle/V5F/User/main.c` — `Bringup_LinkRefreshAckPayload` — L763-L765。
    - 实现事实：`EXAM/GPIO/GPIO_Toggle/V5F/User/bsp_nrf.c` — `NRF_WriteAckPayload` — L930-L942。

11. **PWM 解锁等待宏未生效**：头文件声称解锁时强制最低油门并阻塞 3 s，实际 `PWM_Arm()` 不等待；两个 3000 ms 宏均未被引用。
    - 注释/宏：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.h` — L42-L43；`bsp_pwm.c` — L23。
    - 实现事实：`EXAM/GPIO/GPIO_Toggle/V3F/User/bsp_pwm.c` — `PWM_Arm` — L79-L93。

12. **Mixer 注释 M3/M4 yaw 符号与代码相反**：见本文 6.5；这是安全相关待确认项，不能在面试中把注释矩阵和当前实现同时当真。

## 8. 当前仓库与“历史飞行版本”的可判定边界

### 8.1 当前源码可证明

- 当前检出的分支名是 `150HZ-inner-loop`，HEAD 是 `e459503`；这是 Git 元数据事实，不是某次飞行的烧录记录。
- 当前代码确实实现了 TIM2 约 150 Hz 内环、约 75 Hz姿态外环、25/50 Hz 高度与 XY 控制分频，源码证据见第 2 节。
- 当前 `docs/project-status.md` 明确把自主悬停列为 “Not flight validated”，并说明后续硬件飞行开发停止：`docs/project-status.md` — 项目背景 — L3-L7；状态表 — L27-L30。

### 8.2 当前仓库不能证明

- 哪个 commit / ELF / HEX 实际烧录并完成过哪一次真实飞行。
- 当前工作树参数是否等于某次飞行参数；参数可经 VOFA 运行时修改，且仓库没有对应试验快照。
- IMU 模块此刻实际保存的输出频率是否为 200 Hz；源码只在注释中声明，没有频率设置命令或读回验证。
- `rc_pitch` 与 `rc_throttle` 的交叉控制别名是否来自遥控器通道接线、发包端兼容，还是遗留命名错误。
- 没有 tag、带固件哈希的飞行日志或“flight validated commit”标记，因此不能把 Git 提交主题中的 “flight controller update” 当成飞行验证证据。

### 8.3 若要补齐历史飞行事实，需要的最小证据

1. 每次试验记录：日期、机架/传感器配置、Git commit、V3F/V5F 二进制 SHA-256、运行时参数导出。
2. 烧录记录或串口启动 banner 中包含 commit/构建 ID。
3. 原始日志与试验结论绑定同一 flight ID。
4. 明确区分：编译通过、软件检查、拆桨台架、系留、自由飞行。

---

## 9. 面试速答（只含当前源码可支撑事实）

- **IMU 用哪个串口？** V5F 的 USART4，PC6/PC7，AF7，115200 8N1。
- **IMU 是多少 Hz？** 驱动不配置输出频率；注释声称模块保存为 200 Hz，但当前源码无法证明硬件实际值。不要回答成主循环 stale comment 的 100 Hz。
- **DataReady 是完整姿态组吗？** 不是；任何校验通过的 0x51/0x52/0x53 单帧都会置位，发布时复制当前缓存中的三类数据。
- **TIM2 是多少 Hz？** 配置精确值约 149.9925 Hz，工程名义 150 Hz；外环隔 tick 约 75 Hz。
- **姿态/速率是什么控制器？** Roll/Pitch 姿态外环 P，角速度内环 PD；Yaw 当前是摇杆角速度指令 + 角速度 PID。
- **高度环？** 25 Hz 高度位置 P 产生垂直速度目标，50 Hz 垂直速度 PI 产生油门修正。
- **XY 环？** V3F 25 Hz 位置 P → 速度目标，50 Hz 速度 P → 姿态目标；V5F 另有 200 Hz XY 状态估计/OF2 发布。
- **共享内存地址？** `0x20140000`，两侧 `#pragma pack(1)` 的同布局结构体。
- **哪个字段是提交标记？** `tof_update_tick`；V5F 清零→fence→payload→fence→marker，V3F 首尾双读 marker，并要求连续两个控制 tick 快照一致。
- **其他字段有快照保护吗？** 没有等价于 TOF 的完整协议；IMU、FLOW、XYKF、RC 等大多是裸 `volatile` 多字段访问。
- **NRF 怎么配？** V5F SPI3，PC10/11/12，频道 40，250 kbps，-6 dBm，飞机 PRX；16 B RC 上行，32 B ACK Payload 下行。
- **NRF DataReady 表示有效 RC 包吗？** 不是，只表示 RX FIFO 非空；V5F 随后还做长度、0x5A magic 和 XOR 校验。
- **RC 失联多久停机？** V5F 500 ms 没收到有效包将 `rc_link_ok=0`；V3F 主状态机随后把四路拉回 1000 μs并上锁。
- **LF 光流串口？** V5F USART2，PD5 TX / PD6 RX，AF7，500000 8N1。文件头把 TX/RX 写反，应以初始化代码为准。
- **LF DataReady 是 FLOW 专用吗？** 不是；FLOW/RANGE/IMU/QUAT 都会覆盖同一标志和最后帧 ID。RANGE 另有独立 seqlock/sample_count 路径。
- **XYKF 默认走哪个源？** 初始化默认 source2，即 OF2 直通+偏置/位置处理；TIM3 200 Hz。source0 才走 IMU 预测+flow 修正的 `s_xkf/s_ykf`。
- **解锁条件？** rc_sw=Fly、link_ok=1、控制所用“油门”≤-100、无 V307 过流，然后 `PWM_Arm()`；当前并没有实现注释所称 3 s 等待。
- **Lock 是关闭 PWM 定时器吗？** 不是；TIM4 继续运行，`PWM_Lock()` 把四路 CCR 写成 1000 μs并清软件 armed，API 拒绝未解锁时的更高脉宽。
- **低电压会自动上锁吗？** 当前不会；0xCC 只置 alarm 和蜂鸣。0xDD 过流才阻止解锁并触发 armed 状态紧急 Lock。
- **Mixer 当前实现？** M1 `T+R-P-Y`，M2 `T-R-P+Y`，M3 `T-R+P-Y`，M4 `T+R+P+Y`。注释中的 M3/M4 yaw 符号与代码冲突，需拆桨确认。
