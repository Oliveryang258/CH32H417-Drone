# Hardware and Interface Map

本表只记录当前主动代码路径可以证明的连接。`board_config.h` 还包含历史/预留映射，不能自动视为当前启用硬件。

## Active interfaces

| Device / link | Owner | Peripheral / pins | Configuration | Evidence |
| --- | --- | --- | --- | --- |
| JY61P IMU | V5F | USART4, PC6 TX / PC7 RX | 115200 baud | `V5F/User/bsp_imu.c` |
| Anonymous optical-flow + range module | V5F | USART2, PD5 TX / PD6 RX | 500000 baud; FLOW frame `0x51`, RANGE frame `0x34` | `V5F/User/bsp_lf.c/h` |
| NRF24L01+ RC link | V5F | SPI3 PC10/PC11/PC12; PD0 CSN, PD1 CE, PD2 IRQ | channel 40, 250 kbps, -6 dBm, PRX + ACK payload | `V5F/User/bsp_nrf.c/h`、`main.c` |
| VOFA / serial debug | V3F | USART3, PA13 TX / PA14 RX | 115200 baud; 8-channel JustFloat frame | `V3F/User/bsp_vofa.c/h` |
| V307 link | V3F | USART5, PF5 TX / PE0 RX | 115200 baud | `V3F/User/bsp_comunicate.c/h` |
| ESC PWM | V3F | TIM4, PD12–PD15 | 4 channels; configured 150 Hz period | `V3F/User/bsp_pwm.c/h` |
| LED / buzzer | both project configs | PE11 / PE10 | GPIO | `board_config.h`、`bsp_led_buzz.c` |

JY61P 源码注释写有“saved setting: 200 Hz output”，但初始化代码主要配置输出内容与波特率；README 不把该注释当作每次上电主动配置 200 Hz 的证明。

## Active data ownership

- 光流模块的 RANGE frame 是当前 `tof_*` shared fields 的来源。
- 已删除未进入最终 ELF 的 V5F standalone TOF driver；当前高度数据路径仍是 `bsp_lf.c` RANGE frame。
- 已删除未进入最终 ELF 的 V5F legacy PWM module；最终 motor PWM 只由 V3F 输出。
- V3F `board_config.h` 中 IMU/LF/NRF 等映射不等于这些驱动由 V3F 主动运行；主动实现位于 V5F。

## Hardware facts not proven by active source

以下内容已经无法可靠回忆，公开 README 主动省略，不做推测：

- 机架型号与尺寸；
- 电机 KV、桨规格、电池型号；
- ESC 精确型号与固件版本；
- V307 板卡具体角色和固件版本；
- 调试串口使用的 USB-UART / 无线串口模块型号；
- 比赛视频对应的硬件 revision。
