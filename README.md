# CH32H417 竞赛无人机

基于 CH32H417 双核（V3F + V5F）的四旋翼飞控项目。

## 硬件平台

- **主控**: CH32H417（V3F 飞控核 + V5F 传感器/通信核）
- **电调**: Seekfree BLDC 无刷电调（BEMF）
- **电机**: 2212 1400KV
- **桨叶**: 8045
- **电池**: 3S
- **机架**: F330

## 代码结构

```
EXAM/GPIO/GPIO_Toggle/
├── V3F/User/          ← 飞控核心 (main.c)
│   ├── main.c         PID、混控、PWM输出、安全逻辑
│   ├── bsp_pid.c/h    级联 PID 控制器
│   ├── bsp_pwm.c/h    PWM 输出 & 电调接口
│   ├── bsp_vofa.c/h   VOFA 调试数据上传
│   └── shared_data.h  双核共享数据结构
│
├── V5F/User/          ← 传感器/通信核 (main.c)
│   ├── main.c         IMU、光流、TOF、NRF RC链路
│   └── shared_data.h  双核共享数据结构
│
└── EXAM/SRC/          ← CH32H417 HAL 库 & 启动代码

..\CH32V203C8T6_BLDC_ESC\  ← ESC 固件（独立工程）
```

## 控制器架构

- **外环（角度环）**: P 控制 @ 100Hz — Pitch / Roll
- **内环（角速度环）**: PD 控制 @ 200Hz — 基于四元数误差
- **混控**: X 型四旋翼混控矩阵
- **D 项**: D-on-measurement，防止设定值跳变引起毛刺


## 串口命令（部分）

| 命令 | 说明 |
|------|------|
| `pa <val>` | Pitch 角度外环 P |
| `ra <val>` | Roll 角度外环 P |
| `am1` | 启用 Pitch 角度模式 |
| `rm1` | 启用 Roll 角度模式 |
| `pl <val>` | PID 输出限幅 (µs) |
| `sl <val>` | 电机 slew rate 限制 |

## 安全提示

- 调试前卸桨
- 不要绕过解锁、RC 链路、PWM 限制和上锁逻辑
- 烧录 ESC 固件时断开电池
