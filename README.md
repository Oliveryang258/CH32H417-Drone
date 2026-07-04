# CH32H417-Drone

## 代码结构

```
CH32H417-Drone/
├── EXAM/
│   ├── GPIO/GPIO_Toggle/
│   │   ├── V3F/User/                    ← 飞控核心
│   │   │   ├── main.c                   主程序：PID控制器、混控矩阵、PWM输出、
│   │   │   │                            解锁/上锁逻辑、串口命令解析、故障保护
│   │   │   ├── bsp_pid.c                级联 PID 实现（角度外环 + 角速度内环）
│   │   │   ├── bsp_pid.h                PID 结构体 & 接口声明
│   │   │   ├── bsp_pwm.c                四路 PWM 输出 & 电调接口
│   │   │   ├── bsp_pwm.h                PWM 宏定义 & 安全限幅
│   │   │   ├── bsp_vofa.c               VOFA+ 调试数据发送
│   │   │   ├── bsp_vofa.h               VOFA 协议 & 帧格式
│   │   │   ├── bsp_comunicate.c         双核通信（USART）
│   │   │   ├── bsp_comunicate.h         通信协议定义
│   │   │   ├── bsp_led_buzz.c           LED & 蜂鸣器指示
│   │   │   ├── bsp_led_buzz.h           LED & Buzzer 接口
│   │   │   ├── board_config.h           硬件引脚配置
│   │   │   ├── shared_data.h            双核共享数据结构（与 V5F 同步）
│   │   │   ├── ch32h417_conf.h          芯片外设配置
│   │   │   ├── ch32h417_it.c            中断服务函数
│   │   │   ├── ch32h417_it.h            中断声明
│   │   │   └── system_ch32h417.c        系统时钟初始化
│   │   │
│   │   └── V5F/User/                    ← 传感器 & 通信核
│   │       ├── main.c                   主程序：IMU 读取、光流/TOF 采集、
│   │       │                            NRF RC 接收、双核数据同步
│   │       ├── bsp_led_buzz.c           LED & 蜂鸣器指示
│   │       ├── bsp_led_buzz.h           LED & Buzzer 接口
│   │       └── shared_data.h            双核共享数据结构（与 V3F 同步）
│   │
│   └── SRC/                             ← CH32H417 HAL 库
│       ├── Core/                        内核外设驱动（RISC-V）
│       ├── Peripheral/                  片上外设驱动（GPIO、TIM、USART、SPI 等）
│       ├── Startup/                     启动文件 & 中断向量表
│       └── Ld/                          链接脚本
│
├── tools/                               ← PC 端辅助工具
│   ├── rate_pid_sim.py                  PID 参数仿真脚本
│   ├── rate_pid_visualizer.html         PID 响应可视化
│   └── vofa_pid_advisor.py             VOFA 数据离线分析
│
└── CLAUDE.md                            项目规则 & 安全边界
```

## 关联仓库

[CH32V203C8T6 BLDC ESC](https://github.com/Oliveryang258/CH32V203C8T6_BLDC_ESC) — Seekfree 电调固件
