# 轮式移动机器人底盘 MCU 控制代码（STM32F407）

> 个人轮式移动机器人底盘的底层 MCU 控制固件，基于 **STM32F407VETx + FreeRTOS** 开发，支持多车型（两驱差速 / 四驱无刷），可通过 ROS、手机 APP、航模遥控、PS2 手柄、CAN 总线、串口等多种方式控制。

---

## 一、项目简介

本项目是轮式移动机器人底盘的完整嵌入式固件工程，负责底盘的运动控制、传感器采集、通信协议解析、故障自检与安全保护。代码基于 STM32 标准外设库编写，全部运行在 FreeRTOS 实时操作系统上。

- **MCU**：STM32F407VETx（Cortex-M4F，带 FPU，168MHz）
- **RTOS**：FreeRTOS V9.0.0（系统节拍 1000Hz，总堆 20KB）
- **电机**：无刷轮毂电机 + 霍尔编码器，经 CAN1/CAN2（或 485）与轮毂驱动器通信
- **姿态传感器**：ICM20948（主）/ MPU6050（备）
- **辅助传感器**：OLED 显示屏、超声波模块、RGB 灯带、蜂鸣器、ADC 电压采样、急停开关

## 二、目录结构

```
C63A/
├── USER/                  # 用户层：main.c、中断、启动文件、Keil 工程
│   ├── main.c             # 入口：硬件初始化 + 创建 FreeRTOS 任务
│   ├── stm32f4xx_it.c     # 中断服务函数
│   ├── system_stm32f4xx.c # 系统时钟配置
│   └── *.uvprojx           # Keil MDK5 工程文件（可重命名为自己的项目名）
├── SYSTEM/                # 系统基础模块
│   ├── delay/             # 延时
│   ├── sys/               # GPIO/时钟等底层
│   ├── usart/             # 串口驱动
│   └── dwt/               # Cortex-M 内核计时器
├── CORE/                  # CMSIS 内核文件 + 启动文件（startup_stm32f40_41xxx.s）
├── FWLIB/                 # STM32F4 标准外设库（inc/src）
├── FreeRTOS/              # FreeRTOS 内核
│   ├── include/           # 头文件 + FreeRTOSConfig.h 配置
│   ├── portable/          # 移植层（RVDS/ARM_CM4F 等）
│   └── MemMang/           # 内存管理（工程使用 heap_4）
├── HARDWARE/              # 外设驱动
│   ├── motor.c            # 轮毂电机控制（使能/失能/转速设置）
│   ├── encoder.c          # 编码器读取
│   ├── can.c / can2.c     # CAN1/CAN2 通信
│   ├── usartx.c           # 多路串口（ROS/APP/CAN 转发）
│   ├── oled.c             # OLED 显示
│   ├── Ultrasonic.c       # 超声波测距
│   ├── ICM20948/          # ICM20948 九轴姿态传感器驱动
│   ├── LED.C              # LED/RGB 灯带
│   ├── adc.c              # ADC（电压采样、车型选择电位器）
│   ├── key.c              # 按键（单击切菜单/双击更新陀螺仪零点/长按切换灯带）
│   ├── bsp_can.c          # CAN 底层（驱动器通信帧封装）
│   └── ...                # 其余外设
├── MPU6050/               # MPU6050 六轴传感器驱动（备用 IMU）
├── BALANCE/               # 核心控制层
│   ├── balance.c          # 运动控制主任务（100Hz）、运动学逆解、平滑、PI
│   ├── control.c          # 控制逻辑
│   ├── filter.c           # 姿态滤波
│   ├── robot_select_init.c# 车型选择（电位器 ADC 分段）+ 底盘参数初始化
│   ├── system.c           # 系统初始化、全局变量
│   ├── show.c             # OLED 数据显示任务
│   ├── read_distance.c    # 超声波测距任务
│   └── DataScope_DP.C     # 上位机波形显示协议
├── HUB_DRIVER/            # 轮毂驱动器协议层
│   ├── DriverInit.c       # CAN 驱动器初始化与通信
│   ├── DriverInit_485.c   # 485 驱动器初始化与通信
│   └── HubDriver.c        # 轮毂电机驱动接口（速度/位置/力矩模式）
├── AutoRecharge/          # 自动回充模块（红外引导 + 上位机导航回充）
├── OBJ/                   # Keil 编译输出（.o/.hex/.map 等，不入库）
├── keilkilll.bat          # 一键清理 Keil 编译中间文件脚本
├── *.bin                  # 固件编译产物（不入库，可删除）
└── 更新记录.txt            # 版本更新日志
```

## 三、软件架构

### 3.1 FreeRTOS 任务

| 任务名 | 优先级 | 堆栈(字) | 频率 | 功能 |
|--------|--------|----------|------|------|
| `Balance_task` | 4 | 512 | 100Hz | 核心运动控制：协议解析、运动学逆解、电机使能/失能、安全保护 |
| `ICM20948_task` | 3 | 256 | — | IMU 数据读取与姿态解算 |
| `show_task` | 3 | 2048 | — | OLED 显示（调试数据翻页） |
| `led_task` | 3 | 128 | — | LED 灯闪烁 |
| `data_task` | 4 | 512 | — | 串口1/串口3/ CAN 数据发送 |
| `ReadUS_task` | 4 | 512 | — | 超声波测距（仅 S 系列车型） |
| `Get_MotorState_task` | 4 | 128 | — | 驱动器自检状态发送 |
| `start_task` | 1 | 256 | — | 启动任务：创建上述任务后自删除 |

### 3.2 控制链路

```
遥控/APP/ROS/CAN/串口
        │ 三轴目标速度 (Vx, Vz)
        ▼
Drive_Motor() ── 超声波避障/底线防撞 ── 速度平滑(Smooth_control)
        ▼
运动学逆解（差速：MOTOR_A/B；四驱：MOTOR_A/B/C/D）
        ▼
mm/s → RPM（四舍五入）→ 轮毂驱动器（CAN 0x181 / 0x602）
        ▼
编码器反馈（0x185）→ 增量式 PI 速度闭环
```

### 3.3 系统功能开关（`BALANCE/system.h`）

| 宏 | 默认 | 说明 |
|----|------|------|
| `USE_DWT_CORE` | 1 | 使用 Cortex-M 内核计时器（GD32 平台需关闭） |
| `USE_CAN2` | 1 | 使用 CAN2 与第二路驱动器通信 |
| `OLED_DEBUG_MODE` | 1 | OLED 调试模式，按键翻页查看传感器数据 |
| `USE_US_Avoid` | 1 | 底盘超声波避障功能 |
| `USE_IWDG` | 0 | 独立看门狗监视 |
| `USE_RGB_lights` | 1 | RGB 灯带 |

## 四、支持车型

通过板上电位器（ADC 分段）选择车型，`Car_Mode` 共 6 档：

| 车型 | 说明 |
|------|------|
| S300 | 两驱差速，轮距大 |
| S150 | 两驱差速 |
| S100 | 两驱差速（超声波为用户加装） |
| S200 | 四驱无刷（前/后轮各一对驱动器：COB1 0x601、COB2 0x602） |
| SX04 | 暂定车型 |

参数（轮距/轴距/减速比/编码器精度/轮径）在 `robot_select_init.c` 的 `Robot_Init()` 中初始化，编码器精度 = 编码器倍数 × 编码器线数 × 减速比，周长 = 轮径 × π。

## 五、控制方式

| 控制方式 | 通道 | 说明 |
|----------|------|------|
| ROS 控制 | 串口3 | 直接下发三轴目标速度，走底盘 ROS 驱动 |
| APP 控制 | 蓝牙(串口2) | 手机 APP 遥控，支持方向键 8 方向 + 变速；聊天界面发 `debug` 可输出调试数据 |
| 航模遥控 | 接收机 | 摇杆控制前进/转向，油门通道变速；外八摇杆开启自动回充、内八摇杆切换避障、左右掰杆清除电机报错 |
| PS2 手柄 | SPI | 当前车型不配备（无硬件），代码保留 |
| CAN 控制 | CAN1 | 帧 ID 0x181（控制）、0x185（编码器反馈） |
| 串口控制 | 串口1 | 直接下发三轴目标速度 |

多控制方式通过 `Control_Mode` 位标志位互斥切换（`Set_Control_Mode` / `Get_Control_Mode`）。

## 六、核心功能与安全保护

1. **底盘自检**：`Self_CheckingFlag` 位标志位覆盖左右电机过流/过载/电流异常/编码器异常/速度异常/霍尔未插/驱动器过欠压/驱动器离线/EEPROM 错误/低电量/急停按下/回充装备离线/红外丢失等 20+ 项，开机第 5 秒主动上报一次。
2. **安全策略**：CAN/串口/ROS 控制命令丢失超过 1 秒自动停止小车（上位机可解除，`SecurityPLY=1` 时 LED 常亮提示）。
3. **低电量保护**：电压低于 20V（`MIN_VOL`）持续 2 秒后锁电机，灯带紫色慢闪，开启自动回充时仅允许回充装备控制。
4. **自动回充**：`AutoRecharge` 模块，支持上位机导航回充（无红外时）与充电桩红外引导回充（39ms/52ms 红外信号），回充中实时监测充电电压/电流。
5. **超声波避障与防撞**：`USE_US_Avoid` 下非 ROS/串口/CAN 控制时可通过航模开启避障；`Ultrasonic_safeguard` 为底线防撞——前后 <30cm、左右 <10cm 时禁止前进/后退，任何控制方式下均生效。
6. **驻车模式**：无目标速度且平地静止超 10 秒进入驻车模式（驱动器输出功率减小），收到控制指令自动解除；斜坡上不驻车、不清 PWM，防止溜坡。
7. **斜坡锁轴**：`auto_pwm_clear` 根据加速度融合值判断车身姿态，斜坡上对驱动器设置锁轴报警状态。
8. **电机闭环**：使能/失能带反馈闭环，收到两个驱动器反馈后才结束命令发送；速度环使用增量式 PI（PWM 限幅 ±7200）。
9. **RGB 灯带协议**：可通过 ROS/上位机设置灯带颜色，帧格式 `7B 04 01 R G B 00 00 00 BCC 7D`（详见 `更新记录.txt` 2023.08.29 条目）。
10. **APP 调试**：APP 聊天界面发送 `debug`，小车逐行输出调试数据。

## 七、编译与烧录

1. 安装 **Keil MDK5**，并安装 ST 官方支持包 `Keil.STM32F4xx_DFP`（工程使用版本 2.17.1）。
2. 打开 `USER/` 目录下的 Keil 工程文件（`.uvprojx`，Target 名称：FreeRTOS）。
3. 编译宏已配置：`STM32F40_41xxx, USE_STDPERIPH_DRIVER, __FPU_PRESENT=1, __TARGET_FPU_VFP, ARM_MATH_CM4, __CC_ARM`。
4. 编译通过后使用 **ST-Link / J-Link** 下载 `OBJ/` 目录下生成的 hex 文件，或直接烧录根目录的 bin 固件（编译产物，不入库）。
5. 若工程编译出现中间文件混乱，可运行根目录 `keilkilll.bat` 清理后再重新编译。

> 注意：源码注释中多处提示“GD32 平台需关闭 `USE_DWT_CORE`”，如使用 GD32 兼容芯片请按注释调整。

## 八、版本更新记录

详见仓库内 `更新记录.txt`，主要里程碑：

- 2022.12.16：第 1 次整理底盘资料
- 2023.01.12：新增底盘自检，修复低概率溜车事件
- 2023.08.29：命令丢失 1 秒停车安全保护；支持 ROS 修改 RGB 灯带
- 2023.11.08：CAN 帧 ID 调整（0x183→0x181 控制，0x185 编码器反馈）；低电量紫灯提示并锁电机
- 2023.12.20：修复 CAN 连续发送延迟问题；新增四驱（S200）车型
- 2024.05.22：APP `debug` 调试功能；新增驻车模式
- 2024.10.24：底盘无状态时灯带熄灭省电；S200 参数优化

## 九、Git 版本管理

### 9.1 初始化仓库（首次）

```bash
# 1. 进入项目根目录
cd D:\UserFiles\Desktop\面试\C63A

# 2. 初始化 Git 仓库
git init

# 3. 添加远程仓库（GitHub 或 Gitee 先创建空仓库，把地址替换成你自己的）
git remote add origin https://github.com/你的用户名/wheeled-robot-mcu.git
# 或 Gitee：
# git remote add origin https://gitee.com/你的用户名/wheeled-robot-mcu.git
```

### 9.2 提交代码

```bash
# 查看当前状态（红色 = 未跟踪/已修改）
git status

# 添加全部文件（已配置 .gitignore，会跳过 OBJ/编译产物等）
git add .

# 提交到本地仓库
git commit -m "初始提交：轮式移动机器人底盘 MCU 控制代码"

# 推送（首次加 -u，之后直接 git push）
git push -u origin master
```

### 9.3 日常更新

```bash
git add .
git commit -m "更新说明：..."
git push
```

### 9.4 常用命令速查

| 命令 | 作用 |
|------|------|
| `git init` | 初始化仓库 |
| `git add .` | 暂存所有改动 |
| `git commit -m "说明"` | 提交到本地 |
| `git push` | 推送到远程 |
| `git pull` | 拉取远程更新 |
| `git status` | 查看状态 |
| `git log --oneline` | 查看提交历史 |
| `git clone 地址` | 克隆仓库到本地 |
| `git checkout -b 分支名` | 新建并切换分支 |

### 9.5 注意事项

- **不要提交 `OBJ/` 目录**（编译产物）、根目录 bin 固件、`*.uvguix*`（Keil 个人界面配置）和 `.vscode/`（本机配置），项目已提供 `.gitignore` 自动排除。
- 首次推送前需配置 Git 身份：

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

- 如推送到 GitHub 提示认证失败，推荐使用 **Personal Access Token（PAT）** 作为密码，或安装 [GitHub Desktop](https://desktop.github.com/) 图形化操作。

---

## 十、参考资料

- FreeRTOS 官方文档：<https://www.freertos.org>
- ST STM32F4 标准外设库文档：<https://www.st.com>
- CMSIS：<https://www.arm.com/technology/cmsis>

---

*轮式移动机器人底盘底层 MCU 控制代码，源码注释为双语（中文/英文）。*
