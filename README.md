# LinkArm CLI SDK

🌐 Language: [English](./README_EN.md) | 中文

用于控制 **LinkArm 机械臂** 的 Python 命令行工具与 SDK。

本项目提供：

- 面向终端用户的 **CLI 控制接口**
- 面向开发者的 **Python 库调用接口**
- 面向 AI / Agent 的 **标准化控制入口**
- 面向新手用户的 **快速上手指南**

它适合用于：

- 模块化调试
- 教学演示
- 自动化脚本
- Raspberry Pi / Jetson 机器人本体集成
- AI 模型或代理程序控制机械臂

![](./README/A.png)

---

## 目录

- [项目简介](#项目简介)
- [功能概览](#功能概览)
- [系统架构](#系统架构)
- [硬件与供电说明](#硬件与供电说明)
- [不同型号的接线说明](#不同型号的接线说明)
- [环境准备](#环境准备)
- [第一次连接机械臂](#第一次连接机械臂)
- [查找串口号并修改配置文件](#查找串口号并修改配置文件)
- [舵机中位校准说明（非常重要）](#舵机中位校准说明非常重要)
- [快速让机械臂动起来](#快速让机械臂动起来)
- [配置文件示例](#配置文件示例)
- [CLI 命令总览](#cli-命令总览)
- [CLI 命令详解](#cli-命令详解)
- [交互式 Shell 模式](#交互式-shell-模式)
- [AI 与自动化程序调用建议](#ai-与自动化程序调用建议)
- [Python 脚本如何使用本库](#python-脚本如何使用本库)
- [多机械臂同时控制](#多机械臂同时控制)
- [不同平台上手步骤](#不同平台上手步骤)
- [机械臂贴纸校准示意](#机械臂贴纸校准示意)
- [最小可运行示例](#最小可运行示例)
- [常见问题](#常见问题)

---

## 项目简介

`linkarm.py` 同时具备两种角色：

1. **CLI 工具**  
   直接在终端运行，发送命令控制机械臂。
2. **Python 库**  
   在自己的 Python 脚本中导入并实例化 `RobotController`。

本 SDK 通过串口与机械臂控制板通信，支持：

- 关节空间控制
- 笛卡尔空间逆解控制（IK）
- 正解读取（FK）
- 夹爪控制
- 舵机扭矩开关与扭矩限制
- LED 控制
- PWM 输出控制
- 交互式命令行
- JSON 输出，便于 AI 程序解析
- 多命令批量执行

---

## 功能概览

### 运动控制

- 单关节控制
- 多关节同步控制
- 可靠队列单关节控制
- 笛卡尔插值运动 `ik`
- 笛卡尔立即运动 `ik-now`
- FPV 风格立即运动 `fpv`

### 外设控制

- 夹爪控制 `gripper`
- 板载 LED 控制 `led`
- PWM 输出控制 `pwm`

### 状态与模型能力

- 当前状态读取 `status`
- 当前关节反馈转正解坐标 `fk`

### 配置与维护

- 舵机扭矩开关 `torque-lock`
- 舵机扭矩限制 `torque-limit`
- 全部关节断扭矩 `torque-off-all`
- 记录当前中位 `set-middle`
- 保存中位到配置文件 `save-middle`

### 面向程序与 AI 的能力

- `--json-output` 输出标准 JSON
- `exec "cmd1; cmd2; cmd3"` 一次执行多个动作
- 交互式 shell 模式

---

## 系统架构

LinkArm 的控制系统可以理解为 3 层：

```text
+---------------------------+
|        User Layer         |
|---------------------------|
| Terminal / Scripts / AI   |
+-------------+-------------+
              |
              v
+---------------------------+
|       CLI SDK Layer       |
|---------------------------|
| linkarm.py                |
| - Command parser          |
| - Motion API              |
| - IK / FK                 |
| - Servo control           |
| - LED / PWM               |
+-------------+-------------+
              |
              v
+---------------------------+
|      Hardware Layer       |
|---------------------------|
| Robot Controller Board    |
| Serial communication      |
| Bus servos                |
+---------------------------+
```

通信方式为：

- 主机（PC / Raspberry Pi / Jetson）
- 通过 USB 串口
- 与机械臂控制板通信

---

## 硬件与供电说明

目前我们有两款机械臂：

- **LinkArm-M**
- **LinkArm-LT**

两款产品都采用：

- **12V 直流供电**
- 电源需满足 **3A 供电能力**

同时也支持：

- **3S 锂电池供电**
- 电压范围约 **9V ~ 12.6V**

这使得产品非常适合集成到：

- 移动底盘
- 巡检平台
- 教学机器人
- 远程操作机器人

> 注意：USB 线主要用于通信，不用于给机械臂主动力系统供电。  
> 使用前请先正确接好 12V 电源或 3S 锂电池。

---

## 不同型号的接线说明

### LinkArm-M

如果你使用的是 **LinkArm-M**：

- 请将 USB 线连接到 **TTL Node (A)** 的 **Type-C 接口**
- 保留配置文件中的默认波特率 **500000**
- **不需要修改波特率**

也就是通常保持：

```json
"serial_baudrate": 500000
```

### LinkArm-LT

如果你使用的是 **LinkArm-LT**：

- 请将 USB 线连接到 **Robot Driver 驱动板** 上印有 **UART** 的 **Type-C 接口**
- **不要接错到那个印有 USB 的 Type-C 接口**
- 需要将 `arm_config.json` 中的波特率从 **500000 改为 1000000**

将：

```json
"serial_baudrate": 500000
```

改为：

```json
"serial_baudrate": 1000000
```

---

## 环境准备

### 1. 安装 Python

建议安装：

- Python 3.8 及以上

检查版本：

```bash
python --version
```

### 2. 获取项目

```bash
git clone https://github.com/EffectsMachine/linkarm_python_sdk.git
cd linkarm_module
```

### 3. 创建虚拟环境（推荐）

#### Windows

```bash
python -m venv venv
venv\Scripts\activate
```

#### Linux / Raspberry Pi / Jetson

```bash
python3 -m venv venv
source venv/bin/activate
```

### 4. 安装依赖

```bash
pip install -r requirements.txt
```

---

## 第一次连接机械臂

建议按以下顺序操作：

1. 给机械臂接好 12V 电源或 3s 锂电池
2. 用 USB 线把机械臂连接到电脑 / 树莓派 / Jetson
3. 查找串口号
4. 修改 `arm_config.json`
5. 按贴纸填写 `servo_middle`
6. 运行 `status` 测试通信
7. 先测试夹爪，再测试 IK

---

## 查找串口号并修改配置文件

SDK 会读取 `arm_config.json` 中的：

- `default_device_serial_ports`
- `serial_baudrate`

作为默认连接参数。

例如：

```json
{
  "linkarm": {
    "default_device_serial_ports": "COM42",
    "serial_baudrate": 500000
  }
}
```

### Windows 下查找串口号

#### 方法 1：设备管理器

打开：

- 设备管理器
- 端口（COM 和 LPT）

找到类似：

```text
USB-Enhanced-SERIAL CH343 (COM42)
```

此时串口号就是：

```text
COM42
```

#### 方法 2：PowerShell

```powershell
mode
```

### Linux / Raspberry Pi / Jetson 下查找串口号

插入设备前后分别执行：

```bash
ls /dev/tty*
```

常见设备名：

- `/dev/ttyUSB0`
- `/dev/ttyACM0`

也可以执行：

```bash
dmesg | tail
```

或：

```bash
python -m serial.tools.list_ports
```

### 修改配置文件中的串口号

找到：

```json
"default_device_serial_ports": "COM42"
```

将其改成你自己的实际串口号。

例如：

#### Windows

```json
"default_device_serial_ports": "COM7"
```

#### Linux / Raspberry Pi / Jetson

```json
"default_device_serial_ports": "/dev/ttyUSB0"
```

> 注意：字符串两边必须使用 **英文双引号**。

---

## 舵机中位校准说明（非常重要）

本产品使用的是 **飞特 SCS 系列总线舵机**。

这类舵机的一个重要特点是：

- **舵机中位不能保存在舵机内部**
- 只能保存在 `arm_config.json` 中的 `servo_middle` 数组里

也就是说：

- **每台机械臂的中位都不一样**
- 机械臂出厂时，我们会把该机械臂对应的中位数组打印在贴纸上
- 贴纸会贴在对应机械臂上

例如贴纸上写：

```text
[513,508,327,632]
```

这表示这台机械臂的 4 个舵机中位分别是：

- 513
- 508
- 327
- 632

### 用户需要手动修改 `arm_config.json`

找到配置文件中的：

```json
"servo_middle": [
  511,
  511,
  511,
  511
]
```

将其替换为贴纸上的真实数组，例如：

```json
"servo_middle": [
  513,
  508,
  327,
  632
]
```

### 这一步非常重要

如果 `servo_middle` 填错，可能导致：

- FK 结果错误
- IK 动作不准
- 机械臂姿态偏移
- 关节运动方向 / 幅度异常
- 末端位置不准

### 修改时请特别注意

- **不要输错数字**
- **必须使用英文符号**
- 使用英文方括号：`[ ]`
- 使用英文逗号：`,`
- **不要使用中文逗号 `，`**
- 不要漏数字

### 如何验证中位参数

当使用 `joints 0 0 0 0` 控制机械臂时，机械臂的姿态如下图所示：

![](./README/C.png)

---

## 快速让机械臂动起来

当你已经：

- 接好电源
- 接好 USB
- 改好串口号
- 改好 `servo_middle`
- 设置好正确波特率

就可以开始测试。

### 1. 测试状态读取

```bash
python linkarm.py status
```

### 2. 先测试夹爪（最安全）

注意：即使当前只发送控制夹爪的指令，机械臂也会迅速摆动到初始位置，在发送动作指令前，请保证已经更改中位配置参数，并保证机械臂工作范围内没有易碎物品，远离儿童。

机械臂的角度控制采用弧度制，具体到夹爪的指令，`0`为张开到最大，数值越小张开的范围越小，`-1`为完全闭合，最小值`-1.5`则会闭合得更近，夹爪本身已经做了扭矩限制，来避免长时间用力夹紧导致舵机发热。

关闭夹爪：

```bash
python linkarm.py gripper -1
```

打开夹爪：

```bash
python linkarm.py gripper 0
```

### 3. 测试单关节动作

`--reliable` 代表该动作为可靠动作，可靠动作是发送了就一定会被执行的，在动作序列中拥有最高优先权，不加此参数的动作为高频动作，用于上位机连续控制关节角度（高频动作只有最新的才会被执行）。

当你使用 `python linkarm.py` 这类单次操作的 CLI 命令行交互时，如果不加 `--reliable` 参数，会导致主程序退出运行时，动作指令来不及被执行，所以要加参数来标记要用可靠动作序列来执行。

```bash
python linkarm.py joint 3 -1 --reliable
```

### 4. 测试机械臂笛卡尔运动

`ik-now`后面的参数代表 `[x, y, z]` 坐标值，单位为`mm`，坐标轴定义如下图所示：

![](./README/B.png)

```bash
python linkarm.py ik-now 250 0 60
```

如果该点在逆解范围内，机械臂会运动到对应位置。  
如果超出范围，则会返回：

```text
IK_FAILED
```

---

## 配置文件示例

下面给出一个更完整的 `arm_config.json` 示例。请根据你的设备实际情况修改。

```json
{
  "linkarm": {
    "device_info_keyword": "CH343",
    "default_device_serial_ports": "COM1",
    "serial_baudrate": 500000,
    "joint_type": "scs",
    "joint_id": [
      31,
      32,
      33,
      34
    ],
    "gripper_torque_limit": 200,
    "node_id": 40,
    "servo_middle": [
      511,
      511,
      511,
      511
    ],
    "joint_direction": [
      1,
      1,
      1,
      1
    ],
    "joint_limit": [
      [-1.5708, 1.5708],
      [-1.5708, 1.5708],
      [-0.8, 2],
      [-1.5, 0]
    ],
    "link_ab": 224.0,
    "link_bc": 145.0,
    "link_cd_1": 24.0,
    "link_cd_2": 120.0,
    "link_de": 120.0,
    "link_ef": 25.0,
    "link_bf_1": 24.0,
    "link_bf_2": 120.0
  },
  "joint": {
    "scs": {
      "joint_range_rad": 3.839724777777778,
      "joint_range_steps": 1024,
      "joint_range_angle": 220.0,
      "id_address": 5,
      "torque_limit_address": 16,
      "torque_lock_address": 40
    },
    "hls": {
      "joint_range_rad": 6.28318530718,
      "joint_range_steps": 4096,
      "joint_range_angle": 360.0
    }
  }
}
```

### LinkArm-LT 配置差异

如果是 **LinkArm-LT**，请特别修改：

```json
"serial_baudrate": 1000000
```

---

## CLI 命令总览

### 一次性命令模式

```bash
python linkarm.py <command> [args]
```

### 当前支持的主要命令

- `status`
- `joints`
- `joint`
- `gripper`
- `fk`
- `ik`
- `ik-now`
- `fpv`
- `led`
- `pwm`
- `torque-lock`
- `torque-limit`
- `torque-off-all`
- `set-middle`
- `save-middle`
- `cancel-ik`
- `exec`
- `shell`

---

## CLI 命令详解

### status

读取当前状态：

```bash
python linkarm.py status
```

### joints

多关节同步控制：

```bash
python linkarm.py joints 0 0.2 -0.3 0
```

带速度与加速度：

```bash
python linkarm.py joints 0 0.2 -0.3 0 --speed 200 --acc 50
```

### joint

单关节控制：

```bash
python linkarm.py joint 1 0.3
```

可靠队列模式：

```bash
python linkarm.py joint 3 -1 --reliable
```

> 对夹爪这类低频但不能丢的动作，建议优先使用 `--reliable` 或 `gripper`。

### gripper

夹爪控制：

```bash
python linkarm.py gripper -1
python linkarm.py gripper 0
```

### ik

笛卡尔插值运动：

```bash
python linkarm.py ik 250 0 60
```

### ik-now

笛卡尔立即运动：

```bash
python linkarm.py ik-now 250 0 60
```

### fpv

FPV 风格控制：

此控制方式主要用于将摄像头安装在机械臂末端，采用第一人称视角来控制机械臂时，使用这种控制方式更加直观。

- 第一个参数是底座的旋转角度，角度采用弧度制。
- 第二个参数是末端点向前伸长的距离，单位mm。
- 第三个参数是末端点的高度，单位mm。

```bash
python linkarm.py fpv 1.0 250 60
```

### led

控制板载 LED 颜色，参数范围：`0 ~ 8`。

- 第一个参数是 R-红色通道，数值越大该通道颜色越亮。
- 第一个参数是 G-绿色通道，数值越大该通道颜色越亮。
- 第一个参数是 B-蓝色通道，数值越大该通道颜色越亮。

```bash
python linkarm.py led 8 0 0
python linkarm.py led 0 8 0
python linkarm.py led 0 0 8
python linkarm.py led 8 8 8
```

### pwm

控制 PWM 输出，当前通道支持：

- `0`
- `1`

PWM 数值支持 `0~1024`，对应 `0~100%` 的PWM，用于控制开关接口的输出电压，100% 的 PWM 代表开关口输出总线的供电电压，用户可使用这个接口来控制 LED 补光灯、电磁铁、电磁阀等直流设备，但是需要注意 PWM 与输出电压并非完全线性的关系。

例如：

```bash
python linkarm.py pwm 0 500
python linkarm.py pwm 1 1025
```

### torque-lock

舵机扭矩开关：

- 第一个参数为舵机 ID。
- 第二个参数为 1 是，代表输出扭矩；0 - 关闭扭矩输出。

```bash
python linkarm.py torque-lock 31 1
python linkarm.py torque-lock 31 0
```

### torque-limit

设置某个舵机的扭矩限制：

- 第一个参数为舵机 ID。
- 第二个参数为最大扭矩，1000代表不限制最大扭矩，200代表限制最大扭矩到20%。

```bash
python linkarm.py torque-limit 34 200
```

### torque-off-all

关闭所有关节扭矩：

所有关节的扭矩都关闭后，可以用手调整机械臂的姿态。

```bash
python linkarm.py torque-off-all
```

### set-middle

将当前反馈位置记录为内存中的 `servo_middle`：

```bash
python linkarm.py set-middle
```

### save-middle

把当前内存中的 `servo_middle` 保存到 `arm_config.json`：

```bash
python linkarm.py save-middle
```

### cancel-ik

取消当前插值 IK 任务：

```bash
python linkarm.py cancel-ik
```

### exec

一次执行多个命令：

```bash
python linkarm.py exec "gripper -1; sleep 1; gripper 0; ik-now 250 0 60"
```

### --json-output

输出标准 JSON，便于程序解析：

```bash
python linkarm.py --json-output status
python linkarm.py --json-output fk
python linkarm.py --json-output exec "status; fk; gripper -1"
```

---

## 交互式 Shell 模式

如果你希望连续输入命令，可以进入 shell 模式：

```bash
python linkarm.py shell
```

也可以直接运行：

```bash
python linkarm.py
```

进入后可输入：

```text
status
gripper -1
sleep 1
gripper 0
ik 250 0 60
fk
led 8 0 0
pwm 0 500
exit
```

---

## AI 与自动化程序调用建议

如果你希望让 AI 模型、Agent 或其它程序控制机械臂，推荐使用：

- `--json-output`
- `exec`

例如：

```bash
python linkarm.py --json-output status
```

也可以：

```bash
python linkarm.py --json-output exec "status; fk; gripper -1; sleep 1; gripper 0"
```

### 推荐 AI 使用策略

1. 先读取 `status`
2. 夹爪动作优先用 `gripper`
3. 即时定位优先用 `ik-now`
4. 平滑移动用 `ik`
5. 使用 `fk` 验证末端位置

---

## Python 脚本如何使用本库

除了 CLI 以外，你也可以在 Python 脚本中直接导入并控制机械臂。

### 最简单的用法

```python
from linkarm import RobotController
import time

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    print(arm.get_latest_feedback())
    arm.gripper_ctrl(-1)
    time.sleep(1)
    arm.gripper_ctrl(0)
```

### 关节控制示例

```python
from linkarm import RobotController

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    arm.move_joint_rad(1, 0.3, speed=100, acc=50, blocking=False)
    arm.move_joints_rad_sync([0.0, 0.2, -0.3, 0.0], speed=200, acc=50, blocking=True)
```

### 可靠队列单关节控制

```python
from linkarm import RobotController
import time

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    arm.move_joint_rad_reliable(3, -1.0, speed=100, acc=50)
    time.sleep(1)
    arm.move_joint_rad_reliable(3, 0.0, speed=100, acc=50)
```

### IK 控制示例

```python
from linkarm import RobotController
import time

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    arm.ik_ctrl([250, 0, 60, 0.0], speed=880)
    time.sleep(2)
```

### 立即运动示例

```python
from linkarm import RobotController

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    ok = arm.ik_ctrl_immediate([250, 0, 60, 0.0])
    print("IK result:", ok)
```

### FK 读取示例

```python
from linkarm import RobotController

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    xyz = arm.get_fk_result()
    print("FK:", xyz)
```

### 灯光与 PWM 控制示例

```python
from linkarm import RobotController
import time

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    arm.set_led_async(8, 0, 0)
    time.sleep(1)
    arm.set_led_async(0, 8, 0)
    time.sleep(1)
    arm.set_pwm_async(0, 500)
```

### 扭矩控制示例

```python
from linkarm import RobotController
import time

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    arm.torque_lock_ctrl(31, 0)
    time.sleep(1)
    arm.torque_lock_ctrl(31, 1)
    arm.torque_limit(34, 200)
```

### 中位校准示例

```python
from linkarm import RobotController
import time

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    arm.torque_off_all_joint()
    time.sleep(1)

    middle = arm.set_arm_middle_as_current_pos()
    print("servo_middle =", middle)

    arm.save_joint_middle()
```

---

## 多机械臂同时控制

如果用户想同时控制多个机械臂，不需要改 SDK 主体，只需要：

1. 复制多个 `arm_config.json`
2. 更改为不同名称，例如：
   - `arm_config_1.json`
   - `arm_config_2.json`
3. 每台机械臂分别填写：
   - 不同的串口号
   - 不同的 `servo_middle`

例如：

### arm_config_1.json

```json
{
  "linkarm": {
    "default_device_serial_ports": "COM7",
    "servo_middle": [513, 508, 327, 632]
  }
}
```

### arm_config_2.json

```json
{
  "linkarm": {
    "default_device_serial_ports": "COM8",
    "servo_middle": [512, 510, 330, 629]
  }
}
```

然后在 Python 脚本中分别实例化：

```python
from linkarm import RobotController

arm1 = RobotController(config_path="arm_config_1.json", communication_mode="direct_servo")
arm2 = RobotController(config_path="arm_config_2.json", communication_mode="direct_servo")

arm1.move_joint_rad(1, 0.2)
arm2.move_joint_rad(1, -0.2)
```

---

## 不同平台上手步骤

### Windows

1. 安装 Python
2. 安装 CH343 驱动(一般情况都是自动安装的)
3. 打开设备管理器确认 COM 口
4. 修改 `arm_config.json` 中的 `default_device_serial_ports`
5. 按贴纸修改 `servo_middle`
6. 运行：

```bash
python linkarm.py status
python linkarm.py gripper -1
python linkarm.py ik-now 250 0 60
```

### Raspberry Pi

1. 安装 Python 3
2. 使用 USB 连接机械臂
3. 用 `ls /dev/tty*` 或 `python -m serial.tools.list_ports` 查找串口
4. 修改 `arm_config.json`
5. 运行：

```bash
python3 linkarm.py status
python3 linkarm.py gripper -1
python3 linkarm.py ik-now 250 0 60
```

### Jetson

1. 安装 Python 3
2. 通过 USB 连接机械臂
3. 检查串口权限
4. 修改 `arm_config.json`
5. 如果是 LinkArm-LT，请确认波特率是否为 `1000000`
6. 运行：

```bash
python3 linkarm.py status
python3 linkarm.py gripper -1
python3 linkarm.py ik-now 250 0 60
```

---

## 机械臂贴纸校准示意

产品到用户手中后，请先查看机械臂机身上的贴纸。

例如贴纸内容：

```text
servo_middle:
[513,508,327,632]
```

然后打开 `arm_config.json`，找到：

```json
"servo_middle": [
  511,
  511,
  511,
  511
]
```

将其替换为：

```json
"servo_middle": [
  513,
  508,
  327,
  632
]
```

推荐流程：

1. 先看贴纸
2. 再看串口号
3. 最后检查波特率
4. 完成后先执行 `status`
5. 再执行 `gripper` 与 `ik-now`

---

## 最小可运行示例

### 示例 1：只用 CLI

```bash
python linkarm.py status
python linkarm.py gripper -1
python linkarm.py gripper 0
python linkarm.py ik-now 250 0 60
```

### 示例 2：批量动作

```bash
python linkarm.py exec "gripper -1; sleep 1; gripper 0; ik-now 250 0 60; fk"
```

### 示例 3：JSON 输出给程序解析

```bash
python linkarm.py --json-output exec "status; fk; gripper -1"
```

### 示例 4：最小 Python 控制脚本

```python
from linkarm import RobotController
import time

with RobotController(
    config_path="arm_config.json",
    communication_mode="direct_servo",
) as arm:
    print("Feedback:", arm.get_latest_feedback())
    print("FK:", arm.get_fk_result())

    arm.gripper_ctrl(-1)
    time.sleep(1)
    arm.gripper_ctrl(0)

    ok = arm.ik_ctrl_immediate([250, 0, 60, 0.0])
    print("IK OK:", ok)
```

---

## 常见问题

### 机械臂不动

请检查：

- 是否接好了 12V 电源
- 电源是否能提供 3A
- USB 是否接对接口
- 串口号是否正确
- 波特率是否正确
- `servo_middle` 是否已按贴纸正确填写

### LinkArm-LT 连接正常但不响应

请优先检查：

- USB 是否接到了 **UART** 标识的 Type-C
- 是否误接到了板上 **USB** 标识的 Type-C
- `serial_baudrate` 是否已改为 `1000000`

### IK_FAILED

说明目标位置不在逆解范围内，或者不可达。

可以先尝试更保守的点位，例如：

```bash
python linkarm.py ik-now 200 0 60
```

### FK 结果明显不对

优先检查：

- `servo_middle` 是否正确
- 是否把其它机械臂的贴纸数据填到了当前机械臂
- JSON 里是否误输入了中文符号

### 找不到串口

#### Windows

- 设备管理器查看 COM 口
- 更换 USB 数据线
- 安装 CH343 / CH340 驱动

#### Linux / Raspberry Pi / Jetson

- 查看 `/dev/ttyUSB0` / `/dev/ttyACM0`
- 执行 `dmesg | tail`
- 确认当前用户有串口权限

### 如何确认 CLI 当前支持哪些命令

```bash
python linkarm.py --help
```

或进入 shell 后输入：

```text
help
```

---

## 许可证

请根据你的项目实际情况填写，例如：

```text
MIT License
```

---

## 致开发者

如果你希望将本 SDK 接入：

- GUI 控制界面
- Web 控制界面
- ROS2 节点
- AI Agent
- 云端调度系统

推荐优先复用 `RobotController` 这一层接口，而不是直接操作底层串口逻辑。
