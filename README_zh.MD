# Franka 遥操作数据采集与回放指南

## 简介
本项目是用于 Franka Research 3 机器人（兼容 Franka Panda）的遥操作数据采集工具，嵌入在 [LeRobot](https://github.com/huggingface/lerobot.git) 框架中。

它支持机器人的主从同构映射控制，并以 LeRobot 数据集格式收集遥操作数据。

<p align="center">
  <img src="assets/robot.png" alt="Record" width="600">
  <br>
</p>

### 新功能支持

现在我们支持使用 [SpaceMouse](https://3dconnexion.com/) 设备进行末端位姿控制。

<p align="center">
  <img src="assets/quest3s.jpg" alt="Record" width="600">
  <br>
</p>

SpaceMouse 是一个 6 轴输入设备，可用于控制机器人的末端位姿。左按钮用于打开夹爪，右按钮用于关闭夹爪。

### 新功能支持 2

现在我们支持使用 Oculus Quest 3/3s 设备进行末端位姿控制。

<p align="center">
  <img src="assets/spacemouse.png" alt="Oculus Quest" width="600">
  <br>
</p>

Oculus Quest 控制器提供 6 自由度跟踪，实现直观的末端执行器控制。右手控制器用于机器人操作，控制方式如下：

- **RG（右手握持键）**：必须按下才能启动机器人运动
- **RTr（右手扳机）**：控制夹爪（按下关闭，松开打开）
- **A 按钮**：请求机器人复位
- **右手控制器位姿**：控制末端执行器的增量位姿

## 0. Franka 控制说明
在这个版本中，使用[Polymetis](https://polymetis-docs.github.io/)对Franka 进行控制，由于Polymetis不支持python3.10，并且推荐单独在NUC上运行，因此在这里用Zerorpc进行远程通信实现，具体流程如下：
### 0.1 在NUC上安装[Polymetis](https://polymetis-docs.github.io/)
我们开源了针对franka优化后的[Polymetis](https://github.com/Shenzhaolong1330/fairo-franka/tree/main)源码，详细参考[安装教程](https://github.com/Shenzhaolong1330/fairo-franka/blob/main/polymetis/FRANKA.md)进行安装。
### 0.2 在NUC上启动Server
分别启动[robot interface](https://github.com/Shenzhaolong1330/fairo-franka/blob/main/polymetis/polymetis/python/scripts/launch_robot.py)，[gripper interface](https://github.com/Shenzhaolong1330/fairo-franka/blob/main/polymetis/polymetis/python/scripts/launch_gripper.py)和[interface server](https://github.com/Shenzhaolong1330/fairo-franka/blob/main/polymetis/polymetis/python/polymetis/franka_interface_server.py)

```
<!-- ### 3. 在本地启动[interface client](lerobot_robot_franka\lerobot_robot_franka\franka_interface_client.py)
在尝试链接Franka前，启动[interface client](lerobot_robot_franka\lerobot_robot_franka\franka_interface_client.py)
```
# On the local machine
python franka_interface_client.py
```
[interface client](lerobot_robot_franka\lerobot_robot_franka\franka_interface_client.py)中的函数功能与Polymetis中的函数功能基本一致，使用方法可以参考[Polymetis文档](https://polymetis-docs.github.io/)。 -->


## 1. 环境配置

### 1.1 创建并激活 conda 环境
```bash
conda create -n franka_data python=3.10
conda activate franka_data
```

### 1.2 安装 lerobot
```bash
# 安装指定版本 0.3.4
# git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
pip install -e .
```

### 1.3 克隆并安装 Franka 遥操作控制源码
```bash
mkdir franka_data_collection && cd franka_data_collection
git clone https://github.com/Shenzhaolong1330/lerobot_franka_isoteleop.git
cd lerobot_franka_isoteleop
pip install -e .
```


### 1.4 查看可运行的命令
运行以下命令以查看所有可用的脚本：
```bash
franka-help
# ==================================================
#  Franka Teleoperation Utilities - Command Reference
# ==================================================

# Core Commands:
#   franka-record           Record teleoperation dataset
#   franka-replay           Replay a recorded dataset
#   franka-visualize        Visualize recorded dataset

# Utility Commands:
#   utils-joint-offsets   Compute joint offsets for teleoperation

# Tool Commands:
#   tools-check-dataset   Check local dataset information
#   tools-check-rs        Retrieve connected RealSense camera serial numbers

# Shell Tools:
#   map_gripper.sh        Map Gripper Serial Port
#   check_master_port.sh  Get the Master Arm's Persistent Serial Identifier

# Test Commands:
#   test-gripper-ctrl     Run gripper control command (operate the gripper)

# --------------------------------------------------
#  Tip: Use 'franka-help' anytime to see this summary.
# ==================================================
```
## 2. 获取和配置必要参数

### 2.1 获取 RealSense 相机序列号
请确保每次仅连接一个相机:
```bash
tools-check-rs
```

### 2.2 固定夹爪串口映射
例如，将夹爪映射到 `/dev/franka_left_gripper`, 执行此操作前，请确保仅连接一个夹爪的usb设备。
```bash
map_gripper.sh franka_left_gripper
```
随后，将设定的映射值填入`cfg.yaml`中的`gripper_port`字段
### 2.3 获取主臂的固定串口标识符
执行此操作前，请确保仅连接一个主臂的usb设备。
```bash
check_master_port.sh
```
随后，将获取的串口标识符填入`cfg.yaml`中的`port`字段，并修改设备访问权限:
```bash
sudo chmod 666 <your_master_port>
```
### 2.4 获取主臂-从臂关节角的误差周期
> **⚠️ 注意：在记录数据前，务必完成此步骤以设置正确的误差周期。否则，从臂可能发生意外的动作。**

在执行此操作前，请先手动拖动主臂，使其关节角与从臂的当前关节角尽量保持一致。然后运行以下命令来计算主臂与从臂关节角的误差周期：
```bash
utils-joint-offsets
```
随后，请将脚本输出的`joint_offsets`值填入`cfg.yaml`中的对应配置项

### 2.5 Oculus Quest 设置（用于 Oculus 遥操作模式）

如果您计划使用 Oculus Quest 进行遥操作，请按照以下步骤设置设备：

#### 2.5.1 安装 ADB（Android 调试桥）

ADB 是 Oculus Quest 与计算机之间通信所必需的工具。

```bash
# 在 Ubuntu 上
sudo apt install android-tools-adb

# 验证安装
adb version
```

#### 2.5.2 在 Oculus Quest 上启用开发者模式

1. 在 [Meta for Developers](https://developer.oculus.com/manage/organizations/create/) 创建或加入开发者组织
2. 在手机上打开 Meta Quest 应用
3. 进入 **设置** → 选择您的设备 → **更多设置** → **开发者模式**
4. 启用 **开发者模式** 开关

#### 2.5.3 连接 Oculus Quest 到计算机

**方式 A：USB 连接（推荐用于初始设置）**

1. 使用 USB-C 线缆将 Oculus Quest 连接到计算机
2. 佩戴头显并在提示时允许 USB 调试
3. 勾选 **始终允许来自此计算机**
4. 验证连接：
```bash
adb devices
# 预期输出：
# List of devices attached
# <device_id>    device
```

**方式 B：无线连接（操作更便捷）**

1. 首先通过 USB 线缆连接
2. 确保 Oculus Quest 和计算机连接到同一网络
3. 获取 Oculus Quest 的 IP 地址：
```bash
adb shell ip route
# 查找 "src" 后面的 IP 地址，例如 192.168.110.62
```
4. 在 `record_cfg.yaml` 中配置 IP：
```yaml
teleop:
  oculus_config:
    ip: "192.168.110.62"  # 您的 Oculus Quest IP 地址
```

#### 2.5.4 安装 Oculus Reader APK

Oculus Reader APK 已随本项目预打包。安装方法：

```bash
# 进入 APK 目录
cd lerobot_teleoperator_franka/lerobot_teleoperator_franka/oculus/oculus_reader/APK

# 将 APK 安装到 Oculus Quest
adb install -r teleop-debug.apk
```

安装完成后，应用将出现在您的 Oculus Quest 库中的 **未知来源** 下。

#### 2.5.5 在 record_cfg.yaml 中配置 Oculus 遥操作

```yaml
record:
  control_mode: "oculus"  # 设置控制模式为 oculus
  
  teleop:
    control_mode: "oculus"
    oculus_config:
      ip: "192.168.110.62"  # Oculus Quest IP 地址
      use_gripper: True
      pose_scaler: [2.0, 1.5]  # [位置缩放, 姿态缩放]
      channel_signs: [1, 1, 1, -1, -1, 1]  # 各轴方向符号 [x, y, z, rx, ry, rz]
```

#### 2.5.6 Oculus 控制器操作说明

| 控制键 | 功能 |
|--------|------|
| **RG（右手握持键）** | 按住以启动机器人运动 |
| **RTr（右手扳机）** | 按下关闭夹爪，松开打开夹爪 |
| **A 按钮** | 请求机器人复位 |
| **右手控制器位姿** | 控制末端执行器增量位姿 |

#### 2.5.7 坐标系映射

Oculus 坐标系映射到机器人坐标系如下：

| Oculus 轴 | 机器人轴 | 描述 |
|-----------|----------|------|
| X（右） | -Y（左） | 横向移动 |
| Y（上） | Z（上） | 垂直移动 |
| Z（向后） | X（前） | 前后移动 |

#### 2.5.8 测试 Oculus 连接

在录制之前，测试 Oculus 连接：

```python
from lerobot_teleoperator_franka.lerobot_teleoperator_franka.oculus.oculus_robot import OculusRobot

oculus = OculusRobot(ip='192.168.110.62')
while True:
    action = oculus.get_action()
    print(f"增量位姿: {action[:6]}, 夹爪: {action[6]}")
```

#### 2.5.9 故障排除

**连接问题：**
```bash
# 重启 ADB 服务器
adb kill-server
adb start-server

# 检查已连接设备
adb devices
```

**停止 Oculus 应用：**
```bash
adb shell am force-stop com.rail.oculus.teleop
```

**重新安装 APK：**
```bash
adb uninstall com.rail.oculus.teleop
adb install -r teleop-debug.apk
```

## 3. 数据集记录

### 3.1 上传数据到 Hugging Face（可选）
1. 在 `cfg.yaml` 中设置：
```yaml
push_to_hub: True
```
2. 获取 Hugging Face 账号的 token 并登录：
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} 
huggingface-cli whoami  # 检查是否登录成功
```

### 3.2 开始记录
1. 打开 Franka 示教器上的 **Remote Control**。
2. 确认 `cfg.yaml` 中参数配置正确。
3. 为了确保数据收集过程规范，请先阅读 `9. 数据集命名与记录详解` 以了解数据记录细节，然后再运行以下命令：
```bash
franka-record
```
<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>Figure 1: Record</b>
</p>

## 4. 数据集回放
```bash
franka-replay #注意cfg配置
```

## 5. 数据集可视化
```bash
franka-visualize #注意cfg配置
```
<p align="center">
  <img src="assets/visualize.png" alt="Visualization" width="600">
  <br>
  <b>Figure 2: Visualization</b>
</p>

## 6. 数据集追加与恢复
如果你已经在指定的 `repo_id` 下录制过数据集，可以在 `cfg.yaml` 中将 `resume` 设置为 `True`，并在 `resume_dataset` 中填写要追加的数据集名称，以便在现有数据集的基础上继续录制。然后运行以下命令：
```bash
franka-record
```

## 7. 数据集合并
如果你在不同阶段分别录制了数据集，请确保各阶段的数据集具有不同的 `repo_id`, 完成录制后，可通过以下命令将它们合并为一个数据集: 
```bash
lerobot-edit-dataset 
    --repo_id <merged_repo_id> 
    --operation.type merge 
    --operation.repo_ids "['<repo_id_1>', '<repo_id2>']"
```
- 更多数据集处理命令，请参考 [LeRobot](https://huggingface.co/docs/lerobot/using_dataset_tools)

## 8. 录制控制按键说明
1. **右方向键**  
   1. 按下右方向键: 结束当前 episode 的录制并保存数据，程序将暂停等待。
   2. 进入**复位**阶段: 按回车键以继续（此时从臂会直接跟随主臂运动），请通过主臂将从臂复位至初始位姿。
   3. 再次按下右方向键，开始录制下一个episode
2. **左方向键**  
   - 按下后：重新开始当前 episode 的录制

3. **Esc 键**  
   - 按下后：退出整个录制任务，并保存当前已录制的数据

4. **Ctrl+c或抛出异常**
   - 按下或发生异常时：自动进入异常处理阶段，提示用户是否删除当前未完成的录制数据
  
## 9. 数据集命名与记录详解
### 1. 数据集命名
<p align="center">
  <img src="assets/dataset.png" alt="dataset" width="600">
  <br>
  <b>Figure 3: Dataset</b>
</p>

<p align="center">
  <img src="assets/dataset_info.png" alt="dataset_info" width="600">
  <br>
  <b>Figure 4: Dataset Info</b>
</p>

1. 数据集默认保存在 `~/.cache/huggingface/lerobot` 目录下，包含三类内容：

   - `dataset_info.txt`：自动记录本地数据集信息，包括以下字段：`record_id`、`name`、`task`、`date`、`version`、`user_info` 和 `type`。其中，`user_info` 可以通过 `cfg.yaml` 中的 `user_notes` 进行注解。

   - `dataset_info_backup`：当通过 `tools-check-dataset` 手动更新 `dataset_info.txt` 时，保存旧的记录备份。

   - 数据集文件夹：存放实际的数据集内容。

  
2. 数据集命名格式为 `[description]_[date]_[version]`。其中：
   - `description` 来源于 `cfg.yaml` 中的 `repo_id=<user_name>/<description>`；
   - `date` 会自动生成；
   - `version` 会根据是否存在同名数据集( `repo_id` 相同)自动生成新版本号。
3. description 命名规则：`task.description -> Verb_SourceObject_prep_TargetObject`。  
   即将 `cfg.yaml` 中的 `task.description` 按照上述格式映射生成。例如：  
   `Pick up the green cube and put it into the trash bin -> pick_greencube_into_trashbin`。

### 2. 数据记录说明
1. 完成步骤 `2. 获取和配置必要参数`。
2. 根据任务内容，在 `cfg.yaml` 的 `task.description` 中填写指令，并根据数据集命名规则设置 `repo_id`。
3. 检查并调整 `cfg.yaml` 中的其他参数，确保配置正确。
4. 运行命令 `franka-record`，为确保安全， 请保证主臂与从臂关节角近似，`franka-record` 会自动执行 `2.4 获取主臂-从臂关节角误差周期` 检查，然后按照 `8. 录制控制按键说明` 完成录制操作。
5. 数据记录结束后，数据集同级目录会生成 `dataset_info.txt` 文件，用于保存本地数据集信息。如果手动删除过数据集，可通过以下命令更新记录信息：
```bash
tools-check-dataset