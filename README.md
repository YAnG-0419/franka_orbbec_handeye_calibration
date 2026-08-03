# FR3 + Orbbec 手眼标定（ROS 2 Jazzy）

基于 MoveIt 的手眼标定工作空间，面向 **Franka Research 3（FR3）** 机械臂与 **Orbbec Gemini 435Le**（网口相机）。支持：

- **眼在手上（eye-in-hand）**：相机装在腕部
- **眼在手外（eye-to-hand）**：相机固定，标定板固定在手上

本仓库还提供双臂场景下的 **重力对齐世界坐标系** 估计与静态 TF 发布。

第三方代码以 git submodule 引入：

| Submodule | 作用 |
|-----------|------|
| `franka_ros2` | FR3 驱动与 `franka_fr3_moveit_config` |
| `OrbbecSDK_ROS2` | Orbbec 相机驱动（含 Gemini 435Le launch） |
| `moveit_calibration` | RViz HandEyeCalibration 插件 |
| `franka_description` | FR3 模型描述 |

本仓库一方包为 `src/fr3_handeye_calibration`，负责把上述组件用 launch / RViz 预设串起来。

更细的安装、编译与排错说明见 [docs/setup.md](docs/setup.md)。

---

## 目录结构

```text
scripts/00_install_tools.sh      # 安装 colcon/rosdep、MoveIt 2、Orbbec 系统依赖
scripts/01_build.sh              # 拉取 submodule + rosdep + colcon 编译
scripts/estimate_world_frame.py  # 用 IMU + 手眼结果估计世界坐标系
src/fr3_handeye_calibration/     # 一包：相机 / 标定 launch + RViz 预设
left_hand_pose.launch.py         # 左臂 eye-to-hand 静态 TF（底座 → 相机）
right_hand_pose.launch.py        # 右臂 eye-to-hand 静态 TF（底座 → 相机）
world_frame.launch.py            # 发布 world → 相机 / 左底座 / 右底座
world_frame.json                 # 世界坐标系估计结果（机器可读）
world_frame.md                   # 该次采集的坐标约定说明
docs/setup.md                    # 安装、编译、运行与排错（英文）
```

---

## 环境要求

- Ubuntu 24.04 + ROS 2 Jazzy
- 系统已安装可被 CMake 找到的 **libfranka**（`find_package(Franka)`）
- 实时内核（连接真机 FR3 时需要）
- 能访问 FR3 的 FCI IP，以及 Gemini 435Le 的网口 IP
- 建议使用干净 shell：先 `conda deactivate`，再 `source /opt/ros/jazzy/setup.bash`

当前实验室相机 IP 示例：`172.16.0.7`（请按实际设备修改）。

---

## 安装与编译

在仓库根目录：

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash

./scripts/00_install_tools.sh   # 装工具与系统依赖（需 sudo）
./scripts/01_build.sh           # 初始化 submodule 并编译
source install/setup.bash
```

若 clone 时未加 `--recurse-submodules`，`01_build.sh` 会自动执行 `git submodule update --init --recursive`。

---

## 手眼标定（单臂）

两个终端分别启动相机与机器人，都需要先：

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 终端 1：相机

先关闭 OrbbecViewer 等占用相机的客户端（435Le 为独占连接）。

```bash
ros2 launch fr3_handeye_calibration camera.launch.py \
  enumerate_net_device:=false \
  net_device_ip:=172.16.0.7
```

确认有图像：

```bash
ros2 topic hz /camera/color/image_raw
```

### 终端 2：机器人 + MoveIt + RViz

眼在手外（相机固定）：

```bash
ros2 launch fr3_handeye_calibration calibration.launch.py \
  mode:=eye_to_hand \
  robot_ip:=<FCI_IP> \
  launch_camera:=false
```

眼在手上（相机在腕部）时把 `mode:=eye_in_hand`。

RViz 会加载 HandEyeCalibration 显示；按插件流程采集样本并求解外参。标定结果可写入本仓库根目录的 `left_hand_pose.launch.py` / `right_hand_pose.launch.py`（静态 TF：`fr3_link0` → `camera_color_optical_frame`）。

---

## 双臂世界坐标系

在左右臂各自完成 eye-to-hand 标定后，可用相机静止时的加速度计估计重力方向，再结合双臂底座位姿，得到统一的 `world` 坐标系。

约定（详见 [world_frame.md](world_frame.md)）：

- **原点**：左右 `fr3_link0` 原点中点
- **+Z**：与物理重力相反（静止加速度计向上）
- **+X**：左右臂 +X 均值投影到垂直于 +Z 的平面
- **+Y**：由 `+Z × +X` 得到，保证右手系

### 1. 估计世界系（不发布 TF）

先启动相机，并开启加速度话题（示例）：

```bash
ros2 launch orbbec_camera gemini435_le.launch.py \
  enumerate_net_device:=false \
  net_device_ip:=172.16.0.7 \
  net_device_port:=8090 \
  enable_accel:=true \
  enable_gyro:=true \
  enable_sync_output_accel_gyro:=false
```

保持相机静止，再运行：

```bash
python3 scripts/estimate_world_frame.py \
  --duration 8 \
  --output world_frame.json
```

脚本会读取 `/camera/accel/sample`，结合内置的左右手眼外参，输出 JSON（可同时打印到终端）。

### 2. 发布静态 TF

```bash
ros2 launch world_frame.launch.py
```

将建立如下 TF 树：

```text
world
├── camera_color_optical_frame
├── left_fr3_link0
└── right_fr3_link0
```

**注意：** 不要与 `left_hand_pose.launch.py` / `right_hand_pose.launch.py` 同时启动。后两者会发布「底座 → 相机」，与 `world_frame.launch.py` 中「世界 → 相机」冲突，导致相机坐标系出现两个父节点。

---

## 单独发布某一侧手眼 TF

仅需要单臂底座相对相机的静态外参时：

```bash
ros2 launch left_hand_pose.launch.py
# 或
ros2 launch right_hand_pose.launch.py
```

---

## 常用命令

```bash
# 列出 Orbbec 设备（序列号 / IP）
ros2 run orbbec_camera list_devices_node

# 确认彩色图在出流
ros2 topic hz /camera/color/image_raw

# 查看 TF
ros2 run tf2_tools view_frames
```

---

## 常见问题（摘要）

| 现象 | 处理 |
|------|------|
| 找不到 `colcon` / `rosdep` | 运行 `scripts/00_install_tools.sh`，或安装 `ros-dev-tools` |
| 编译报 `catkin_pkg` / 用到了 conda Python | `conda deactivate` 后再编（需清掉 `CONDA_PREFIX`） |
| 相机节点起来但没有 `/camera/...` 图像 | 关闭 OrbbecViewer 等其它客户端；确认 `net_device_ip` |
| `libfranka: UDP receive: Timeout` | 检查实时内核、FCI 已激活、能 `ping` 到机器人 IP |
| 标定板轴线乱、提示 intrinsics 异常 | 使用本仓库的 `moveit_calibration` fork 并重新编译、重启 RViz |

完整排错见 [docs/setup.md](docs/setup.md)。

---

## 版本说明（简要）

- `franka_ros2` 需要系统 libfranka ≥ 0.19.0；不要把 libfranka 再做成 submodule 或重编覆盖系统版。
- `moveit_calibration` 使用适配 Jazzy 的 `ros2` 分支（本仓库指向可接受 Orbbec 畸变系数长度的 fork）。
- `OrbbecSDK_ROS2` 必须用 `v2-main`（含 `gemini435_le.launch.py`）；从源码编译以支持网口 435Le。
