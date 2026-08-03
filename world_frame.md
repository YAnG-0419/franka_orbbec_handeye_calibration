# 世界坐标系采集说明

本说明对应 `world_frame.json`。JSON 保持为机器可读格式，不能直接写入
注释；本文件记录该次采集的坐标约定和结果含义。

## 本次采集质量

- 采集时长：8 秒。
- 接收 IMU 样本：1501；接受样本：1500。
- 平均加速度模长：`9.782311954 m/s²`，相对标准重力加速度 `9.80665 m/s²`
  的误差约为 0.25%，可用于定义竖直方向。
- 机械臂 X 轴的实测夹角：`1.453001°`。
- 输出坐标系为右手正交系：`X × Y = Z`。

## 世界坐标系约定

- **原点**：左右机械臂 `fr3_link0` 原点的中点。
- **+Z**：相机静止时，`camera_accel_optical_frame` 加速度均值经 TF 变换到
  `camera_color_optical_frame` 后的单位方向。按静止加速度计测得支撑加速度
  （向上）的约定，它与物理重力方向相反，因此为世界 `+Z`。
- **+X**：左右机械臂各自 `+X` 方向的平均值，投影到垂直于世界 `+Z` 的平面后
  归一化得到。该定义保留两臂约 1.45° 的实际安装差异，而不强制修改任一手眼标定。
- **+Y**：由 `+Z × +X` 计算，并再用 `+X = +Y × +Z` 正交化。

`world_frame.json` 的 `world_axes_expressed_in_camera` 给出的三个向量，是世界
三个单位轴分别在 `camera_color_optical_frame` 中的表达。

## 变换的读取方式

JSON 中的 `world_to_*` 均遵循 ROS TF 的父到子变换：

```text
p_world = R_world_child · p_child + t_world_child
```

因此：

- `world_to_camera_color_optical_frame`：用于发布 `world` 为相机光学系父坐标的
  静态 TF。
- `world_to_left_base` 与 `world_to_right_base`：左右底座相对于该世界系的结果；
  原点取中点，所以两者的平移严格互为相反数。
- 四元数顺序为 ROS 约定的 `(x, y, z, w)`，平移单位为米。

## 使用前提与限制

1. 采样期间相机必须静止；移动、撞击或振动会污染重力方向。
2. IMU 只能提供俯仰和横滚约束；绕世界 Z 的偏航由双臂 X 轴方向确定。
3. 该结果依赖左右手眼标定以及
   `camera_accel_optical_frame -> camera_color_optical_frame` 的 TF 正确。
4. 若相机或任一机械臂底座被移动，应重新采样并重新生成结果。
