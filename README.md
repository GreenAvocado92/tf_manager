# tf_manager

`tf_manager` 维护相机与机器人之间的标定 TF 链，并将点云从相机光学坐标系转换到机器人基座坐标系。

## 组件与默认坐标链

| 节点 | 职责 |
| --- | --- |
| `calibration_tf_broadcaster` | 从 YAML 发布静态 TF，并将 `PoseStamped` 转发为动态 TF。 |
| `pointcloud_tf_transformer` | 查询 TF，逐点转换 `PointCloud2` 的 `x`、`y`、`z` 字段。 |
| `pointcloud_consumer` | `pointcloud_tf_transformer` 的 Python 实现：查询 TF 并逐点转换 `PointCloud2`。 |
| `tf_matrix_consumer` | 定期输出指定两 frame 间的 4×4 齐次矩阵，供诊断使用。 |

默认配置由 `config/calibration.yaml` 和 `config/realsense_pointcloud_transform.yaml` 提供：

```text
base_link --(/ur/tcp_pose)--> tool0 --(static)--> camera_link --(static)--> camera_optical_frame
```

因此，默认点云转换查询并应用：

```text
T_base_link_camera_optical_frame
```

其中 `/ur/tcp_pose` 的消息类型必须为 `geometry_msgs/msg/PoseStamped`，其位置与姿态应表达为 `tool0` 在 `base_link` 中的位姿。

## 构建与启动

在工作区根目录执行：

```bash
colcon build --packages-select tf_manager
source install/setup.bash
ros2 launch tf_manager system_tf.launch.py
```

`system_tf.launch.py` 同时启动标定 TF 广播和点云转换。可替换配置文件：

```bash
ros2 launch tf_manager system_tf.launch.py \
  calibration_yaml:=/absolute/path/calibration.yaml \
  pointcloud_config:=/absolute/path/realsense_pointcloud_transform.yaml
```

## 私有 TF 话题

通过 `system_tf.launch.py` 启动时，本项目不会使用全局 `/tf`、`/tf_static`，而是使用私有话题：

```text
/tf_manager/tf
/tf_manager/tf_static
```

`calibration_tf_broadcaster` 在这些话题发布，`pointcloud_tf_transformer` 也只从这些话题查询。这样全局 TF 中其他节点发布的同名 frame 不会参与项目内的点云变换。

隔离后，项目自己的 TF 链必须完整：`/ur/tcp_pose` 需要持续提供 `base_link -> tool0`，否则无法获得 `T_base_link_camera_optical_frame`。

### 其他节点接入私有 TF 链

需要使用该标定链的其他节点，也必须同时重映射 `/tf` 和 `/tf_static`。这适用于通过 `tf2_ros::TransformListener` 查询 TF 的消费者，也适用于通过 `TransformBroadcaster` 发布 TF 的广播者。

在 launch 文件中为节点添加：

```python
Node(
    package='your_package',
    executable='your_node',
    remappings=[
        ('/tf', '/tf_manager/tf'),
        ('/tf_static', '/tf_manager/tf_static'),
    ],
)
```

直接运行节点时，可传入 ROS remapping 参数：

```bash
ros2 run your_package your_node \
  --ros-args \
  -r /tf:=/tf_manager/tf \
  -r /tf_static:=/tf_manager/tf_static
```

同一条私有 TF 链上的发布者和消费者必须使用相同 remapping；若一个节点仍向全局 `/tf` 发布，而另一个节点从 `/tf_manager/tf` 查询，两者无法互相发现。使用 RViz 查看该链时也应重映射：

```bash
rviz2 --ros-args \
  -r /tf:=/tf_manager/tf \
  -r /tf_static:=/tf_manager/tf_static
```

若第三方节点未使用标准 `tf2_ros` 的 `/tf`、`/tf_static`，请按其自身提供的 TF 话题或参数接口配置。

## 标定配置

### 动态 TF

`config/calibration.yaml` 的 `dynamic_transforms` 定义 Pose 输入及目标 TF：

```yaml
dynamic_transforms:
  - topic: /ur/tcp_pose
    parent_frame: base_link
    child_frame: tool0
```

当前实现的 parent 选择规则为：若消息 `header.frame_id` 为空，使用配置的 `parent_frame`；若非空，直接使用消息的 `header.frame_id`。因此生产环境中应确保上游始终填写 `base_link`，或不填写该字段。错误的 `header.frame_id` 会改变实际发布的 parent frame。

用于测试默认链路的示例：

```bash
ros2 topic pub --rate 10 /ur/tcp_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.3, y: 0.1, z: 0.4}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

不要让其他节点同时向 `/tf_manager/tf` 发布以 `tool0` 为 child 的 TF。

### 静态 TF

`static_transforms` 使用 4×4 齐次矩阵定义 `parent_frame -> child_frame`：

```yaml
static_transforms:
  - parent_frame: tool0
    child_frame: camera_link
    transform_matrix:
      - [r11, r12, r13, tx]
      - [r21, r22, r23, ty]
      - [r31, r32, r33, tz]
      - [0.0, 0.0, 0.0, 1.0]
```

平移单位为米，左上 3×3 为旋转矩阵。节点要求矩阵为 4×4，且最后一行是 `[0, 0, 0, 1]`。配置方仍需自行保证旋转矩阵正交、行列式为 `+1`，以及所有数值有限。

## 点云转换

默认输入与输出话题为：

```text
输入：/camera/camera/depth/color/points
输出：/camera/depth/points_in_base
输入 frame：camera_optical_frame
输出 frame：base_link
```

`realsense_pointcloud_transform.yaml` 支持以下字段：

```yaml
input_xyz_unit: m       # 仅接受 m 或 mm
cloud_topic: /input
output_topic: /output
source_frame_id: source_frame
target_frame_id: target_frame
```

节点根据输入点云时间戳查询 `T_target_source`，最长等待 100 ms。查询成功时发布转换后的点云；查询失败时，仍会向 `output_topic` 原样发布输入点云，不修改其 header、XYZ 或其他字段。因此下游节点必须检查 `header.frame_id`：`base_link` 表示该帧已转换，其他 frame 表示该帧为未转换的透传数据。

### 单位约定

点云转换内部及输出统一使用**毫米（mm）**：

- `input_xyz_unit: m`：输入 XYZ 先乘以 1000；
- `input_xyz_unit: mm`：输入 XYZ 直接使用；
- 标定 TF 平移仍以米配置，节点计算时转为 mm；
- 输出 `PointCloud2` 的 XYZ 字段始终是 mm。

`PointCloud2` 的 header 没有单位字段。所有订阅输出点云的节点都必须按 mm 解释 XYZ；未适配的 ROS/PCL 节点常默认米制，直接接入会产生 1000 倍尺度错误。

当前实现不会校验输入点云 `header.frame_id`，而是始终按 YAML 的 `source_frame_id` 解释点坐标，并将输出 header 写为 `target_frame_id`。因此必须保证 YAML 与实际输入点云坐标系相符。

可单独运行点云节点。若希望仍使用项目私有 TF，需显式添加同样的 remapping：

```bash
ros2 run tf_manager pointcloud_tf_transformer \
  --ros-args \
  -p pointcloud_config:=$(ros2 pkg prefix tf_manager)/share/tf_manager/config/realsense_pointcloud_transform.yaml \
  -r /tf:=/tf_manager/tf \
  -r /tf_static:=/tf_manager/tf_static
```

### Python 点云转换实现

`pointcloud_consumer` 是 `pointcloud_tf_transformer` 的 Python 等价实现。它使用同一份 YAML
配置，按输入点云时间戳查询 `T_target_source`，逐点转换 `float32` 的 `x`、`y`、`z` 字段，并将
成功转换后的输出统一为 **mm**。它保留其他点字段和点云元数据；TF 不可用时原样透传输入点云。

先构建并加载环境：

```bash
colcon build --packages-select tf_manager
source install/setup.bash
```

启动示例：

```bash
ros2 run tf_manager pointcloud_consumer \
  --ros-args \
  -p pointcloud_config:=$(ros2 pkg prefix tf_manager)/share/tf_manager/config/realsense_pointcloud_transform.yaml \
  -r /tf:=/tf_manager/tf \
  -r /tf_static:=/tf_manager/tf_static
```

和 C++ 节点一样，`pointcloud_config` 为必填参数；如需使用项目私有 TF 链，必须同时重映射
`/tf` 与 `/tf_static`。

## 查询与诊断

`tf_matrix_consumer` 默认查询 `T_camera_link_base_link`。查询默认标定链中的相机相对基座变换时，可运行：

```bash
ros2 run tf_manager tf_matrix_consumer \
  --ros-args \
  -p target_frame:=base_link \
  -p source_frame:=camera_optical_frame \
  -r /tf:=/tf_manager/tf \
  -r /tf_static:=/tf_manager/tf_static
```

`T_target_source` 表示把 source frame 中表达的点转换到 target frame。输出矩阵中的平移遵循 TF 约定，为米。

同样，使用 `tf2_echo` 诊断私有链路时需要 remapping：

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame \
  --ros-args \
  -r /tf:=/tf_manager/tf \
  -r /tf_static:=/tf_manager/tf_static
```
