# SO101 Xavier NX 运行工程

这是面向 Jetson Xavier NX、Ubuntu 20.04、ROS2 Foxy、Python 3.8 的轻量运行工程。
它不导入 LeRobot、PyTorch、Hugging Face 数据集或视频编码器。任一时刻仅一个运行模式
占用机械臂串口。

```text
so101_nx/
  calibration.py       读取 LeRobot 标定 JSON，并在角度与编码器刻度间转换
  feetech.py           STS3215 串口控制、标定校验与扭矩控制
  ros_camera.py        订阅 RealSense ROS2 彩色和对齐深度图像
  recorder.py          主臂到从臂遥操作及原始数据采集
  topic_control.py     ROS2 控制话题到从臂的执行和状态发布
  cloud_gateway.py     云端动作校验并转发至本地控制话题
```

## 安装

通过 apt/ROS 安装 ROS2 Foxy、`realsense2_camera`、`cv_bridge`、`sensor_msgs`、
`std_msgs` 和 `python3-opencv`。请保留 apt/ROS 提供的 OpenCV；用 pip 安装
`opencv-python` 或 `opencv-python-headless` 可能破坏 `cv_bridge` 使用的二进制 ABI。

```bash
source /opt/ros/foxy/setup.bash
cd lerobot_nx
/usr/bin/python3 -m pip install --user -r requirements.txt
```

若系统没有 `cv2`，使用系统包安装：

```bash
sudo apt install python3-opencv
```

运行用户需要串口权限。临时配置方式如下，之后需要重新登录：

```bash
sudo usermod -aG dialout "$USER"
```

将由 LeRobot 为这对实际主从臂生成的标定 JSON 复制到 NX。禁止使用其他机械臂的标定文件。
启动时，工程会对比 JSON 与电机 EEPROM 中的关节行程；不一致时会在使能扭矩前停止。

## 模式一：主从采集

该模式读取已标定的主臂、控制从臂并记录原始数据。先在另一个终端启动 RealSense：

```bash
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

随后采集一个 episode：

```bash
source /opt/ros/foxy/setup.bash
cd lerobot_nx
/usr/bin/python3 record_so101_realsense.py \
  --leader-port /dev/ttyACM0 \
  --follower-port /dev/ttyACM1 \
  --leader-calibration calibration/leader.json \
  --follower-calibration calibration/follower.json \
  --output-root /data/so101_raw \
  --task "抓取方块" \
  --fps 30
```

默认相机话题为 `/camera/camera/color/image_raw` 和
`/camera/camera/aligned_depth_to_color/image_raw`。首次使用前通过
`ros2 topic list | grep image_raw` 检查实际名称；不同时使用 `--color-topic` 与
`--depth-topic` 覆盖。

采集结果是供工作站转换脚本使用的中间格式：

```text
episode_YYYYMMDD_HHMMSS/
  episode.json                         完成标记与任务元数据
  samples.jsonl                        追加写入的关节与图像索引
  leader_calibration.json
  follower_calibration.json
  rgb/000000_<monotonic-ns>.jpg
  depth/000000_<monotonic-ns>.png
```

`action_position_deg` 是经单周期安全限制后实际发给从臂的命令，转换时应写为
LeRobot 的 `action`；`follower_position_deg` 应写为 `observation.state`。`Ctrl+C`
会正常结束 episode 并失能从臂扭矩。异常中断的 episode 会维持 `recording` 状态，转换器应默认拒绝。

## 模式二：话题控制

该模式供 NX 上的 LeRobot 推理节点、其他本地控制程序或云端命令桥接使用。它不订阅相机，
也不记录数据。

```bash
source /opt/ros/foxy/setup.bash
cd lerobot_nx
/usr/bin/python3 topic_control_so101.py \
  --follower-port /dev/ttyACM1 \
  --follower-calibration calibration/follower.json \
  --fps 30
```

仅一个进程可以使用 `/dev/ttyACM1`，不可与采集模式同时启动。控制器收到有效命令前保持扭矩失能；
默认连续 0.5 秒未收到有效命令时再次失能。只有确认动作发布频率稳定后，才调整
`--command-timeout-s`。

所有关节数组使用以下严格顺序，数值单位为标定后的角度：

```text
[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
```

| 话题 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `/so101/control/command` | `std_msgs/Float64MultiArray` | 订阅 | 期望从臂关节位置，六个角度值 |
| `/so101/follower/state` | `std_msgs/Float64MultiArray` | 发布 | 当前从臂关节位置，六个角度值 |
| `/so101/leader/state` | `std_msgs/Float64MultiArray` | 发布 | 同时提供主臂参数时发布主臂状态 |
| `/so101/control/action` | `std_msgs/Float64MultiArray` | 发布 | 实际发送给从臂的安全限制后动作 |
| `/so101/status` | `std_msgs/String` | 发布 | JSON 格式的状态、关节顺序和硬件错误 |

可用下列命令进行台架测试。发布频率必须高于超时阈值：

```bash
ros2 topic pub -r 30 /so101/control/command std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## 云端推理入口

`cloud_command_gateway.py` 是云端推理传输与机械臂执行之间的明确边界。它接收
`/so101/cloud/action` 上的六个标定角度值，检查长度和绝对值范围后，转发到
`/so101/control/command`。硬件控制器始终是唯一串口拥有者，并继续执行新鲜度和相对位移检查。

```bash
source /opt/ros/foxy/setup.bash
cd lerobot_nx
/usr/bin/python3 cloud_command_gateway.py
```

| 话题 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `/so101/cloud/action` | `std_msgs/Float64MultiArray` | 订阅 | 由云端传输适配器送入的动作 |
| `/so101/control/command` | `std_msgs/Float64MultiArray` | 发布 | 经校验后供话题控制器消费的本地命令 |
| `/so101/cloud/status` | `std_msgs/String` | 发布 | JSON 格式的转发或拒绝状态 |

云端传输本身有意保持独立。部署在 NX 的 WebSocket、MQTT、gRPC 或 HTTP 适配器只能向
`/so101/cloud/action` 发布，不能打开串口或直接写电机总线。这样可以将传输重连、鉴权和模型
版本切换隔离在硬件安全边界之外。
