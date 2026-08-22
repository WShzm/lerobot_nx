# SO101 Xavier NX 统一运行时

这是面向 Jetson Xavier NX、Ubuntu 20.04、ROS2 Foxy、Python 3.8 的轻量运行工程。
工程仅有一个启动脚本：`lerobot_nx.py`。程序是一个包含两种状态的状态机：

| 状态 | 作用 | 进入方式 |
|---|---|---|
| 从臂控制 | 执行 ROS2 话题命令，或使用预留的云端控制后端 | 程序启动默认进入；示教时按 `c` 返回 |
| 主从示教 | 主臂控制从臂，同时记录相机和关节原始数据 | 在从臂控制状态按 `r` |

按 `q` 或 `Ctrl+C` 安全退出。任意切换都会先失能从臂扭矩，因此两个状态不会同时占用或控制机械臂。

```text
lerobot_nx.py           唯一入口
config.py               命令行参数、参数校验与 ROS2 话题约定
state/
  runtime.py            双状态状态机与状态切换
  teaching.py           主从示教会话的节拍调度与编排
  storage.py            episode 目录、样本行与图像写入
  control/
    base.py             控制后端生命周期接口
    ros2_backend.py     ROS2 话题控制后端
    cloud_backend.py    云端控制预留接口
hardware/
  motors/
    joints.py           关节名称、电机 ID 与编码器分辨率约定
    calibration.py      LeRobot 标定 JSON 与角度/编码器刻度转换
    safety.py           单周期目标增量限制
    feetech.py          STS3215 串口控制、标定校验与扭矩控制
  camera.py             RealSense ROS2 彩色和对齐深度图像订阅
  keyboard.py           r、c、q 非阻塞键盘读取
```

该工程不导入 LeRobot、PyTorch、Hugging Face 数据集或视频编码器。

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

## 启动

示教状态需要 RealSense。建议在另一个终端启动相机；控制状态不依赖相机：

```bash
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

启动统一运行时。默认控制后端是 ROS2 话题控制：

```bash
source /opt/ros/foxy/setup.bash
cd lerobot_nx
/usr/bin/python3 lerobot_nx.py \
  --leader-port /dev/ttyACM0 \
  --follower-port /dev/ttyACM1 \
  --leader-calibration calibration/leader.json \
  --follower-calibration calibration/follower.json \
  --output-root /data/so101_raw \
  --task "抓取方块" \
  --fps 30
```

程序启动后默认处于从臂控制状态。终端无需按回车即可识别下列按键：

| 按键 | 行为 |
|---|---|
| `r` | 结束当前控制后端，进入示教并创建新 episode |
| `c` | 完成当前 episode，返回从臂控制状态 |
| `q` | 结束当前状态并退出程序 |

首次启动需要在终端按回车确认工作空间安全。对于无人值守启动，可追加
`--skip-start-prompt`，但仅应在已验证机械臂周围安全的场景使用。

## 从臂控制状态

默认 `--control-backend ros2`。控制器订阅控制命令，并以设置的 `--fps` 发布当前状态。
收到第一条合法命令前保持从臂扭矩失能；连续默认 0.5 秒未收到新命令时再次失能，避免执行陈旧动作。

所有关节数组使用以下严格顺序，数值单位为标定后的角度：

```text
[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]
```

| 话题 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `/so101/control/command` | `std_msgs/Float64MultiArray` | 订阅 | 期望从臂关节位置，六个角度值 |
| `/so101/follower/state` | `std_msgs/Float64MultiArray` | 发布 | 当前从臂关节位置，六个角度值 |
| `/so101/leader/state` | `std_msgs/Float64MultiArray` | 发布 | 当前主臂关节位置，六个角度值 |
| `/so101/control/action` | `std_msgs/Float64MultiArray` | 发布 | 实际发送给从臂的安全限制后动作 |
| `/so101/status` | `std_msgs/String` | 发布 | JSON 格式的状态、关节顺序和硬件错误 |

台架测试命令如下。发布频率必须高于超时阈值：

```bash
ros2 topic pub -r 30 /so101/control/command std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

使用 `--control-backend cloud` 可选择云端转发控制的预留后端。当前它只完成相同的
生命周期和安全停机结构，不连接网络也不向电机下发动作。未来 WebSocket、MQTT、gRPC 或
HTTP 适配器应实现为该后端，而不是直接操作串口。

## 主从示教状态

按 `r` 后，程序创建一个 episode，并开始订阅默认的 RealSense 话题：

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`

首次使用前通过 `ros2 topic list | grep image_raw` 检查实际名称；不同时传入
`--color-topic` 与 `--depth-topic`。需要仅记录 RGB 时传入 `--no-depth`。

在收到新鲜的彩色和深度帧以前，从臂保持扭矩失能。采集时，主臂位置驱动从臂，程序写入
RGB JPEG、深度 PNG、关节状态、实际动作、时间戳和两份标定文件。按 `c` 后正常结束 episode：

```text
episode_YYYYMMDD_HHMMSS/
  episode.json                         完成标记与任务元数据
  samples.jsonl                        追加写入的关节与图像索引
  leader_calibration.json
  follower_calibration.json
  rgb/000000_<monotonic-ns>.jpg
  depth/000000_<monotonic-ns>.png
```

该目录是工作站转换脚本的中间格式。`action_position_deg` 是实际发给从臂的命令，转换时应写为
LeRobot 的 `action`；`follower_position_deg` 应写为 `observation.state`。
