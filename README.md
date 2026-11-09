# SO101 Xavier NX

Jetson Xavier NX、Ubuntu 20.04、ROS2 Foxy、Python 3.8，so101机械臂控制工程。  
入口：`lerobot_nx.py`：

| 状态 | 作用 | 进入方式 |
|---|---|---|
| 从臂控制 | 执行 ROS2 话题命令，或云端控制 | 启动默认进入；示教时按 `c` 返回 |
| 主从示教 | 主臂控制从臂，同时记录相机和关节原始数据 | 在从臂控制状态按 `r` |

按 `q` 或 `Ctrl+C` 退出，任意切换都会先失能从臂扭矩。

```text
lerobot_nx.py           入口
config.py               启动参数
state/
  runtime.py            状态机与状态切换
  teaching.py           主从示教
  storage.py            episode 目录、样本行与图像写入
  control/
    base.py             控制后端
    ros2_backend.py     ROS2 话题控制
    cloud_backend.py    云端 VLA 推理控制
    vla_client.py        裸 TCP NDJSON 客户端（观测上行 / chunk 下行）
hardware/
  motors/
    joints.py           关节名称、电机 ID 与编码器分辨率约定
    calibration.py      LeRobot 标定 JSON 与角度/编码器刻度转换
    safety.py           单周期目标增量限制
    feetech.py          STS3215 串口控制、标定校验与扭矩控制
  camera.py             RealSense ROS2 彩色和对齐深度图像订阅
  keyboard.py           r、c、q 键盘控制接收
```

## 安装
```bash
source /opt/ros/foxy/setup.bash
cd lerobot_nx
/usr/bin/python3 -m pip install --user -r requirements.txt
```

运行用户需要串口权限。临时配置方式：

```bash
sudo usermod -aG dialout "$USER"
```

## 启动

```bash
python3 ./lerobot_nx.py
```

主臂配置（`leader_port`、`leader_calibration`）保持 `None` 时为仅从臂模式

RealSense
```bash
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

程序启动后默认处于从臂控制状态。首次启动需要在终端按回车确认工作空间安全。
## 从臂控制状态

默认以60hz发布当前状态；示教采集与主从跟随频率由 `Config.fps`（默认 30）控制。收到第一条合法命令前保持从臂扭矩失能；连续默认 0.5 秒未收到新命令时再次失能。

关节数组控制顺序，单位为标定后的角度：

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

测试命令（发布频率必须高于超时阈值）：

```bash
ros2 topic pub -r 30 /so101/control/command std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

在 `config.py` 中设置 `control_backend = "cloud"` 可选择云端 VLA 推理后端：NX 持续上传观测包，
VLA 服务返回动作 chunk，后端在 60Hz 主循环里按时间步进执行。限幅、扭矩生命周期与异常兜底
与 ROS2 后端完全相同（继承自基类）。

### 云端 VLA 控制

- **观测端口**（`vla_obs_url`）：NX 主动连接并逐行推送观测包，服务端只收不回。包体：
  ```json
  {"protocol_version": 1, "robot_id": "so101_nx_01", "timestamp_ns": 0,
   "image_stamp_ns": 0, "image_age_ms": 0.0, "image_jpeg_b64": "...",
   "joint_names": ["shoulder_pan", "..."], "joint_degrees": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
  ```
  图像为 base64 JPEG（`jpeg_quality` 控制质量），超过 `max_image_age_ms` 的陈旧帧不组包，
  实际图像年龄随包上报。关节位置与图像在同一处读取，但两者并非严格同刻，以
  `image_age_ms` 为准。
- **chunk 端口**（`vla_chunk_url`）：NX 主动连接后阻塞读行，服务端按自身频率推送：
  ```json
  {"protocol_version": 1, "robot_id": "so101_nx_01", "chunk_id": 1,
   "obs_timestamp_ns": 0, "actions": [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
   "fps": 30.0, "exec_steps_hint": 0}
  ```

内部数据流：组包线程（生产者）→ 有界队列（满丢最旧）→ 上传线程（消费者）→ 观测端口；
chunk 订阅线程校验后按到达时间展开成带截止时间的步进队列，60Hz 主循环按时间戳消费。
chunk 间隔内在最后目标上位置保持；断线按指数退避（上限 5 秒）自动重连；超过
`inference_timeout_s` 没有新 chunk 时失能从臂，恢复后重新使能。


## 主从示教状态

仅记录 RGB 时设置 `record_depth = False`。

在收到新鲜的彩色和深度帧以前，从臂保持扭矩失能。采集时程序写入
RGB、深度、关节状态、实际动作、时间戳和两份标定文件。按 `c` 后正常结束 episode：

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
