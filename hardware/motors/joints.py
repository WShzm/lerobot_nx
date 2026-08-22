"""所有 NX 端组件共享的 SO101 关节约定。"""

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
MOTOR_IDS = dict(zip(JOINT_NAMES, range(1, len(JOINT_NAMES) + 1)))
STS3215_RESOLUTION = 4095
