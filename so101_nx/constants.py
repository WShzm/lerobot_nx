"""所有 NX 端组件共享的 SO101 约定。"""

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

COMMAND_TOPIC = "/so101/control/command"
FOLLOWER_STATE_TOPIC = "/so101/follower/state"
LEADER_STATE_TOPIC = "/so101/leader/state"
ACTION_TOPIC = "/so101/control/action"
STATUS_TOPIC = "/so101/status"
CLOUD_ACTION_TOPIC = "/so101/cloud/action"
CLOUD_STATUS_TOPIC = "/so101/cloud/status"
