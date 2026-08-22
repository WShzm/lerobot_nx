"""替代完整 LeRobot 硬件栈的精简 STS3215 协议 0 客户端。"""

from __future__ import annotations

from typing import Dict, Tuple

try:
    import scservo_sdk as scs
except ImportError as exc:
    raise SystemExit("缺少 Feetech SDK，请使用 ROS2 的 Python 解释器安装 requirements.txt。") from exc

from .calibration import Calibration, raw_to_degrees
from .joints import JOINT_NAMES, MOTOR_IDS

# STS3215 控制表地址。
ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11
ADDR_P_COEFFICIENT = 21
ADDR_D_COEFFICIENT = 22
ADDR_I_COEFFICIENT = 23
ADDR_OPERATING_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56


class FeetechArm:
    """通过串口适配器访问一台已标定的六关节 SO101 机械臂。"""

    def __init__(self, port_name: str, calibration: Dict[str, Calibration]) -> None:
        self.port_name = port_name
        self.calibration = calibration
        self.port = scs.PortHandler(port_name)
        self.packet = scs.PacketHandler(0)
        self.connected = False

    def connect(self) -> None:
        if not self.port.openPort():
            raise ConnectionError("无法打开串口 {}".format(self.port_name))
        if not self.port.setBaudRate(1_000_000):
            self.port.closePort()
            raise ConnectionError("无法将串口 {} 设置为 1 Mbps".format(self.port_name))
        self.connected = True
        self.read_positions_raw()

    def close(self) -> None:
        if self.connected:
            self.port.closePort()
            self.connected = False

    def _check(self, comm_result: int, error: int, operation: str) -> None:
        if comm_result != scs.COMM_SUCCESS:
            raise ConnectionError(
                "{}: {}: {}".format(self.port_name, operation, self.packet.getTxRxResult(comm_result))
            )
        if error:
            raise RuntimeError(
                "{}: {}: {}".format(self.port_name, operation, self.packet.getRxPacketError(error))
            )

    def _read_2byte(self, motor_id: int, address: int, operation: str) -> int:
        value, comm_result, error = self.packet.read2ByteTxRx(self.port, motor_id, address)
        self._check(comm_result, error, operation)
        return int(value)

    def read_positions_raw(self) -> Dict[str, int]:
        return {
            joint: self._read_2byte(motor_id, ADDR_PRESENT_POSITION, "读取 {} 的当前位置".format(joint))
            for joint, motor_id in MOTOR_IDS.items()
        }

    def read_positions_degrees(self) -> Tuple[Dict[str, int], Dict[str, float]]:
        raw = self.read_positions_raw()
        degrees = {joint: raw_to_degrees(raw[joint], self.calibration[joint]) for joint in JOINT_NAMES}
        return raw, degrees

    def verify_position_limits(self) -> None:
        """拒绝与控制器 EEPROM 不匹配的 JSON 标定文件。"""
        for joint, motor_id in MOTOR_IDS.items():
            minimum = self._read_2byte(
                motor_id, ADDR_MIN_POSITION_LIMIT, "读取 {} 的最小位置限制".format(joint)
            )
            maximum = self._read_2byte(
                motor_id, ADDR_MAX_POSITION_LIMIT, "读取 {} 的最大位置限制".format(joint)
            )
            expected = self.calibration[joint]
            if minimum != expected.range_min or maximum != expected.range_max:
                raise RuntimeError(
                    "{}：{} 的标定不匹配：电机行程为 [{}, {}]，JSON 行程为 [{}, {}]。"
                    "请重新写入匹配的 LeRobot 标定。".format(
                        self.port_name,
                        joint,
                        minimum,
                        maximum,
                        expected.range_min,
                        expected.range_max,
                    )
                )

    def write_register(self, address: int, value: int, length: int = 1) -> None:
        for joint, motor_id in MOTOR_IDS.items():
            if length == 1:
                comm_result, error = self.packet.write1ByteTxRx(self.port, motor_id, address, value)
            else:
                comm_result, error = self.packet.write2ByteTxRx(self.port, motor_id, address, value)
            self._check(comm_result, error, "向 {} 写入寄存器 {}".format(joint, address))

    def disable_torque(self) -> None:
        self.write_register(ADDR_TORQUE_ENABLE, 0)
        self.write_register(ADDR_LOCK, 0)

    def enable_torque(self) -> None:
        self.write_register(ADDR_TORQUE_ENABLE, 1)
        self.write_register(ADDR_LOCK, 1)

    def configure_follower(self) -> None:
        """应用 LeRobot 的默认位置模式设置，不修改标定信息。"""
        self.disable_torque()
        self.write_register(ADDR_OPERATING_MODE, 0)
        self.write_register(ADDR_P_COEFFICIENT, 16)
        self.write_register(ADDR_I_COEFFICIENT, 0)
        self.write_register(ADDR_D_COEFFICIENT, 32)

    def write_goal_positions_raw(self, goals: Dict[str, int]) -> None:
        writer = scs.GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 2)
        try:
            for joint, motor_id in MOTOR_IDS.items():
                value = int(goals[joint])
                params = [value & 0xFF, (value >> 8) & 0xFF]
                if not writer.addParam(motor_id, params):
                    raise RuntimeError("无法为 {} 加入目标位置队列".format(joint))
            comm_result = writer.txPacket()
            if comm_result != scs.COMM_SUCCESS:
                raise ConnectionError(
                    "{}：写入目标位置失败：{}".format(self.port_name, self.packet.getTxRxResult(comm_result))
                )
        finally:
            writer.clearParam()
