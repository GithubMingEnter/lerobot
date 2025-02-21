from lerobot.common.robot_devices.robots.configs import KochRobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.motors.configs import DynamixelMotorsBusConfig
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus

leader_config = DynamixelMotorsBusConfig(
    port="/dev/ttyACM1",
    motors={
        # name: (index, model)
        "shoulder_pan": (1, "xl330-m077"),
        "shoulder_lift": (2, "xl330-m077"),
        "elbow_flex": (3, "xl330-m077"),
        "wrist_flex": (4, "xl330-m077"),
        "wrist_roll": (5, "xl330-m077"),
        "gripper": (6, "xl330-m077"),
    },
)

follower_config = DynamixelMotorsBusConfig(
    port="/dev/ttyACM2",
    motors={
        # name: (index, model)
        "shoulder_pan": (1, "xl430-w250"),
        "shoulder_lift": (2, "xl430-w250"),
        "elbow_flex": (3, "xl330-m288"),
        "wrist_flex": (4, "xl330-m288"),
        "wrist_roll": (5, "xl330-m288"),
        "gripper": (6, "xl330-m288"),
    },
)
robot_config = KochRobotConfig(
    leader_arms={"main": leader_config},
    follower_arms={"main": follower_config},
    cameras={},  # We don't use any camera for now
)
robot = ManipulatorRobot(robot_config)

robot.connect()
