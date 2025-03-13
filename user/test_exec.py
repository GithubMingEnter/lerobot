from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.scripts.control_robot import busy_wait
import torch
import time
from lerobot.common.robot_devices.robots.configs import KochRobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.motors.configs import DynamixelMotorsBusConfig
from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.robot_devices.cameras.configs import OpenCVCameraConfig

leader_config = DynamixelMotorsBusConfig(
    port="/dev/ttyLeaderL",
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
    port="/dev/ttyFollowerR",
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
leader_arm = DynamixelMotorsBus(leader_config)
follower_arm = DynamixelMotorsBus(follower_config)
robot_config = KochRobotConfig(
    leader_arms={"main": leader_config},
    follower_arms={"main": follower_config},
    cameras={"laptop": OpenCVCameraConfig(0, fps=30, width=640, height=480),
        "phone": OpenCVCameraConfig(2, fps=30, width=640, height=480),
        },  
)
robot = ManipulatorRobot(robot_config)

robot.connect()

inference_time_s = 60
fps = 30
device = "cuda"  # TODO: 在 Mac 上，使用 "mps" 或 "cpu"

ckpt_path = "outputs/train/act_koch_test2/checkpoints/last/pretrained_model"
policy = ACTPolicy.from_pretrained(ckpt_path)
policy.to(device)

for _ in range(inference_time_s * fps):
    start_time = time.perf_counter()

    # 读取跟随臂状态并访问相机的帧
    observation = robot.capture_observation()

    # 转换为 pytorch 格式：通道优先，浮点数 [0,1]，带有批量维度
    for name in observation:
        if "image" in name:
            observation[name] = observation[name].type(torch.float32) / 255
            observation[name] = observation[name].permute(2, 0, 1).contiguous()
        observation[name] = observation[name].unsqueeze(0)
        observation[name] = observation[name].to(device)

    # 根据当前观察计算下一个动作
    action = policy.select_action(observation)
    # 去掉批量维度
    action = action.squeeze(0)
    # 移动到 CPU，如果还未移动
    action = action.to("cpu")
    # 告诉机器人移动
    robot.send_action(action)

    dt_s = time.perf_counter() - start_time
    busy_wait(1 / fps - dt_s)