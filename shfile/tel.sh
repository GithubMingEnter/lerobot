export robot_type=$1

python lerobot/scripts/control_robot.py teleoperate \
       --robot-path lerobot/configs/robot/$robot_type.yaml
    --robot-overrides '~cameras' \
    --display-cameras 0
    