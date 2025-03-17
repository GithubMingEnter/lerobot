export repo_name="dm_test$1"
# --force-override 1\
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/dm.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/$repo_name \
  --tags dm food eval \
  --warmup-time-s  5\
  --episode-time-s 100 \
  --reset-time-s 10 \
  --num-episodes 50 \
  --push-to-hub 0