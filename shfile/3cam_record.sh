export repo_name="act_dm_3cam_test$1"
# --force-override 1\
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/dm_3cam.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/${repo_name} \
  --tags dm tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 100 \
  --reset-time-s 10 \
  --num-episodes 50 \
  --push-to-hub 0