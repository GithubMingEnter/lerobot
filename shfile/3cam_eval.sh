export repo_name="act_dm_3cam_test$1"
# --force-override 1 \
# \  # \
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/dm_3cam.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/eval_${repo_name}_r1 \
  --tags dm tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 120 \
  --reset-time-s 10 \
  --num-episodes 5 \
  -p outputs/train/act_dm_3cam_test5_2_0111_2117/checkpoints/last/pretrained_model\
  --push-to-hub 0