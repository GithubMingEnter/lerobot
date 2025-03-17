export repo_name="dm_test$1"
echo $repo_name
  # --force-override 1 \
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/dm.yaml \
  --fps 30 \
  --root data \
  --repo-id ${HF_USER}/eval_${repo_name}_$(date +%m%d)_2 \
  --tags dm tutorial eval \
  --warmup-time-s 5 \
  --episode-time-s 100 \
  --reset-time-s 10 \
  --num-episodes 5 \
  -p outputs/train/dm_test7_0227_1433/checkpoints/080000/pretrained_model \
  --push-to-hub 0