export repo_name="act_dm_3cam_test$1"
# --force-override 1 \
python lerobot/scripts/control_robot.py record \
  --robot-path lerobot/configs/robot/dm_3cam.yaml \
  --fps 30 \
  --repo-id ${HF_USER}/eval_diffusion_${repo_name}q \
  --tags dm dp tutorial eval \
  --warmup-time-s 10 \
  --episode-time-s 130 \
  --reset-time-s 10 \
  --num-episodes 10 \
  --push-to-hub 0 \
  --force-override 1 \
  -p outputs/train/diffusion_${repo_name}q/checkpoints/last/pretrained_model