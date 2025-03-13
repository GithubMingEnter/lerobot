python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/koch_test2 \
  --policy.type=act \
  --output_dir=outputs/train/act_koch_test2 \
  --job_name=act_koch_test2 \
  --device=cuda \
  --wandb.enable=false \
  --dataset.local_files_only=true