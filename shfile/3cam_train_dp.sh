export repo_name="act_dm_3cam_test$1"
  # resume=true\
python lerobot/scripts/train.py \
  dataset_repo_id=${HF_USER}/${repo_name} \
  policy=diffusion_dm_3cam_real \
  env=dm_real \
  hydra.run.dir=outputs/train/diffusion_${repo_name} \
  hydra.job.name=diffusion_${repo_name} \
  device=cuda:0 \
  wandb.enable=false\