export repo_name="act_dm_3cam_test$1"
python lerobot/scripts/train.py \
  dataset_repo_id=${HF_USER}/${repo_name} \
  policy=act_dm_3cam_real6 \
  env=dm_real \
  hydra.run.dir=outputs/train/${repo_name}_6$(date +%m%d_%H%M) \
  hydra.job.name=act_dm_${repo_name}_6_$(date +%m%d_%H%M) \
  device=cuda:1 \
  wandb.enable=false\