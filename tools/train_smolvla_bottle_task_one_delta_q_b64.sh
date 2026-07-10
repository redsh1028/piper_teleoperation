#!/usr/bin/env bash
set -euo pipefail
source ~/piper_smolvla/.venv/bin/activate

ROOT=/home/hong/lerobot_datasets/bottle_task_one_delta_q
GOOD_EPISODES=$(python - <<'PY'
from pathlib import Path
import json

root = Path('/home/hong/lerobot_datasets/bottle_task_one_delta_q')
info = json.loads((root / 'meta/info.json').read_text())
bad = set()
bad_path = root / 'bad_episodes.txt'
if bad_path.exists():
    bad = {
        int(x.strip())
        for x in bad_path.read_text().splitlines()
        if x.strip() and not x.strip().startswith('#')
    }
good = [i for i in range(int(info['total_episodes'])) if i not in bad]
print(str(good).replace(' ', ''))
PY
)
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/home/hong/piper_smolvla/outputs/smolvla_bottle_task_one_delta_q_b64_${STAMP}
LOG=/home/hong/piper_smolvla/logs/smolvla_bottle_task_one_delta_q_b64_${STAMP}.log

mkdir -p /home/hong/piper_smolvla/logs /home/hong/piper_smolvla/outputs

echo "$LOG" > /home/hong/piper_smolvla/logs/latest_train_bottle_task_one_delta_q_b64.log

echo "Output: $OUT"
echo "Log: $LOG"
echo "Dataset root: $ROOT"
echo "Good episodes: $GOOD_EPISODES"
python - <<'PY'
from pathlib import Path
import json
from collections import Counter

root = Path('/home/hong/lerobot_datasets/bottle_task_one_delta_q')
info = json.loads((root / 'meta/info.json').read_text())
bad_path = root / 'bad_episodes.txt'
meta_path = root / 'meta/episode_metadata.jsonl'
bad = set()
if bad_path.exists():
    bad = {
        int(x.strip())
        for x in bad_path.read_text().splitlines()
        if x.strip() and not x.strip().startswith('#')
    }
meta = [json.loads(x) for x in meta_path.read_text().splitlines() if x.strip()]
good_meta = [m for m in meta if int(m['episode_index']) not in bad]
print('Dataset total_episodes:', info.get('total_episodes'))
print('Dataset total_frames:', info.get('total_frames'))
print('Bad episodes:', sorted(bad))
print('Good episodes:', len(good_meta))
print('Task labels:', dict(Counter(m.get('task_label', '') for m in good_meta)))
print('Delta action:', info.get('delta_q_action'))
PY

CUDA_VISIBLE_DEVICES=0 lerobot-train \
  --dataset.repo_id local/piper_bottle_task_one_delta_q \
  --dataset.root "$ROOT" \
  --dataset.episodes "$GOOD_EPISODES" \
  --policy.type smolvla \
  --policy.push_to_hub false \
  --policy.use_amp true \
  --policy.optimizer_lr 1e-4 \
  --policy.scheduler_warmup_steps 1000 \
  --policy.scheduler_decay_steps 20000 \
  --policy.scheduler_decay_lr 2.5e-6 \
  --output_dir "$OUT" \
  --job_name smolvla_bottle_task_one_delta_q_b64 \
  --batch_size 64 \
  --num_workers 2 \
  --steps 20000 \
  --log_freq 20 \
  --eval_freq 100000 \
  --save_freq 2500 \
  --wandb.enable true \
  --wandb.project piper_smolvla 2>&1 | tee "$LOG"
