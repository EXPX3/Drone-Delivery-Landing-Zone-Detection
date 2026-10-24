#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
model_dir="${repo_root}/pcds/semantic_experiment/models"
mkdir -p "${model_dir}"
docker build -t ddlzd-semantic:2026 -f "${repo_root}/scripts/semantic_lz/Dockerfile" "${repo_root}"
docker run --rm --gpus all -v "${model_dir}:/models" -w /models \
  --entrypoint bash ddlzd-semantic:2026 -lc \
  'python3 -c '\''from ultralytics import YOLOE; m=YOLOE("yoloe-11s-seg.pt"); m.set_classes(["person","cargo box","cardboard box","plastic crate","car"])'\'''

