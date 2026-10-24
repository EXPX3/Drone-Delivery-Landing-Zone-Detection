#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
image="ddlzd-semantic:2026"
weights="${repo_root}/pcds/semantic_experiment/models/yoloe-11s-seg.pt"
clip_weights="${repo_root}/pcds/semantic_experiment/models/mobileclip_blt.ts"

if ! docker image inspect "${image}" >/dev/null 2>&1; then
  docker build -t "${image}" -f "${repo_root}/scripts/semantic_lz/Dockerfile" "${repo_root}"
fi
if [[ ! -f "${weights}" || ! -f "${clip_weights}" ]]; then
  echo "Semantic model files are missing under pcds/semantic_experiment/models." >&2
  echo "Run scripts/semantic_lz/prepare_models.sh first." >&2
  exit 1
fi

export DDLZD_RVIZ_IMAGE="${image}"
exec "${repo_root}/scripts/lz_rviz_viewer/run_rviz_replay.sh" "$@"

