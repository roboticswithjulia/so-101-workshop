#!/usr/bin/env bash
# Torna a baixar els vídeos de muntatge del SO-101 (Hugging Face LeRobot).
set -euo pipefail
cd "$(dirname "$0")"
BASE="https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/lerobot"
for nom in Joint1_v2 Joint2_v2 Joint3_v2 Joint4_v2 Joint5_v2 Gripper_v2; do
    echo "Baixant $nom.mp4"
    curl -sSfL -o "$nom.mp4" "$BASE/$nom.mp4"
done
