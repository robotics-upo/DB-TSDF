#!/usr/bin/env bash
set -euo pipefail

# ---------------------------------------------------------------------------
# DB-TSDF: MaiCity full dataset fetcher (all sequences)
#
# Fully self-contained: downloads the official MaiCity archive, extracts every
# sequence bag, converts each from ROS 1 (.bag) to ROS 2 (rosbag2) and removes
# every intermediate file, leaving ready-to-play datasets behind.
#
# Datasets are workspace-level artifacts: when the repo lives at the standard
# ROS 2 location <ws>/src/db_tsdf, datasets are placed at <ws>/datasets/
# (alongside build/install/log); otherwise it falls back to <repo>/datasets/.
#
# Source dataset: https://www.ipb.uni-bonn.de/data/mai-city-dataset/
#
# Usage:
#   ./scripts/download_mai_city.sh
#   ros2 bag play <printed dataset path>/mai_city/01
# ---------------------------------------------------------------------------

readonly REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Datasets are workspace-level artifacts (like build/install/log), not source
# code. When the repo follows the standard ROS 2 layout (<ws>/src/db_tsdf),
# place them at the workspace root; otherwise fall back to <repo>/datasets.
if [[ "$(basename "$(dirname "${REPO_DIR}")")" == "src" ]]; then
    DATASETS_ROOT="$(cd "${REPO_DIR}/../.." && pwd)/datasets"
else
    DATASETS_ROOT="${REPO_DIR}/datasets"
fi
readonly DATASETS_ROOT

readonly DATASET_ROOT="${DATASETS_ROOT}/mai_city"
readonly TARBALL_URL="https://www.ipb.uni-bonn.de/html/projects/mai_city/mai_city.tar.gz"

readonly WORK_DIR="$(mktemp -d)"
readonly TARBALL="${WORK_DIR}/mai_city.tar.gz"

trap 'rm -rf "${WORK_DIR}"' EXIT

log()  { printf '[%(%H:%M:%S)T] %s\n' -1 "$*"; }
step() { printf '\n[%(%H:%M:%S)T] >> %s\n' -1 "$*"; }

step "[1/4] Resolving dependencies"
if command -v rosbags-convert >/dev/null 2>&1; then
    log "rosbags-convert already available, skipping installation"
else
    log "Installing 'rosbags' (provides rosbags-convert, ROS 1 -> ROS 2 bag converter)"
    # --user keeps the install confined to this account. --break-system-packages
    # is required on PEP 668 distros (Ubuntu 23.10+/Debian 12+, e.g. the host)
    # but is unknown to older pip versions (e.g. Ubuntu 22.04 inside the Docker
    # image), so only pass it when pip actually supports it.
    PIP_EXTRA_ARGS=()
    if pip3 install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
        PIP_EXTRA_ARGS+=(--break-system-packages)
    fi
    pip3 install --user --quiet "${PIP_EXTRA_ARGS[@]}" rosbags
    export PATH="${HOME}/.local/bin:${PATH}"
fi

step "[2/4] Downloading full MaiCity dataset archive (~3.4 GB)"
log "Source: ${TARBALL_URL}"
log "This will take a while depending on your connection; the download is resumable"
wget --show-progress -q -c -O "${TARBALL}" "${TARBALL_URL}"
log "Download complete"

step "[3/4] Extracting and converting all sequences (ROS 1 .bag -> ROS 2 rosbag2)"
log "Extracting bags from archive..."
tar -xzf "${TARBALL}" -C "${WORK_DIR}" --wildcards "*/bags/*.bag"

mkdir -p "${DATASET_ROOT}"

mapfile -d '' -t BAG_FILES < <(find "${WORK_DIR}" -type f -name "*.bag" -print0)
if [[ "${#BAG_FILES[@]}" -eq 0 ]]; then
    echo "ERROR: no .bag files found inside the archive." >&2
    exit 1
fi
log "Found ${#BAG_FILES[@]} sequence bag(s); converting each to rosbag2..."

for bag in "${BAG_FILES[@]}"; do
    seq="$(basename "${bag}" .bag | grep -oE '[0-9]+' | tail -n1)"
    dst="${DATASET_ROOT}/${seq}"
    log "  - sequence ${seq}: $(basename "${bag}") -> ${dst}/"
    rm -rf "${dst}"
    rosbags-convert --src "${bag}" --dst "${dst}" >/dev/null
done
log "All sequences converted"

step "[4/4] Cleaning up temporary files"
rm -rf "${WORK_DIR}"
log "Removed downloaded archive and intermediate ROS 1 bags"

echo
echo "============================================================================"
echo " MaiCity dataset ready at: ${DATASET_ROOT}/<sequence>"
echo " Play sequence 01 with:  ros2 bag play ${DATASET_ROOT}/01"
echo "============================================================================"
