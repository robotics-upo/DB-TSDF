#!/usr/bin/env bash
set -euo pipefail

# ---------------------------------------------------------------------------
# DB-TSDF — MaiCity quick-test dataset fetcher (sequence 01 only)
#
# Fully self-contained: downloads the official MaiCity archive, extracts only
# the sequence 01 bag, converts it from ROS 1 (.bag) to ROS 2 (rosbag2) and
# removes every intermediate file, leaving a single ready-to-play dataset at
# datasets/mai_city/01/.
#
# Source dataset: https://www.ipb.uni-bonn.de/data/mai-city-dataset/
#
# Usage:
#   ./scripts/download_test.sh
#   ros2 bag play datasets/mai_city/01
# ---------------------------------------------------------------------------

readonly SEQ="01"
readonly ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DATASET_DIR="${ROOT_DIR}/datasets/mai_city/${SEQ}"
readonly TARBALL_URL="https://www.ipb.uni-bonn.de/html/projects/mai_city/mai_city.tar.gz"

readonly WORK_DIR="$(mktemp -d)"
readonly TARBALL="${WORK_DIR}/mai_city.tar.gz"

trap 'rm -rf "${WORK_DIR}"' EXIT

log()  { printf '[%(%H:%M:%S)T] %s\n' -1 "$*"; }
step() { printf '\n[%(%H:%M:%S)T] >> %s\n' -1 "$*"; }

step "[1/4] Resolving dependencies"
if command -v rosbags-convert >/dev/null 2>&1; then
    log "rosbags-convert already available — skipping installation"
else
    log "Installing 'rosbags' (provides rosbags-convert, ROS 1 -> ROS 2 bag converter)"
    # --user keeps the install confined to this account; --break-system-packages
    # is required on PEP 668 distros (Ubuntu 23.10+/Debian 12+) and is safe here
    # since nothing is written to the system site-packages.
    pip3 install --user --quiet --break-system-packages rosbags
    export PATH="${HOME}/.local/bin:${PATH}"
fi

step "[2/4] Downloading MaiCity sequence ${SEQ} (~215 MB) from the official archive"
log "Source: ${TARBALL_URL}"
log "Note: the archive bundles every sequence; only the ${SEQ} bag is extracted below"
wget --show-progress -q -c -O "${TARBALL}" "${TARBALL_URL}"
log "Download complete"

step "[3/4] Extracting and converting sequence ${SEQ} (ROS 1 .bag -> ROS 2 rosbag2)"
log "Extracting matching bag from archive..."
tar -xzf "${TARBALL}" -C "${WORK_DIR}" --wildcards "*/bags/*${SEQ}*.bag"

BAG_FILE="$(find "${WORK_DIR}" -type f -name "*${SEQ}*.bag" | head -n1)"
if [[ -z "${BAG_FILE}" ]]; then
    echo "ERROR: could not locate a .bag file for sequence ${SEQ} inside the archive." >&2
    exit 1
fi
log "Found $(basename "${BAG_FILE}") — converting to rosbag2..."

mkdir -p "$(dirname "${DATASET_DIR}")"
rm -rf "${DATASET_DIR}"
rosbags-convert --src "${BAG_FILE}" --dst "${DATASET_DIR}" >/dev/null
log "Conversion complete"

step "[4/4] Cleaning up temporary files"
rm -rf "${WORK_DIR}"
log "Removed downloaded archive and intermediate ROS 1 bag"

echo
echo "============================================================================"
echo " MaiCity sequence ${SEQ} is ready at: datasets/mai_city/${SEQ}"
echo " Play it with:  ros2 bag play datasets/mai_city/${SEQ}"
echo "============================================================================"
