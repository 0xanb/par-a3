#!/usr/bin/env bash
# One-shot bootstrap: installs everything the Mac host needs.
# Re-runnable; skips what is already in place.

set -euo pipefail

need { command -v "$1" >/dev/null 2>&1; }
step { echo; echo "==> $*"; }

step "Homebrew"
if ! need brew; then
 /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
fi

step "Colima + Docker CLI"
brew install --quiet colima docker docker-compose docker-buildx

step "Starting Colima (skipped if already running)"
colima status >/dev/null 2>&1 || colima start --cpu 4 --memory 8 --disk 60 --vm-type vz --mount-type virtiofs

step "VS Code extensions"
for ext in \
 ms-iot.vscode-ros \
 ms-python.python \
 ms-python.vscode-pylance \
 ms-vscode.cpptools-extension-pack \
 ms-azuretools.vscode-docker \
 redhat.vscode-yaml \
 twxs.cmake \
 ms-vscode-remote.remote-containers; do
 code --install-extension "$ext" --force >/dev/null 2>&1 || true
done

step "Host Python helpers (report generation, local OAK-D sanity checks)"
python3 -m pip install --quiet --upgrade pip
python3 -m pip install --quiet --user \
 depthai \
 pyzbar \
 opencv-contrib-python \
 matplotlib \
 pandas \
 numpy || true

step "Dev container image"
docker build -t par-a3-dev -f "$(dirname "$0")//.devcontainer/Dockerfile" "$(dirname "$0")//.devcontainer"

step "Done. Next: ./scripts/dev.sh up"
