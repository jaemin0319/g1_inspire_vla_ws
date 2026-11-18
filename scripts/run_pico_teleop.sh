#!/usr/bin/env bash
# Real PICO 4 Ultra controllers via the official televuer WebXR server. Prints the URL to open on the
# headset. --diag prints raw buttons/triggers/sticks to verify the mapping (docs/PICO_CONNECTION.md).
#   scripts/run_pico_teleop.sh [--loco-source keyboard|xr] [--diag] [--record]
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/_env_common.sh"
g1iv_certs
echo "== PICO connection =="
for ip in $(ip -4 -o addr show scope global | awk '{split($4,a,"/"); print a[1]}'); do
  echo "  open on the headset browser:  https://$ip:8012/?ws=wss://$ip:8012"
done
echo "  certificate SANs: $(openssl x509 -in "$XR_TELEOP_CERT" -noout -ext subjectAltName 2>/dev/null | tail -1 | xargs)"
echo "  waiting for the sim + headset; arms engage with left X (or R in the hub window)."
exec "$(dirname "${BASH_SOURCE[0]}")/_hub_launch.sh" pico --arm-source pico "$@"
