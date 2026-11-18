#!/usr/bin/env bash
# Create a local root CA + server certificate for the WebXR (televuer, :8012) and WebRTC image (:60001-3)
# HTTPS/WSS endpoints. SANs: localhost, 127.0.0.1 and every global IPv4 of this PC (looked up now).
# Files go to $G1IV_WS/certs (git-ignored). Existing files are never overwritten unless --force.
# The PICO browser must trust certs/rootCA.pem (or accept the warning) - see docs/PICO_CONNECTION.md.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../_env_common.sh"
D="$G1IV_WS/certs"
mkdir -p "$D"
chmod 700 "$D"
if [[ -f "$D/cert.pem" && "${1:-}" != "--force" ]]; then
  echo "certs exist in $D (use --force to re-create, e.g. after the PC IP changed)"
  openssl x509 -in "$D/cert.pem" -noout -subject -enddate -ext subjectAltName 2>/dev/null || true
  exit 0
fi
IPS=$(ip -4 -o addr show scope global | awk '{split($4,a,"/"); print a[1]}')
SAN="DNS:localhost,IP:127.0.0.1"
for ip in $IPS; do SAN="$SAN,IP:$ip"; done
cd "$D"
openssl genrsa -out rootCA.key 2048 2>/dev/null
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 365 -out rootCA.pem -subj "/CN=g1iv-xr-teleoperate-local-CA"
openssl genrsa -out key.pem 2048 2>/dev/null
openssl req -new -key key.pem -out server.csr -subj "/CN=g1iv-sim-pc"
printf 'authorityKeyIdentifier=keyid,issuer\nbasicConstraints=CA:FALSE\nkeyUsage=digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth\nsubjectAltName=%s\n' "$SAN" > server_ext.cnf
openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key -CAcreateserial -out cert.pem -days 365 -sha256 -extfile server_ext.cnf 2>/dev/null
chmod 600 rootCA.key key.pem
echo "created $D/{rootCA.pem,cert.pem,key.pem}  SAN=$SAN"
