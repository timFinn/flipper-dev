#!/usr/bin/env bash
# Pin the ufbt SDK to a known channel/version for this monorepo.
#
# Keep this script authoritative — don't run `ufbt update` with ad-hoc
# flags on one machine and not the other, or the dev box (remote) and
# carbon (flashing host) will build against different SDKs.
set -euo pipefail

CHANNEL="${UFBT_CHANNEL:-release}"
HW_TARGET="${UFBT_HW_TARGET:-f7}"

if ! command -v ufbt >/dev/null 2>&1; then
    echo "ufbt not installed. Install with: pipx install ufbt  (or: pip install --user ufbt)" >&2
    exit 1
fi

echo "Updating ufbt SDK: channel=${CHANNEL} hw_target=${HW_TARGET}"
ufbt update --channel="${CHANNEL}" --hw-target="${HW_TARGET}"
echo "Done. SDK cached at ~/.ufbt/"
