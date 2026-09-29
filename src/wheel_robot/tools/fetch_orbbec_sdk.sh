#!/usr/bin/env bash
set -euo pipefail

package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk_repo="$package_root/vendor/OrbbecSDK_ROS2"
sdk_commit="3812e0f5adbb5a3796242bdff0ef90d173f7ca2f"

if [[ ! -d "$sdk_repo/.git" ]]; then
  git clone https://github.com/orbbec/OrbbecSDK_ROS2.git "$sdk_repo"
fi

git -C "$sdk_repo" fetch origin "$sdk_commit"
git -C "$sdk_repo" checkout --detach "$sdk_commit"

printf 'Orbbec legacy SDK ready: %s\n' "$sdk_repo/orbbec_camera/SDK"
