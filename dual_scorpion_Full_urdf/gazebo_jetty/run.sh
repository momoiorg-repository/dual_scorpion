#!/bin/sh
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export GZ_SIM_RESOURCE_PATH="$here/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim -v 3 "$here/worlds/scorpion.sdf" "$@"
