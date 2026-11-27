#!/bin/sh
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
export GZ_SIM_RESOURCE_PATH="$here/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# Assimp supplies the normals that DART requires for the convex OBJ meshes.
export GZ_MESH_FORCE_ASSIMP="${GZ_MESH_FORCE_ASSIMP:-true}"
exec gz sim -v 3 "$here/worlds/scorpion.sdf" "$@"
