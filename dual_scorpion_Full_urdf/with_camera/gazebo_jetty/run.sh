#!/bin/sh
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
stock=$(CDPATH= cd -- "$here/../../default" && pwd)
export GZ_SIM_RESOURCE_PATH="$here/models:$stock/gazebo_jetty/models:$stock${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
# Assimp supplies the normals that DART requires for the shared convex OBJ meshes.
export GZ_MESH_FORCE_ASSIMP="${GZ_MESH_FORCE_ASSIMP:-true}"
exec gz sim -v 3 "$here/worlds/scorpion.sdf" "$@"
