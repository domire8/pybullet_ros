#!/usr/bin/env bash

IMAGE_NAME=pybullet_ros
MULTISTAGE_TARGET="ros-user"
USE_NVIDIA_TOOLKIT=false

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

xhost +
docker run \
  ${GPU_FLAG} \
  --privileged \
  -it \
  --rm \
  --net="host" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTH:$XAUTH" \
  --env XAUTHORITY="$XAUTH" \
  --env DISPLAY="${DISPLAY}" \
  "$IMAGE_NAME:$MULTISTAGE_TARGET"
