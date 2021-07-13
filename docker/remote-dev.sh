#!/usr/bin/env bash

# change to true if using nvidia graphic cards
USE_NVIDIA_TOOLKIT=false

IMAGE_NAME=pybullet-ros
MULTISTAGE_TARGET=develop
CONTAINER_NAME="${IMAGE_NAME/\//-}"
CONTAINER_NAME="${CONTAINER_NAME/:/-}-${MULTISTAGE_TARGET}"

path=$(echo "${PWD}" | rev | cut -d'/' -f-2 | rev)
if [ "${path}" != "pybullet_ros/docker" ]; then
  echo "Run this script from within the directory pybullet_ros/docker !"
  echo "You are currently in ${path}"
  exit 1
fi

# BUILD
BUILD_FLAGS=()
while getopts 'r' opt; do
  case $opt in
    r) BUILD_FLAGS+=(--no-cache) ;;
    *) echo 'Error in command line parsing' >&2
       exit 1
  esac
done
shift "$(( OPTIND - 1 ))"

if [[ "$OSTYPE" != "darwin"* ]]; then
  USER_ID="$(id -u "${USER}")"
  GROUP_ID="$(id -g "${USER}")"
  BUILD_FLAGS+=(--build-arg UID="${USER_ID}")
  BUILD_FLAGS+=(--build-arg GID="${GROUP_ID}")
fi

BUILD_FLAGS+=(-t "${IMAGE_NAME}:${MULTISTAGE_TARGET}")
BUILD_FLAGS+=(--target "${MULTISTAGE_TARGET}")

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" .. || exit

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/../" \
    --opt o="bind" \
    "${IMAGE_NAME}_ros_pkg_vol"

RUN_FLAGS=(-u ros)
if [[ "$OSTYPE" == "darwin"* ]]; then
  RUN_FLAGS+=(-e DISPLAY=host.docker.internal:0)
else
  xhost +
  RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
  RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
  RUN_FLAGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  RUN_FLAGS+=(-v "${IMAGE_NAME}"_ros_pkg_vol:/home/ros/ros_ws/src/pybullet_ros)
fi

xhost +
docker run -it --rm --privileged \
  ${GPU_FLAG} \
  "${RUN_FLAGS[@]}" \
  --hostname "${CONTAINER_NAME}" \
  "${IMAGE_NAME}:${MULTISTAGE_TARGET}" /bin/bash
