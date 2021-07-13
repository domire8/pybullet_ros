#!/usr/bin/env bash

IMAGE_NAME=pybullet-ros
MULTISTAGE_TARGET=develop
CONTAINER_NAME="${IMAGE_NAME/\//-}"
CONTAINER_NAME="${CONTAINER_NAME/:/-}-ssh"

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

# RUN
SSH_PORT=2222
USERNAME=ros

PUBLIC_KEY=$(cat "$HOME/.ssh/id_rsa.pub")
COMMAND_FLAGS=()
COMMAND_FLAGS+=(--key "${PUBLIC_KEY}")
COMMAND_FLAGS+=(--user "${USERNAME}")

RUN_FLAGS=()
if [[ "$OSTYPE" != "darwin"* ]]; then
  USER_ID=$(id -u "${USER}")
  GROUP_ID=$(id -g "${USER}")
  COMMAND_FLAGS+=(--uid "${USER_ID}")
  COMMAND_FLAGS+=(--gid "${GROUP_ID}")

  xhost +
  RUN_FLAGS+=(--privileged)
  RUN_FLAGS+=(-e DISPLAY="${DISPLAY}")
  RUN_FLAGS+=(-e XAUTHORITY="${XAUTHORITY}")
  RUN_FLAGS+=(--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

docker container stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1

echo "Starting background container with access port ${SSH_PORT} for user ${USERNAME}"
docker run -d --rm --cap-add sys_ptrace \
  --publish 127.0.0.1:"${SSH_PORT}":22 \
  --name "${CONTAINER_NAME}" \
  --hostname "${CONTAINER_NAME}" \
  "${RUN_FLAGS[@]}" \
  "${IMAGE_NAME}:${MULTISTAGE_TARGET}" /sshd_entrypoint.sh "${COMMAND_FLAGS[@]}"

echo "${CONTAINER_NAME}"
