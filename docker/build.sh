#!/usr/bin/env bash

MULTISTAGE_TARGET="ros-user"

path=$(echo "$PWD" | rev | cut -d'/' -f-2 | rev)
if [ "$path" != "pybullet_ros/docker" ]; then
  echo "Run this script from within the directory pybullet_ros/docker !"
  echo "You are currently in $path"
  exit 1
fi

IMAGE_NAME=pybullet_ros

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${MULTISTAGE_TARGET}")

if [ "$REBUILD" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" ..
