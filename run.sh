#!/usr/bin/env bash

IMAGE_NAME="ece4078"
USER_NAME="ece4078"
XSOCK=/tmp/.X11-unix
REPO_DIR="$(pwd)"

docker run -it --rm \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env QT_X11_NO_MITSHM=1 \
    --env DISPLAY="$DISPLAY" \
    --device /dev/video0:/dev/video0 \
    --network=host \
    --privileged \
    --volume "$XSOCK":"$XSOCK" \
    --volume "$HOME"/.Xauthority:/root/.Xauthority \
    --volume "$REPO_DIR":/home/$USER_NAME/ece4078-team2-05/ \
    --gpus all "$IMAGE_NAME" \
    bash
