#!/bin/bash

IMAGE_NAME=ros2_orb_slam3:latest
WORKSPACE_DIR=$(pwd)
PORT_MAPPING=""

# Optional: handle port mapping
if [ "$1" = "--port" ] && [ -n "$2" ] && [ -n "$3" ]; then
    PORT_MAPPING="-p $2:$3"
    shift 3
fi

# ARCH=$(uname -m)

# if [ "$ARCH" != "x86_64" ]; then
#     echo "This container is built for x86_64 only. Current architecture: $ARCH"
#     exit 1
# fi

echo "Running ros2_orb_slam3 container on x86_64 (CPU-only)..."
# --rm
docker run -it  \
    --platform=linux/amd64 \
    --network compose_my_bridge_network \
    --env-file .env \
    -v "$WORKSPACE_DIR:/root/ros2_ws/src" \
    $PORT_MAPPING \
    $IMAGE_NAME 
    # /bin/bash