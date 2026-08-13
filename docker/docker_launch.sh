#!/usr/bin/env bash

# Allow local Docker containers to connect to X server
xhost +local:docker >/dev/null 2>&1

docker run --name DDLZD --rm -it --privileged \
  --gpus all --network host \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=$XAUTHORITY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Documents/robotspace/DDLZD_ws/Drone-Delivery-Landing-Zone-Detection:/home/airsim_user/Drone-Delivery-Landing-Zone-Detection \
  -v ~/Documents/robotspace/ws_2025_3dmapoctoserver/bt_pcds:/home/airsim_user/Drone-Delivery-Landing-Zone-Detection/bt_pcds \
  --entrypoint /bin/bash \
  giri6937/lam:latest

# Optional: tighten security again after container exits (uncomment if desired)
# xhost -local:docker >/dev/null 2>&1