#!/bin/bash

xhost +local:docker

docker build -t ros2-rnr .

docker run -it --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros2-rnr
