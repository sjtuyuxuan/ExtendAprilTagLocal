#!/bin/bash

docker rm -f `docker ps -a | grep exaprillocal | cut -d " " -f 1`
docker run --name=exaprillocal -d -it \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   -e DISPLAY=unix$DISPLAY \
   -e GDK_SCALE \
   -e GDK_DPI_SCALE \
   -v $PWD/..:/home/test/workspace/src/ExtendAprilTagLocal exaprillocal:v0 roscore
