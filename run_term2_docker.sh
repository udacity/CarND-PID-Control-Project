#!/usr/bin/env bash
docker run\
  --net=host\
  -e SHELL\
  -e DISPLAY\
  -e DOCKER=1\
  -v "`pwd`:/src"\
  -p 4567:4567\
  -it --rm carnd-term2 $SHELL
