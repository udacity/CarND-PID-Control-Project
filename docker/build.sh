#!/usr/bin/env bash
docker build\
  --build-arg user=$USER\
  --build-arg uid=$UID\
  --build-arg home=$HOME\
  --build-arg shell=$SHELL\
  -t carnd-term2 .
