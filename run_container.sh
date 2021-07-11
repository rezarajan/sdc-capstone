#!/bin/sh
docker run -p 4567:4567 -v "$(pwd):/capstone" -v "/tmp/log:/root/.ros/" --rm -it capstone