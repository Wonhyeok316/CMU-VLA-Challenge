#!/bin/bash

export DISPLAY=143.248.84.96:0

docker compose -f compose_gpu.yml up --build -d