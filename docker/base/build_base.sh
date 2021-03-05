#!/bin/bash

IMAGE_NAME=ros-deploy-base

docker build -t "${IMAGE_NAME}" -f base.Dockerfile .
