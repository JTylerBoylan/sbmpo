#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Build the Docker image from the Dockerfile in the current directory
docker build -t sbmpo:latest -f "${SCRIPT_DIR}/Dockerfile.sbmpo" "${SCRIPT_DIR}"

# Start the Docker container with the current directory mounted to /app, and automatically remove the container when it is stopped or exited
docker run --rm -it --mount type=bind,source="${SCRIPT_DIR}",target=/sbmpo_ws/sbmpo/ sbmpo:latest
