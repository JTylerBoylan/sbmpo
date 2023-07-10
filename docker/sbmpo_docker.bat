@echo off

REM Get the absolute path to the directory containing this script
set SCRIPT_DIR=%~dp0

REM Replace backslashes with forward slashes in the path
set SCRIPT_DIR=%SCRIPT_DIR:\=/%

REM Build the Docker image from the Dockerfile in the current directory
docker build -t sbmpo:latest -f "%SCRIPT_DIR%/Dockerfile.sbmpo" "%SCRIPT_DIR%"

REM Start the Docker container with the current directory mounted to /app, and automatically remove the container when it is stopped or exited
docker run --rm -it --mount type=bind,source="%SCRIPT_DIR%/../",target=/sbmpo_ws/ sbmpo:latest
