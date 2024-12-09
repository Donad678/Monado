#!/usr/bin/env bash
# SPDX-License-Identifier: CC0-1.0
# SPDX-FileCopyrightText: 2018-2022 Collabora, Ltd. and the Monado contributors
set -e
set -x

# must be run from the build directory
echo "Run from directory ${PWD}"

export XDG_RUNTIME_DIR=/tmp

WAYLAND_SOCKET_NAME=monado-ci
export WAYLAND_DISPLAY="${WAYLAND_SOCKET_NAME}"

weston --use-pixman --backends headless --no-config --socket=${WAYLAND_SOCKET_NAME} &
WESTON_PID="$!"

# TODO: watch for weston startup finish? File: $XDG_RUNTIME_DIR/$WAYLAND_SOCKET
sleep 2

# TODO: manually kill monado-service for now

# export IPC_EXIT_ON_DISCONNECT=1
export XRT_COMPOSITOR_LOG=debug
export VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json
export XRT_NO_STDIN=1
export P_OVERRIDE_ACTIVE_CONFIG=remote

export XR_RUNTIME_JSON="${PWD}"/openxr_monado-dev.json

src/xrt/targets/service/monado-service &
# TODO: watch for monado pid file and/or socket?
sleep 3
# TODO: $! seems to record the wrong PID.
MONADO_SERVICE_PID=$(cat "${XDG_RUNTIME_DIR}"/monado.pid)

#export IPC_LOG=trace

# Vulkan requires external fence on monado, which is not supported on lvp. Vulkan2 does not require it.
# hello_xr really wants an stdin, therefore the <(cat) trick
hello_xr -G Vulkan2 < <(cat) &
HELLOXR_PID="$!"

# Run hello_xr for 3 seconds
sleep 3

# || true to make sure we actually kill everything and don't leave dangling processes in the CI
echo "Kill hello_xr"
kill ${HELLOXR_PID} || true

sleep 1
echo "Kill monado-service"
kill ${MONADO_SERVICE_PID} || true
echo "weston"
kill ${WESTON_PID} || true