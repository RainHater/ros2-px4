#!/bin/bash

ROOT = $(PWD)
JOBS := $(shell nproc)

.PHONY: all
all: build

.PHONY: build
build:
	echo "CPU 核心数量: ${JOBS}"
	colcon build \
		--base-paths $(ROOT) \
		--parallel-workers $(JOBS) \
		--packages-skip simulation

.PHONY: simulation
simulation:
	colcon build \
		--base-paths $(ROOT) \
		--parallel-workers $(JOBS)

.PHONY: sync
sync:
	bash $(ROOT)/scripts/sync/ros2_ws.sh
