.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: clean
clean:
	@rm -rf build/ install/ log/ logs/

.PHONY: clean-test
clean-test:
	colcon test-result --delete-yes

.PHONY: build-debug
build-debug:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

.PHONY: build
build:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

.PHONY: test
test:
	colcon test; colcon test-result --verbose

.PHONY: reformat
reformat:
	ament_clang_format --config .clang-format --reformat src/
	ament_clang_format --config .clang-format --reformat include/

.PHONY: rosdep-install
rosdep-install:
	sudo apt update
	rosdep update --include-eol-distros
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths .