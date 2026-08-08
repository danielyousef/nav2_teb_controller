SHELL := /bin/bash
WS     := $(HOME)/git/ros2_ws
PKG    := nav2_teb_controller
SRC    := src

.PHONY: help build test format format-fix lint lint-fix docs docs-check all

help:
	@echo "Available commands:"
	@echo "  make build       		- colcon build"
	@echo "  make test        		- colcon test"
	@echo "  make test-with-log		- colcon test"
	@echo "  make format      		- clang-format check (kein Fix)"
	@echo "  make format-fix  		- clang-format mit Fix"
	@echo "  make lint        		- clang-tidy check (kein Fix)"
	@echo "  make lint-fix    		- clang-tidy mit Fix"
	@echo "  make docs        		- regenerate doc/parameters.md from the parameter schema"
	@echo "  make docs-check  		- fail if doc/parameters.md is out of date"
	@echo "  make all         		- format + lint + build + test + docs-check"

build:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon build \
		--packages-select $(PKG) \
		--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

build-clean:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon build \
		--packages-select $(PKG) \
		--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		--cmake-clean-first

test-with-log:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon test --packages-select $(PKG) --event-handlers console_direct+ && \
	colcon test-result

test:
	source /opt/ros/jazzy/setup.bash && \
	cd $(WS) && colcon test --packages-select $(PKG) && \
	colcon test-result

format:
	find $(SRC) -name "*.cpp" -o -name "*.hpp" | \
	xargs clang-format --dry-run --Werror --style=file

format-fix:
	find $(SRC) -name "*.cpp" -o -name "*.hpp" | \
	xargs clang-format -i --style=file

lint:
	run-clang-tidy \
		-p $(WS)/build/$(PKG) \
		-config-file .clang-tidy \
		-header-filter=".*nav2_teb_controller/(g2o_types|core|obstacles|homotopy|planner).*" \
		$(shell find $(SRC) -name "*.cpp")

lint-fix:
	run-clang-tidy \
		-p $(WS)/build/$(PKG) \
		-config-file .clang-tidy \
		-header-filter=".*nav2_teb_controller/(g2o_types|core|obstacles|homotopy|planner).*" \
		-fix \
		$(shell find $(SRC) -name "*.cpp")
		--fix-errors

# Regenerate doc/parameters.md from config/teb_controller_parameters.yaml
# (the generate_parameter_library schema is the single source of truth).
docs:
	source /opt/ros/jazzy/setup.bash && \
	python3 scripts/gen_params_docs.py \
		--input_yaml_file config/teb_controller_parameters.yaml \
		--output_markdown_file doc/parameters.md

docs-check:
	@tmp=$$(mktemp -d) && \
	source /opt/ros/jazzy/setup.bash && \
	python3 scripts/gen_params_docs.py \
		--input_yaml_file config/teb_controller_parameters.yaml \
		--output_markdown_file $$tmp/parameters.md && \
	diff -u doc/parameters.md $$tmp/parameters.md && \
	echo "doc/parameters.md is up to date" && \
	rm -rf $$tmp

all: format lint build test docs-check