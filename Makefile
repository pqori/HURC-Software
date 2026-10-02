.PHONY: image ros stop update build build-%

ifeq ("$(wildcard /.dockerenv)", "")
image:
	docker build . --tag hurc-ros

ros:
	docker compose up -d
	- docker compose attach ros

stop:
	docker compose down

restart: stop ros
endif

update:
	/ros_entrypoint.sh rosdep update
	/ros_entrypoint.sh rosdep install --from-paths src -y -r --ignore-src

build:
	/ros_entrypoint.sh colcon build --continue-on-error

build-%:
	/ros_entrypoint.sh colcon build --packages-select $*
