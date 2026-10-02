FROM osrf/ros:humble-desktop

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN useradd -m -s /bin/bash -G sudo rover

USER rover
ENV USER=rover

WORKDIR /home/rover
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh -s 1.3.1
RUN /home/rover/bin/arduino-cli config init --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
RUN /home/rover/bin/arduino-cli core install teensy:avr@1.57.3

WORKDIR /tmp
ADD --chown=rover https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.7-humble.tar.gz /tmp/micro_ros_arduino.tar.gz
RUN tar -xzf micro_ros_arduino.tar.gz
RUN mv micro_ros_arduino-2.0.7-humble /home/rover/.arduino15/packages/teensy/hardware/avr/1.57.3/libraries/micro_ros_arduino

ADD --chown=rover https://github.com/SolidGeek/VescUart/archive/refs/heads/master.tar.gz /tmp/vesc_uart.tar.gz
RUN tar -xzf vesc_uart.tar.gz
RUN mv VescUart-master /home/rover/.arduino15/packages/teensy/hardware/avr/1.57.3/libraries/VescUart

RUN sudo apt-get update && sudo apt-get install -y zstd
ADD --chown=rover https://download.stereolabs.com/zedsdk/5.1/cu13/ubuntu22 /tmp/zed_sdk_installer.run
RUN chmod +x zed_sdk_installer.run
RUN sudo mkdir -p /etc/udev/rules.d/
RUN ./zed_sdk_installer.run -- silent skip_cuda

# RUN sudo rm -rf /tmp/*

WORKDIR /home/rover/ros_ws
COPY --chown=rover . ./src
RUN ln -s src/Makefile Makefile

RUN make update
# RUN /ros_entrypoint.sh rosdep update
# RUN /ros_entrypoint.sh rosdep install --from-paths src -y -r --ignore-src
# RUN /ros_entrypoint.sh colcon build --continue-on-error; exit 0

ENTRYPOINT [ "/ros_entrypoint.sh" ]
