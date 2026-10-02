cmake_minimum_required(VERSION 3.16)
project(arm_firmware)

# Find ament (ROS 2 build system)
find_package(ament_cmake REQUIRED)

# Path to Teensyduino install
set(TEENSYDUINO_VER 1.57.3)
set(TEENSY_TOOLCHAIN_VER 5.4.1)
set(TEENSY_ROOT $ENV{HOME}/.arduino15/packages/teensy)
set(TEENSY_BASE ${TEENSY_ROOT}/hardware/avr/${TEENSYDUINO_VER})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(TEENSY_LIBS
  AccelStepper
  micro_ros_arduino
  NativeEthernet
  FNET
)

# Toolchain (override compiler for this target)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m7)
set(CMAKE_C_COMPILER ${TEENSY_ROOT}/tools/teensy-compile/${TEENSY_TOOLCHAIN_VER}/arm/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${TEENSY_ROOT}/tools/teensy-compile/${TEENSY_TOOLCHAIN_VER}/arm/bin/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${TEENSY_ROOT}/tools/teensy-compile/${TEENSY_TOOLCHAIN_VER}/arm/bin/arm-none-eabi-gcc)

# Common compiler flags
set(TEENSY_FLAGS "-mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16")
set(CMAKE_C_FLAGS   "${TEENSY_FLAGS} -Wall -g -O2 -ffunction-sections -fdata-sections -MMD")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-exceptions -fno-rtti -fno-use-cxa-atexit")
set(CMAKE_ASM_FLAGS "${TEENSY_FLAGS}")

# Preprocessor defines for Teensy 4.1
add_definitions(
  -DF_CPU=600000000
  -DUSB_SERIAL
  -DLAYOUT_US_ENGLISH
  -DTEENSYDUINO=159
  -DARDUINO=10813
  -DARDUINO_TEENSY41
  -D__IMXRT1062__
)

# Include paths
include_directories(
  include
  ${TEENSY_BASE}/cores/teensy4
  ${TEENSY_BASE}/variants/teensy41
)

# Gather sources
file(GLOB APP_SRC src/*.cpp src/*.c)
file(GLOB TEENSY_CORE_SRC
  ${TEENSY_BASE}/cores/teensy4/*.c
  ${TEENSY_BASE}/cores/teensy4/*.cpp
)

# Include libraries
set(TEENSY_LIB_SRC "")
foreach(LIB ${TEENSY_LIBS})
  include_directories(${TEENSY_BASE}/libraries/${LIB}/src)
  file(GLOB_RECURSE LIB_SRC
    ${TEENSY_BASE}/libraries/${LIB}/src/*.c
    ${TEENSY_BASE}/libraries/${LIB}/src/*.cpp
  )
  set(TEENSY_LIB_SRC ${TEENSY_LIB_SRC} ${LIB_SRC})
endforeach()

# Build firmware
add_executable(${PROJECT_NAME}.elf
  ${APP_SRC}
  ${TEENSY_CORE_SRC}
  ${TEENSY_LIB_SRC}
)

# Linker script
set(LINKER_SCRIPT ${TEENSY_BASE}/cores/teensy4/imxrt1062_t41.ld)
set_target_properties(${PROJECT_NAME}.elf PROPERTIES
  LINK_FLAGS "-T${LINKER_SCRIPT} -specs=nosys.specs -Wl,--gc-sections"
)

# Link against required runtime libraries
target_link_libraries(${PROJECT_NAME}.elf
  gcc
  c
  m
  stdc++
  nosys
  ${TEENSY_BASE}/libraries/micro_ros_arduino/src/imxrt1062/fpv5-d16-hard/libmicroros.a
)

# HEX output
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${TEENSY_ROOT}/tools/teensy-compile/${TEENSY_TOOLCHAIN_VER}/arm/bin/arm-none-eabi-objcopy -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
  COMMENT "Generating HEX firmware"
)

# Output the HEX file into the ROS 2 install space
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
  DESTINATION share/${PROJECT_NAME}/firmware
)

# Install scripts to manage the arm's microcontroller
install(
  PROGRAMS scripts/upload_firmware.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Normal ROS 2 package bits
ament_package()
