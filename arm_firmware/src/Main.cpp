#include <Arduino.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

#include "MicroROS.h"
#include "Messages.h"

#define NODE_NAME "arm_teensy"
#define NAMESPACE ""

MultiStepper steppers;
AccelStepper azimuthStepper(AccelStepper::DRIVER, 9, 10);
AccelStepper gripperStepper(AccelStepper::DRIVER, 11, 12);

IntMsg azimuthSubMsg;
uROS::Subscriber AzimuthSub("/arm/set_base_rotate", azimuthSubMsg);

IntMsg azimuthPubMsg;
uROS::Publisher AzimuthPub("/arm/get_base_rotate", &azimuthPubMsg);

IntMsg gripperSubMsg;
uROS::Subscriber GripperSub("/arm/set_gripper", gripperSubMsg);

IntMsg gripperPubMsg;
uROS::Publisher GripperPub("/arm/get_gripper", &gripperPubMsg);

void setup() {

  set_microros_transports();
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(500);
  }

  uROS::Init_MicroROS(NODE_NAME, NAMESPACE);

  azimuthSubMsg.Init();
  azimuthPubMsg.Init();
  // leftFrontRpmMsg.Init();
  // leftBackRpmMsg.Init();
  gripperSubMsg.Init();
  gripperPubMsg.Init();
  // rightFrontRpmMsg.Init();
  // rightBackRpmMsg.Init();

  AzimuthSub.Init(); // &uROS::QOS_Teleop
  AzimuthPub.Init(&uROS::QOS_Teleop);
  // LeftFrontRpmPub.Init(&uROS::QOS_Teleop);
  // LeftBackRpmPub.Init(&uROS::QOS_Teleop);
  GripperSub.Init(); // &uROS::QOS_Teleop
  GripperPub.Init(&uROS::QOS_Teleop);
  // RightFrontRpmPub.Init(&uROS::QOS_Teleop);
  // RightBackRpmPub.Init(&uROS::QOS_Teleop);

  // delay(250);
  // logMsg.Set("[INFO] uROS initialized");
  // LogPub.Publish();

  azimuthStepper.setMaxSpeed(10000);
  azimuthStepper.setMinPulseWidth(5);
  steppers.addStepper(azimuthStepper);

  gripperStepper.setMaxSpeed(4000);
  gripperStepper.setMinPulseWidth(5);
  steppers.addStepper(gripperStepper);
}

unsigned long lastUpdate = 0;

void loop() {
  if (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
    SCB_AIRCR = 0x05FA0004;
  }

  uROS::Spin(0);

  int32_t azimuthTarget = azimuthSubMsg.GetValue();
  int32_t gripperTarget = gripperSubMsg.GetValue();

  long targets[2];
  targets[0] = azimuthTarget;
  targets[1] = gripperTarget;

  steppers.moveTo(targets);
  steppers.run();

  azimuthPubMsg._msg.data = azimuthStepper.currentPosition();
  gripperPubMsg._msg.data = gripperStepper.currentPosition();

  AzimuthPub.Publish();
  GripperPub.Publish();

  // leftBackRpmMsg._msg.data = 0.0;
  // if (VESC1.getVescValues(0)) {
  //   leftBackRpmMsg._msg.data = VESC1.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // LeftBackRpmPub.Publish();
  // leftFrontRpmMsg._msg.data = 0.0;
  // if (VESC1.getVescValues(1)) {
  //   leftFrontRpmMsg._msg.data = VESC1.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // LeftFrontRpmPub.Publish();

  // rightBackRpmMsg._msg.data = 0.0;
  // if (VESC2.getVescValues(0)) {
  //   rightBackRpmMsg._msg.data = VESC2.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // RightBackRpmPub.Publish();
  // rightFrontRpmMsg._msg.data = 0.0;
  // if (VESC2.getVescValues(1)) {
  //   rightFrontRpmMsg._msg.data = VESC2.data.rpm / DRIVE_POLE_PAIRS;
  // }
  // RightFrontRpmPub.Publish();
}
