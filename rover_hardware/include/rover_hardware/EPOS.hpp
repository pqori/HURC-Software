#ifndef HURC_ROVER_HARDWARE_EPOS_HPP_
#define HURC_ROVER_HARDWARE_EPOS_HPP_

#include "EPOS_Definitions.h"

namespace EPOS
{

    enum OperatingMode {
        UNKNOWN, POSITION, VELOCITY
    };

    // using ::VCS_OpenDevice;
    // using ::VCS_CloseAllDevices;
    // using ::VCS_SetEnableState;
    // using ::VCS_ActivateProfilePositionMode;
    // using ::VCS_ActivateProfileVelocityMode;
    // using ::VCS_MoveWithVelocity;
    // using ::VCS_MoveToPosition;
    // using ::VCS_HaltVelocityMovement;
    // using ::VCS_SetDisableState;

    inline void* OpenDevice(char* deviceName, char* protocolStackName, char* interfaceName, char* portName, unsigned int* errorCode) {
        // return nullptr;
        return VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portName, errorCode);
    }

class Motor {
public:
    Motor(void** deviceHandle, unsigned short nodeId);

    bool enable();
    bool enableProfilePositionMode();
    bool enableProfileVelocityMode();
    // bool setVelocity(long velocity);
    bool setPosition(long position, bool absolute = true, bool immediately = true);
    bool getPosition(int* outPosition);
    bool halt();
    bool disable();
    unsigned short nodeId() const;
    unsigned int lastError() const;

private:
    void** deviceHandle_;
    unsigned short nodeId_;
    OperatingMode currentMode_;
    unsigned int errorCode_;
    bool enabled_;
};
}

#endif