#include "rover_hardware/EPOS.hpp"

namespace EPOS {

    Motor::Motor(void** deviceHandle, unsigned short nodeId)
        : deviceHandle_(deviceHandle), nodeId_(nodeId), currentMode_(UNKNOWN), errorCode_(0), enabled_(false) {};

    bool Motor::enable() {
        if (enabled_ || !deviceHandle_) return false;
        enabled_ = VCS_SetEnableState(*deviceHandle_, nodeId_, &errorCode_);
        return enabled_;
    }

    bool Motor::enableProfilePositionMode() {
        if (!enabled_) return false;
        currentMode_ = POSITION;
        return VCS_ActivateProfilePositionMode(*deviceHandle_, nodeId_, &errorCode_) != 0;
    }

    bool Motor::setPosition(long position, bool absolute, bool immediately) {
        if (!enabled_) return false;
        return VCS_MoveToPosition(
            *deviceHandle_,
            nodeId_,
            position,
            absolute,
            immediately,
            &errorCode_
        ) != 0;
    }

    bool Motor::getPosition(int* outPosition) {
        // if (!enabled_) return false;
        return VCS_GetPositionIs(
            *deviceHandle_,
            nodeId_,
            outPosition,
            &errorCode_
        ) != 0;
    }

    bool Motor::halt() {
        if (!enabled_) return false;
        switch (currentMode_) {
        case POSITION:
            return VCS_HaltPositionMovement(*deviceHandle_, nodeId_, &errorCode_) != 0;
        case VELOCITY:
            return VCS_HaltVelocityMovement(*deviceHandle_, nodeId_, &errorCode_) != 0;
        default:
            return false;
        }
    }

    bool Motor::disable() {
        if (!enabled_) return false;
        return VCS_SetDisableState(*deviceHandle_, nodeId_, &errorCode_) != 0;
    }

    unsigned short Motor::nodeId() const {
        return nodeId_;
    }

    unsigned int Motor::lastError() const {
        return errorCode_;
    }

}