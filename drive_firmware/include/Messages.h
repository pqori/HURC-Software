#pragma once
#include "MicroROS.h"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

struct IntMsg: public uROS::Msg {
public:
    std_msgs__msg__Int32 _msg;
    const rosidl_message_type_support_t* GetTypeSupport() {
        return ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, Int32)();
    }
    void Init() { _msg.data = 0; };
    void* GetRawMsg() {
        return &_msg;
    }
    int32_t GetValue() {
        return _msg.data;
    }
};

struct FloatMsg: public uROS::Msg {
public:
    std_msgs__msg__Float32 _msg;
    const rosidl_message_type_support_t* GetTypeSupport() {
        return ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, Float32)();
    }
    void Init() { _msg.data = 0.0; };
    void* GetRawMsg() {
        return &_msg;
    }
};

struct StringMsg: public uROS::Msg {
public:
    std_msgs__msg__String *_msg;
    const rosidl_message_type_support_t* GetTypeSupport() {
    return ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, String)();
    }
    void Init() {
        _msg = std_msgs__msg__String__create();
    }
    void Set(const char* value);
    void* GetRawMsg() {
        return _msg;
    }
};