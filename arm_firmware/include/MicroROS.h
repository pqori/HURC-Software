#pragma once

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <type_traits>
#include <functional>

namespace uROS {
    extern rcl_allocator_t allocator;

    const rmw_qos_profile_t QOS_Teleop = {
        RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT,
        RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
        false
    };

    void Init_MicroROS(const char* node_name, const char* node_namespace);
    void Spin(int ms);
    
    struct Msg {
        const virtual rosidl_message_type_support_t* GetTypeSupport();
        virtual void Init();
        virtual void* GetRawMsg();
    };

    class Publisher {
    public:
        template<typename MsgType>
        Publisher(const char* name, MsgType* msg)
            : _name(name) {
            static_assert(std::is_convertible<MsgType, Msg>::value, "");
            _msg = msg;
        };
        void Init();
        void Init(const rmw_qos_profile_t* profile);
        bool Publish();

    private:
        const char* _name;
        rcl_publisher_t _publisher;
        Msg* _msg;
        // std_msgs__msg__Int32 _out_msg;
    };

    class Subscriber {
    public:
        template<typename MsgType>
        Subscriber(const char* name, MsgType &msg)
            : _name(name), _msg(msg), _callback(nullptr) {
            static_assert(std::is_convertible<MsgType, Msg>::value, "");
            _callback = [](const void* msgin){};
            extern int registered_handles;
            registered_handles++;
        };
        template<typename MsgType>
        Subscriber(const char* name, MsgType &msg, rclc_subscription_callback_t callback)
            : _name(name), _msg(msg), _callback(callback) {
            static_assert(std::is_convertible<MsgType, Msg>::value, "");
            // _msg = msg;
            extern int registered_handles;
            registered_handles++;
        };
        void Init();
        void Init(const rmw_qos_profile_t* profile);

    private:
        const char* _name;
        rcl_subscription_t _subscriber;
        Msg &_msg;
        rclc_subscription_callback_t _callback;

        // void _Internal_Callback(const void*);
    };

}
// #endif

// #pragma once

// #include <micro_ros_arduino.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// namespace uROS {
//     void Init_MicroROS(const char* node_name, const char* node_namespace);

//     class Publisher {
//     public:
//         // Publisher(const char* name);
//         void Init(const char* name);
//         bool PublishRaw(void* out_msg);

//     private:
//         rcl_publisher_t _publisher;
//         // std_msgs__msg__Int32 _out_msg;
//     };

// }
// // #endif