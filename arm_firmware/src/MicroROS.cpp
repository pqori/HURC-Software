#include "MicroROS.h"

namespace uROS {

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
int registered_handles = 0;

// #include <std_msgs/msg/int32.h>

void Init_MicroROS(const char* node_name, const char* node_namespace) {
    // set_microros_transports();

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, node_name, node_namespace, &support);

    rclc_executor_init(&executor, &support.context, registered_handles, &allocator);
}

void Spin(int ms) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(ms));
}

void Publisher::Init() {
    rclc_publisher_init_default(
        &_publisher,
        &node,
        _msg->GetTypeSupport(),
        _name
    );
}

void Publisher::Init(const rmw_qos_profile_t* profile) {
    rclc_publisher_init(
        &_publisher,
        &node,
        _msg->GetTypeSupport(),
        _name,
        profile
    );
}

bool Publisher::Publish() {
    // _out_msg.data = data;
    rcl_ret_t rc = rcl_publish(&_publisher, _msg->GetRawMsg(), NULL);
    return rc == RCL_RET_OK;
}

void Subscriber::Init() {
    rclc_subscription_init_default(
        &_subscriber,
        &node,
        _msg.GetTypeSupport(),
        _name
    );
    rclc_executor_add_subscription(&executor, &_subscriber, _msg.GetRawMsg(), _callback, ON_NEW_DATA);
}

void Subscriber::Init(const rmw_qos_profile_t* profile) {
    rclc_subscription_init(
        &_subscriber,
        &node,
        _msg.GetTypeSupport(),
        _name,
        profile
    );
    rclc_executor_add_subscription(&executor, &_subscriber, _msg.GetRawMsg(), _callback, ON_NEW_DATA);
}



}

// #include "MicroROS.h"

// namespace uROS {

// rcl_allocator_t allocator;
// rclc_support_t support;
// rcl_node_t node;

// void Init_MicroROS(const char* node_name, const char* node_namespace) {
//     set_microros_transports();

//     allocator = rcl_get_default_allocator();

//     rclc_support_init(&support, 0, NULL, &allocator);
//     rclc_node_init_default(&node, node_name, node_namespace, &support);
// }

// void Publisher::Init(const char* name) {
//     rclc_publisher_init_default(
//         &_publisher,
//         &node,
//         ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, Int32)(),
//         name
//     );
// }

// bool Publisher::PublishRaw(void* out_msg) {
//     // _out_msg.data = data;
//     rcl_ret_t rc = rcl_publish(&_publisher, out_msg, NULL);
//     return rc == RCL_RET_OK;
// }


// }