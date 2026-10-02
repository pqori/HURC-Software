#include "Messages.h"

void StringMsg::Set(const char* value) {
    if (_msg == nullptr) return;
    RCL_CHECK_ALLOCATOR(&uROS::allocator, return;);
    _msg->data.size = strlen(value);
    if (_msg->data.data == nullptr) {
      _msg->data.data = (char*)uROS::allocator.allocate(_msg->data.size + 1, uROS::allocator.state);
      _msg->data.capacity = _msg->data.size + 1;
    }
    if (_msg->data.size >= _msg->data.capacity) {
      _msg->data.data = (char*)uROS::allocator.reallocate(_msg->data.data, _msg->data.size + 1, uROS::allocator.state);
      _msg->data.capacity = _msg->data.size + 1;
    }

    strcpy(_msg->data.data, value);
  }