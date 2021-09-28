// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drive_controller_msgs:msg/Swerve.idl
// generated code does not contain a copyright notice

#ifndef DRIVE_CONTROLLER_MSGS__MSG__DETAIL__SWERVE__STRUCT_H_
#define DRIVE_CONTROLLER_MSGS__MSG__DETAIL__SWERVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/Swerve in the package drive_controller_msgs.
typedef struct drive_controller_msgs__msg__Swerve
{
  std_msgs__msg__Header header;
  float ws[4];
  float wa[4];
} drive_controller_msgs__msg__Swerve;

// Struct for a sequence of drive_controller_msgs__msg__Swerve.
typedef struct drive_controller_msgs__msg__Swerve__Sequence
{
  drive_controller_msgs__msg__Swerve * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drive_controller_msgs__msg__Swerve__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRIVE_CONTROLLER_MSGS__MSG__DETAIL__SWERVE__STRUCT_H_
