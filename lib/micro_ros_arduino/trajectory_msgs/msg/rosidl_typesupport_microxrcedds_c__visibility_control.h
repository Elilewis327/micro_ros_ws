// generated from
// rosidl_typesupport_microxrcedds_c/resource/rosidl_typesupport_microxrcedds_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef TRAJECTORY_MSGS__MSG__ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__VISIBILITY_CONTROL_H_
#define TRAJECTORY_MSGS__MSG__ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_EXPORT_trajectory_msgs __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_IMPORT_trajectory_msgs __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_EXPORT_trajectory_msgs __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_IMPORT_trajectory_msgs __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_BUILDING_DLL_trajectory_msgs
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_trajectory_msgs ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_EXPORT_trajectory_msgs
  #else
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_trajectory_msgs ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_IMPORT_trajectory_msgs
  #endif
#else
  #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_EXPORT_trajectory_msgs __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_IMPORT_trajectory_msgs
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_trajectory_msgs __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_trajectory_msgs
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // TRAJECTORY_MSGS__MSG__ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__VISIBILITY_CONTROL_H_
