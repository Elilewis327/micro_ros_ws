#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3_stamped.h>

#include <micro_ros_arduino.h>

#include <Arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#define MPU_FRAMEID "base_imu"

rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;