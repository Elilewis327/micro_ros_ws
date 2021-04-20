#include <micro_ros_arduino.h>

#include <Arduino.h>
#if __STDC_VERSION__ >= 199901L
#define _XOPEN_SOURCE 600
#else
#define _XOPEN_SOURCE 500
#endif /* __STDC_VERSION__ */
#include <ctime>
#include <sys/_timespec.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "mpu6050_node.h"

#include <drive_controller_msgs/msg/tank.h>
#include <drive_controller_msgs/msg/swerve.h>
#include <sensor_msgs/msg/imu.h>

unsigned long previous_time = 0;
unsigned long current_time = 0;
const unsigned wait_time = 20;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Micro Ros declarations
rcl_subscription_t drive_sub;
rcl_publisher_t imu_pub;
drive_controller_msgs__msg__Tank drive_msg;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// I/O
mpu6050_node imu;

const unsigned int timer_timeout = 1000;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

// Callbacks
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

        // Update header timestamp
        struct timespec tv = {0};
        clock_gettime(0, &tv);
        imu_msg.header.stamp.nanosec = tv.tv_nsec;
        imu_msg.header.stamp.sec = tv.tv_sec;
        
        imu.update(imu_msg);
    }
}

void drive_sub_callback(const void * msgin)
{  
  const drive_controller_msgs__msg__Tank * drive_msg = (const drive_controller_msgs__msg__Tank *)msgin;
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);
  Serial.println("Initializing allocator");
  allocator = rcl_get_default_allocator();
  digitalWrite(LED_PIN, LOW);

  // create init_options
  Serial.print("Initializing default support init");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  Serial.print("Initializing creating teensy node");
  RCCHECK(rclc_node_init_default(&node, "teensy_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &imu_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "mpu6050_pub"
  ));

  // create subscriber
  Serial.print("Initializing Tank Subscriber");
  RCCHECK(rclc_subscription_init_default(
    &drive_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(drive_controller_msgs, msg, Tank),
    "tank_sub"));

  // Create timer,
  Serial.print("Initializing timer");
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(wait_time),
      imu_timer_callback
  ));

  // create executor
  Serial.print("Initializing executor");
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // handles equal to number of topics/subscriptions
  RCCHECK(rclc_executor_add_subscription(&executor, &drive_sub, &drive_msg, &drive_sub_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // set imu_msg header
  imu_msg.header.frame_id.data = (char*)malloc(100*sizeof(char));
  char frame[] = "/IMU";
  memcpy(imu_msg.header.frame_id.data, frame, strlen(frame) + 1);
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
  imu_msg.header.frame_id.capacity = 100;
}

void loop() {
  current_time = millis();
  if(current_time - previous_time > wait_time){
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    
    previous_time = current_time;
  }
}