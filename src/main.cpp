#include <micro_ros_arduino.h>

#include <Arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// #include <std_msgs/msg/header.h>
#include <drive_controller_msgs/msg/tank.h>

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const drive_controller_msgs__msg__Tank * msg1 = (const drive_controller_msgs__msg__Tank *)msgin;
	// const std_msgs__msg__Header * msg1 = (const std_msgs__msg__Header *)msgin;
  Serial1.printf("%s[%d,%d]: (%f,%f)\n", msg1->header.frame_id.data, msg1->header.stamp.sec, msg1->header.stamp.nanosec, msg1->left, msg1->right);
	// Serial1.printf("%s: (%d,%d)\n", msg1->frame_id.data, msg1->stamp.sec, msg1->stamp.nanosec);
	// Serial1.printf("%s: (%d,%d)\n", msg.frame_id.data, msg.stamp.sec, msg.stamp.nanosec);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  
}

void setup() {
	rcl_subscription_t subscriber;
	// drive_controller_msgs__msg__Tank msg;
	
	rclc_executor_t executor;
	rclc_support_t support;
	drive_controller_msgs__msg__Tank msg;
	rcl_node_t node;
	Serial1.begin(9600);
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  rcl_allocator_t allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(drive_controller_msgs, msg, Tank),
    // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
	"micro_ros_arduino_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

	// Assigning dynamic memory to the frame_id char sequence
	msg.header.frame_id.capacity = 100;
	msg.header.frame_id.data = (char*) malloc(msg.header.frame_id.capacity * sizeof(char));
	msg.header.frame_id.size = 0;

	// Assigning value to the frame_id char sequence
	strcpy(msg.header.frame_id.data, "Hello World");
	msg.header.frame_id.size = strlen(msg.header.frame_id.data);

	while (1) {
		delay(100);
  		RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
	}
}

void loop() {
  
}