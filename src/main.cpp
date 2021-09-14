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

#include <std_msgs/msg/header.h>

unsigned long previous_time = 0;
const unsigned wait_time = 200;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define STRING_BUFFER_LEN 50

// Micro Ros declarations
rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
rcl_subscription_t pong_subscriber;

std_msgs__msg__Header incoming_ping;
std_msgs__msg__Header outcoming_ping;
std_msgs__msg__Header incoming_pong;

int device_id;
int seq_no;
int pong_count;

const unsigned int timer_timeout = 1000;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

// Callbacks
void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {

		// seq_no = rand();
		// sprintf(outcoming_ping.frame_id.data, "%d_%d", seq_no, device_id);
    //TODO
    char s[] = "ping";
		memcpy(outcoming_ping.frame_id.data, s, strlen(s) + 1);
    outcoming_ping.frame_id.size = strlen(outcoming_ping.frame_id.data);

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outcoming_ping.stamp.sec = ts.tv_sec;
		outcoming_ping.stamp.nanosec = ts.tv_nsec;

		// Reset the pong count and publish the ping message
		// pong_count = 0;
		rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL);
		// printf("Ping send seq %s\n", outcoming_ping.frame_id.data);
	}
}

void ping_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

	// Dont pong my own pings
	// if(strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) != 0){

    char s[] = "pong";
		memcpy(msg->frame_id.data, s, strlen(s) + 1);
    msg->frame_id.size = strlen(outcoming_ping.frame_id.data);

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outcoming_ping.stamp.sec = ts.tv_sec;
		outcoming_ping.stamp.nanosec = ts.tv_nsec;

		printf("Ping received with seq %s. Answering.\n", msg->frame_id.data);
		rcl_publish(&pong_publisher, (const void*)msg, NULL);
    digitalWrite(LED_PIN, !digitalRead(LED_BUILTIN));
	// }
}


void pong_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;

	if(strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) == 0) {
		pong_count++;
		printf("Pong for seq %s (%d)\n", msg->frame_id.data, pong_count);
	}
  
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  delay(2000);    //time to delay micro_ros agent startup
  Serial.println("Initializing allocator");
  digitalWrite(LED_PIN, LOW);

  rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

	// Create a reliable ping publisher
	RCCHECK(rclc_publisher_init_default(&ping_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

	// Create a best effort pong publisher
	RCCHECK(rclc_publisher_init_best_effort(&pong_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));

	// Create a best effort ping subscriber
	RCCHECK(rclc_subscription_init_best_effort(&ping_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping"));

	// Create a best effort  pong subscriber
	RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));


	// Create a 3 seconds ping timer timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(400), ping_timer_callback));


	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping,
		&ping_subscription_callback, ON_NEW_DATA));
	// RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong,
	// 	&pong_subscription_callback, ON_NEW_DATA));

	// Create and allocate the pingpong messages
	char outcoming_ping_buffer[STRING_BUFFER_LEN];
	outcoming_ping.frame_id.data = outcoming_ping_buffer;
	outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

	char incoming_ping_buffer[STRING_BUFFER_LEN];
	incoming_ping.frame_id.data = incoming_ping_buffer;
	incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

	char incoming_pong_buffer[STRING_BUFFER_LEN];
	incoming_pong.frame_id.data = incoming_pong_buffer;
	incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

	device_id = rand();

	while(1){
		unsigned long t = millis();
    if (t > previous_time + wait_time) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      previous_time = t;
      
    }
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&ping_publisher, &node));
	RCCHECK(rcl_publisher_fini(&pong_publisher, &node));
	RCCHECK(rcl_subscription_fini(&ping_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&pong_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}

void loop() {
  
}