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

#include <drive_controller_msgs/msg/tank.h>

unsigned long previous_time = 0;
const unsigned wait_time = 200;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial1.printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc); error_loop();}} //error_loop();
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial1.printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

#define STRING_BUFFER_LEN 50

// Micro Ros declarations
rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
rcl_subscription_t pong_subscriber;

drive_controller_msgs__msg__Tank incoming_ping;
drive_controller_msgs__msg__Tank outcoming_ping;
drive_controller_msgs__msg__Tank incoming_pong;

int device_id;
int seq_no;
int pong_count;

float left = -1.0;
float right = -1.0;


const unsigned int timer_timeout = 1000;

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

// Callbacks
void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);

	if (timer != NULL) {
    	// char s[] = "ping";
		// memcpy(outcoming_ping.header.frame_id.data, s, strlen(s) + 1);
    	// outcoming_ping.header.frame_id.size = strlen(outcoming_ping.header.frame_id.data);
		

		// Fill the message timestamp
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		outcoming_ping.header.stamp.sec = ts.tv_sec;
		outcoming_ping.header.stamp.nanosec = ts.tv_nsec;

		left += 0.1;
		right += 0.2;
		if (left > 1.0001 ) {
			left = -1.0;
		}

		if (right > 1.0001 ) {
			right = -1.0;
		}

    	outcoming_ping.left = left;
    	outcoming_ping.right = right;

		seq_no = rand();
		Serial1.printf("%s [%d_%d] L: %f, R: %f \n", outcoming_ping.header.frame_id.data, seq_no, device_id, outcoming_ping.left, outcoming_ping.right);
		// Reset the pong count and publish the ping message
		pong_count = 0;
		rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL);
	}
}

void ping_subscription_callback(const void * msgin)
{
	const drive_controller_msgs__msg__Tank * msg = (const drive_controller_msgs__msg__Tank *)msgin;

  Serial1.printf("Ping received with seq %s [%d:%d] L: %f, R: %f Answering.\n", msg->header.frame_id.data, msg->header.stamp.sec, msg->header.stamp.nanosec, msg->left, msg->right);
  rcl_publish(&pong_publisher, (const void*)msg, NULL);
    
}


void pong_subscription_callback(const void * msgin)
{
	const drive_controller_msgs__msg__Tank * msg = (const drive_controller_msgs__msg__Tank *)msgin;
  pong_count++;
  Serial1.printf("Pong for seq %s (%d) L: %f, R: %f\n", msg->header.frame_id.data, pong_count, msg->left, msg->right);
  
}

void setup() {
  Serial1.begin(9600);
  set_microros_transports();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  delay(2000);    //time to delay micro_ros agent startup
  Serial.println("Initializing allocator");
  digitalWrite(LED_BUILTIN, LOW);

  rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "pingpong_node", "", &support));

	// Create a reliable ping publisher
	RCCHECK(rclc_publisher_init_default(&ping_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drive_controller_msgs, msg, Tank), "/microROS/ping"));

	// Create a best effort pong publisher
	RCCHECK(rclc_publisher_init_best_effort(&pong_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drive_controller_msgs, msg, Tank), "/microROS/pong"));

	// Create a best effort ping subscriber
	RCCHECK(rclc_subscription_init_best_effort(&ping_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drive_controller_msgs, msg, Tank), "/microROS/ping"));

	// Create a best effort  pong subscriber
	RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(drive_controller_msgs, msg, Tank), "/microROS/pong"));


	// Create a 3 seconds ping timer timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(3000), ping_timer_callback));


	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping,
		&ping_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong,
		&pong_subscription_callback, ON_NEW_DATA));

	// Create and allocate the pingpong messages
	char outcoming_ping_buffer[] = "ping";
	outcoming_ping.header.frame_id.data = outcoming_ping_buffer;
	outcoming_ping.header.frame_id.size = strlen(outcoming_ping_buffer);
	outcoming_ping.header.frame_id.capacity = strlen(outcoming_ping_buffer);

	char incoming_ping_buffer[] = "ping";
	incoming_ping.header.frame_id.data = incoming_ping_buffer;
	incoming_ping.header.frame_id.capacity = STRING_BUFFER_LEN;

	char incoming_pong_buffer[STRING_BUFFER_LEN] = "pong";
	incoming_pong.header.frame_id.data = incoming_pong_buffer;
	incoming_pong.header.frame_id.capacity = STRING_BUFFER_LEN;

	device_id = rand();

	while(1){
		unsigned long t = millis();
    if (t > previous_time + wait_time) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      previous_time = t;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
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