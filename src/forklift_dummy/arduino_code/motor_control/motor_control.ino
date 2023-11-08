#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <message_interfaces/msg/pwm.h>

rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
std_msgs__msg__Int16 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
//
//int pwm1 = 0;
//int pwm2 = 0;

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
}

void subscription_callback_1(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  int pwm = int(msg->data);
  move_left_wheel(pwm);
 
}

void subscription_callback_2(const void * msgin)
{
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  int pwm = int(msg->data);
  move_right_wheel(pwm);
}

void setup() {
  // put your setup code here, to run once:

  set_microros_transports();

  pinMode(7, OUTPUT); //dir1
  pinMode(8, OUTPUT);  //pwm1
  pinMode(9, OUTPUT); //dir2
  pinMode(10, OUTPUT); //pwm2

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber1,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
            "/pwm1"));

  RCCHECK(rclc_subscription_init_default(
            &subscriber2,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
            "/pwm2"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg, &subscription_callback_1, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg, &subscription_callback_2, ON_NEW_DATA));
}

void move_left_wheel(int pwm) {

  if (pwm < 0) {

    digitalWrite(7, LOW);

  }
  else {

    digitalWrite(7, HIGH);

  }

  analogWrite(8, abs(pwm));

}

void move_right_wheel(int pwm) {

  if (pwm > 0) {

    digitalWrite(9, LOW);

  }
  else {

    digitalWrite(9, HIGH);

  }

  analogWrite(10, abs(pwm));

}

void loop() {

//  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

}
