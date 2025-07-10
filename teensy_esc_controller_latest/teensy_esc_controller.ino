#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

// RCL objects
rcl_subscription_t subscriber;
rcl_subscription_t key_subscriber;
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Messages
geometry_msgs__msg__Twist msg_sub;
std_msgs__msg__String key_msg;
std_msgs__msg__String msg_pub;

// ESC and Motor Control - Using Servo library for ESC control
const int ESC1_PIN = 9;   // PWM pin for ESC 1
const int ESC2_PIN = 10;  // PWM pin for ESC 2
const int REVERSE1_PIN = 7; // Reverse pin for ESC 1
const int REVERSE2_PIN = 8; // Reverse pin for ESC 2

// PWM frequency for ESC control (50Hz = 20ms period)
const int PWM_FREQUENCY = 50;

// Motor control variables
const int NEUTRAL_PWM = 1000;
const int PWM_MIN_LIMIT = 1000;  // Minimum safe PWM (max forward)
const int PWM_MAX_LIMIT = 2000;  // Maximum safe PWM

// Incremental PWM control variables
int current_esc1_pwm = 1000;  // Current PWM for ESC1 - starts at 1000
int current_esc2_pwm = 1000;  // Current PWM for ESC2 - starts at 1000
bool reverse1_active = false;        // Reverse state for ESC1
bool reverse2_active = false;        // Reverse state for ESC2

// PWM increment/decrement value
const int PWM_STEP = 20;

// Status message buffer
char status_buffer[200];

// Turn timer variables
unsigned long turn_start_time = 0;
bool turn_in_progress = false;
const unsigned long TURN_DURATION = 1000; // 1 second turn duration

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Declare global variables to store original PWM values before turning
int original_esc1_pwm = 1000;
int original_esc2_pwm = 1000;

// Function to write microseconds using Teensy native PWM
void writeMicrosecondsTeensy(int pin, int microseconds) {
  // Convert microseconds to PWM value for 50Hz (20ms period)
  // PWM range: 0-65535 (16-bit)
  int pwm_value = map(microseconds, 0, 20000, 0, 65535);
  analogWrite(pin, pwm_value);
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Function to handle incremental key commands
void handleIncrementalKeyCommand(char key) {
  switch(key) {
    case 'i': // Increase both ESCs by +20
      current_esc1_pwm = 1200;
      current_esc2_pwm = 1200;
      break;

    case 'u': // Increase ESC1 by +20, no change to ESC2
      current_esc1_pwm = constrain(current_esc1_pwm + PWM_STEP, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
      break;

    case 'o': // Increase ESC2 by +20, no change to ESC1
      current_esc2_pwm = constrain(current_esc2_pwm + PWM_STEP, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
      break;

    case 'j': // Turn left (increase ESC1 by 50 PWM)
      current_esc1_pwm = 1200;
      current_esc2_pwm = 1250;
      break;

    case 'l': // Turn right (increase ESC2 by 50 PWM)
      current_esc1_pwm = 1250;
      current_esc2_pwm = 1200;
      break;

    case 'k': // Decrease both ESCs by -20
      current_esc1_pwm = constrain(current_esc1_pwm - PWM_STEP, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
      current_esc2_pwm = constrain(current_esc2_pwm - PWM_STEP, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
      break;

    case 'm': // Decrease ESC1 by -20, no change to ESC2
      current_esc1_pwm = constrain(current_esc1_pwm - PWM_STEP, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
      break;

    case '.': // Decrease ESC2 by -20, no change to ESC1
      current_esc2_pwm = constrain(current_esc2_pwm - PWM_STEP, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
      break;

    case ',': // Reverse command (toggle reverse for both ESCs)
      reverse1_active = !reverse1_active;
      reverse2_active = !reverse2_active;
      break;

    case ' ': // Spacebar - Stop system, set both to 1000
      current_esc1_pwm = 1000;
      current_esc2_pwm = 1000;
      reverse1_active = false;
      reverse2_active = false;
      turn_in_progress = false;
      break;

    default:
      // No action for other keys
      return; // Exit early for unknown keys
  }

  // Apply the reverse pin states
  digitalWrite(REVERSE1_PIN, reverse1_active ? HIGH : LOW);
  digitalWrite(REVERSE2_PIN, reverse2_active ? HIGH : LOW);

  // Apply PWM values to ESCs
  writeMicrosecondsTeensy(ESC1_PIN, current_esc1_pwm);
  writeMicrosecondsTeensy(ESC2_PIN, current_esc2_pwm);

  // Update status message
  snprintf(status_buffer, sizeof(status_buffer), 
           "Key: %c | PWM: ESC1=%d ESC2=%d | Rev: ESC1=%s ESC2=%s", 
           key, current_esc1_pwm, current_esc2_pwm,
           reverse1_active ? "ON" : "OFF",
           reverse2_active ? "ON" : "OFF");

  // Visual feedback
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

// Function to handle turn completion
void handleTurnCompletion() {
  if (turn_in_progress && (millis() - turn_start_time >= TURN_DURATION)) {
    // Reset the PWM values for ESC1 and ESC2 to their original values after the turn
    current_esc1_pwm = original_esc1_pwm;  // Restore original PWM for ESC1
    current_esc2_pwm = original_esc2_pwm;  // Restore original PWM for ESC2
    
    // Apply the restored PWM values
    writeMicrosecondsTeensy(ESC1_PIN, current_esc1_pwm);
    writeMicrosecondsTeensy(ESC2_PIN, current_esc2_pwm);
    
    turn_in_progress = false;
    
    // Update status message
    snprintf(status_buffer, sizeof(status_buffer), 
             "Turn completed | PWM: ESC1=%d ESC2=%d | Rev: ESC1=%s ESC2=%s", 
             current_esc1_pwm, current_esc2_pwm,
             reverse1_active ? "ON" : "OFF",
             reverse2_active ? "ON" : "OFF");
  }
}

// Timer callback to publish status and handle turn completion
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Handle turn completion
    handleTurnCompletion();
    
    // Publish current status
    msg_pub.data.data = status_buffer;
    msg_pub.data.size = strlen(status_buffer);
    msg_pub.data.capacity = sizeof(status_buffer);

    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
  }
}

// Define the callback for cmd_vel subscription
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Simple logic to determine key from twist values
  char key = 'x'; // Default unknown key
  
  if (msg->linear.x > 0.4 && fabs(msg->angular.z) < 0.1) {
    key = 'i'; // Forward
  } else if (msg->linear.x > 0.4 && msg->angular.z > 0.4) {
    key = 'u'; // Forward-left
  } else if (msg->linear.x > 0.4 && msg->angular.z < -0.4) {
    key = 'o'; // Forward-right
  } else if (fabs(msg->linear.x) < 0.1 && msg->angular.z > 0.4) {
    key = 'j'; // Turn left (preset values)
  } else if (fabs(msg->linear.x) < 0.1 && msg->angular.z < -0.4) {
    key = 'l'; // Turn right (preset values)
  } else if (fabs(msg->linear.x) < 0.1 && fabs(msg->angular.z) < 0.1) {
    key = ' '; // Stop (spacebar equivalent)
  } else if (msg->linear.x < -0.4) {
    key = ','; // Reverse
  }
  
  // Process the key command
  if (key != 'x') {
    handleIncrementalKeyCommand(key);
  }
}

// Define the callback for key_commands subscription
void key_subscription_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  if (msg->data.size > 0) {
    char key = msg->data.data[0]; // Get first character
    handleIncrementalKeyCommand(key);
  }
}

void setup() {
  // Initialize serial for debugging (optional)
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(REVERSE1_PIN, OUTPUT);
  pinMode(REVERSE2_PIN, OUTPUT);
  pinMode(ESC1_PIN, OUTPUT);
  pinMode(ESC2_PIN, OUTPUT);
  
  // Setup PWM for ESC control on Teensy
  analogWriteResolution(16);  // Use 16-bit PWM resolution
  analogWriteFrequency(ESC1_PIN, PWM_FREQUENCY);  // 50Hz for ESC
  analogWriteFrequency(ESC2_PIN, PWM_FREQUENCY);  // 50Hz for ESC
  
  // Initialize ESCs to 1000 PWM (start position)
  writeMicrosecondsTeensy(ESC1_PIN, 1000);
  writeMicrosecondsTeensy(ESC2_PIN, 1000);
  digitalWrite(REVERSE1_PIN, LOW);
  digitalWrite(REVERSE2_PIN, LOW);
  
  // Initialize status message
  strcpy(status_buffer, "Teensy PWM Controller Ready - Starting at 1000 PWM");
  
  // Wait for ESCs to initialize
  delay(2000);
  
  // Configure micro-ROS transport
  set_microros_transports();
  
  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  
  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "teensy_incremental_pwm_controller", "", &support));
  
  // Create subscriber for cmd_vel (traditional teleop)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  // Create subscriber for direct key commands
  RCCHECK(rclc_subscription_init_default(
    &key_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "key_commands"));
  
  // Create publisher for status
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "drive_status"));
  
  // Create timer for publishing status every 100ms (more frequent for better turn handling)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Create executor with 3 handles (2 subscriptions + 1 timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub,
    &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &key_subscriber, &key_msg,
    &key_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Initialize status message
  msg_pub.data.data = status_buffer;
  msg_pub.data.size = 0;
  msg_pub.data.capacity = sizeof(status_buffer);
  
  // Initial status update
  snprintf(status_buffer, sizeof(status_buffer), 
           "Ready - Started at 1000 PWM | PWM: ESC1=%d ESC2=%d | Rev: ESC1=OFF ESC2=OFF", 
           current_esc1_pwm, current_esc2_pwm);
}

void loop() {
  delay(10); // Reduced delay for more responsive operation
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}