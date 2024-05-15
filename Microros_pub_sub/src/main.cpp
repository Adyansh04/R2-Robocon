#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <Wire.h>
#include <TFLI2C.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int64_multi_array.h> 
#include <std_srvs/srv/set_bool.h>




#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#endif


// define pins for ultrasonic 
int trigPin = 52;    // TRIG pin
int echoPin = 53;    // ECHO pin
float duration_us, distance_cm=0;
bool reached_height = false;
bool reached_below = false;

// Define pins for LSA08 connection 
float kp = 5;   // 3
double kd = 3;  // 5
float ki = 0;   // 0

float prev_err = 0;
float curr_err;
float area = 0;
float prop;
float deriv;
float integral;
float final_PID;
int prev_data;
const int baseSpeed = 200;  // base pwm value for motors
int midValue = 35;

#define en 4          
const int jPulse = 2; // Hardware interrupt pin Due
int nodeCount = 0;
byte read=0;  
bool reached_zone_3  = false;
int val=0;

// extern TwoWire Wire1;
TFLI2C tflI2C;   // create object for tfluna library
int16_t  tfDist_1;   // distance in centimeters
int16_t  tfDist_2;  
int16_t  tfDist_3; 
int16_t  tfDist_4;     
int16_t  tfAddr1 = 0x11;  // Use this default I2C address
int16_t  tfAddr2 = 0x12;
int16_t  tfAddr3 = 0x13;
int16_t  tfAddr4 = 0x14;


// declaring motor pwm variables (these are used for sending pwm signals to motor driver)
int FL_motor;
int FR_motor;
int BL_motor;
int BR_motor;

// declaring pins for motor driver (Arduino_Due)
#define mPinFL 11
#define mPinFR 10
#define mPinBL 8
#define mPinBR 9
#define dirFL 33
#define dirFR 31
#define dirBL 27
#define dirBR 29

// declarings pins for lift and claw 
#define pwmPin1 46
#define dir1 48
#define pwmPin2 44
#define dir2 42

int count = 0, last_value = 0;


rcl_publisher_t publisher_line;
rcl_publisher_t publisher_luna;
rcl_publisher_t publisher_junction;

std_msgs__msg__Int8 junc;
std_msgs__msg__Int32 lsa08; 
std_msgs__msg__Int64MultiArray msg_luna;        


rcl_subscription_t subscriber;       
geometry_msgs__msg__Twist sub_msg;

rclc_executor_t executor;
rclc_executor_t executor1;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_node_t node2;


rcl_timer_t timer_line;
rcl_timer_t timer_luna;
rcl_timer_t timer_junc;

rcl_service_t service;
rcl_service_t service1;
rcl_service_t service2;
rcl_wait_set_t wait_set;

std_srvs__srv__SetBool_Response req;
std_srvs__srv__SetBool_Response res;

std_srvs__srv__SetBool_Response req1;
std_srvs__srv__SetBool_Response res1;

std_srvs__srv__SetBool_Response req2;
std_srvs__srv__SetBool_Response res2;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void pulse_detected()
{
  nodeCount++;
}

void pinSetup()

{

  pinMode(en,OUTPUT);
  digitalWrite(en,LOW);
  pinMode(jPulse, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(jPulse), pulse_detected, RISING);
  pinMode(mPinFL, OUTPUT);
  pinMode(mPinFR, OUTPUT);
  pinMode(mPinBL, OUTPUT);
  pinMode(mPinBR, OUTPUT);

  pinMode(dirFL, OUTPUT);
  pinMode(dirFR, OUTPUT);
  pinMode(dirBL, OUTPUT);
  pinMode(dirBR, OUTPUT);

  pinMode(dir1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
 

}
// Error handle loop
void error_loop()
 {
  while(1) {
    delay(100);
  }
}

void rotate_clockwise()
{

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration_us = pulseIn(echoPin, HIGH);
 
  // calculate the distance
  distance_cm = 0.017 * duration_us;
  delay(100);

  if(distance_cm>10.0)
  {
  digitalWrite(dir1, HIGH);
  analogWrite(pwmPin1, 255);

  }
  else
  {
       analogWrite(pwmPin1, 0);
       reached_height= true;

  }

}
void rotate_anticlockwise()
{
 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration_us = pulseIn(echoPin, HIGH);
  // calculate the distance
  distance_cm = 0.017 * duration_us;
  delay(100);

  if(distance_cm<58.0)
  {
      digitalWrite(dir1, LOW);
      analogWrite(pwmPin1, 255);

  }
  else
  { 
    analogWrite(pwmPin1, 0);
    reached_below =true;

  }

}

void rotate_clockwise_claw(bool &success)
{
  

  digitalWrite(dir2, HIGH);
  analogWrite(pwmPin2, 150);
  delay(2000);
  val=1;
  analogWrite(pwmPin2, 0);
  success = true;
}
void rotate_anticlockwise_claw(bool &success)
{

  digitalWrite(dir2, LOW);
  analogWrite(pwmPin2, 150);
  delay(2000);
  val=0;
  analogWrite(pwmPin2, 0);
  success = true;
}
void leftTurn(int PID)
{
  int lS = min(baseSpeed-PID,255);
  int RS = min(baseSpeed+PID,255);
  int leftSpeed = max(0,lS);
  int RightSpeed = max(0,RS);
  analogWrite(mPinFL, leftSpeed);
  analogWrite(mPinFR, RightSpeed);
  analogWrite(mPinBL, leftSpeed);
  analogWrite(mPinBR, RightSpeed);
  last_value = leftSpeed;
}

void rightTurn(int PID)
{
  int lS = min(baseSpeed+PID,255);
  int RS = min(baseSpeed-PID,255);
  int leftSpeed = max(0,lS);
  int RightSpeed = max(0,RS);
  analogWrite(mPinFL, leftSpeed);
  analogWrite(mPinFR, RightSpeed);
  analogWrite(mPinBL, leftSpeed);
  analogWrite(mPinBR, RightSpeed);
  last_value = leftSpeed;
  
}
void pidFunction(byte data)
{
  digitalWrite(dirFL,LOW);
  digitalWrite(dirFR,LOW);
  digitalWrite(dirBL,LOW);
  digitalWrite(dirBR,LOW);

  float error = data - midValue;
  // curr_err = map(error, -35, 35, -maxSteeringValue, maxSteeringValue);
  curr_err = error;

  // *********** PID code :
  prop = kp*curr_err;    // propotional equation
  deriv = kd*(curr_err - prev_err);   // derivative equation
  integral = ki*(area + curr_err);      // integral equation
  
  final_PID = prop + deriv + integral;    // adding proportional, derivative and integral values

  // which direction to turn will be decided by the variable error 
  if(error < 0)
  {
    leftTurn(int(abs(final_PID)));    // turning left if sensor's reading less than 35 (error is -ve)
  }
  
  else
  {
    rightTurn(int(abs(final_PID)));;   // turning right if sensor's reading greater than 35 (error is +ve) 
  }
}

void move_little_forward(int last_value)
{
  digitalWrite(dirFL,LOW);
  digitalWrite(dirFR,LOW);
  digitalWrite(dirBL,LOW);
  digitalWrite(dirBR,LOW);
  analogWrite(mPinFL, last_value);
  analogWrite(mPinFR, last_value);
  analogWrite(mPinBL, last_value);
  analogWrite(mPinBR, last_value);
  delay(100);
}

void move_to_zone_3()
{     

     while(Serial1.available() > 0)
     {
        int read2=Serial1.read();

        if((nodeCount==1) && (read2==255))
        {
            if(count == 0)
            {
              move_little_forward(last_value);
              count++;
            }

            digitalWrite(dirFL,LOW);
            digitalWrite(dirFR,HIGH);
            digitalWrite(dirBL,HIGH);
            digitalWrite(dirBR,LOW);
            analogWrite(mPinFL, 150);
            analogWrite(mPinFR, 150);
            analogWrite(mPinBL, 150);
            analogWrite(mPinBR, 150);
            
        }
        else if(nodeCount>=2)
        {
                analogWrite(mPinFL, 0);
                analogWrite(mPinFR, 0);
                analogWrite(mPinBL, 0);
                analogWrite(mPinBR, 0);
                reached_zone_3 = true;
                break;
        }
        else
        {
            pidFunction(read2);
        }
     }
    
}

void timer_callback_junc(rcl_timer_t * timer, int64_t last_call_time)
{
      RCLC_UNUSED(last_call_time);
      
      junc.data=nodeCount;

      if (timer!= NULL){

       RCSOFTCHECK(rcl_publish(&publisher_junction, &junc, NULL));
      
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
     
  Serial1.begin(115200);
  digitalWrite(en,LOW);
  while(Serial1.available()<=0);
  
  read=Serial1.read();
  // Serial.println(read);
  lsa08.data=read;
  digitalWrite(en,HIGH);
  
  
  if (timer!= NULL){

    RCSOFTCHECK(rcl_publish(&publisher_line, &lsa08, NULL));
      
  }
}

// luna timer callback
void timer_callback_multiarray(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  
   while(Wire1.available())
   {
    // Clear previous data
    msg_luna.data.size = 4;
    msg_luna.data.capacity = 4;
    static int64_t memory[4];
    msg_luna.data.data= memory;
   // Add new data
      tflI2C.getData(tfDist_1, tfAddr1);
      tflI2C.getData(tfDist_2, tfAddr2);
      tflI2C.getData(tfDist_3, tfAddr3);
      tflI2C.getData(tfDist_4, tfAddr4);
      msg_luna.data.data[0] = tfDist_1;
      msg_luna.data.data[1] = tfDist_2;
      msg_luna.data.data[2] = tfDist_3;
      msg_luna.data.data[3] = tfDist_4;

   }
  if (timer != NULL) {
      // Publish the message
      RCSOFTCHECK(rcl_publish(&publisher_luna, &msg_luna, NULL));
    }
 // }
}

void subscription_callback(const void *msgin) {

  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    

   float x1 = msg->linear.x;
   float y1 = msg->linear.y;
   float z1 = msg->angular.z;

  float mapped_leftHatx =  (255.0/2.0)*x1;
  float mapped_leftHaty = (255.0/2.0)* y1;
  float mapped_rightHatz = (255.0/1.0)* z1;

    FL_motor = mapped_leftHatx - mapped_rightHatz - mapped_leftHaty;
    BR_motor = mapped_leftHatx + mapped_rightHatz - mapped_leftHaty;
    FR_motor = mapped_leftHatx + mapped_rightHatz + mapped_leftHaty;
    BL_motor = mapped_leftHatx - mapped_rightHatz + mapped_leftHaty;
    
    // constraining motor variables between -255 to 255
    FL_motor = constrain(FL_motor, -255, 255);
    BR_motor = constrain(BR_motor, -255, 255);
    FR_motor = constrain(FR_motor, -255, 255);
    BL_motor = constrain(BL_motor, -255, 255);

    // assigning direction values
    // HIGH - Backward && LOW - Forward
    if(FL_motor < 0)
    {
      digitalWrite(dirFL,HIGH);
      // Serial.println("HIGH");
    } else {
      digitalWrite(dirFL,LOW);
      // Serial.println("LOW");
    }

    if(BR_motor < 0)
    {
      digitalWrite(dirBR,HIGH);
    } else {
      digitalWrite(dirBR,LOW);
    }

    if(FR_motor < 0)
    {
      digitalWrite(dirFR,HIGH);
      // Serial.println("HIGH");
    } else {
      digitalWrite(dirFR,LOW);
    }

    if(BL_motor < 0)
    {
      digitalWrite(dirBL,HIGH);
      // Serial.println("HIGH");
    } else {
      digitalWrite(dirBL,LOW);
    }

    FL_motor = abs(FL_motor);
    BR_motor = abs(BR_motor);
    FR_motor = abs(FR_motor);
    BL_motor = abs(BL_motor);
    
    // analogWrite those pwm values and write cases for Direction pins
    analogWrite(mPinFL,FL_motor);
    analogWrite(mPinFR,FR_motor);
    analogWrite(mPinBL,BL_motor);
    analogWrite(mPinBR,BR_motor);

}

void service_callback_lift(const void * req, void * res){
  
  
  std_srvs__srv__SetBool_Request * req_in=(std_srvs__srv__SetBool_Request *) req;
  std_srvs__srv__SetBool_Response * res_in=(std_srvs__srv__SetBool_Response *) res;

  if(req_in->data == 1)
  {
      
      while(reached_height==false)
      {
        rotate_clockwise();
      }

      res_in->success = true;
      reached_height = false;
    
  }

  else
  {   
      
      while(reached_below==false)
      {
        rotate_anticlockwise();
      }

      res_in->success = true;
      reached_below = false;
  }

}

void service_callback_claw(const void * req1, void * res1){
  
  
  std_srvs__srv__SetBool_Request * req_in1=(std_srvs__srv__SetBool_Request *) req1;
  std_srvs__srv__SetBool_Response * res_in1=(std_srvs__srv__SetBool_Response *) res1;

  bool success = false;

  if(req_in1->data == 1)
  {
//    res_in1->success = true;
//    res_in1->message = response_str;

      rotate_clockwise_claw(success);
      res_in1->success = success;
    
  }

  else
  {
//    res_in1->success = true;
//    res_in1->message = "f0";

      rotate_anticlockwise_claw(success);
      res_in1->success = success;
  }

}
void service_callback_line(const void * req2, void * res2){
  
  
  std_srvs__srv__SetBool_Request * req_in2=(std_srvs__srv__SetBool_Request *) req2;
  std_srvs__srv__SetBool_Response * res_in2=(std_srvs__srv__SetBool_Response *) res2;

  
  if(req_in2->data == 1)
  {
      
      
      move_to_zone_3();
      while(reached_zone_3==false)
      {
        move_to_zone_3();
      }

      res_in2->success = true;
  
  }


}

void setup() {
   
  pinSetup();
  // Configure serial transport
  

  set_microros_serial_transports(Serial);
  delay(1000);
  
  Wire1.begin(); 
  Serial.begin(115200);
  Serial1.begin(115200);


  
 // Try to initialize!

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node 
  RCCHECK(rclc_node_init_default(&node, "r2_vrc", "", &support));

  // create node 2 for subscriber 

  RCCHECK(rclc_node_init_default(&node2,"subscribe","",&support));

  // create service for lift2
   RCCHECK(rclc_service_init_default(
    &service, 
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), 
    "/service_lift"));

     //create service for claw
    RCCHECK(rclc_service_init_default(
    &service1, 
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), 
    "/service_claw"));

    //create service for line follower
    RCCHECK(rclc_service_init_default(
    &service2, 
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), 
    "/service_line_follow"));
  
  // create publisher for junction
    RCCHECK(rclc_publisher_init_default(
    &publisher_junction,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "Junction_count"));

  // create publisher lsa08 data
    RCCHECK(rclc_publisher_init_default(
    &publisher_line,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "line_lsa"));

 //create publisher for lunar sensor

    RCCHECK(rclc_publisher_init_default(
    &publisher_luna,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
    "luna_data"));

  // subscriber for cmd vel
    RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node2,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

//  create timer for lunar sensor

   const unsigned int timer_timeout_luna = 50;
    RCCHECK(rclc_timer_init_default(
    &timer_luna,
    &support,
    RCL_MS_TO_NS(timer_timeout_luna),
    timer_callback_multiarray));

  // // create timmer for junction
    const unsigned int timer_timeout_junc= 1;
    RCCHECK(rclc_timer_init_default(
    &timer_junc,
    &support,
    RCL_MS_TO_NS(timer_timeout_junc),
    timer_callback_junc));

  // create timer for LSA08,
    const unsigned int timer_timeout = 1;  // initially 100
    RCCHECK(rclc_timer_init_default(
    &timer_line,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  
RCCHECK(rclc_executor_init(&executor, &support.context,4, &allocator));
RCCHECK(rclc_executor_init(&executor1, &support.context,1, &allocator));

//RCCHECK(rclc_executor_add_timer(&executor, &timer_line));
// RCCHECK(rclc_executor_add_timer(&executor, &timer_junc));
RCCHECK(rclc_executor_add_timer(&executor, &timer_luna));

RCCHECK(rclc_executor_add_subscription(&executor1, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));

RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback_lift));
RCCHECK(rclc_executor_add_service(&executor, &service1, &req1, &res1, service_callback_claw));
RCCHECK(rclc_executor_add_service(&executor, &service2, &req2, &res2, service_callback_line));

}

void loop() {

  delay(100);

  RCSOFTCHECK(rclc_executor_spin_some(&executor1, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));


}
