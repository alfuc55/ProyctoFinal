#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>

ros::NodeHandle nh;
Servo myservo1;
Servo myservo2;


// configuracion de servomotor 
const int servo1 = 5;
const int servo2 = 3;
// Configuración de pines para el primer motor
const int enablePinMotor1 = 11; // Pin de habilitación del puente H para el primer motor
const int in1PinMotor1 = 8;   // Primer pin de entrada del puente H para el primer motor
const int in2PinMotor1 = 7;   // Segundo pin de entradxa del puente H para el primer motor

// Configuración de pines para e l segundo motor
const int enablePinMotor2 = 6; // Pin de habilitación del puente H para el segundo motor
const int in1PinMotor2 = 4;    // Primer pin de entrada del puente H para el segundo motor
const int in2PinMotor2 = 2;    // Segundo pin de entrada del puente H para el segundo motor

int motorSpeedMotor1 = 0; // Variable para almacenar la velocidad del primer motor
int motorSpeedMotor2 = 0; // Variable para almacenar la velocidad del segundo motor
int posicionServo1 = 0;
int posicionServo2 = 0;
void servo1_cb(const std_msgs::Int16& cmd_msg){
    posicionServo1 = cmd_msg.data;
    myservo1.write(posicionServo1);    
    delay(10);
}
void servo2_cb(const std_msgs::Int16& cmd_msg){
    posicionServo2 = cmd_msg.data;
    myservo2.write(posicionServo2);    
    delay(10);
}
ros::Subscriber<std_msgs::Int16> subServo1("servo1", servo1_cb);
ros::Subscriber<std_msgs::Int16> subServo2("servo2", servo2_cb);

void motor_cb1(const std_msgs::Int16& cmd_msg) {
  // El mensaje de ROS proporciona un valor de velocidad de 0 a 255
  motorSpeedMotor1 = cmd_msg.data;
  // Control del primer motor
  if (motorSpeedMotor1 > 0) { // ir hacia adelante 
    digitalWrite(in1PinMotor1, HIGH);
    digitalWrite(in2PinMotor1, LOW);
  } else {       // ir hacia atras cambiando el sentido del giro 
    digitalWrite(in1PinMotor1, LOW);
    digitalWrite(in2PinMotor1, HIGH);
  }
  analogWrite(enablePinMotor1, abs(motorSpeedMotor1)); // Configurar la velocidad del primer motor mediante PWM
}
void motor_cb2(const std_msgs::Int16& cmd_msg) {
  // El mensaje de ROS proporciona un valor de velocidad de 0 a 255
  motorSpeedMotor2 = cmd_msg.data;
  // Control del segundo motor
  if (motorSpeedMotor2 > 0) {
    digitalWrite(in1PinMotor2, HIGH);
    digitalWrite(in2PinMotor2, LOW);
  } else {
    digitalWrite(in1PinMotor2, LOW);
    digitalWrite(in2PinMotor2, HIGH);
  }
  analogWrite(enablePinMotor2, abs(motorSpeedMotor2)); // Configurar la velocidad del segundo motor mediante PWM
}

ros::Subscriber<std_msgs::Int16> subMotor1("motor_speed1", motor_cb1);
ros::Subscriber<std_msgs::Int16> subMotor2("motor_speed2", motor_cb2);

void setup() {
  pinMode(enablePinMotor1, OUTPUT);
  pinMode(in1PinMotor1, OUTPUT);
  pinMode(in2PinMotor1, OUTPUT);

  pinMode(enablePinMotor2, OUTPUT);
  pinMode(in1PinMotor2, OUTPUT);
  pinMode(in2PinMotor2, OUTPUT);
  myservo1.attach(servo1);
  myservo2.attach(servo2);
  nh.initNode();
  nh.subscribe(subMotor1);
  nh.subscribe(subMotor2);
  nh.subscribe(subServo1);
  nh.subscribe(subServo2);
}

void loop() {
  nh.spinOnce();
  // No es necesario agregar un delay en este caso, pero puedes hacerlo si lo deseas
}