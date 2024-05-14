# ProyctoFinal
# Proyecto Final Diseño de sistemas Roboticos 
EL siguiente proyecto presenta los códigos necesarios para controlar un robot de eje diferencial mediante ROS y Arduino. 
Hardware requerido: 
    •	Raspberry 
    •	Arduino uno 
    •	Pc master
Actuadores 
    •	2 motores DC
    •	2 servomotores  
Para ejecutar estos códigos se necesita una PC máster con una distribución de Linux, y una raspberry 4 con Ubuntu Server 20.04 como sistema operativo y tener ROS noetic instalado.
Software requerido: 
    •	Linux en Pc máster
    •	Linux Ubuntu server 20.04 en Raspberry
    •	Ros Noetic instalado en raspberry 
La comunicación entre la raspberry y el Arduino UNO se realiza mediante el puerto serial, lo cual requiere paquetes específicos de ROS, los cuales se instalan siguiendo el siguiente tutorial: 
https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

 El Siguiente código es cargado en el Arduino: 
           
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

Y en la raspberry se debera ejecutar el core de ROS, y el nodo de comunicacion serial para el Arduino, Una vez hecho esto se ejecuta el siguiente codigo de python el cual permite la teleopreacion del robot. 

            #!/usr/bin/env python
            license removed for brevity
            import rospy
            from std_msgs.msg import String
            from std_msgs.msg import Int16
            from geometry_msgs.msg import Twist
            from turtlesim.srv import TeleportAbsolute, Teleport Relative
            from pynput import keyboard as kb
            import termios, sys, os
            import tty
            import select
            posicionS1 = 60
            posicionS20
            def motor_control():
                rospy.init_node('motor_control',anonymous=True # iniciar el nodo
                pub = rospy. Publisher('motor_speed1', Int16, queue_size=10) #ropic que publicara la velocidad del motor 1
                pub2 =rospy.Publisher('motor_speed2', Int16, queue_size=10) #topic que publivara la velocidad del motoe 2
                pubservo1 = rospy. Publisher('servo1', Int16, queue_size=10) # topic para publivar posicion de servo
                pubservo2 = rospy. Publisher('servo2', Int16, queue_size=10) # topic para publicar posicion del servo 2
                rate = rospy.Rate (20) 20hz
                while not rospy.is_shutdown():
                    key = getKey() # obtener tevla presionada
                    if key == "r":
                        break
                    #COM
                    control movimiento (key) # variables obtenidas dependiendo de la tecla presioanda
                    motor_speed1, motor_speed2, posicionS1, posicions2 = control # asignar variables
                    rospy.loginfo("spd1: %d spd2: %d PS1=%d PS2-%d ", motor_speed1, motor_speed2 posicionS1, posicions2)
                    pub.publish(motor_speed1)
                    pub2.publish(motor_speed2)
                    pubservo1.publish(posicionS1)
                    pubservo2.publish(posicionS2)
                    rate.sleep()
            def getkey():
                tty.setraw(sys.stdin.fileno())
                select.select([sys.stdin], [], [], 0)
                key = sys.stdin.read(1)
                termios.tcsetattr(sys.stdin, termios. TCSADRAIN, settings)
                return key
            def movimiento(tecla):
            	print('Se ha pulsado la tecla ' + str(tecla))
                global posicionS1
                global posicionS2
                motor_speed1 = 0;
                motor_spedd2 = 0;
                if tecla == 'w'
                    motorspeed1 = 250
                    motorspeed2 = 250
                elif tecla == 's'
                    motorspeed1 = -250
                    motorspeed2 = -250
                elif tecla == 'a'
                    motorspeed1 = -230
                    motorspeed2 = 230
                elif tecla == 'd'
                    motorspeed1 = 230
                    motorspeed2 = -230
                elif tecla == 'q'
                    motorspeed1 = 0
                    motorspeed2 = 0
                elif tecla == 't'
                    posicionS1 = posicionS1+1
                elif tecla == 'g'
                    posicionS1 = posicionS1-1
                elif tecla == 'y'
                    posicionS2 = posicionS2+1
                elif tecla == 'h'
                    posicionS2 = posicionS2-1
                elif tecla == 'r'
                    motorspeed1 = 0
                    motorspeed2 = 0
                return motor_speed1, motor_speed2, posicionS1, posicionS2

            def talker():
                while(1):
                    key = getKey()
                    if key=="r":
                        break
                    movimiento(key)

            if _name_ == '_main_':
                settings = termios.tcgetattr(sys.stdin)
                twist = Twist()
                try:
                    key = getKey()
                    motor_control()
                except rospy.ROSInterruptException:
                    pass

El mapeo de las teclas es: 
    -W: Adelante  
    -A: izquierda 
    -S: atrás
    -D: derecha
    -Q: detener 
    -T: movimiento arriba garra 
    -G: movimiento abajo garra 
    -Y: movimiento arriba pala
    -H: movimiento abajo pala
    -R: salir del programa
De esta manera el robot es completamente pilotado en modo tele operación
