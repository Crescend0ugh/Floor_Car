#include <AccelStepper.h>
#include <MultiStepper.h>

#include <Vex.h>

#include <Adafruit_MotorShield.h>
#include <Servo.h>

#include <SerialTransfer.h>

#include "src/write_buffer.h"
//#include "src/motor_driver.h"

#include <Wire.h>

#define MotorInterfaceType 4

AccelStepper stepper = AccelStepper(MotorInterfaceType, 3, 4, 5, 6);

Vex robot;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m3 = AFMS.getMotor(3);
Adafruit_DCMotor *m4 = AFMS.getMotor(4);

SerialTransfer transfer;

const uint8_t delta_servo_pos = 1;
uint8_t servo_pos = 90;
bool servo_going_cw = true;
Servo servo;

const unsigned long update_rate_ms = 50;
double delta_time = 0.01;

const int motor_speed = 20;

const uint8_t message_length = 32;

const uint8_t microphone_input_header = 0x02;
const uint8_t log_string_header = 0x03;
const uint8_t rc_header_byte = 0xFF;

enum rc_command : uint8_t
{
    none = 0,
    stop,
    w,
    s,
    a,
    d,
    pick_up,
    servo_ccw,
    servo_cw
};

// A makeshift Serial.println. Outputs to the Raspberry Pi terminal.
void send_log(String string)
{
    write_buffer writer(log_string_header);
    char string_buffer[string.length() + 1];
    string.toCharArray(string_buffer, sizeof(string_buffer));
    writer.write(string_buffer, sizeof(string_buffer));

    transfer.sendDatum(writer.buffer);
}

void stop_motors()
{
    m1->run(RELEASE);
    m4->run(RELEASE);
    m2->run(RELEASE);
    m3->run(RELEASE);
}

void handle_rc_command(rc_command command)
{
    switch (command)
    {
    case (rc_command::stop):
    {
        send_log("STOP");
        stop_motors();
        break;
    }
    case (rc_command::w):
    {
        send_log("W");
        m1->run(FORWARD);
        m4->run(FORWARD);
        m2->run(FORWARD);
        m3->run(FORWARD);
        break;
    }
    case (rc_command::s):
    {
        send_log("S");
        m1->run(BACKWARD);
        m4->run(BACKWARD);
        m2->run(BACKWARD);
        m3->run(BACKWARD);
        break;
    }
    case (rc_command::a):
    {
        send_log("A");
        m1->run(FORWARD);
        m4->run(FORWARD);

        m2->run(BACKWARD);
        m3->run(BACKWARD);
        break;
    }
    case (rc_command::d):
    {
        send_log("D");
        m1->run(BACKWARD);
        m4->run(BACKWARD);

        m2->run(FORWARD);
        m3->run(FORWARD);
        //driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::pick_up):
    {
        send_log("PICK UP");

        stop_motors();

        stepper.moveTo(1000);
        stepper.run();
        break;
    }
    case (rc_command::servo_ccw):
    {
        send_log("CCW");
        stepper.moveTo(1000);
        stepper.run();     
        //stepper.moveTo(0);

        break;
    }
     case (rc_command::servo_cw):
    {
        send_log("CW");
        stepper.moveTo(-1000);
        stepper.run();
        //stepper.moveTo(0);  
        break;
    }
    default:
        int received = static_cast<int>(command); 
        send_log("Unknown command: " + String(received));
        break;
    }
}

void setup() 
{
    Serial.begin(115200);
    transfer.begin(Serial);

    send_log("Setup begin.");

    delay(500);

    robot.begin();

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    // Motor speed setup
    m1->setSpeed(255);
    m2->setSpeed(255);
    m3->setSpeed(255);
    m4->setSpeed(255);

    // Stepper motor
    stepper.setMaxSpeed(750);
    stepper.setAcceleration(150);

    // Attached to Servo1 = pin 10
    servo.attach(10);
    servo.write(90);

    send_log("Setup complete.");
}

void loop() 
{
    bool command_processed_this_loop = false;

    char receive_buffer[8];
    if (transfer.available())
    {
        transfer.rxObj(receive_buffer);
        handle_rc_command(receive_buffer[0]);
        command_processed_this_loop = true;
    }
    

    if (!command_processed_this_loop)
    {
        // No commands = stop moving
        stop_motors();
        //stepper.moveTo(0);
    }

    stepper.setSpeed(500);
    stepper.move(200);
    stepper.run();

    // Back and forth servo rotation
    servo_pos = constrain(servo_going_cw ? servo_pos + delta_servo_pos : servo_pos - delta_servo_pos, 0, 180);

    if (servo_pos == 180)
    {
        servo_going_cw = false;
    }
    else if (servo_pos == 0)
    {
        servo_going_cw = true;
    }

    servo.write(servo_pos);
}