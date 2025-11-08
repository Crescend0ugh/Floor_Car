//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SerialTransfer.h>

#include "src/write_buffer.h"
#include "src/motor_driver.h"

#include <Wire.h>

motor_driver driver;
Servo scooper_servo;
SerialTransfer transfer;

const unsigned long update_rate_ms = 16;
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

void send_microphone_data()
{

}

// A makeshift Serial.println. Outputs to the Raspberry Pi terminal.
void send_log(String string)
{
    write_buffer writer(log_string_header);
    char string_buffer[string.length() + 1];
    string.toCharArray(string_buffer, sizeof(string_buffer));
    writer.write(string_buffer, sizeof(string_buffer));

    transfer.sendDatum(writer.buffer);
}

void handle_rc_command(rc_command command)
{
    switch (command)
    {
    case (rc_command::stop):
    {
        send_log("STOP");
        driver.stop();
        break;
    }
    case (rc_command::w):
    {
        send_log("W");
        driver.set_direction(left | right, forward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::s):
    {
        send_log("S");
        driver.set_direction(left | right, backward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::a):
    {
        send_log("A");
        driver.set_left_direction(backward);
        driver.set_right_direction(forward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::d):
    {
        send_log("D");
        driver.set_left_direction(forward);
        driver.set_right_direction(backward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::pick_up):
    {
        send_log("PICK UP");
        driver.stop();
        // TODO
        break;
    }
    case (rc_command::servo_ccw):
    {
        send_log("CCW");
        scooper_servo.write(180);
        break;
    }
     case (rc_command::servo_cw):
    {0
        send_log("CW");
        scooper_servo.write(0);
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
    Serial.begin(9600);
    transfer.begin(Serial);
    send_log("Starting...");
    
    Wire.begin();

    driver = motor_driver(2, 3, 4, 7, 5, 6);
    driver.stop();

    scooper_servo.attach(9);
    scooper_servo.write(90);

    delay(1000);
    send_log("Arduino setup complete.");
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
        driver.stop();
        scooper_servo.write(90);
    }

    delay(50);
}