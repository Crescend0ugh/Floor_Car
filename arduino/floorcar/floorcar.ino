#include "src/write_buffer.h"
#include "src/motor_driver.h"

#include <MPU9250.h>
#include <Wire.h>

motor_driver driver;
// MPU9250 imu(Wire, 0x68);

const unsigned long update_rate_ms = 16;

const unsigned char imu_data_header = 0x01;
const unsigned char microphone_input_header = 0x02;
const unsigned char log_string_header = 0x03;

// Commands and their IDs
enum command
{
    move_distance = 0,
    move_for_seconds,
    rotate_to_heading,
    rotate_by,
    rotate_for_seconds
};

const unsigned char rc_header_byte = 0xFF;
enum rc_command
{
    none = 0,
    stop,
    w,
    s,
    a,
    d,
    pick_up
};

// Sometimes, the serial has 3 bytes available when we expect 4, for whatever reason, so this is a failsafe
bool had_read_error = false;

float position[3];
float velocity[3];
float acceleration[3];

float angular_velocity[3];
float rotation[3];

void euler_method(float* x, float* dx, float delta_time)
{
    x[0] += dx[0] * delta_time;
    x[1] += dx[1] * delta_time;
    x[2] += dx[2] * delta_time;
}

void send_imu_data()
{
    write_buffer writer(imu_data_header);
    writer.write(position, sizeof(position));
    writer.write(rotation, sizeof(rotation));
    writer.transmit();
}

void send_microphone_data()
{

}

// A makeshift Serial.println. Outputs to the Raspberry Pi terminal.
void send_log(String string)
{
    write_buffer writer(log_string_header);
    char string_buffer[string.length() + 1];
    string.toCharArray(string_buffer, string.length() + 1);
    writer.write(string_buffer, sizeof(string_buffer));
    writer.transmit();
}

float read_float()
{
    if (Serial.available() < 4)
    {
        had_read_error = true;
        return 0.0f / 0.0f;
    }

    union float_bytes
    {
        float f;
        unsigned char bytes[4];
    };

    float_bytes data;
    for (int i = 0; i < 4; i++) 
    {
        data.bytes[i] = Serial.read();
    }

    return data.f;
}

void clear_serial_buffer()
{
    while (Serial.available() > 0) 
    {
        Serial.read();
    }
}

void handle_rc_command()
{
    switch (Serial.read())
    {
    case (rc_command::stop):
    {
        driver.stop();
        break;
    }
    case (rc_command::w):
    {
        driver.set_direction(left | right, forward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::s):
    {
        driver.set_direction(left | right, backward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::a):
    {
        driver.set_left_direction(backward);
        driver.set_right_direction(forward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::d):
    {
        driver.set_left_direction(forward);
        driver.set_right_direction(backward);
        driver.set_speed(left | right, 255);
        break;
    }
    case (rc_command::pick_up):
    {
        driver.stop();
        // TODO
        break;
    }
    default:
        break;
    }
}

void setup() 
{
    Serial.begin(115200);
    while (!Serial) {}

    Wire.begin();

    driver = motor_driver(5, 7, 6, 3, 1, 2);
    driver.stop();

    randomSeed(1001);

    /*
    while (imu.begin() != INV_SUCCESS) 
    {
        log("Failed to initialize MPU9250");
        delay(1000);
    }
    */

    delay(1000);
}

void loop() 
{
    //driver.set_direction(left | right, forward);
    //driver.set_speed(left | right, 255);

    // Or do millis() - previous_loop_time
    float delta_time = update_rate_ms / 1000.0f;

    acceleration[0] = random(-1000, 1000) / 13056.0;
    acceleration[1] = random(-1000, 1000) / 1660.0;
    acceleration[2] = random(-1000, 1000) / 232.0;
    angular_velocity[0] = random(-1000, 1000) / 1400.0;
    angular_velocity[1] = random(-1000, 1000) / 43.0;
    angular_velocity[2] = random(-1000, 1000) / 54.0;
    // TODO: Read IMU sensor data into above data (however it's done)

    euler_method(velocity, acceleration, delta_time);
    euler_method(position, velocity, delta_time);
    euler_method(rotation, angular_velocity, delta_time);

    send_imu_data();

    // Read commands from Raspberry Pi
    had_read_error = false;

    if (Serial.available() > 0)
    {
        int command_id = Serial.read();

        switch (command_id)
        {
        case (rc_header_byte):
        {
            handle_rc_command();
            break;
        }
        case (move_distance):
        {
            float distance = read_float();
            break;
        }
        case (move_for_seconds):
        {
            float seconds = read_float();

            if (!had_read_error)
            {
                // log(String(seconds, 2));
            }

            break;    
        }
        case (rotate_to_heading):
        {
            float heading = read_float();
            break;
        }
        case (rotate_by):
        {
            float delta_heading = read_float();
            float progress = read_float();
            break;
        }
        case (rotate_for_seconds):
        {
            float seconds = read_float();
            break;
        }
        default:
        {
            // Occassionally, we hit this for no reason.
            send_log("Unknown command.");
            break;
        }   
        }

        clear_serial_buffer();
    }
    else
    {
        // No commands = stop moving
        driver.stop();
    }

    delay(update_rate_ms);
}