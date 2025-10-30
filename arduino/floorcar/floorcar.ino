enum direction
{
    backward,
    forward
};

enum side
{
    left = 1 << 0,
    right = 1 << 1
};

struct motor_driver
{
    motor_driver() : en_a(-1), in_1(-1), in_2(-1), en_b(-1), in_3(-1), in_4(-1) {}
    motor_driver(int en_a, int in_1, int in_2, int en_b, int in_3, int in_4) :
            en_a(en_a), in_1(in_1), in_2(in_2), en_b(en_b), in_3(in_3), in_4(in_4)
    {
        pinMode(en_a, OUTPUT);
        pinMode(in_1, OUTPUT);
        pinMode(in_2, OUTPUT);
        pinMode(en_b, OUTPUT);
        pinMode(in_3, OUTPUT);
        pinMode(in_4, OUTPUT);

    }
    ;
    void set_speed(side s, unsigned char val)
    {
        if(s & left)
        {
            analogWrite(en_a, val);
        }
        if(s & right)
        {
            analogWrite(en_b, val);
        }
    }
    void set_direction(side s, direction dir)
    {
        if(s & left)
        {
            set_left_direction(dir);
        }
        if(s & right)
        {
            set_right_direction(dir);
        }
    }
    void set_left_direction(direction dir)
    {
        if(dir & forward)
        {
            digitalWrite(in_1, HIGH);
            digitalWrite(in_2, LOW);
        }
        if(dir & backward)
        {
            digitalWrite(in_1, LOW);
            digitalWrite(in_2, HIGH);
        }
    };
    void set_right_direction(direction dir)
    {
        if(dir & forward)
        {
            digitalWrite(in_3, HIGH);
            digitalWrite(in_4, LOW);
        }
        if(dir & backward)
        {
            digitalWrite(in_3, LOW);
            digitalWrite(in_4, HIGH);
        }
    };

    int en_a;
    int in_1;
    int in_2;
    direction left_dir;
    direction right_dir;
    unsigned char left_speed;
    unsigned char right_speed;
    int en_b;
    int in_3;
    int in_4;
};

motor_driver driver;

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
    // [payload size in bytes] [message type] [position vector3] [rotation vector3]
    unsigned short packet_size = sizeof(unsigned short) + sizeof(unsigned char) + 6 * sizeof(float);
    unsigned char buffer[packet_size];
    unsigned char* cursor = buffer;

    unsigned short payload_size = packet_size - sizeof(unsigned short);
    memcpy(cursor, &payload_size, sizeof(unsigned short));
    cursor += sizeof(unsigned short);

    // Write header
    memcpy(cursor, &imu_data_header, sizeof(unsigned char));
    cursor += sizeof(unsigned char);

    // Position vector
    memcpy(cursor, position, sizeof(position));
    cursor += sizeof(position);

    // Rotation vector
    memcpy(cursor, rotation, sizeof(rotation));
    cursor += sizeof(rotation);

    if (Serial.availableForWrite() > 0)
    {
        Serial.write(buffer, sizeof(buffer));
    }
}

void send_microphone_data()
{

}

// A makeshift Serial.println. Outputs to the Raspberry Pi terminal.
void log(String string)
{
    unsigned short packet_size = sizeof(unsigned short) + sizeof(unsigned char) + string.length() + 1;
    unsigned char buffer[packet_size];
    unsigned char* cursor = buffer;

    unsigned short payload_size = packet_size - sizeof(unsigned short);
    memcpy(cursor, &payload_size, sizeof(unsigned short));
    cursor += sizeof(unsigned short);

    memcpy(cursor, &log_string_header, sizeof(unsigned char));
    cursor += sizeof(unsigned char);

    string.toCharArray(cursor, packet_size);

    if (Serial.availableForWrite() > 0)
    {
        Serial.write(buffer, sizeof(buffer));
    }
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

void setup() 
{
    Serial.begin(115200);
    while (!Serial) {}

    driver = motor_driver(5, 7, 6, 3, 1, 2);

    randomSeed(1001);

    delay(500);
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

    // TODO: Something special needs to be do for the orientation

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
            log("Unknown command.");
            break;
        }   
        }

        clear_serial_buffer();
    }

    delay(update_rate_ms);
}