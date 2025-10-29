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

unsigned long last_imu_read_time = 0;

void send_imu_data(float a_x, float a_y, float a_z, float g_x, float g_y, float g_z)
{
    unsigned long current_time = millis();
    unsigned long delta_time = current_time - last_imu_read_time;

    last_imu_read_time = current_time;

    // [payload size in bytes] [message type] [delta time] [acceleration vector3] [angular velocity vector3]
    size_t packet_size = sizeof(size_t) + sizeof(unsigned char) + sizeof(unsigned long) + 6 * sizeof(float);
    unsigned char buffer[packet_size];
    unsigned char* cursor = buffer;

    size_t payload_size = packet_size - sizeof(size_t);
    memcpy(cursor, &payload_size, sizeof(size_t));
    cursor += sizeof(size_t);

    // Write 0x01 so the Pi knows that this is IMU data as opposed to microphone analog input
    memcpy(cursor, &imu_data_header, sizeof(unsigned char));
    cursor += sizeof(unsigned char);

    // Write delta time for position and velocity estimation
    memcpy(cursor, &delta_time, sizeof(unsigned long));
    cursor += sizeof(unsigned long);

    // Write accleration values
    memcpy(cursor, &a_x, sizeof(float));
    cursor += sizeof(float);

    memcpy(cursor, &a_y, sizeof(float));
    cursor += sizeof(float);

    memcpy(cursor, &a_z, sizeof(float));
    cursor += sizeof(float);

    // Finally, write angular velocity values
    memcpy(cursor, &g_x, sizeof(float));
    cursor += sizeof(float);

    memcpy(cursor, &g_y, sizeof(float));
    cursor += sizeof(float);

    memcpy(cursor, &g_z, sizeof(float));
    cursor += sizeof(float);

    Serial.write(buffer, sizeof(buffer));
}

void setup() 
{
    Serial.begin(9600);

    driver = motor_driver(5, 7, 6, 3, 1, 2);

    delay(500);
}

void loop() 
{
    //driver.set_direction(left | right, forward);
    //driver.set_speed(left | right, 255);

    float a_x, a_y, a_z;
    float g_x, g_y, g_z; 
    // TODO: Read IMU sensor data into above variables (however it's done)

    send_imu_data(a_x, a_y, a_z, g_x, g_y, g_z);

    // Read commands from Raspberry Pi
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        Serial.print("RECEIVED: ");
        Serial.println(command);

        if (command.equals("W"))
        {
            driver.set_direction(left | right, forward);
           
        }
        else if (command.equals("A"))
        {
            driver.set_direction(left, forward);
        }
    }

    delay(update_rate_ms);
}