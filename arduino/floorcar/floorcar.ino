 @ -0,0 +1,100 @@
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
            pinMode(in_1, HIGH);
            pinMode(in_2, LOW);
        }
        if(dir & backward)
        {
            pinMode(in_1, LOW);
            pinMode(in_2, HIGH);
        }
    };
    void set_right_direction(direction dir)
    {
        if(dir & forward)
        {
            pinMode(in_3, HIGH);
            pinMode(in_4, LOW);
        }
        if(dir & backward)
        {
            pinMode(in_3, LOW);
            pinMode(in_4, HIGH);
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
void setup() {
    driver = motor_driver(5, 4, 3, 0, 2, 1);

    // put your setup code here, to run once:
    delay(500);
}

void loop() {
    driver.set_direction(left | right, forward);
    driver.set_speed(left | right, 255);
}