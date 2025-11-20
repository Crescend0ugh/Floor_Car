#include "ldlidar_driver/lidar_driver.h"
int main()
{
    robo::ld19::lidar_data_interface ld19("");
    ld19.open();
    while (true)
    {
        ld19.poll();
    }
    ld19.close();

    // generate commands
    // send commands



}
