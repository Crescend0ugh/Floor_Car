#ifdef __linux__

#include "lidar.h"

static uint64_t get_system_timestamp()
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());

    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());

    return ((uint64_t)tmp.count());
}

lidar::lidar()
{
    node.RegisterGetTimestampFunctional(std::bind(&get_system_timestamp));
    node.EnableFilterAlgorithnmProcess(true);
}

bool lidar::initialize_communication()
{
    // node.Start(lidar_type, ip, port_str, ldlidar::COMM_TCP_CLIENT_MODE);
    if (node.Start(lidar_type, "/dev/ttyUSB0", 230400, ldlidar::COMM_SERIAL_MODE))
    {
        LDS_LOG_INFO("LiDar node initialization succeeded", "");
        power_on();
    }
    else
    {
        LD_LOG_ERROR("LiDar node initialization failed!", "");
        power_off();
        return false;
    }

    if (node.WaitLidarCommConnect(3500))
    {
        LDS_LOG_INFO("LiDar communication is normal.", "");
    }
    else
    {
        LDS_LOG_ERROR("LiDar communication is abnormal.", "");
        node.Stop();
        return false;
    }

    return true;
}

void lidar::power_on()
{

}

void lidar::power_off()
{

}

void lidar::get_2d_points()
{
    if (!ldlidar::LDLidarDriver::IsOk())
    {
        LD_LOG_ERROR("Something went wrong with the LiDar node", "");
        return;
    }

    ldlidar::Points2D laser_scan_points;

    switch (node.GetLaserScanData(laser_scan_points, 1000))
    {
    case (ldlidar::LidarStatus::NORMAL):
    {
        double lidar_scan_freq = 0;
        node.GetLidarScanFreq(lidar_scan_freq);

        LDS_LOG_INFO("speed(Hz):%f,size:%d,stamp_front:%lu, stamp_back:%lu",
            lidar_scan_freq, laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);

        if (laser_scan_points.front().stamp >= laser_scan_points.back().stamp)
        {
            LDS_LOG_ERROR("LiDar ran into a timestamp error! The first laser scan point had an earlier timestamp than the last.", "");
            node.Stop();
        }

        break;
    }
    case (ldlidar::LidarStatus::DATA_TIME_OUT):
    {
        LDS_LOG_ERROR("LiDar timed out.", "");
        node.Stop();
        break;
    }
    case (ldlidar::LidarStatus::DATA_WAIT):
    {
        break;
    }
    default:
    {
        break;
    }
    }
}

#endif