#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <thread>
#include <chrono>

#include <gpiod.h>

#define i2c_path "/dev/i2c-1"
#define lcd_addr "0x27"

std::string exec(const char* cmd) 
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) 
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }

    if (!result.empty() && result.back() == '\n') 
    {
        result.pop_back();
    }

    return result;
}

int main()
{
    std::string ip_address;
    while (ip_address.empty() || ip_address == "127.0.0.1")
    {
        ip_address = exec("hostname -I");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    

	return 0;
}