#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <thread>
#include <chrono>
#include <cstring>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

extern "C"
{
    #include <i2c/smbus.h>
    #include <linux/i2c-dev.h>
    #include <linux/i2c.h>
}

const std::string i2c_path = "/dev/i2c-1";
constexpr uint8_t lcd_address = 0x27;

// LCD Commands
constexpr uint8_t LCD_CLEARDISPLAY = 0x01;
constexpr uint8_t LCD_RETURNHOME = 0x02;
constexpr uint8_t LCD_ENTRYMODESET = 0x04;
constexpr uint8_t LCD_DISPLAYCONTROL = 0x08;
constexpr uint8_t LCD_FUNCTIONSET = 0x20;
constexpr uint8_t LCD_SETDDRAMADDR = 0x80;

// Flags for display entry mode
constexpr uint8_t LCD_ENTRYLEFT = 0x02;
constexpr uint8_t LCD_ENTRYSHIFTDECREMENT = 0x00;

// Flags for display on/off control
constexpr uint8_t LCD_DISPLAYON = 0x04;
constexpr uint8_t LCD_DISPLAYOFF = 0x00;
constexpr uint8_t LCD_CURSOROFF = 0x00;
constexpr uint8_t LCD_BLINKOFF = 0x00;

// Flags for function set
constexpr uint8_t LCD_4BITMODE = 0x00;
constexpr uint8_t LCD_2LINE = 0x08;
constexpr uint8_t LCD_5x8DOTS = 0x00;

// Flags for backlight control
constexpr uint8_t LCD_BACKLIGHT = 0x08;
constexpr uint8_t LCD_NOBACKLIGHT = 0x00;

constexpr uint8_t en_bit = 0b00000100;  // Enable bit
constexpr uint8_t reg_bit = 0b00000001;  // Register select bit

class i2c_lcd
{
private:
    // Write a byte
    bool write_byte(uint8_t data) const
    {
        __s32 result = i2c_smbus_write_byte(bus_fd, data);
        return result >= 0;
    }

    // Send data to LCD with enable pulse
    bool lcd_strobe(uint8_t data)
    {
        // Enable high
        if (!write_byte(data | en_bit | backlight_state))
        {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));

        // Enable low
        if (!write_byte((data & ~en_bit) | backlight_state))
        {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));

        return true;
    }

    // Write 4 bits to LCD
    bool lcd_write_four_bits(uint8_t data)
    {
        if (!write_byte(data | backlight_state))
        {
            return false;
        }

        return lcd_strobe(data);
    }

    // Send a command or data to LCD
    bool lcd_write(uint8_t cmd, uint8_t mode = 0)
    {
        // Write high 4 bits
        if (!lcd_write_four_bits(mode | (cmd & 0xF0)))
        {
            return false;
        }

        // Write low 4 bits
        if (!lcd_write_four_bits(mode | ((cmd << 4) & 0xF0)))
        {
            return false;
        }

        return true;
    }

public:
    int bus_fd;
    uint8_t lcd_address;
    uint8_t backlight_state;
    bool is_connected;

    i2c_lcd(const std::string& i2c_path, uint8_t address):
        lcd_address(address),
        backlight_state(LCD_BACKLIGHT),
        is_connected(false)
    {
        bus_fd = open(i2c_path.c_str(), O_RDWR);

        if (bus_fd < 0)
        {
            std::cerr << "Failed to open I2C bus: " << i2c_path << std::endl;
        }
    }

    ~i2c_lcd()
    {
        if (bus_fd >= 0)
        {
            close(bus_fd);
        }
    }

    // Check if LCD is connected and responding
    bool test_connection() const
    {
        if (bus_fd < 0)
        {
            return false;
        }

        // Set I2C slave address
        if (ioctl(bus_fd, I2C_SLAVE, lcd_address) < 0)
        {
            return false;
        }

        // Write a byte to test connection
        __s32 result = i2c_smbus_write_byte(bus_fd, 0x00);
        return result >= 0;
    }

    // Initialize the LCD
    bool initialize()
    {
        if (!test_connection())
        {
            is_connected = false;
            return false;
        }

        try
        {
            if (ioctl(bus_fd, I2C_SLAVE, lcd_address) < 0)
            {
                return false;
            }

            // LCD initialization sequence (HD44780 standard)
            // https://web.alfredstate.edu/faculty/weimandn/lcd/lcd_initialization/lcd_initialization_index.html
            
            // Step 1
            std::this_thread::sleep_for(std::chrono::milliseconds(150));

            // Steps 2-5
            lcd_write_four_bits(0x03 << 4);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            lcd_write_four_bits(0x03 << 4);
            std::this_thread::sleep_for(std::chrono::microseconds(200));

            lcd_write_four_bits(0x03 << 4);
            std::this_thread::sleep_for(std::chrono::microseconds(200));

            lcd_write_four_bits(0x02 << 4);  // Set to 4-bit mode
            std::this_thread::sleep_for(std::chrono::microseconds(200));

            // Step 6: Function set: 4-bit mode, 2 lines, 5x8 dots
            lcd_write(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            // Step 7
            lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF);
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            // Step 8
            clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            // Step 9
            lcd_write(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            // Turn display on and we're done
            lcd_write(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            is_connected = true;
            return true;
        }
        catch (...)
        {
            is_connected = false;
            return false;
        }
    }

    // Clear the display
    bool clear()
    {
        if (!is_connected)
        {
            return false;
        }

        if (!lcd_write(LCD_CLEARDISPLAY))
        {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return true;
    }

    // Set cursor to home position
    bool home()
    {
        if (!is_connected)
        {
            return false;
        }

        if (!lcd_write(LCD_RETURNHOME))
        {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return true;
    }

    // Set cursor position (row: 0-1, col: 0-15 for 16x2 LCD)
    bool set_cursor(uint8_t row, uint8_t col)
    {
        if (!is_connected)
        {
            return false;
        }

        const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

        if (row >= 2)
        {
            row = 1;
        }

        return lcd_write(LCD_SETDDRAMADDR | (col + row_offsets[row]));
    }

    // Print a string to the LCD
    bool print(const std::string& text)
    {
        if (!is_connected)
        {
            return false;
        }

        for (char c : text)
        {
            if (!lcd_write(c, reg_bit))
            {
                return false;
            }
        }

        return true;
    }

    // Turn backlight on/off
    void set_backlight(bool on)
    {
        backlight_state = on ? LCD_BACKLIGHT : LCD_NOBACKLIGHT;
        if (is_connected)
        {
            write_byte(backlight_state);
        }
    }
};

static std::string get_ip_address()
{
    std::array<char, 128> buffer = { 0 };
    std::string result;

    FILE* pipe_file = popen("hostname -I", "r");
    if (!pipe_file)
    {
        return "";
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe_file) != nullptr)
    {
        result += buffer.data();
    }

    pclose(pipe_file);

    // Remove trailing whitespace/newline
    while (!result.empty() && (result.back() == '\n' || result.back() == ' '))
    {
        result.pop_back();
    }

    return result;
}

static bool is_valid_ip_address(const std::string& ip)
{
    return !ip.empty() && ip != "127.0.0.1" && ip != "255.255.255.255";
}

int main()
{
    i2c_lcd lcd(i2c_path, lcd_address);

    std::string current_ip;
    std::string displayed_ip;
    bool lcd_was_connected = false;

    while (true)
    {
        current_ip = get_ip_address();

        bool is_connected = lcd.test_connection();
        if (is_connected && !lcd_was_connected)
        {
            // Just plugged in
            if (lcd.initialize())
            {
                lcd_was_connected = true;
                displayed_ip = "";  // Force redisplay since screen was cleared
            }
            else
            {
                is_connected = false;
            }
        }
        else if (!is_connected && lcd_was_connected)
        {
            // Just disconnected
            lcd_was_connected = false;
            displayed_ip = "";
        }

        if (is_connected && is_valid_ip_address(current_ip) && current_ip != displayed_ip)
        {
            // Clear display and show IP
            if (lcd.clear())
            {
                lcd.set_cursor(0, 0);
                lcd.print("IP Address");
                lcd.set_cursor(1, 0);

                if (lcd.print(current_ip))
                {
                    displayed_ip = current_ip;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

	return 0;
}