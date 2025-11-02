#include "client/connection_screen.h"

const int input_max_chars = 15;
std::regex ip_address_regex(
    "^((25[0-5]|2[0-4][0-9]|1[0-9]{2}|[1-9]?[0-9])\\.){3}"
    "(25[0-5]|2[0-4][0-9]|1[0-9]{2}|[1-9]?[0-9])$"
);

const int font_size = 40;

ui::connection_screen::connection_screen():
    text_box(Rectangle())
{
}

bool ui::connection_screen::is_input_valid() const
{
    return std::regex_match(input, ip_address_regex);
}

void ui::connection_screen::draw()
{
    input_entered = (IsKeyPressed(KEY_ENTER) && is_input_valid());
	text_box = Rectangle { GetScreenWidth() / 2.0f - 250, GetScreenHeight() / 2.0f, 500, 75 };

	is_mouse_on_text = (CheckCollisionPointRec(GetMousePosition(), text_box));

    if (input_entered)
    {
        submitted_input = input;
        input.clear();
        frame_counter = 0;
    }
    else
    {
        frame_counter++;

        // Set the window's cursor to the I-Beam
        SetMouseCursor(MOUSE_CURSOR_IBEAM);
        int key = GetCharPressed();

        // Check if more characters have been pressed on the same frame
        while (key > 0)
        {
            // Only allow keys in range [32..125]
            if ((key >= 32) && (key <= 125) && (input.size() < input_max_chars))
            {
                input.push_back(key);
            }

            key = GetCharPressed();  // Check next character in the queue
        }

        if (IsKeyPressed(KEY_BACKSPACE) && input.size() > 0)
        {
            input.pop_back();
        }
    }
    
    // Draw the boxes

    DrawRectangleRec(text_box, LIGHTGRAY);
    if (is_mouse_on_text)
    {
        DrawRectangleLines((int)text_box.x, (int)text_box.y, (int)text_box.width, (int)text_box.height, RED);
    }
    else
    {
        DrawRectangleLines((int)text_box.x, (int)text_box.y, (int)text_box.width, (int)text_box.height, DARKGRAY);
    }

    DrawText(input.c_str(), (int)text_box.x + 5, (int)text_box.y + 8, font_size, MAROON);

    if (input.size() < input_max_chars)
    {
        // Draw blinking underscore char
        if (((frame_counter / 20) % 2) == 0)
        {
            DrawText("_", (int)text_box.x + 8 + MeasureText(input.c_str(), font_size), (int)text_box.y + 12, font_size, MAROON);
        }
    }
        
    if (input.size() > 0 && !is_input_valid())
    {
        DrawText(
            "Invalid IP address",
            text_box.x,
            text_box.y + text_box.height + 20,
            20, MAROON
        );
    }
    
    DrawText(
        "Enter IP address:",
        text_box.x,
        text_box.y - 20,
        20, GRAY
    );
}

std::optional<std::string> ui::connection_screen::get_submitted_input()
{
    if (submitted_input.size() == 0)
    {
        return std::nullopt;
    }

    return submitted_input;
}

void ui::connection_screen::reset()
{
    input.clear();
    submitted_input.clear();
    frame_counter = 0;
}