#pragma once

#include <raylib.h>

#include <string>
#include <regex>
#include <optional>

namespace ui
{
	class connection_screen
	{
	private:
		Rectangle text_box;
		bool is_mouse_on_text = false;
		int frame_counter = 0;
		bool input_entered = false;
		std::string input;
		std::string submitted_input;

		bool is_input_valid() const;

	public:
		connection_screen();
		void draw();
		void clear_input();
		std::optional<std::string> get_submitted_input();
		void reset();
	};
}
