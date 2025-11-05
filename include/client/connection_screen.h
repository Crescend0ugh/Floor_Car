#if defined(_WIN32)
#define NOGDI             // All GDI defines and routines
#define NOUSER            // All USER defines and routines
#endif

#pragma once

#include "network.h"

#include <raylib.h>

#include <string>
#include <regex>
#include <optional>

namespace ui
{
	class connection_screen
	{
	private:
		network::client& client;

		Rectangle text_box;
		bool is_mouse_on_text = false;
		int frame_counter = 0;
		bool input_entered = false;
		std::string input;
		std::string submitted_input;

		bool is_input_valid() const;

	public:
		connection_screen(network::client& client);
		void draw();
		void clear_input();
		std::optional<std::string> get_submitted_input();
		void reset();
	};
}
