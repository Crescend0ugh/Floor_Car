#include <iostream>
#include <chrono>

#include "network.h"

struct some_data 
{
	double x{};
	double y{};
	double z{};
	std::string name;
};

struct other_data
{
	std::vector<float> floats;
};

enum protocol
{
	one = 0,
	two,
	max_protocols
};

int main(int argc, char* argv[]) { 
	if (argc > 1) {
		std::cout << "client" << std::endl;

		std::string ip = "127.0.0.1";

		asio::io_context io_context;
		network::client client(io_context, ip, 12345);
		std::thread thread([&io_context] {
			io_context.run();
		});

		while (1) {
			network::received_data data;

			while (client.poll(data)) {

				switch (data.protocol_id)
				{

				case one: 
				{
					some_data s;
					network::deserialize(s, data);
					std::cout << std::format("received protocol::one! some_data [ x: {}, y: {}, z: {}, name: {} ]\n", s.x, s.y, s.z, s.name) << std::endl;
					break;
				}

				case two:
				{
					other_data o;
					network::deserialize(o, data);
					std::cout << std::format("received protocol::two! other_data [ floats is size {} ]\n", o.floats.size()) << std::endl;
					break;
				}

				default:
					break;
				}

			}

			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
	}
	else {
		std::cout << "server" << std::endl;

		asio::io_context io_context;
		network::server server(io_context, 12345);
		std::thread thread([&io_context] {
			io_context.run();
		});

		int which_protocol = 0;
		while (1) {
			
			switch (which_protocol)
			{
			case one: 
			{
				some_data s { 6.7, 5.555, -4243, "Jim" };
				std::cout << std::format("sending protocol::one : some_data [ x: {}, y: {}, z: {}, name: {} ]\n", s.x, s.y, s.z, s.name) << std::endl;
				server.send(protocol::one, s);
				break;
			}
			case two:
			{
				other_data o{ std::vector<float> { 5.0f, 6.0f } };
				std::cout << std::format("sending protocol::two : other_data [ floats is size {} ]\n", o.floats.size()) << std::endl;
				server.send(protocol::two, o);
				break;
			}
			default:
				break;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			which_protocol = (which_protocol + 1) % protocol::max_protocols;
		}
	}

	return 0;
}
