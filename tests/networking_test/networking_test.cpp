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

		auto current_time = std::chrono::high_resolution_clock::now();
		auto next_send_time = current_time + std::chrono::seconds(1);

		while (1) {
			current_time = std::chrono::high_resolution_clock::now();

			if (current_time >= next_send_time) {
				double random_number = (double)rand();
				std::cout << "\nclient send: the random number is: " << random_number << std::endl;
				client.send(one, some_data{ random_number, random_number * 2, random_number * 3, "I'M THE CLIENT!" });
				next_send_time += std::chrono::seconds(1);
			}

			network::received_data data;

			while (client.poll(data)) {
				switch (data.protocol_id)
				{

				case one:
				{
					some_data s;
					network::deserialize(s, data);
					std::cout << std::format("[client received one] some_data [ x: {}, y: {}, z: {}, name: {} ]\n", s.x, s.y, s.z, s.name) << std::endl;
					break;
				}

				case two:
				{
					other_data o;
					network::deserialize(o, data);
					std::cout << std::format("[client received two] other_data [ floats is size {} ]\n", o.floats.size()) << std::endl;
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
			// SEND

			switch (which_protocol)
			{
			case one: 
			{
				some_data s { 6.7, 5.555, -4243, "Jim" };
				std::cout << std::format("[server sending one] some_data [ x: {}, y: {}, z: {}, name: {} ]\n", s.x, s.y, s.z, s.name) << std::endl;
				server.send(protocol::one, s);
				break;
			}
			case two:
			{
				other_data o{ std::vector<float> { 5.0f, 6.0f } };
				std::cout << std::format("[server sending two] other_data [ floats is size {} ]\n", o.floats.size()) << std::endl;
				server.send(protocol::two, o);
				break;
			}
			default:
				break;
			}

			// READ
			network::received_data data;
			while (server.poll(data))
			{
				switch (data.protocol_id)
				{

				case one:
				{
					some_data s;
					network::deserialize(s, data);
					std::cout << std::format("[server received one] some_data [ x: {}, y: {}, z: {}, name: {} ]", s.x, s.y, s.z, s.name) << std::endl;
					break;
				}

				case two:
				{
					other_data o;
					network::deserialize(o, data);
					std::cout << std::format("[server received two] other_data [ floats is size {} ]", o.floats.size()) << std::endl;
					break;
				}

				default:
					break;
				}
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(500));

			which_protocol = (which_protocol + 1) % protocol::max_protocols;
		}
	}

	return 0;
}
