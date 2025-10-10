#include <iostream>

#include "network.h"

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
			std::string message;
			while (client.poll(message)) {
				std::cout << message;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}
	else {
		std::cout << "server" << std::endl;

		asio::io_context io_context;
		network::server server(io_context, 12345);
		std::thread thread([&io_context] {
			io_context.run();
		});

		while (1) {
			std::string data = "hello client!\n";
			server.send(data);

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
	}

	return 0;
}
