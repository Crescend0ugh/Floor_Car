/*
	network.h

	Implements two-way asynchronous TCP socket communication between a server and a client.
	Message types and protocols are defined in network_data.h.
	To send a message, the message's corresponding message type (defined in robo::network::protocol) must be passed as the first argument.
	 
	This allows the FloorCar server to send positional, visual and environmental data to connected clients 
	for visualization and more seamless debugging.

	In early stages of prototyping, the server also accepts client messages to enable more control over various components.
*/

#pragma once

#include <exception>
#include <memory>
#include <list>

#include "asio.hpp"
#include "zpp_bits.h"

using asio::ip::tcp;

namespace network
{
	using bytes = std::vector<std::byte>;

	struct received_data
	{
		int protocol_id;
		bytes serialized_data;
	};

	class io : public std::enable_shared_from_this<io>
	{
	protected:
		tcp::socket socket;
		asio::streambuf buffer;
		std::list<bytes> send_queue;
		std::list<bytes> read_queue;

	public:
		io(tcp::socket socket):
			socket(std::move(socket))
		{}

		bool enqueue(bytes data, bool at_front)
		{
			at_front &= !send_queue.empty(); // no difference
			if (at_front)
				send_queue.insert(std::next(std::begin(send_queue)), std::move(data));
			else
				send_queue.push_back(std::move(data));

			return send_queue.size() == 1;
		}

		bool dequeue()
		{
			assert(!send_queue.empty());
			send_queue.pop_front();
			return !send_queue.empty();
		}

		void send(bytes data, bool at_front = false)
		{
			post(socket.get_executor(), 
				[=, this] 
				{
					if (enqueue(std::move(data), at_front))
					{
						write_loop();
					}
				}
			);
		}

		bool pop_read_queue(received_data& data)
		{
			if (read_queue.empty()) 
			{
				return false;
			}

			bytes received(read_queue.front()); // I give up. Just copy the vector.
			data.serialized_data = received;
			data.protocol_id = static_cast<int>(data.serialized_data[0]);
			read_queue.pop_front();

			return true;
		}

		virtual void write_loop() = 0;

		virtual void read_loop() = 0;
	};

	class connection : public io
	{
	private:
		void write_loop() override;
		void read_loop() override;

	public:
		using io::io;
		void start();

		void close() 
		{
			asio::error_code ec;
			socket.shutdown(asio::ip::tcp::socket::shutdown_both, ec);
			socket.close(ec);
		}
	};

	class server
	{
	private:
		asio::io_context io_context;
		tcp::acceptor acceptor;

		using connection_ptr = std::shared_ptr<connection>;
		using weak_connection_ptr = std::weak_ptr<connection>;

		std::mutex mutex;
		std::vector<weak_connection_ptr> registered_connections;

		size_t register_connection(weak_connection_ptr ptr);

		template <typename F>
		size_t for_each_active(F f);

		void accept();
		size_t send_bytes(bytes data);

	public:
		server(asio::io_context& io_context, short port);

		bool poll(received_data& data);
		void shutdown();

		size_t get_client_count()
		{
			return registered_connections.size();
		}

		size_t send(int protocol, auto data)
		{
			bytes serialized;
			zpp::bits::out out(serialized);

			size_t size_header = 0;
			out(size_header).or_throw();

			out(protocol).or_throw();
			out(data).or_throw();

			// Go back and write the actual size
			out.reset();
			out(serialized.size() - sizeof(size_header)).or_throw();

			return send_bytes(serialized);
		}
	};

	class client : public io
	{
	private:
		tcp::resolver resolver;
		std::string port;

		void resolve_loop(std::string& ip_address, const std::string& port);
		void connect_loop(const tcp::resolver::results_type& endpoints);
		void write_loop() override;
		void read_loop() override;

		void send_bytes(bytes data);

	public:
		asio::steady_timer retry_timer;
		bool is_connected = false;
		std::string ip;

		client(asio::io_context& io_context, const std::string& ip_address, const std::string& port);

		void set_ip_address(const std::string& new_ip);

		void disconnect();

		bool poll(received_data& data);

		void send(int protocol, auto data)
		{
			bytes serialized;
			zpp::bits::out out(serialized);

			size_t size_header = 0;
			out(size_header).or_throw();

			out(protocol).or_throw();
			out(data).or_throw();

			out.reset();
			out(serialized.size() - sizeof(size_header)).or_throw();

			send_bytes(serialized);
		}
	};

	void deserialize(auto& target, received_data& data)
	{
		zpp::bits::in in(data.serialized_data);

		int protocol_id = 0;
		in(protocol_id).or_throw();

		in(target).or_throw();
	}
};