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

	bytes serialize(int protocol, const auto& data)
	{
		bytes serialized;
		zpp::bits::out out(serialized);

		size_t size_header = 0;
		out(size_header).or_throw();

		out(protocol).or_throw();
		out(data).or_throw();

		out.reset();
		out(serialized.size() - sizeof(size_header)).or_throw();

		return serialized;
	}

	void deserialize(auto& target, received_data& data)
	{
		zpp::bits::in in(data.serialized_data);

		int protocol_id = 0;
		in(protocol_id).or_throw();

		in(target).or_throw();
	}

	// Shared functions for server connections and clients
	class io : public std::enable_shared_from_this<io>
	{
	protected:
		tcp::socket socket;
		asio::streambuf buffer;
		std::list<bytes> send_queue;
		std::list<bytes> read_queue;

	public:
		io(tcp::socket socket);
		bool enqueue(bytes data, bool at_front);
		bool dequeue();
		void send(bytes data, bool at_front = false);
		bool pop_read_queue(received_data& data);
		void close();

		virtual void write_loop() = 0;
		virtual void read_loop() = 0;
		virtual void handle_read_error(const asio::error_code& error) = 0;
	};

	// Represents a connected client on the server
	class connection : public io
	{
	private:
		void write_loop() override;
		void read_loop() override;
		void handle_read_error(const asio::error_code& error) override;

	public:
		using io::io;
		void start();
	};

	class server
	{
	private:
		using connection_ptr = std::shared_ptr<connection>;
		using weak_connection_ptr = std::weak_ptr<connection>;

		asio::io_context io_context;
		tcp::acceptor acceptor;
		std::mutex mutex;
		std::vector<weak_connection_ptr> registered_connections;

		size_t register_connection(weak_connection_ptr ptr);

		template <typename F>
		size_t for_each_active(F f);

		void accept();
		size_t send_to_all(bytes data);

	public:
		server(asio::io_context& io_context, short port);

		bool poll(received_data& data);
		void shutdown();

		size_t get_client_count() const;

		void send(int protocol, auto data)
		{
			send_to_all(network::serialize(protocol, data));
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

		void send(bytes data);
		void handle_read_error(const asio::error_code& error) override;

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
			send(network::serialize(protocol, data));
		}
	};
};