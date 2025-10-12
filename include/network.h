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

	class connection : public std::enable_shared_from_this<connection>
	{
	private:
		tcp::socket socket;
		asio::streambuf buffer;
		std::list<bytes> send_queue;
		std::list<bytes> read_queue;

		bool enqueue(bytes data, bool at_front);
		bool dequeue();
		void write_loop();
		void read_loop();
		
	public:
		connection(tcp::socket socket);
		void start();

		void pop_read_queue(received_data& data);
		void send(bytes data, bool at_front);
		std::list<bytes> get_read_queue() { return read_queue; };
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

		// Anything with auto in parameters needs to be in the header...
		size_t send(int protocol, auto data)
		{
			auto [serialized, out] = zpp::bits::data_out();
			out(protocol).or_throw();
			out(data).or_throw();
			return send_bytes(serialized);
		}
	};

	class client
	{
	private:
		tcp::socket socket;
		asio::streambuf buffer;
		std::list<bytes> send_queue;
		std::list<bytes> read_queue;

		bool enqueue(bytes data, bool at_front);
		bool dequeue();
		void write_loop();
		void read_loop();

		void send_bytes(bytes data);

	public:
		client(asio::io_context& io_context, std::string& ip_address, short port);
		~client();

		void disconnect();

		bool poll(received_data& data);

		void send(int protocol, auto data)
		{
			auto [serialized, out] = zpp::bits::data_out();
			out(protocol).or_throw();
			out(data).or_throw();
			send_bytes(serialized);
		}

	};

	void deserialize(auto& target, received_data& data)
	{
		zpp::bits::in in(data.serialized_data);

		int protocol_id = 0;
		in(protocol_id);

		in(target);
	}
};