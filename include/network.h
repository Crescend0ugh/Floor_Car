#pragma once

#include <exception>
#include <memory>
#include <list>

#include "asio.hpp"

using asio::ip::tcp;

namespace network
{
	class connection : public std::enable_shared_from_this<connection>
	{
	private:
		tcp::socket socket;
		asio::streambuf buffer;
		std::list<std::string> send_queue;

		bool enqueue(std::string data, bool at_front);
		bool dequeue();
		void write_loop();
		void read_loop();
	public:
		connection(tcp::socket socket);
		void start();

		void send(std::string data, bool at_front);
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

	public:
		server(asio::io_context& io_context, short port);
		size_t send(std::string data);
	};

	class client
	{
	private:
		tcp::socket socket;
		asio::streambuf buffer;
		std::list<std::string> read_queue;

		void read_loop();

	public:
		client(asio::io_context& io_context, std::string& ip_address, short port);
		~client();

		void send(std::string data);
		bool poll(std::string& target);
		void disconnect();
	};
};
