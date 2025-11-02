#include <iostream>
#include <thread>
#include "asio/post.hpp"

#include "network.h"

#define LOG 0

using namespace network;

void connection::start()
{
    read_loop();
}

void connection::write_loop()
{
    asio::async_write(socket, asio::buffer(send_queue.front()), 
        [this, self = shared_from_this()](asio::error_code error, size_t bytes_written) 
        {
            if (!error && dequeue()) 
            {
                write_loop();
            }
        }
    );
}

void connection::read_loop()
{
    asio::async_read(socket, buffer, asio::transfer_exactly(sizeof(size_t)), 
        [this, self = shared_from_this()](asio::error_code error, size_t bytes_read)
        {
            if (!error) 
            {
                // Read payload size
                size_t payload_size = 0;
                std::memcpy(&payload_size, buffer.data().data(), sizeof(payload_size));

                buffer.consume(buffer.size());

                // Read payload
                asio::read(socket, buffer, asio::transfer_exactly(payload_size));

                bytes data(buffer.size());
                asio::buffer_copy(asio::buffer(data), buffer.data());

                read_queue.push_back(data);

                buffer.consume(buffer.size());
                read_loop();
            }
            else if (error == asio::error::eof) 
            {
                std::cout << "Session terminated." << std::endl;
                return;
            }
            else 
            {
                return;
            }
        }
    );
}

server::server(asio::io_context& io_context, short port) :
    acceptor(io_context)
{
    auto endpoint = tcp::endpoint(tcp::v4(), port);
    acceptor.open(endpoint.protocol());
    acceptor.set_option(tcp::acceptor::reuse_address());
    acceptor.bind(endpoint);
    acceptor.listen();
    accept();
}

size_t server::register_connection(weak_connection_ptr ptr)
{
    std::lock_guard<std::mutex> lock(mutex);
    registered_connections.push_back(ptr);
    return registered_connections.size();
}

template <typename F>
size_t server::for_each_active(F f)
{
    std::vector<connection_ptr> active;
    {
        std::lock_guard<std::mutex> lock(mutex);

        for (auto& w : registered_connections)
        {
            if (auto c = w.lock())
            {
                active.push_back(c);
            }
        } 
    }

    for (auto& c : active) 
    {
        f(*c);
    }

    return active.size();
}

void server::accept()
{
    acceptor.async_accept(
        [this](asio::error_code error, tcp::socket socket) 
        {
            if (!error) 
            {
                std::cout << "Creating session on: "
                    << socket.remote_endpoint().address().to_string()
                    << ":" << socket.remote_endpoint().port() << std::endl;

                auto session = std::make_shared<connection>(std::move(socket));
                register_connection(session);
                session->start();
                accept();
            }
            else 
            {
                std::cout << "Network error: " << error.message() << std::endl;
            }
        }
    );
}

size_t server::send_bytes(bytes data)
{
    return for_each_active([data](connection& c) { c.send(data, true); });
}

bool server::poll(received_data& data)
{
    std::vector<connection_ptr> active;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto& w : registered_connections)
        {
            if (auto c = w.lock())
            {
                active.push_back(c);
            }
        }
    }

    std::list<bytes>* active_read_queue = nullptr;

    for (auto& c : active) 
    {
        if (c->pop_read_queue(data))
        {
            return true;
        }
    }

    return false;
}

client::client(asio::io_context& io_context, const std::string& ip_address, const std::string& port) : 
    io::io((tcp::socket)io_context), 
    resolver(io_context),
    retry_timer(io_context, std::chrono::seconds(3)),
    ip(ip_address),
    port(port)
{
    resolve_loop(ip, port);
}

void client::set_ip_address(const std::string& new_ip)
{
    if (new_ip == ip)
    {
        return;
    }

    retry_timer.cancel();
    resolver.cancel(); // Stop current connect loop
    socket.close();

    ip = new_ip;
    resolve_loop(ip, port);
}

void client::resolve_loop(std::string& ip_address, const std::string& port)
{
    is_connected = false;

    resolver.async_resolve(ip_address, port,
        [&, this](const asio::error_code& error, const tcp::resolver::results_type& endpoints)  
        {
            if (error && error != asio::error::operation_aborted)
            {
                std::cerr << "Error resolving IP address and port: " << error.message() << std::endl;
                std::cerr << "Retrying in 3 seconds" << std::endl;

                retry_timer.expires_at(std::chrono::steady_clock::now() + std::chrono::seconds(3));
                retry_timer.async_wait(
                    [&, this](const asio::error_code& error)
                    {
                        resolve_loop(ip_address, port);
                    }
                );
            }
            else if (!error)
            {
                connect_loop(endpoints);
            }
        }
    );
}

void client::connect_loop(const tcp::resolver::results_type& endpoints)
{
    asio::async_connect(socket, endpoints,
        [&, this](const asio::error_code& error, const asio::ip::tcp::endpoint& /*endpoint*/) 
        {
            if (error && error != asio::error::operation_aborted)
            {
                is_connected = false;

                std::cerr << "Connection error: " << error.message() << std::endl;
                std::cerr << "Scheduling reconnect in 3 seconds" << std::endl;

                socket.close();

                retry_timer.expires_at(std::chrono::steady_clock::now() + std::chrono::seconds(3));
                retry_timer.async_wait(
                    [&, this](const asio::error_code& error)
                    {
                        resolve_loop(ip, port);
                    }
                );
            }
            else if (!error)
            {
                is_connected = true;
                read_loop();
            }
        }
    );
}

void client::read_loop()
{
    asio::async_read(socket, buffer, asio::transfer_exactly(sizeof(size_t)), 
        [this](asio::error_code error, size_t bytes_read) 
        {
            if (!error && error != asio::error::operation_aborted)
            {
                // Read payload size
                size_t payload_size = 0;
                std::memcpy(&payload_size, buffer.data().data(), sizeof(payload_size));

                buffer.consume(buffer.size());

                // Read payload
                asio::read(socket, buffer, asio::transfer_exactly(payload_size));

                bytes data(buffer.size());
                asio::buffer_copy(asio::buffer(data), buffer.data());

                read_queue.push_back(data);

                buffer.consume(buffer.size());
                read_loop();
            }
            else 
            {
                is_connected = false;
                if (error == asio::error::eof)
                {
                    std::cout << "Session terminated." << std::endl;
                }
                else
                {
                    std::cout << "Read error: " << error.message() << std::endl;
                }

                resolve_loop(ip, port);
            }
        }
    );
}

void client::write_loop()
{
    asio::async_write(socket, asio::buffer(send_queue.front()), 
        [this](asio::error_code error, size_t bytes_written) 
        {
            if (!error && dequeue()) {
                write_loop();
            }
        }
    );
}

void client::send_bytes(bytes data)
{
    post(socket.get_executor(), [=, this] 
        {
            if (enqueue(std::move(data), true))
            {
                write_loop();
            }
        }
    );
}

void client::disconnect()
{
    if (!is_connected)
    {
        return;
    }

    asio::error_code error; // This error doesn't matter
    socket.shutdown(tcp::socket::shutdown_both, error);
    socket.close();
}

bool client::poll(received_data& data)
{
    return pop_read_queue(data);
}