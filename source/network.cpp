#include <iostream>
#include <thread>
#include "asio/post.hpp"

#include "network.h"

#define LOG 0

using namespace network;

connection::connection(tcp::socket socket):
    socket(std::move(socket))
{
}

void connection::start()
{
	read_loop();
}

void connection::send(bytes data, bool at_front = false)
{
    post(socket.get_executor(), [=] {
    if (enqueue(std::move(data), at_front))
        write_loop();
    });
}

// Returns true if need to start write loop
bool connection::enqueue(bytes data, bool at_front)
{ 
    at_front &= !send_queue.empty(); // no difference
    if (at_front)
        send_queue.insert(std::next(std::begin(send_queue)), std::move(data));
    else
        send_queue.push_back(std::move(data));

    return send_queue.size() == 1;
}

// Returns true if more messages pending after dequeue
bool connection::dequeue()
{ 
    assert(!send_queue.empty());
    send_queue.pop_front();
    return !send_queue.empty();
}

void connection::write_loop()
{
    asio::async_write(socket, asio::buffer(send_queue.front()), [this, self = shared_from_this()](asio::error_code error, size_t bytes_written) {
#if LOG
        std::cout << "Server sent: " << bytes_written << " bytes (" << error.message() << ")" << std::endl;
#endif
        if (!error && dequeue()) {
            write_loop();
        }
    });
}

void connection::read_loop()
{
    buffer.consume(buffer.size());

    asio::async_read_until(socket, buffer, "\n", [this, self = shared_from_this()](asio::error_code error, size_t bytes_read) {
#if LOG
        std::cout << "Server read: " << bytes_read << " bytes (" << error.message() << ")" << std::endl;
#endif

        if (!error) {
            bytes data(buffer.size());
            asio::buffer_copy(asio::buffer(data), buffer.data());

            read_queue.push_back(data);
            read_loop();
        } 
        else if (error == asio::error::eof) {
            std::cout << "session terminated" << std::endl;
            return;
        }
        else {
            return;
        }
    });
}

void connection::pop_read_queue(received_data& data)
{
    bytes received = std::move(read_queue.front());
    data.serialized_data = received;
    data.protocol_id = static_cast<int>(data.serialized_data[0]);
    read_queue.pop_front();
}

server::server(asio::io_context& io_context, short port) :
    acceptor(io_context, tcp::endpoint(tcp::v4(), port))
{
    acceptor.set_option(tcp::acceptor::reuse_address());
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
            if (auto c = w.lock())
                active.push_back(c);
    }

    for (auto& c : active) {
        f(*c);
    }

    return active.size();
}

void server::accept()
{
    acceptor.async_accept([this](asio::error_code error, tcp::socket socket) {
        if (!error) {
#if LOG
            std::cout << "creating session on: "
                << socket.remote_endpoint().address().to_string()
                << ":" << socket.remote_endpoint().port() << '\n';
#endif

            auto session = std::make_shared<connection>(std::move(socket));
            register_connection(session);
            session->start();
            accept();
        }
        else {
            std::cout << "network error: " << error.message() << std::endl;
        }
     });
}

size_t server::send_bytes(bytes data)
{
    data.push_back((std::byte)'\n');
    return for_each_active([data](connection& c) { c.send(data, true); });
}

bool server::poll(received_data& data)
{
    std::vector<connection_ptr> active;
    {
        std::lock_guard<std::mutex> lock(mutex);

        for (auto& w : registered_connections)
            if (auto c = w.lock())
                active.push_back(c);
    }

    std::list<bytes>* active_read_queue = nullptr;

    for (auto& c : active) {
        std::list<bytes> read_queue = c->get_read_queue();
        if (read_queue.empty()) {
            continue;
        }

        c->pop_read_queue(data);
        return true;
    }

    return false;
}

client::client(asio::io_context& io_context, std::string& ip_address, short port):
    socket(io_context)
{
    tcp::resolver resolver(io_context);
    asio::connect(socket, resolver.resolve(ip_address, std::to_string(port)));

    read_loop();
}

client::~client()
{
    disconnect();
}

void client::read_loop()
{
    buffer.consume(buffer.size());

    asio::async_read_until(socket, buffer, "\n", [this](asio::error_code error, size_t bytes_read) {
#if LOG
        std::cout << "Client read: " << bytes_read << " bytes (" << error.message() << ")" << std::endl;
#endif

        if (!error) {
            bytes data(buffer.size());
            asio::buffer_copy(asio::buffer(data), buffer.data());

            read_queue.push_back(data);
            read_loop();
        }
        else if (error == asio::error::eof) {
            std::cout << "server terminated" << std::endl;
        }
        else {
            std::cout << "read error: " << error.message() << std::endl;
        }
    });
}

// Returns true if need to start write loop
bool client::enqueue(bytes data, bool at_front)
{
    at_front &= !send_queue.empty(); // no difference
    if (at_front)
        send_queue.insert(std::next(std::begin(send_queue)), std::move(data));
    else
        send_queue.push_back(std::move(data));

    return send_queue.size() == 1;
}

// Returns true if more messages pending after dequeue
bool client::dequeue()
{
    assert(!send_queue.empty());
    send_queue.pop_front();
    return !send_queue.empty();
}

void client::write_loop()
{
    asio::async_write(socket, asio::buffer(send_queue.front()), [this](asio::error_code error, size_t bytes_written) {
        if (!error && dequeue()) {
            write_loop();
        }
    });
}

void client::send_bytes(bytes data)
{
    data.push_back((std::byte)'\n');
    post(socket.get_executor(), [=] {
        if (enqueue(std::move(data), true))
            write_loop();
    });
}

void client::disconnect()
{
    asio::error_code error; // This error doesn't matter
    socket.shutdown(tcp::socket::shutdown_both, error);
    socket.close();
}

bool client::poll(received_data& data)
{
    if (read_queue.empty()) {
        return false;
    }

    bytes received = std::move(read_queue.front());
    data.serialized_data = received;
    data.protocol_id = static_cast<int>(data.serialized_data[0]);
    read_queue.pop_front();

    return true;
}