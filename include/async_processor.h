/*
    async_processor.h

    Base class for components that perform asynchronous processing with callbacks.
    Used as the base class for vision and voice_detection.
    Encapsulates ASIO threading, timers, and lifecycle management.

    Derived classes need to implement:
    - process_impl() - The actual processing logic
    - Optional: on_init() - Custom initialization
    - Optional: on_shutdown() - Custom cleanup
*/

#pragma once

#include <asio.hpp>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>
#include <iostream>

// Base class for asynchronous processors
template<typename ResultType>
class async_processor
{
public:
    using result_callback = std::function<void(const ResultType&)>;

protected:
    // ASIO infrastructure
    asio::io_context io_context;
    asio::steady_timer timer;
    asio::executor_work_guard<asio::io_context::executor_type> work_guard;
    std::unique_ptr<std::thread> io_thread;

    // State management
    std::atomic<bool> is_running{ false };
    std::atomic<bool> processing_in_progress{ false };
    std::atomic<bool> is_initialized{ false };

    // Result callback
    result_callback on_result_ready;

    // Mutex for result protection (if needed by derived classes)
    mutable std::mutex result_mutex;

    // Internal processing handler
    void run_processing_async() 
    {
        if (!is_running.load() || processing_in_progress.load()) 
        {
            return;
        }

        processing_in_progress.store(true);

        try 
        {
            // Call derived class implementation
            ResultType result = process_impl();

            // Invoke callback if registered
            if (on_result_ready) 
            {
                on_result_ready(result);
            }
        }
        catch (const std::exception& e) 
        {
            std::cerr << "Error in async processing: " << e.what() << std::endl;
            on_processing_error(e);
        }

        processing_in_progress.store(false);
    }

    // To be implemented by derived classes
    virtual ResultType process_impl() = 0;

    // Optional hooks for derived classes
    virtual bool on_init() { return true; }
    virtual void on_shutdown() {}
    virtual void on_processing_error(const std::exception& e) 
    {
        std::cerr << "Processing error: " << e.what() << std::endl;
    }

    // Start the io_context thread if not already running
    void ensure_thread_running() 
    {
        is_running.store(true);

        if (!io_thread || !io_thread->joinable()) 
        {
            io_thread = std::make_unique<std::thread>(
                [this]() 
                {
                    io_context.run();
                }
            );
        }
    }

public:
    async_processor():
        timer(io_context),
        work_guard(asio::make_work_guard(io_context))
    {
    }

    virtual ~async_processor()
    {
        shutdown();
    }

    // Prevent copying
    async_processor(const async_processor&) = delete;
    async_processor& operator=(const async_processor&) = delete;

    // Initialize the processor
    bool initialize() 
    {
        if (is_initialized.load())
        {
            std::cerr << "Warning: Processor already initialized" << std::endl;
            return true;
        }

        bool success = on_init();
        if (success) 
        {
            is_initialized.store(true);
        }

        return success;
    }

    // Start continuous processing loop
    void start_continuous(result_callback callback, std::chrono::milliseconds interval = std::chrono::milliseconds(100)) 
    {
        if (!is_initialized.load()) 
        {
            std::cerr << "Error: Processor not initialized" << std::endl;
            return;
        }

        if (is_running.load())
        {
            std::cerr << "Warning: Continuous processing already running" << std::endl;
            return;
        }

        on_result_ready = callback;
        ensure_thread_running();

        // Create a shared_ptr to the schedule function to enable recursion
        auto schedule_next = std::make_shared<std::function<void()>>();
        *schedule_next = [this, interval, schedule_next]()
        {
            if (!is_running.load())
            {
                return;
            }

            run_processing_async();

            timer.expires_after(interval);
            timer.async_wait(
                [schedule_next](const asio::error_code& ec)
                {
                    if (!ec)
                    {
                        (*schedule_next)();
                    }
                }
            );
        };

        // Start the loop (initial delay is 100ms)
        timer.expires_after(std::chrono::milliseconds(100));
        timer.async_wait(
            [schedule_next](const asio::error_code& ec) 
            {
                if (!ec) 
                {
                    (*schedule_next)();
                }
            }
        );
    }

    // Stop continuous processing
    void stop_continuous() 
    {
        is_running.store(false);
        timer.cancel();
        on_result_ready = nullptr;
    }

    // Trigger a single processing operation
    void trigger_single(result_callback callback) 
    {
        if (!is_initialized.load())
        {
            std::cerr << "Error: Processor not initialized" << std::endl;
            return;
        }

        if (processing_in_progress.load()) 
        {
            std::cerr << "Warning: Processing already in progress" << std::endl;
            return;
        }

        ensure_thread_running();

        // Post single processing operation
        asio::post(io_context, 
            [this, callback]() 
            {
                processing_in_progress.store(true);

                try {
                    ResultType result = process_impl();

                    if (callback) {
                        callback(result);
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Error in single processing: " << e.what() << std::endl;
                    on_processing_error(e);
                }

                processing_in_progress.store(false);
            }
        );
    }

    // Check if processing is currently running
    bool is_processing() const 
    {
        return processing_in_progress.load();
    }

    // Check if initialized
    bool initialized() const
    {
        return is_initialized.load();
    }

    // Shutdown and cleanup
    void shutdown() 
    {
        if (is_running.load()) 
        {
            stop_continuous();
        }

        // Call derived class cleanup
        on_shutdown();

        work_guard.reset();
        io_context.stop();

        if (io_thread && io_thread->joinable()) 
        {
            io_thread->join();
        }

        is_initialized.store(false);
    }
};
