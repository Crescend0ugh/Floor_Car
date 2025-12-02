#pragma once

#include "async_processor.h"
#include "common-sdl.h"
#include "common.h"
#include "whisper.h"
#include "grammar-parser.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

struct whisper_params 
{
    int32_t n_threads = std::min(2, (int32_t)std::thread::hardware_concurrency());
    int32_t command_ms = 3000;
    int32_t capture_id = -1;
    int32_t max_tokens = 32;
    int32_t audio_ctx = 512;

    float vad_thold = 0.98f;
    float freq_thold = 1200.0f;

    float grammar_penalty = 100.0f;

    grammar_parser::parse_state grammar_parsed;

    bool translate = false;
    bool print_special = false;
    bool print_energy = false;
    bool no_timestamps = true;
    bool use_gpu = false;
    bool flash_attn = true;

    std::string language = "en";
    std::string model = "content/voice_models/ggml-tiny.en.bin";
};

namespace robo
{
    struct voice_result
    {
        std::string command;
        float confidence = 0.0f;
        std::chrono::milliseconds processing_time;
        bool detected = false;
    };

    class voice_detection : public async_processor<voice_result>
    {
    private:
        whisper_params params;
        struct whisper_context* context = nullptr;
        std::vector<std::string> allowed_commands;

        std::vector<std::vector<whisper_token>> allowed_tokens;
        uint32_t token_max_len = 0;

        audio_async audio;

        std::vector<float> pcmf32_cur;
        std::vector<float> pcmf32_prompt;

        // Minimum confidence threshold
        float min_confidence = 0.75f;

        // Base class overrides
        voice_result process_impl() override;
        bool on_init() override;
        void on_shutdown() override;

    public:
        voice_detection(std::vector<std::string>& commands);
    };
}
