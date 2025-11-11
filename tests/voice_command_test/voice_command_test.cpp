// Voice assistant example
//
// Speak short text commands to the microphone.
// This program will detect your voice command and convert them to text.
//
// ref: https://github.com/ggml-org/whisper.cpp/issues/171
//

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

// command-line parameters
struct whisper_params {
    int32_t n_threads = std::min(4, (int32_t)std::thread::hardware_concurrency());
    int32_t prompt_ms = 5000;
    int32_t command_ms = 3000;
    int32_t capture_id = -1;
    int32_t max_tokens = 32;
    int32_t audio_ctx = 0;

    float vad_thold = 0.9f;
    float freq_thold = 100.0f;

    float grammar_penalty = 100.0f;

    grammar_parser::parse_state grammar_parsed;

    bool translate = false;
    bool print_special = false;
    bool print_energy = false;
    bool no_timestamps = true;
    bool use_gpu = true;
    bool flash_attn = true;

    std::string language = "en";
    std::string model = "content/voice_models/ggml-tiny.en.bin";
    std::string fname_out;
    std::string commands;
    std::string prompt;
    std::string context;
    std::string grammar;
};

// command-list mode
// guide the transcription to match the most likely command from a provided list
static int process_command_list(struct whisper_context* ctx, audio_async& audio, const whisper_params& params, std::ofstream& fout) {
    std::vector<std::string> allowed_commands = { "apple", "orange", "ball", "hello" };

    int max_len = 0;

    std::vector<std::vector<whisper_token>> allowed_tokens;

    for (const auto& cmd : allowed_commands) {
        whisper_token tokens[1024];
        allowed_tokens.emplace_back();

        for (int l = 0; l < (int)cmd.size(); ++l) {
            // NOTE: very important to add the whitespace !
            //       the reason is that the first decoded token starts with a whitespace too!
            std::string ss = std::string(" ") + cmd.substr(0, l + 1);

            const int n = whisper_tokenize(ctx, ss.c_str(), tokens, 1024);
            if (n < 0) {
                fprintf(stderr, "%s: error: failed to tokenize command '%s'\n", __func__, cmd.c_str());
                return 3;
            }

            if (n == 1) {
                allowed_tokens.back().push_back(tokens[0]);
            }
        }

        max_len = std::max(max_len, (int)cmd.size());
    }

    /*
    std::string k_prompt = "select one from the available words: ";
    for (int i = 0; i < (int)allowed_commands.size(); ++i) {
        if (i > 0) {
            k_prompt += ", ";
        }
        k_prompt += allowed_commands[i];
    }
    k_prompt += ". selected word: ";

    // tokenize prompt
    std::vector<whisper_token> k_tokens;
    {
        k_tokens.resize(1024);
        const int n = whisper_tokenize(ctx, k_prompt.c_str(), k_tokens.data(), 1024);
        if (n < 0) {
            fprintf(stderr, "%s: error: failed to tokenize prompt '%s'\n", __func__, k_prompt.c_str());
            return 4;
        }
        k_tokens.resize(n);
    }

    fprintf(stderr, "\n");
    fprintf(stderr, "%s: prompt: '%s'\n", __func__, k_prompt.c_str());
    fprintf(stderr, "%s: tokens: [", __func__);
    for (const auto& token : k_tokens) {
        fprintf(stderr, " %d", token);
    }
    fprintf(stderr, " ]\n");
    */

    fprintf(stderr, "\n");
    fprintf(stderr, "%s: listening for a command ...\n", __func__);
    fprintf(stderr, "\n");

    bool is_running = true;

    std::vector<float> pcmf32_cur;
    std::vector<float> pcmf32_prompt;

    // main loop
    while (is_running) {
        // handle Ctrl + C
        is_running = sdl_poll_events();

        // delay
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        audio.get(2000, pcmf32_cur);

        if (::vad_simple(pcmf32_cur, WHISPER_SAMPLE_RATE, 1000, params.vad_thold, params.freq_thold, params.print_energy)) {
            fprintf(stdout, "%s: Speech detected! Processing ...\n", __func__);

            const auto t_start = std::chrono::high_resolution_clock::now();

            whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);

            wparams.print_progress = false;
            wparams.print_special = params.print_special;
            wparams.print_realtime = false;
            wparams.print_timestamps = !params.no_timestamps;
            wparams.translate = params.translate;
            wparams.no_context = true;
            wparams.single_segment = true;
            wparams.max_tokens = 1;
            wparams.language = params.language.c_str();
            wparams.n_threads = params.n_threads;

            wparams.audio_ctx = params.audio_ctx;

            //wparams.prompt_tokens = k_tokens.data();
            //wparams.prompt_n_tokens = k_tokens.size();

            // run the transformer and a single decoding pass
            if (whisper_full(ctx, wparams, pcmf32_cur.data(), pcmf32_cur.size()) != 0) {
                fprintf(stderr, "%s: ERROR: whisper_full() failed\n", __func__);
                break;
            }

            // estimate command probability
            // NOTE: not optimal
            {
                const auto* logits = whisper_get_logits(ctx);

                std::vector<float> probs(whisper_n_vocab(ctx), 0.0f);

                // compute probs from logits via softmax
                {
                    float max = -1e9;
                    for (int i = 0; i < (int)probs.size(); ++i) {
                        max = std::max(max, logits[i]);
                    }

                    float sum = 0.0f;
                    for (int i = 0; i < (int)probs.size(); ++i) {
                        probs[i] = expf(logits[i] - max);
                        sum += probs[i];
                    }

                    for (int i = 0; i < (int)probs.size(); ++i) {
                        probs[i] /= sum;
                    }
                }

                std::vector<std::pair<float, int>> probs_id;

                double psum = 0.0;
                for (int i = 0; i < (int)allowed_commands.size(); ++i) {
                    probs_id.emplace_back(probs[allowed_tokens[i][0]], i);
                    for (int j = 1; j < (int)allowed_tokens[i].size(); ++j) {
                        probs_id.back().first += probs[allowed_tokens[i][j]];
                    }
                    probs_id.back().first /= allowed_tokens[i].size();
                    psum += probs_id.back().first;
                }

                // normalize
                for (auto& p : probs_id) {
                    p.first /= psum;
                }

                // sort descending
                {
                    using pair_type = decltype(probs_id)::value_type;
                    std::sort(probs_id.begin(), probs_id.end(), [](const pair_type& a, const pair_type& b) {
                        return a.first > b.first;
                        });
                }

                // print the commands and the respective probabilities
                {
                    fprintf(stdout, "\n");
                    for (const auto& cmd : probs_id) {
                        fprintf(stdout, "%s: %s%-*s%s = %f | ", __func__, "\033[1m", max_len, allowed_commands[cmd.second].c_str(), "\033[0m", cmd.first);
                        for (int token : allowed_tokens[cmd.second]) {
                            fprintf(stdout, "'%4s' %f ", whisper_token_to_str(ctx, token), probs[token]);
                        }
                        fprintf(stdout, "\n");
                    }
                }

                // best command
                {
                    const auto t_end = std::chrono::high_resolution_clock::now();

                    const float prob = probs_id[0].first;
                    const int index = probs_id[0].second;
                    const char* best_command = allowed_commands[index].c_str();

                    fprintf(stdout, "\n");
                    fprintf(stdout, "%s: detected command: %s%s%s | p = %f | t = %d ms\n", __func__,
                        "\033[1m", best_command, "\033[0m", prob,
                        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count());
                    fprintf(stdout, "\n");
                    if (fout.is_open()) {
                        fout << best_command << std::endl;
                    }
                }
            }

            audio.clear();
        }
    }

    return 0;
}

int main(int argc, char** argv) {
    ggml_backend_load_all();

    whisper_params params;

    // whisper init

    struct whisper_context_params cparams = whisper_context_default_params();

    cparams.use_gpu = params.use_gpu;
    cparams.flash_attn = params.flash_attn;

    struct whisper_context* ctx = whisper_init_from_file_with_params(params.model.c_str(), cparams);
    if (ctx == nullptr) {
        fprintf(stderr, "error: failed to initialize whisper context\n");
        return 2;
    }

    // print some info about the processing
    {
        fprintf(stderr, "\n");
        if (!whisper_is_multilingual(ctx)) {
            if (params.language != "en" || params.translate) {
                params.language = "en";
                params.translate = false;
                fprintf(stderr, "%s: WARNING: model is not multilingual, ignoring language and translation options\n", __func__);
            }
        }
        fprintf(stderr, "%s: processing, %d threads, lang = %s, task = %s, timestamps = %d ...\n",
            __func__,
            params.n_threads,
            params.language.c_str(),
            params.translate ? "translate" : "transcribe",
            params.no_timestamps ? 0 : 1);

        fprintf(stderr, "\n");
    }

    // init audio

    audio_async audio(30 * 1000);
    if (!audio.init(params.capture_id, WHISPER_SAMPLE_RATE)) {
        fprintf(stderr, "%s: audio.init() failed!\n", __func__);
        return 1;
    }

    audio.resume();

    // wait for 1 second to avoid any buffered noise
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    audio.clear();

    int  ret_val = 0;

    if (!params.grammar.empty()) {
        auto& grammar = params.grammar_parsed;
        if (is_file_exist(params.grammar.c_str())) {
            // read grammar from file
            std::ifstream ifs(params.grammar.c_str());
            const std::string txt = std::string((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            grammar = grammar_parser::parse(txt.c_str());
        }
        else {
            // read grammar from string
            grammar = grammar_parser::parse(params.grammar.c_str());
        }

        // will be empty (default) if there are parse errors
        if (grammar.rules.empty()) {
            ret_val = 1;
        }
        else {
            fprintf(stderr, "%s: grammar:\n", __func__);
            grammar_parser::print_grammar(stderr, grammar);
            fprintf(stderr, "\n");
        }
    }

    std::ofstream fout;
    if (params.fname_out.length() > 0) {
        fout.open(params.fname_out);
        if (!fout.is_open()) {
            fprintf(stderr, "%s: failed to open output file '%s'!\n", __func__, params.fname_out.c_str());
            return 1;
        }
    }

    if (ret_val == 0) {
        if (!params.commands.empty()) {
            ret_val = process_command_list(ctx, audio, params, fout);
        }
        else {
            //ret_val = process_general_transcription(ctx, audio, params, fout);
            //ret_val = always_prompt_transcription(ctx, audio, params, fout);
            ret_val = process_command_list(ctx, audio, params, fout);
        }
    }

    audio.pause();

    whisper_print_timings(ctx);
    whisper_free(ctx);

    return ret_val;
}