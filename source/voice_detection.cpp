#include "voice_detection.h"

#include <iostream>

robo::voice_detection::voice_detection():
    allowed_commands({ "apple", "orange", "hello" }),
    audio(30 * 1000)
{
}

bool robo::voice_detection::init()
{
    ggml_backend_load_all();
    struct whisper_context_params cparams = whisper_context_default_params();

    cparams.use_gpu = params.use_gpu;
    cparams.flash_attn = params.flash_attn;

    context = whisper_init_from_file_with_params(params.model.c_str(), cparams);

    if (context == nullptr)
    {
        fprintf(stderr, "error: failed to initialize whisper context\n");
        return false;
    }

    // Convert the commands into token streams we're listening for
    for (const auto& cmd : allowed_commands)
    {
        whisper_token tokens[1024];
        allowed_tokens.emplace_back();

        for (int l = 0; l < (int)cmd.size(); ++l)
        {
            // NOTE: very important to add the whitespace !
            //       the reason is that the first decoded token starts with a whitespace too!
            std::string ss = std::string(" ") + cmd.substr(0, l + 1);

            const int n = whisper_tokenize(context, ss.c_str(), tokens, 1024);
            if (n < 0)
            {
                fprintf(stderr, "%s: error: failed to tokenize command '%s'\n", __func__, cmd.c_str());
                return false;
            }

            if (n == 1)
            {
                allowed_tokens.back().push_back(tokens[0]);
            }
        }

        token_max_len = std::max(token_max_len, (uint32_t)cmd.size());
    }

    if (!audio.init(params.capture_id, WHISPER_SAMPLE_RATE))
    {
        fprintf(stderr, "audio.init() failed!\n");
        return true;
    }

    audio.resume();

    // Wait for 1 second to avoid any buffered noise
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    audio.clear();

    return true;
}

std::string robo::voice_detection::process()
{
    audio.get(2000, pcmf32_cur);

    // Manual energy check because I guess vad_simple doesn't do enough?
    float energy = 0.0f;
    for (float sample : pcmf32_cur) 
    {
        energy += sample * sample;
    }
    energy = sqrtf(energy / pcmf32_cur.size());

    const float min_energy = 0.001f;
    if (energy < min_energy) 
    {
        return "";  // Too quiet, ignore
    }

    fprintf(stdout, "Audio energy: %f\n", energy);

    if (!::vad_simple(pcmf32_cur, WHISPER_SAMPLE_RATE, 1000, params.vad_thold, params.freq_thold, params.print_energy))
    {
        return "";
    }

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

    // run the transformer and a single decoding pass
    if (whisper_full(context, wparams, pcmf32_cur.data(), pcmf32_cur.size()) != 0)
    {
        fprintf(stderr, "%s: ERROR: whisper_full() failed\n", __func__);
        return "";
    }

    // estimate command probability
    // NOTE: not optimal
    
    const auto* logits = whisper_get_logits(context);

    std::vector<float> probs(whisper_n_vocab(context), 0.0f);

    // compute probs from logits via softmax
    {
        float max = -1e9;
        for (int i = 0; i < (int)probs.size(); ++i) 
        {
            max = std::max(max, logits[i]);
        }

        float sum = 0.0f;
        for (int i = 0; i < (int)probs.size(); ++i) 
        {
            probs[i] = expf(logits[i] - max);
            sum += probs[i];
        }

        for (int i = 0; i < (int)probs.size(); ++i)
        {
            probs[i] /= sum;
        }
    }

    std::vector<std::pair<float, int>> probs_id;

    double psum = 0.0;
    for (int i = 0; i < (int)allowed_commands.size(); ++i) 
    {
        probs_id.emplace_back(probs[allowed_tokens[i][0]], i);
        for (int j = 1; j < (int)allowed_tokens[i].size(); ++j) 
        {
            probs_id.back().first += probs[allowed_tokens[i][j]];
        }
        probs_id.back().first /= allowed_tokens[i].size();
        psum += probs_id.back().first;
    }

    // normalize
    for (auto& p : probs_id)
    {
        p.first /= psum;
    }

    // sort descending
    {
        using pair_type = decltype(probs_id)::value_type;
        std::sort(probs_id.begin(), probs_id.end(), 
            [](const pair_type& a, const pair_type& b) 
            {
                return a.first > b.first;
            }
        );
    }

    // print the commands and the respective probabilities
    {
        fprintf(stdout, "\n");
        for (const auto& cmd : probs_id)
        {
            fprintf(stdout, "%s: %s%-*s%s = %f | ", __func__, "\033[1m", token_max_len, allowed_commands[cmd.second].c_str(), "\033[0m", cmd.first);
            for (int token : allowed_tokens[cmd.second]) 
            {
                fprintf(stdout, "'%4s' %f ", whisper_token_to_str(context, token), probs[token]);
            }
            fprintf(stdout, "\n");
        }
    }

    // best command
    const auto t_end = std::chrono::high_resolution_clock::now();

    const float prob = probs_id[0].first;
    const int index = probs_id[0].second;
    const char* best_command = allowed_commands[index].c_str();

    const float min_confidence = 0.75f;

    // Reject if the maximum confidence is too low
    if (prob < min_confidence) {
        fprintf(stdout, "%s: best command '%s' has confidence %f, below threshold %f. Ignoring.\n",
            __func__, allowed_commands[probs_id[0].second].c_str(), probs_id[0].first, min_confidence);
        audio.clear();
        return "";
    }

    fprintf(stdout, "\n");
    fprintf(stdout, "%s: detected command: %s%s%s | p = %f | t = %d ms\n", __func__,
        "\033[1m", best_command, "\033[0m", prob,
        (int)std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count());
    fprintf(stdout, "\n");
    
    audio.clear();
    
    return std::string(best_command);
}