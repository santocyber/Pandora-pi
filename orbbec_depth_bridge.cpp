#include "libobsensor/hpp/Pipeline.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "libobsensor/hpp/Error.hpp"
#include "libobsensor/hpp/StreamProfile.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

static std::atomic<bool> running(true);

static void handle_signal(int) {
    running = false;
}

static uint64_t wall_now_ms() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return static_cast<uint64_t>(ms.count());
}

static uint64_t steady_now_ms() {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return static_cast<uint64_t>(ms.count());
}

static int to_int(const char *s, int fallback) {
    if(s == nullptr) return fallback;
    try { return std::stoi(std::string(s)); }
    catch(...) { return fallback; }
}

static int env_int(const char *name, int fallback) {
    const char *value = std::getenv(name);
    return to_int(value, fallback);
}

static std::string json_escape(const std::string &s) {
    std::ostringstream out;

    for(char c : s) {
        switch(c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << c; break;
        }
    }

    return out.str();
}

/*
 * Rename atomico sem remove().
 * O bridge antigo removia o arquivo final antes do rename, criando uma janela
 * em que o Python podia ver RAW/META como inexistente. Aqui o rename substitui
 * atomicamente quando tmp e final estao no mesmo filesystem.
 */
static bool write_binary_atomic(const std::string &path, const void *data, size_t size) {
    const std::string tmp = path + ".tmp";

    {
        std::ofstream out(tmp.c_str(), std::ios::binary | std::ios::trunc);
        if(!out.is_open()) {
            std::cerr << "[bridge] erro abrindo tmp binario: " << tmp
                      << " errno=" << errno << " " << std::strerror(errno) << std::endl;
            return false;
        }

        out.write(static_cast<const char *>(data), static_cast<std::streamsize>(size));

        if(!out.good()) {
            std::cerr << "[bridge] erro escrevendo tmp binario: " << tmp
                      << " errno=" << errno << " " << std::strerror(errno) << std::endl;
            return false;
        }
    }

    if(std::rename(tmp.c_str(), path.c_str()) != 0) {
        std::cerr << "[bridge] erro rename binario: " << tmp << " -> " << path
                  << " errno=" << errno << " " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

static bool write_text_atomic(const std::string &path, const std::string &text) {
    const std::string tmp = path + ".tmp";

    {
        std::ofstream out(tmp.c_str(), std::ios::out | std::ios::trunc);
        if(!out.is_open()) {
            std::cerr << "[bridge] erro abrindo tmp texto: " << tmp
                      << " errno=" << errno << " " << std::strerror(errno) << std::endl;
            return false;
        }

        out << text;

        if(!out.good()) {
            std::cerr << "[bridge] erro escrevendo tmp texto: " << tmp
                      << " errno=" << errno << " " << std::strerror(errno) << std::endl;
            return false;
        }
    }

    if(std::rename(tmp.c_str(), path.c_str()) != 0) {
        std::cerr << "[bridge] erro rename texto: " << tmp << " -> " << path
                  << " errno=" << errno << " " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

struct DepthFrameCopy {
    bool valid = false;
    uint32_t width = 0;
    uint32_t height = 0;
    OBFormat format = OB_FORMAT_UNKNOWN;
    float scale = 1.0f;
    size_t data_size = 0;
    uint64_t frame_index = 0;
    uint64_t device_timestamp_us = 0;
    uint64_t captured_steady_ms = 0;
    uint64_t captured_wall_ms = 0;
    uint64_t input_frame_count = 0;
    std::vector<uint16_t> raw_y16;
};

struct SharedState {
    std::mutex mutex;
    std::condition_variable cv;

    DepthFrameCopy latest;

    uint64_t input_frame_count = 0;
    uint64_t capture_timeout_count = 0;
    uint64_t null_depth_count = 0;
    uint64_t bad_frame_count = 0;
    uint64_t published_count = 0;
    uint64_t stale_published_count = 0;
    uint64_t skipped_same_frame_count = 0;

    uint64_t last_new_frame_steady_ms = 0;
    uint64_t last_publish_frame_index = 0;

    double capture_fps = 0.0;
    double writer_fps = 0.0;

    bool capture_running = false;
    bool writer_running = false;
    std::string last_error;
};

static void print_depth_profiles(ob::Pipeline &pipe) {
    try {
        auto profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);

        if(!profiles) {
            std::cerr << "[bridge] lista de perfis depth indisponivel" << std::endl;
            return;
        }

        std::cerr << "[bridge] perfis depth suportados: " << profiles->count() << std::endl;

        for(uint32_t i = 0; i < profiles->count(); i++) {
            auto profile = profiles->getProfile(i);

            try {
                auto vp = profile->as<ob::VideoStreamProfile>();
                std::cerr << "[bridge] profile[" << i << "] "
                          << vp->width() << "x" << vp->height()
                          << " fps=" << vp->fps()
                          << " format=" << static_cast<int>(vp->format())
                          << std::endl;
            }
            catch(...) {
                std::cerr << "[bridge] profile[" << i << "] nao-video" << std::endl;
            }
        }
    }
    catch(ob::Error &e) {
        std::cerr << "[bridge] nao conseguiu listar perfis depth: " << e.getMessage() << std::endl;
    }
    catch(std::exception &e) {
        std::cerr << "[bridge] nao conseguiu listar perfis depth: " << e.what() << std::endl;
    }
}

struct DepthProfileCandidate {
    int width;
    int height;
    int fps;
    const char *name;
};

static void add_candidate(
    std::vector<DepthProfileCandidate> &items,
    int width,
    int height,
    int fps,
    const char *name
) {
    for(size_t i = 0; i < items.size(); i++) {
        if(items[i].width == width && items[i].height == height && items[i].fps == fps) {
            return;
        }
    }

    DepthProfileCandidate item;
    item.width = width;
    item.height = height;
    item.fps = fps;
    item.name = name;
    items.push_back(item);
}

static bool read_actual_depth_profile(
    ob::Pipeline &pipe,
    int &actual_width,
    int &actual_height,
    int &actual_fps
) {
    actual_width = 0;
    actual_height = 0;
    actual_fps = 0;

    try {
        auto enabled = pipe.getEnabledStreamProfileList();

        if(enabled && enabled->count() > 0) {
            for(uint32_t i = 0; i < enabled->count(); i++) {
                auto p = enabled->getProfile(i);

                if(p && p->type() == OB_STREAM_DEPTH) {
                    auto vp = p->as<ob::VideoStreamProfile>();
                    actual_width = static_cast<int>(vp->width());
                    actual_height = static_cast<int>(vp->height());
                    actual_fps = static_cast<int>(vp->fps());

                    std::cerr << "[bridge] depth habilitado "
                              << actual_width << "x" << actual_height
                              << "@" << actual_fps
                              << " format=" << static_cast<int>(vp->format())
                              << std::endl;

                    return actual_width > 0 && actual_height > 0;
                }
            }
        }
    }
    catch(std::exception &e) {
        std::cerr << "[bridge] aviso lendo perfil habilitado: "
                  << e.what() << std::endl;
    }

    return false;
}

static bool start_pipeline_depth(
    std::unique_ptr<ob::Pipeline> &pipe,
    int req_width,
    int req_height,
    int req_fps,
    int &actual_width,
    int &actual_height,
    int &actual_fps,
    bool show_profiles
) {
    std::vector<DepthProfileCandidate> candidates;

    // Ordem importante: tenta o solicitado; depois perfis mais leves.
    // Isso evita cair direto no perfil SDK padrao 160x120 quando 640x480
    // fica ocupado no endpoint USB.
    add_candidate(candidates, req_width, req_height, req_fps, "requested");
    add_candidate(candidates, 320, 240, 30, "safe_320x240_30");
    add_candidate(candidates, 320, 240, 15, "safe_320x240_15");
    add_candidate(candidates, 640, 480, 15, "safe_640x480_15");
    add_candidate(candidates, 640, 480, 30, "safe_640x480_30");
    add_candidate(candidates, 160, 120, 30, "last_resort_160x120_30");
    add_candidate(candidates, 0, 0, 0, "sdk_default");

    for(size_t i = 0; i < candidates.size(); i++) {
        DepthProfileCandidate c = candidates[i];

        try {
            pipe.reset(new ob::Pipeline());

            if(show_profiles && i == 0) {
                print_depth_profiles(*pipe);
            }

            auto config = std::make_shared<ob::Config>();

            std::cerr << "[bridge] tentando perfil depth "
                      << c.name << " ";

            if(c.width > 0 && c.height > 0 && c.fps > 0) {
                std::cerr << c.width << "x" << c.height << "@" << c.fps
                          << " Y16" << std::endl;

                config->enableVideoStream(
                    OB_STREAM_DEPTH,
                    c.width,
                    c.height,
                    c.fps,
                    OB_FORMAT_Y16
                );
            }
            else {
                std::cerr << "SDK default" << std::endl;
                config->enableVideoStream(OB_STREAM_DEPTH);
            }

            pipe->start(config);

            if(read_actual_depth_profile(*pipe, actual_width, actual_height, actual_fps)) {
                std::cerr << "[bridge] perfil aceito: " << c.name << std::endl;
                return true;
            }

            try {
                pipe->stop();
            }
            catch(...) {
            }
        }
        catch(ob::Error &e) {
            std::cerr << "[bridge] perfil falhou " << c.name
                      << ": " << e.getMessage() << std::endl;

            try {
                if(pipe) {
                    pipe->stop();
                }
            }
            catch(...) {
            }
        }
        catch(std::exception &e) {
            std::cerr << "[bridge] perfil falhou " << c.name
                      << ": " << e.what() << std::endl;

            try {
                if(pipe) {
                    pipe->stop();
                }
            }
            catch(...) {
            }
        }
    }

    actual_width = 0;
    actual_height = 0;
    actual_fps = 0;

    return false;
}

static void convert_y16_to_mm(
    const uint16_t *src,
    uint32_t width,
    uint32_t height,
    float scale,
    std::vector<uint16_t> &dst,
    uint16_t &min_mm,
    uint16_t &max_mm,
    uint16_t &center_mm,
    uint64_t &valid_count
) {
    const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);

    dst.resize(count);

    min_mm = std::numeric_limits<uint16_t>::max();
    max_mm = 0;
    center_mm = 0;
    valid_count = 0;

    for(size_t i = 0; i < count; i++) {
        const float mmf = static_cast<float>(src[i]) * scale;
        uint32_t mm = 0;

        if(mmf > 0.0f) {
            mm = static_cast<uint32_t>(std::lround(mmf));
            if(mm > 65535U) {
                mm = 65535U;
            }
        }

        const uint16_t value = static_cast<uint16_t>(mm);
        dst[i] = value;

        if(value > 0) {
            valid_count++;
            if(value < min_mm) min_mm = value;
            if(value > max_mm) max_mm = value;
        }
    }

    if(valid_count == 0) {
        min_mm = 0;
        max_mm = 0;
    }

    const size_t center_index =
        static_cast<size_t>(height / 2) * static_cast<size_t>(width) +
        static_cast<size_t>(width / 2);

    if(center_index < dst.size()) {
        center_mm = dst[center_index];
    }
}

static std::string make_meta_json(
    const DepthFrameCopy &frame,
    int req_width,
    int req_height,
    int req_fps,
    int actual_width,
    int actual_height,
    int actual_fps,
    int publish_fps,
    size_t output_size,
    uint64_t published_count,
    uint64_t stale_published_count,
    uint64_t input_frame_count,
    uint64_t capture_timeout_count,
    uint64_t null_depth_count,
    uint64_t bad_frame_count,
    uint16_t min_mm,
    uint16_t max_mm,
    uint16_t center_mm,
    uint64_t valid_count,
    double convert_ms,
    double write_ms,
    double capture_fps,
    double writer_fps,
    uint64_t frame_age_ms,
    const std::string &status,
    const std::string &last_error
) {
    std::ostringstream meta;

    meta << "{";
    meta << "\"ok\":true,";
    meta << "\"status\":\"" << json_escape(status) << "\",";
    meta << "\"width\":" << frame.width << ",";
    meta << "\"height\":" << frame.height << ",";
    meta << "\"requested_width\":" << req_width << ",";
    meta << "\"requested_height\":" << req_height << ",";
    meta << "\"requested_fps\":" << req_fps << ",";
    meta << "\"actual_width\":" << actual_width << ",";
    meta << "\"actual_height\":" << actual_height << ",";
    meta << "\"actual_fps\":" << actual_fps << ",";
    meta << "\"publish_fps\":" << publish_fps << ",";
    meta << "\"data_size\":" << output_size << ",";
    meta << "\"timestamp_ms\":" << wall_now_ms() << ",";
    meta << "\"steady_timestamp_ms\":" << steady_now_ms() << ",";
    meta << "\"captured_timestamp_ms\":" << frame.captured_wall_ms << ",";
    meta << "\"frame_age_ms\":" << frame_age_ms << ",";
    meta << "\"frame_count\":" << published_count << ",";
    meta << "\"input_frame_count\":" << input_frame_count << ",";
    meta << "\"stale_published_count\":" << stale_published_count << ",";
    meta << "\"capture_timeout_count\":" << capture_timeout_count << ",";
    meta << "\"null_depth_count\":" << null_depth_count << ",";
    meta << "\"bad_frame_count\":" << bad_frame_count << ",";
    meta << "\"frame_index\":" << frame.frame_index << ",";
    meta << "\"device_timestamp_us\":" << frame.device_timestamp_us << ",";
    meta << "\"format\":" << static_cast<int>(frame.format) << ",";
    meta << "\"raw_value_scale\":" << frame.scale << ",";
    meta << "\"output_unit\":\"mm_uint16\",";
    meta << "\"min_mm\":" << min_mm << ",";
    meta << "\"max_mm\":" << max_mm << ",";
    meta << "\"center_mm\":" << center_mm << ",";
    meta << "\"valid_count\":" << valid_count << ",";
    meta << "\"convert_ms\":" << convert_ms << ",";
    meta << "\"write_ms\":" << write_ms << ",";
    meta << "\"capture_fps\":" << capture_fps << ",";
    meta << "\"writer_fps\":" << writer_fps << ",";
    meta << "\"last_error\":\"" << json_escape(last_error) << "\"";
    meta << "}";

    return meta.str();
}

/*
 * Thread de captura: faz apenas waitForFrames + copia o frame mais recente.
 * Ela nao converte e nao escreve arquivo. Isso evita que I/O e conversao travem
 * a leitura do sensor e causem backlog/timeouts no SDK.
 */
static void capture_loop(
    ob::Pipeline *pipe,
    SharedState *state,
    int timeout_ms,
    std::atomic<bool> *local_running
) {
    uint64_t fps_window_ms = steady_now_ms();
    uint64_t fps_window_frames = 0;

    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->capture_running = true;
    }

    while(running && local_running->load()) {
        try {
            auto frame_set = pipe->waitForFrames(static_cast<uint32_t>(timeout_ms));

            if(frame_set == nullptr) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->capture_timeout_count++;
                continue;
            }

            auto depth_frame = frame_set->depthFrame();

            if(depth_frame == nullptr) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->null_depth_count++;
                continue;
            }

            const uint32_t width = depth_frame->width();
            const uint32_t height = depth_frame->height();
            const OBFormat format = depth_frame->format();
            void *data = depth_frame->data();
            const size_t data_size = depth_frame->dataSize();

            if(data == nullptr || width == 0 || height == 0 || data_size == 0) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->bad_frame_count++;
                continue;
            }

            if(format != OB_FORMAT_Y16) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->bad_frame_count++;
                state->last_error = "formato depth nao Y16: " + std::to_string(static_cast<int>(format));
                continue;
            }

            const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);

            if(data_size < pixel_count * sizeof(uint16_t)) {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->bad_frame_count++;
                state->last_error = "data_size menor que esperado";
                continue;
            }

            float scale = depth_frame->getValueScale();
            if(scale <= 0.0f) {
                scale = 1.0f;
            }

            DepthFrameCopy copy;
            copy.valid = true;
            copy.width = width;
            copy.height = height;
            copy.format = format;
            copy.scale = scale;
            copy.data_size = pixel_count * sizeof(uint16_t);
            copy.frame_index = depth_frame->index();
            copy.device_timestamp_us = depth_frame->timeStampUs();
            copy.captured_steady_ms = steady_now_ms();
            copy.captured_wall_ms = wall_now_ms();
            copy.raw_y16.resize(pixel_count);

            std::memcpy(copy.raw_y16.data(), data, pixel_count * sizeof(uint16_t));

            {
                std::lock_guard<std::mutex> lock(state->mutex);
                state->input_frame_count++;
                copy.input_frame_count = state->input_frame_count;
                state->latest = std::move(copy);
                state->last_new_frame_steady_ms = steady_now_ms();
                state->last_error.clear();

                fps_window_frames++;
                const uint64_t now = steady_now_ms();

                if(now - fps_window_ms >= 1000) {
                    state->capture_fps = static_cast<double>(fps_window_frames) * 1000.0 /
                                         static_cast<double>(std::max<uint64_t>(1, now - fps_window_ms));
                    fps_window_ms = now;
                    fps_window_frames = 0;
                }
            }

            state->cv.notify_all();
        }
        catch(ob::Error &e) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->last_error = std::string("capture OrbbecSDK: ") + e.getMessage();
            state->bad_frame_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        catch(std::exception &e) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->last_error = std::string("capture exception: ") + e.what();
            state->bad_frame_count++;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->capture_running = false;
    }

    state->cv.notify_all();
}

/*
 * Thread de publicacao: converte e escreve em FPS controlado.
 * Se a captura tiver microtravadas, ela pode republicar o ultimo frame por
 * alguns segundos para o Python nao ficar reiniciando o bridge sem necessidade.
 */
static void writer_loop(
    SharedState *state,
    const std::string &raw_path,
    const std::string &meta_path,
    int req_width,
    int req_height,
    int req_fps,
    int actual_width,
    int actual_height,
    int actual_fps,
    int publish_fps,
    int stale_republish_ms,
    std::atomic<bool> *local_running
) {
    const uint64_t publish_period_ms =
        publish_fps > 0 ? static_cast<uint64_t>(1000 / std::max(1, publish_fps)) : 0;

    uint64_t next_publish_ms = steady_now_ms();
    uint64_t last_published_frame_index = std::numeric_limits<uint64_t>::max();

    uint64_t fps_window_ms = steady_now_ms();
    uint64_t fps_window_frames = 0;

    std::vector<uint16_t> depth_mm;

    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->writer_running = true;
    }

    while(running && local_running->load()) {
        const uint64_t now = steady_now_ms();

        if(publish_period_ms > 0 && now < next_publish_ms) {
            std::unique_lock<std::mutex> lock(state->mutex);
            state->cv.wait_for(lock, std::chrono::milliseconds(std::min<uint64_t>(10, next_publish_ms - now)));
            continue;
        }

        if(publish_period_ms > 0) {
            const uint64_t after_wait = steady_now_ms();

            if(after_wait > next_publish_ms + publish_period_ms * 2) {
                next_publish_ms = after_wait + publish_period_ms;
            }
            else {
                next_publish_ms += publish_period_ms;
            }
        }

        DepthFrameCopy frame;
        uint64_t input_frame_count = 0;
        uint64_t capture_timeout_count = 0;
        uint64_t null_depth_count = 0;
        uint64_t bad_frame_count = 0;
        uint64_t stale_published_count = 0;
        uint64_t published_count = 0;
        double capture_fps = 0.0;
        double writer_fps = 0.0;
        std::string last_error;

        {
            std::lock_guard<std::mutex> lock(state->mutex);

            if(!state->latest.valid) {
                continue;
            }

            frame = state->latest;
            input_frame_count = state->input_frame_count;
            capture_timeout_count = state->capture_timeout_count;
            null_depth_count = state->null_depth_count;
            bad_frame_count = state->bad_frame_count;
            stale_published_count = state->stale_published_count;
            published_count = state->published_count;
            capture_fps = state->capture_fps;
            writer_fps = state->writer_fps;
            last_error = state->last_error;
        }

        const uint64_t frame_age_ms =
            frame.captured_steady_ms > 0 ? steady_now_ms() - frame.captured_steady_ms : 0;

        bool stale = false;

        if(frame.frame_index == last_published_frame_index) {
            stale = true;
        }

        if(stale && frame_age_ms > static_cast<uint64_t>(stale_republish_ms)) {
            // Evita escrever para sempre frame muito antigo. Se o Python ainda
            // mostrar imagem antiga, o meta indicara idade alta.
            continue;
        }

        uint16_t min_mm = 0;
        uint16_t max_mm = 0;
        uint16_t center_mm = 0;
        uint64_t valid_count = 0;

        const uint64_t convert_start = steady_now_ms();

        convert_y16_to_mm(
            frame.raw_y16.data(),
            frame.width,
            frame.height,
            frame.scale,
            depth_mm,
            min_mm,
            max_mm,
            center_mm,
            valid_count
        );

        const uint64_t convert_end = steady_now_ms();

        const size_t output_size = depth_mm.size() * sizeof(uint16_t);

        const bool raw_ok = write_binary_atomic(raw_path, depth_mm.data(), output_size);
        const uint64_t write_end = steady_now_ms();

        if(!raw_ok) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->last_error = "falha gravando raw";
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->published_count++;

            if(stale) {
                state->stale_published_count++;
            }

            state->last_publish_frame_index = frame.frame_index;

            fps_window_frames++;
            const uint64_t fps_now = steady_now_ms();

            if(fps_now - fps_window_ms >= 1000) {
                state->writer_fps = static_cast<double>(fps_window_frames) * 1000.0 /
                                    static_cast<double>(std::max<uint64_t>(1, fps_now - fps_window_ms));
                fps_window_ms = fps_now;
                fps_window_frames = 0;
            }

            stale_published_count = state->stale_published_count;
            published_count = state->published_count;
            writer_fps = state->writer_fps;
        }

        const std::string status = stale ? "stale_republish" : "ok";

        const std::string meta_json = make_meta_json(
            frame,
            req_width,
            req_height,
            req_fps,
            actual_width,
            actual_height,
            actual_fps,
            publish_fps,
            output_size,
            published_count,
            stale_published_count,
            input_frame_count,
            capture_timeout_count,
            null_depth_count,
            bad_frame_count,
            min_mm,
            max_mm,
            center_mm,
            valid_count,
            static_cast<double>(convert_end - convert_start),
            static_cast<double>(write_end - convert_end),
            capture_fps,
            writer_fps,
            frame_age_ms,
            status,
            last_error
        );

        if(!write_text_atomic(meta_path, meta_json)) {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->last_error = "falha gravando meta";
            continue;
        }

        last_published_frame_index = frame.frame_index;

        if(published_count % 30 == 0) {
            std::cerr << "[bridge] publish=" << published_count
                      << " input=" << input_frame_count
                      << " stale=" << stale_published_count
                      << " size=" << frame.width << "x" << frame.height
                      << " center_mm=" << center_mm
                      << " min_mm=" << min_mm
                      << " max_mm=" << max_mm
                      << " valid=" << valid_count
                      << " age_ms=" << frame_age_ms
                      << " convert_ms=" << (convert_end - convert_start)
                      << " write_ms=" << (write_end - convert_end)
                      << " capture_fps=" << capture_fps
                      << " writer_fps=" << writer_fps
                      << " status=" << status
                      << std::endl;
        }
    }

    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->writer_running = false;
    }
}

static int run_once(
    const std::string &raw_path,
    const std::string &meta_path,
    int req_width,
    int req_height,
    int req_fps,
    int timeout_ms,
    int publish_fps,
    int restart_after_ms,
    int stale_republish_ms,
    bool show_profiles
) {
    std::unique_ptr<ob::Pipeline> pipe;

    int actual_width = 0;
    int actual_height = 0;
    int actual_fps = 0;

    const bool started = start_pipeline_depth(
        pipe,
        req_width,
        req_height,
        req_fps,
        actual_width,
        actual_height,
        actual_fps,
        show_profiles
    );

    if(!started) {
        std::cerr << "[bridge] nao conseguiu iniciar nenhum perfil depth" << std::endl;
        return 5;
    }

    std::cerr << "[bridge] depth iniciado" << std::endl;

    SharedState state;
    state.last_new_frame_steady_ms = steady_now_ms();

    std::atomic<bool> local_running(true);

    std::thread capture_thread(capture_loop, pipe.get(), &state, timeout_ms, &local_running);
    std::thread writer_thread(
        writer_loop,
        &state,
        raw_path,
        meta_path,
        req_width,
        req_height,
        req_fps,
        actual_width,
        actual_height,
        actual_fps,
        publish_fps,
        stale_republish_ms,
        &local_running
    );

    uint64_t last_watchdog_log_ms = 0;

    while(running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        uint64_t last_new = 0;
        uint64_t input_count = 0;
        uint64_t timeout_count = 0;
        uint64_t published_count = 0;
        std::string last_error;

        {
            std::lock_guard<std::mutex> lock(state.mutex);
            last_new = state.last_new_frame_steady_ms;
            input_count = state.input_frame_count;
            timeout_count = state.capture_timeout_count;
            published_count = state.published_count;
            last_error = state.last_error;
        }

        const uint64_t now = steady_now_ms();
        const uint64_t elapsed = last_new > 0 ? now - last_new : 0;

        if(now - last_watchdog_log_ms >= 3000) {
            last_watchdog_log_ms = now;

            std::cerr << "[bridge] watchdog input=" << input_count
                      << " published=" << published_count
                      << " timeouts=" << timeout_count
                      << " elapsed_no_new_ms=" << elapsed
                      << " last_error=" << last_error
                      << std::endl;
        }

        if(restart_after_ms > 0 && elapsed > static_cast<uint64_t>(restart_after_ms)) {
            std::cerr << "[bridge] sem frame novo por "
                      << elapsed
                      << "ms. Reiniciando pipeline com cooldown suave."
                      << std::endl;
            break;
        }
    }

    local_running = false;
    state.cv.notify_all();

    if(capture_thread.joinable()) {
        capture_thread.join();
    }

    if(writer_thread.joinable()) {
        writer_thread.join();
    }

    try {
        pipe->stop();
    }
    catch(...) {
    }

    return running ? 5 : 0;
}

int main(int argc, char **argv) {
    std::string raw_path = "/tmp/orbbec_depth.raw";
    std::string meta_path = "/tmp/orbbec_depth_meta.json";

    if(argc >= 2) raw_path = argv[1];
    if(argc >= 3) meta_path = argv[2];

    int req_width = argc >= 4 ? to_int(argv[3], 320) : env_int("ORBBEC_DEPTH_WIDTH", 320);
    int req_height = argc >= 5 ? to_int(argv[4], 240) : env_int("ORBBEC_DEPTH_HEIGHT", 240);

    // Padrão seguro: 320x240@30 reduz carga USB e evita fallback cego para 160x120.
    int req_fps = argc >= 6 ? to_int(argv[5], 30) : env_int("ORBBEC_DEPTH_FPS", 30);

    int timeout_ms = argc >= 7 ? to_int(argv[6], 1000) : env_int("ORBBEC_DEPTH_TIMEOUT_MS", 1000);
    int publish_fps = argc >= 8 ? to_int(argv[7], 15) : env_int("ORBBEC_DEPTH_PUBLISH_FPS", 15);

    // Reinicia somente depois de muito tempo sem frame novo.
    // Isso evita apagar/reacender o sensor por microtravadas.
    int restart_after_ms = argc >= 9 ? to_int(argv[8], 45000) : env_int("ORBBEC_DEPTH_RESTART_AFTER_MS", 45000);

    // Durante pequenas quedas, republica ultimo frame para manter Python/UI estaveis.
    int stale_republish_ms = argc >= 10 ? to_int(argv[9], 20000) : env_int("ORBBEC_DEPTH_STALE_REPUBLISH_MS", 20000);

    int max_attempts = argc >= 11 ? to_int(argv[10], 0) : env_int("ORBBEC_DEPTH_MAX_ATTEMPTS", 0);

    if(req_width <= 0) req_width = OB_WIDTH_ANY;
    if(req_height <= 0) req_height = OB_HEIGHT_ANY;
    if(req_fps <= 0) req_fps = OB_FPS_ANY;
    if(timeout_ms < 20) timeout_ms = 20;
    if(publish_fps < 0) publish_fps = 0;
    if(restart_after_ms < 0) restart_after_ms = 0;
    if(stale_republish_ms < 0) stale_republish_ms = 0;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    std::cerr << "[bridge] OrbbecSDK depth bridge BANDWIDTH SAFE" << std::endl;
    std::cerr << "[bridge] raw=" << raw_path << std::endl;
    std::cerr << "[bridge] meta=" << meta_path << std::endl;
    std::cerr << "[bridge] requested=" << req_width << "x" << req_height
              << "@" << req_fps
              << " timeout_ms=" << timeout_ms
              << " publish_fps=" << publish_fps
              << " restart_after_ms=" << restart_after_ms
              << " stale_republish_ms=" << stale_republish_ms
              << " max_attempts=" << max_attempts
              << std::endl;

    int attempt = 0;
    int cooldown_ms = 1200;

    while(running) {
        attempt++;

        try {
            std::cerr << "[bridge] tentativa=" << attempt << std::endl;

            const bool show_profiles = (attempt == 1);

            const int rc = run_once(
                raw_path,
                meta_path,
                req_width,
                req_height,
                req_fps,
                timeout_ms,
                publish_fps,
                restart_after_ms,
                stale_republish_ms,
                show_profiles
            );

            if(rc == 0) {
                return 0;
            }
        }
        catch(ob::Error &e) {
            std::cerr << "[bridge] erro OrbbecSDK" << std::endl;
            std::cerr << "function: " << e.getName() << std::endl;
            std::cerr << "args: " << e.getArgs() << std::endl;
            std::cerr << "message: " << e.getMessage() << std::endl;
            std::cerr << "type: " << e.getExceptionType() << std::endl;
        }
        catch(std::exception &e) {
            std::cerr << "[bridge] std::exception: " << e.what() << std::endl;
        }
        catch(...) {
            std::cerr << "[bridge] excecao desconhecida" << std::endl;
        }

        if(max_attempts > 0 && attempt >= max_attempts) {
            std::cerr << "[bridge] falhou apos max_attempts=" << max_attempts << std::endl;
            return 2;
        }

        if(!running) {
            break;
        }

        std::cerr << "[bridge] cooldown antes de nova tentativa: "
                  << cooldown_ms << "ms" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(cooldown_ms));

        cooldown_ms = std::min(8000, cooldown_ms + 800);
    }

    return 0;
}
