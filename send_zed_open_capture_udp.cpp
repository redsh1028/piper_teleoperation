#define VIDEO_MOD_AVAILABLE
#include "videocapture.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

namespace {

constexpr const char* MAGIC = "RSIMG1";

struct Address {
    sockaddr_storage storage{};
    socklen_t length = 0;
    std::string text;
};

struct Options {
    std::string host = "172.16.64.161";
    int port = 5020;
    int device_id = -1;
    std::string resolution = "HD720";
    int fps = 30;
    std::string crop = "right-half";
    int jpeg_quality = 60;
    int max_packet_size = 1400;
};

std::vector<std::string> split(const std::string& text, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        if (!item.empty()) {
            parts.push_back(item);
        }
    }
    return parts;
}

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [--host IP[,IP...]] [--port PORT]\n"
        << "  [--device-id N] [--resolution HD720|HD1080|HD2K|VGA]\n"
        << "  [--fps 15|30|60|100] [--crop none|left-half|right-half]\n"
        << "  [--jpeg-quality 1..100] [--max-packet-size BYTES]\n";
}

Options parse_args(int argc, char** argv) {
    Options opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(name + " requires a value");
            }
            return argv[++i];
        };

        if (arg == "--host") {
            opts.host = require_value(arg);
        } else if (arg == "--port") {
            opts.port = std::stoi(require_value(arg));
        } else if (arg == "--device-id") {
            opts.device_id = std::stoi(require_value(arg));
        } else if (arg == "--resolution") {
            opts.resolution = require_value(arg);
        } else if (arg == "--fps") {
            opts.fps = std::stoi(require_value(arg));
        } else if (arg == "--crop") {
            opts.crop = require_value(arg);
        } else if (arg == "--jpeg-quality") {
            opts.jpeg_quality = std::stoi(require_value(arg));
        } else if (arg == "--max-packet-size") {
            opts.max_packet_size = std::stoi(require_value(arg));
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (opts.port <= 0 || opts.port > 65535) {
        throw std::runtime_error("--port must be in 1..65535");
    }
    if (opts.jpeg_quality < 1 || opts.jpeg_quality > 100) {
        throw std::runtime_error("--jpeg-quality must be in 1..100");
    }
    if (opts.max_packet_size < 512) {
        throw std::runtime_error("--max-packet-size must be at least 512");
    }
    if (opts.crop != "none" && opts.crop != "left-half" && opts.crop != "right-half") {
        throw std::runtime_error("--crop must be none, left-half, or right-half");
    }
    return opts;
}

sl_oc::video::RESOLUTION parse_resolution(const std::string& value) {
    if (value == "HD2K") return sl_oc::video::RESOLUTION::HD2K;
    if (value == "HD1080") return sl_oc::video::RESOLUTION::HD1080;
    if (value == "HD720") return sl_oc::video::RESOLUTION::HD720;
    if (value == "VGA") return sl_oc::video::RESOLUTION::VGA;
    throw std::runtime_error("Unsupported resolution: " + value);
}

sl_oc::video::FPS parse_fps(int value) {
    if (value == 15) return sl_oc::video::FPS::FPS_15;
    if (value == 30) return sl_oc::video::FPS::FPS_30;
    if (value == 60) return sl_oc::video::FPS::FPS_60;
    if (value == 100) return sl_oc::video::FPS::FPS_100;
    throw std::runtime_error("Unsupported fps");
}

std::vector<Address> parse_addresses(const std::string& hosts, int port) {
    std::vector<Address> addresses;
    for (const auto& host : split(hosts, ',')) {
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;

        addrinfo* result = nullptr;
        int rc = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &result);
        if (rc != 0) {
            throw std::runtime_error("getaddrinfo failed for " + host + ": " + gai_strerror(rc));
        }

        Address address;
        std::memcpy(&address.storage, result->ai_addr, result->ai_addrlen);
        address.length = static_cast<socklen_t>(result->ai_addrlen);
        address.text = host + ":" + std::to_string(port);
        addresses.push_back(address);
        freeaddrinfo(result);
    }
    if (addresses.empty()) {
        throw std::runtime_error("--host must contain at least one address");
    }
    return addresses;
}

double now_seconds() {
    using clock = std::chrono::system_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

cv::Mat crop_image(const cv::Mat& image, const std::string& crop) {
    if (crop == "none") {
        return image;
    }
    int half_width = image.cols / 2;
    if (crop == "left-half") {
        return image(cv::Rect(0, 0, half_width, image.rows));
    }
    return image(cv::Rect(image.cols - half_width, 0, half_width, image.rows));
}

std::string json_header(
    uint64_t frame_id,
    double timestamp,
    int chunk_index,
    int chunk_count,
    int total_size,
    int payload_size,
    int width,
    int height) {
    std::ostringstream out;
    out << "{\"magic\":\"" << MAGIC << "\""
        << ",\"frame_id\":" << frame_id
        << ",\"timestamp\":" << timestamp
        << ",\"chunk_index\":" << chunk_index
        << ",\"chunk_count\":" << chunk_count
        << ",\"total_size\":" << total_size
        << ",\"payload_size\":" << payload_size
        << ",\"stream\":\"color\""
        << ",\"encoding\":\"jpeg\""
        << ",\"width\":" << width
        << ",\"height\":" << height
        << ",\"format\":\"bgr8\"}";
    return out.str();
}

void send_image(
    int sock,
    const std::vector<Address>& addresses,
    uint64_t frame_id,
    double timestamp,
    const std::vector<uchar>& encoded,
    int width,
    int height,
    int max_packet_size) {
    int chunk_size = max_packet_size - 512;
    int total_size = static_cast<int>(encoded.size());
    int chunk_count = static_cast<int>(std::ceil(total_size / static_cast<double>(chunk_size)));

    for (int chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        int offset = chunk_index * chunk_size;
        int payload_size = std::min(chunk_size, total_size - offset);
        std::string header = json_header(
            frame_id,
            timestamp,
            chunk_index,
            chunk_count,
            total_size,
            payload_size,
            width,
            height);

        std::vector<uint8_t> packet;
        packet.reserve(header.size() + 1 + payload_size);
        packet.insert(packet.end(), header.begin(), header.end());
        packet.push_back('\n');
        packet.insert(packet.end(), encoded.begin() + offset, encoded.begin() + offset + payload_size);

        for (const auto& address : addresses) {
            sendto(
                sock,
                packet.data(),
                packet.size(),
                0,
                reinterpret_cast<const sockaddr*>(&address.storage),
                address.length);
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Options opts = parse_args(argc, argv);
        std::vector<Address> addresses = parse_addresses(opts.host, opts.port);

        sl_oc::video::VideoParams params;
        params.res = parse_resolution(opts.resolution);
        params.fps = parse_fps(opts.fps);
        params.verbose = sl_oc::VERBOSITY::INFO;

        sl_oc::video::VideoCapture cap(params);
        if (!cap.initializeVideo(opts.device_id)) {
            std::cerr << "Cannot initialize ZED Open Capture video" << std::endl;
            return EXIT_FAILURE;
        }

        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            throw std::runtime_error("Failed to create UDP socket");
        }

        std::cout << "ZED Open Capture started: sn=" << cap.getSerialNumber()
                  << " name=" << cap.getDeviceName()
                  << " resolution=" << opts.resolution
                  << " fps=" << opts.fps
                  << " crop=" << opts.crop
                  << " to ";
        for (size_t i = 0; i < addresses.size(); ++i) {
            if (i) std::cout << ",";
            std::cout << addresses[i].text;
        }
        std::cout << std::endl;

        uint64_t last_timestamp = 0;
        while (true) {
            const sl_oc::video::Frame& frame = cap.getLastFrame(500);
            if (frame.data == nullptr || frame.timestamp == 0 || frame.timestamp == last_timestamp) {
                continue;
            }
            last_timestamp = frame.timestamp;

            cv::Mat yuv(frame.height, frame.width, CV_8UC2, frame.data);
            cv::Mat bgr;
            cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_YUYV);
            cv::Mat image = crop_image(bgr, opts.crop);

            std::vector<uchar> encoded;
            std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, opts.jpeg_quality};
            if (!cv::imencode(".jpg", image, encoded, encode_params)) {
                std::cerr << "Warning: failed to JPEG encode frame" << std::endl;
                continue;
            }

            send_image(
                sock,
                addresses,
                frame.frame_id,
                now_seconds(),
                encoded,
                image.cols,
                image.rows,
                opts.max_packet_size);
        }

        close(sock);
        return EXIT_SUCCESS;
    } catch (const std::exception& exc) {
        std::cerr << "Error: " << exc.what() << std::endl;
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }
}
