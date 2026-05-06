/*
 * video_sender_pc.cpp  —  PC webcam H.264 TCP video sender
 * Compatible with XRoboToolkit Unity Client "Remote Vision"
 *
 * Protocol: [4-byte big-endian length][H264 encoded frame]
 *
 * Usage:
 *   # Listen mode (VR connects to PC):
 *   ./VideoSenderPC --listen 0.0.0.0:13579
 *
 *   # ROS2 mode (read from image topics, coexists with AnyCam2Ros):
 *   ./VideoSenderPC --listen 0.0.0.0:13579 --ros \
 *     --topic /cam/head/color/image_raw \
 *     --topic2 /cam/left_wrist/color/image_raw \
 *     --topic3 /cam/right_wrist/color/image_raw
 *
 *   # Direct send mode:
 *   ./VideoSenderPC --send --server <VR_IP> --port 12345
 *
 * Build:
 *   mkdir -p build && cd build && cmake .. && make
 */

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>

#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#endif

// ─── globals ────────────────────────────────────────────────────────────────
static std::atomic<bool> g_stop{false};
static std::atomic<int>  g_client_fd{-1};   // listen mode: accepted client

static void sig_handler(int) { g_stop = true; }

// ─── TCP helpers ────────────────────────────────────────────────────────────
static bool send_all(int fd, const uint8_t *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(fd, buf + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) return false;
        sent += n;
    }
    return true;
}

static bool send_frame(int fd, const uint8_t *data, size_t size) {
    uint8_t hdr[4];
    hdr[0] = (size >> 24) & 0xFF;
    hdr[1] = (size >> 16) & 0xFF;
    hdr[2] = (size >>  8) & 0xFF;
    hdr[3] = (size      ) & 0xFF;
    if (!send_all(fd, hdr, 4)) return false;
    return send_all(fd, data, size);
}

// ─── Listen mode: TCP server + command parsing ──────────────────────────────
struct CamConfig {
    int width = 1280, height = 720, fps = 30, bitrate = 4000000;
    std::string ip;
    int port = 12345;
};

static int32_t read_le32(const uint8_t *p) {
    return p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24);
}

static std::string read_compact_str(const uint8_t *&p, const uint8_t *end) {
    if (p >= end) return {};
    uint8_t len = *p++;
    if (p + len > end) return {};
    std::string s(reinterpret_cast<const char*>(p), len);
    p += len;
    return s;
}

static bool parse_open_camera(const uint8_t *data, size_t sz, CamConfig &cfg) {
    // Magic 0xCA 0xFE, version 1, then 7 int32 LE, then 2 compact strings
    if (sz < 31) return false;
    if (data[0] != 0xCA || data[1] != 0xFE || data[2] != 1) return false;
    const uint8_t *p = data + 3;
    cfg.width       = read_le32(p);      p += 4;
    cfg.height      = read_le32(p);      p += 4;
    cfg.fps         = read_le32(p);      p += 4;
    cfg.bitrate     = read_le32(p);      p += 4;
    /* enableMvHevc */ read_le32(p);      p += 4;
    /* renderMode  */ read_le32(p);      p += 4;
    cfg.port        = read_le32(p);      p += 4;
    const uint8_t *end = data + sz;
    /* camera type */ read_compact_str(p, end);
    cfg.ip = read_compact_str(p, end);
    return !cfg.ip.empty();
}

// Parse the command sent by VR in listen mode
static bool parse_command(const uint8_t *raw, size_t raw_sz, CamConfig &cfg) {
    if (raw_sz < 4) return false;
    uint32_t body_len = (uint32_t(raw[0])<<24) | (uint32_t(raw[1])<<16)
                      | (uint32_t(raw[2])<<8)  | uint32_t(raw[3]);
    if (4 + body_len > raw_sz) return false;
    const uint8_t *body = raw + 4;
    if (body_len < 8) return false;

    // NetworkDataProtocol: [4-byte LE cmd_len][cmd string][4-byte LE data_len][data]
    int32_t cmd_len = read_le32(body);
    if (cmd_len < 0 || size_t(4 + cmd_len + 4) > body_len) return false;
    std::string cmd(reinterpret_cast<const char*>(body+4), cmd_len);
    size_t doff = 4 + cmd_len;
    int32_t data_len = read_le32(body + doff);
    doff += 4;
    if (data_len < 0 || doff + data_len > body_len) return false;

    if (cmd.find("OPEN_CAMERA") != std::string::npos) {
        return parse_open_camera(body + doff, data_len, cfg);
    }
    return false;
}

// ─── Encoder (FFmpeg) ───────────────────────────────────────────────────────
struct Encoder {
    const AVCodec    *codec = nullptr;
    AVCodecContext   *ctx   = nullptr;
    AVFrame          *frame = nullptr;
    AVPacket         *pkt   = nullptr;
    SwsContext       *sws   = nullptr;

    bool try_open(int w, int h, int fps, int bitrate, const char *enc_name) {
        codec = avcodec_find_encoder_by_name(enc_name);
        if (!codec) return false;

        ctx = avcodec_alloc_context3(codec);
        ctx->width     = w;
        ctx->height    = h;
        ctx->time_base = {1, fps};
        ctx->framerate = {fps, 1};
        ctx->pix_fmt   = AV_PIX_FMT_YUV420P;
        ctx->bit_rate  = bitrate;
        ctx->gop_size  = fps;
        ctx->max_b_frames = 0;

        if (std::string(enc_name) == "libx264") {
            av_opt_set(ctx->priv_data, "preset", "ultrafast", 0);
            av_opt_set(ctx->priv_data, "tune",   "zerolatency", 0);
        } else {
            av_opt_set(ctx->priv_data, "preset", "p1", 0);
            av_opt_set(ctx->priv_data, "tune",   "ull", 0);
            av_opt_set(ctx->priv_data, "rc",     "cbr", 0);
        }

        if (avcodec_open2(ctx, codec, nullptr) < 0) {
            avcodec_free_context(&ctx); ctx = nullptr; codec = nullptr;
            return false;
        }
        return true;
    }

    bool init(int w, int h, int fps, int bitrate, bool use_nvenc) {
        if (use_nvenc && try_open(w, h, fps, bitrate, "h264_nvenc")) {
            // nvenc OK
        } else {
            if (use_nvenc) std::cerr << "h264_nvenc failed, falling back to libx264\n";
            if (!try_open(w, h, fps, bitrate, "libx264")) {
                std::cerr << "No working H264 encoder\n"; return false;
            }
        }
        frame = av_frame_alloc();
        frame->format = ctx->pix_fmt;
        frame->width  = w;
        frame->height = h;
        av_frame_get_buffer(frame, 0);
        pkt = av_packet_alloc();

        sws = sws_getContext(w, h, AV_PIX_FMT_BGR24,
                             w, h, AV_PIX_FMT_YUV420P,
                             SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
        std::cout << "Encoder: " << codec->name << " " << w << "x" << h
                  << " @" << fps << "fps " << bitrate/1000 << "kbps\n";
        return true;
    }

    // Encode one BGR frame; returns encoded packets via callback
    void encode(const cv::Mat &bgr, int64_t pts,
                const std::function<void(const uint8_t*, size_t)> &on_packet) {
        av_frame_make_writable(frame);
        const uint8_t *src[1] = { bgr.data };
        int src_stride[1] = { (int)bgr.step };
        sws_scale(sws, src, src_stride, 0, bgr.rows,
                  frame->data, frame->linesize);
        frame->pts = pts;

        avcodec_send_frame(ctx, frame);
        while (avcodec_receive_packet(ctx, pkt) == 0) {
            on_packet(pkt->data, pkt->size);
            av_packet_unref(pkt);
        }
    }

    ~Encoder() {
        if (sws) sws_freeContext(sws);
        if (pkt) av_packet_free(&pkt);
        if (frame) av_frame_free(&frame);
        if (ctx) avcodec_free_context(&ctx);
    }
};

// ─── Thread-safe latest-frame buffer (for ROS2 mode) ────────────────────────
struct FrameBuffer {
    std::mutex mtx;
    cv::Mat frame;
    bool fresh = false;
    void push(const cv::Mat &f) { std::lock_guard<std::mutex> lk(mtx); f.copyTo(frame); fresh = true; }
    bool pop(cv::Mat &out) { std::lock_guard<std::mutex> lk(mtx); if (!fresh) return false; frame.copyTo(out); fresh = false; return true; }
};

// ─── Helper: open a V4L2 camera with MJPG ──────────────────────────────────
static bool open_cam(cv::VideoCapture &cap, int dev, int w, int h, int fps) {
    cap.open(dev, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (!cap.isOpened()) { std::cerr << "Cannot open /dev/video" << dev << "\n"; return false; }
    return true;
}

// ─── Streaming thread ───────────────────────────────────────────────────────
static void stream_to(int fd, int cam_dev, int cam_dev2, int cam_dev3,
                      int w, int h, int fps, int bitrate,
                      bool preview, bool use_nvenc) {
    // Side-by-side stereo: if width > 2*height, capture at half width and duplicate
    bool sbs = (w > h * 2 - 100);  // e.g. 2560x720 → capture 1280x720, duplicate
    int cap_w = sbs ? w / 2 : w;
    int cap_h = h;

    // Main camera
    cv::VideoCapture cap;
    if (!open_cam(cap, cam_dev, cap_w, cap_h, fps)) return;

    // Extra gripper cameras
    bool has_cam2 = (cam_dev2 >= 0), has_cam3 = (cam_dev3 >= 0);
    cv::VideoCapture cap2, cap3;
    if (has_cam2 && !open_cam(cap2, cam_dev2, 640, 480, fps)) has_cam2 = false;
    if (has_cam3 && !open_cam(cap3, cam_dev3, 640, 480, fps)) has_cam3 = false;
    int n_extra = (has_cam2 ? 1 : 0) + (has_cam3 ? 1 : 0);
    bool grid = (n_extra > 0);

    // Grid layout within each eye:
    //   Top ~2/3: main camera    Bottom ~1/3: gripper cameras side-by-side
    int main_h = grid ? cap_h * 2 / 3 : cap_h;
    int sub_h  = cap_h - main_h;
    int sub_w  = (n_extra == 2) ? cap_w / 2 : cap_w;

    Encoder enc;
    if (!enc.init(w, h, fps, bitrate, use_nvenc)) return;

    if (sbs) std::cout << "Side-by-side mode: capture " << cap_w << "x" << cap_h
                       << " → encode " << w << "x" << h << "\n";
    if (grid) std::cout << "Grid mode: main " << cap_w << "x" << main_h
                        << " + " << n_extra << " sub-cam(s) " << sub_w << "x" << sub_h << "\n";

    cv::Mat frame_bgr, frame_main, frame2, frame3, grid_frame, sbs_frame;
    if (grid) grid_frame.create(cap_h, cap_w, CV_8UC3);
    if (sbs) sbs_frame.create(h, w, CV_8UC3);
    int64_t pts = 0;
    auto t_next = std::chrono::steady_clock::now();
    const auto interval = std::chrono::microseconds(1000000 / fps);

    std::cout << "Streaming to fd=" << fd << " ...\n";
    while (!g_stop) {
        cap >> frame_main;
        if (frame_main.empty()) continue;

        cv::Mat frame_cap;  // the composed single-eye frame

        if (grid) {
            grid_frame = cv::Scalar(0, 0, 0);

            // Main camera → top portion
            cv::Mat resized_main;
            cv::resize(frame_main, resized_main, {cap_w, main_h});
            resized_main.copyTo(grid_frame(cv::Rect(0, 0, cap_w, main_h)));

            // Gripper cameras → bottom portion
            if (has_cam2) {
                cap2 >> frame2;
                if (!frame2.empty()) {
                    cv::Mat r2;
                    cv::resize(frame2, r2, {sub_w, sub_h});
                    cv::putText(r2, "L Gripper", {4, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
                    r2.copyTo(grid_frame(cv::Rect(0, main_h, sub_w, sub_h)));
                }
            }
            if (has_cam3) {
                cap3 >> frame3;
                if (!frame3.empty()) {
                    cv::Mat r3;
                    cv::resize(frame3, r3, {sub_w, sub_h});
                    cv::putText(r3, "R Gripper", {4, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
                    int x_off = has_cam2 ? sub_w : 0;
                    r3.copyTo(grid_frame(cv::Rect(x_off, main_h, sub_w, sub_h)));
                }
            }

            // Draw separator lines (black)
            cv::line(grid_frame, {0, main_h}, {cap_w, main_h}, {0,0,0}, 2);
            if (has_cam2 && has_cam3)
                cv::line(grid_frame, {sub_w, main_h}, {sub_w, cap_h}, {0,0,0}, 2);
            frame_cap = grid_frame;
        } else {
            if (frame_main.cols != cap_w || frame_main.rows != cap_h)
                cv::resize(frame_main, frame_main, {cap_w, cap_h});
            frame_cap = frame_main;
        }

        if (sbs) {
            // Duplicate: left half = right half = same composed image
            frame_cap.copyTo(sbs_frame(cv::Rect(0, 0, cap_w, cap_h)));
            frame_cap.copyTo(sbs_frame(cv::Rect(cap_w, 0, cap_w, cap_h)));
            frame_bgr = sbs_frame;
        } else {
            frame_bgr = frame_cap;
        }

        bool ok = true;
        enc.encode(frame_bgr, pts++, [&](const uint8_t *data, size_t sz) {
            if (ok) ok = send_frame(fd, data, sz);
        });
        if (!ok) { std::cout << "Client disconnected\n"; break; }

        if (preview) {
            cv::imshow("Preview", frame_bgr);
            if (cv::waitKey(1) == 27) { g_stop = true; break; }
        }

        t_next += interval;
        std::this_thread::sleep_until(t_next);
    }
    cap.release();
    if (has_cam2) cap2.release();
    if (has_cam3) cap3.release();
    if (preview) cv::destroyAllWindows();
}

#ifdef USE_ROS2
// ─── ROS2 streaming thread ──────────────────────────────────────────────────
static void stream_to_ros(int fd, const std::string &topic1, const std::string &topic2,
                          const std::string &topic3, int w, int h, int fps, int bitrate,
                          bool preview, bool use_nvenc) {
    bool sbs = (w > h * 2 - 100);
    int cap_w = sbs ? w / 2 : w;
    int cap_h = h;

    bool has_t2 = !topic2.empty(), has_t3 = !topic3.empty();
    int n_extra = (has_t2 ? 1 : 0) + (has_t3 ? 1 : 0);
    bool grid = (n_extra > 0);

    int main_h = grid ? cap_h * 2 / 3 : cap_h;
    int sub_h  = cap_h - main_h;
    int sub_w  = (n_extra == 2) ? cap_w / 2 : cap_w;

    Encoder enc;
    if (!enc.init(w, h, fps, bitrate, use_nvenc)) return;

    if (sbs) std::cout << "Side-by-side mode: " << cap_w << "x" << cap_h
                       << " → encode " << w << "x" << h << "\n";
    if (grid) std::cout << "Grid mode: main " << cap_w << "x" << main_h
                        << " + " << n_extra << " sub-cam(s) " << sub_w << "x" << sub_h << "\n";

    // ROS2 subscribers
    auto node = rclcpp::Node::make_shared("video_sender_pc");
    FrameBuffer buf1, buf2, buf3;

    auto qos = rclcpp::QoS(1).reliable();
    auto sub1 = node->create_subscription<sensor_msgs::msg::Image>(
        topic1, qos, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            buf1.push(cv_ptr->image);
        });
    std::cout << "Subscribed to: " << topic1 << "\n";

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub2, sub3;
    if (has_t2) {
        sub2 = node->create_subscription<sensor_msgs::msg::Image>(
            topic2, qos, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
                auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                buf2.push(cv_ptr->image);
            });
        std::cout << "Subscribed to: " << topic2 << "\n";
    }
    if (has_t3) {
        sub3 = node->create_subscription<sensor_msgs::msg::Image>(
            topic3, qos, [&](const sensor_msgs::msg::Image::SharedPtr msg) {
                auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                buf3.push(cv_ptr->image);
            });
        std::cout << "Subscribed to: " << topic3 << "\n";
    }

    // Spin ROS in a background thread
    std::atomic<bool> session_stop{false};
    std::thread ros_thread([&]() {
        while (rclcpp::ok() && !g_stop && !session_stop) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    cv::Mat frame_bgr, frame_main, frame2, frame3, grid_frame, sbs_frame;
    cv::Mat last_r2, last_r3;  // cached gripper frames for reuse
    if (grid) grid_frame.create(cap_h, cap_w, CV_8UC3);
    if (sbs) sbs_frame.create(h, w, CV_8UC3);
    int64_t pts = 0;
    auto t_next = std::chrono::steady_clock::now();
    const auto interval = std::chrono::microseconds(1000000 / fps);

    std::cout << "Streaming (ROS2) to fd=" << fd << " ...\n";
    while (!g_stop) {
        // Wait for main camera frame
        if (!buf1.pop(frame_main)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        cv::Mat frame_cap;
        if (grid) {
            grid_frame = cv::Scalar(0, 0, 0);
            cv::Mat resized_main;
            cv::resize(frame_main, resized_main, {cap_w, main_h});
            resized_main.copyTo(grid_frame(cv::Rect(0, 0, cap_w, main_h)));

            if (has_t2) {
                cv::Mat tmp2;
                if (buf2.pop(tmp2)) {
                    cv::resize(tmp2, last_r2, {sub_w, sub_h});
                    cv::putText(last_r2, "L Gripper", {4, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
                }
                if (!last_r2.empty())
                    last_r2.copyTo(grid_frame(cv::Rect(0, main_h, sub_w, sub_h)));
            }
            if (has_t3) {
                cv::Mat tmp3;
                if (buf3.pop(tmp3)) {
                    cv::resize(tmp3, last_r3, {sub_w, sub_h});
                    cv::putText(last_r3, "R Gripper", {4, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
                }
                int x_off = has_t2 ? sub_w : 0;
                if (!last_r3.empty())
                    last_r3.copyTo(grid_frame(cv::Rect(x_off, main_h, sub_w, sub_h)));
            }
            cv::line(grid_frame, {0, main_h}, {cap_w, main_h}, {0,0,0}, 2);
            if (has_t2 && has_t3)
                cv::line(grid_frame, {sub_w, main_h}, {sub_w, cap_h}, {0,0,0}, 2);
            frame_cap = grid_frame;
        } else {
            cv::resize(frame_main, frame_main, {cap_w, cap_h});
            frame_cap = frame_main;
        }

        if (sbs) {
            frame_cap.copyTo(sbs_frame(cv::Rect(0, 0, cap_w, cap_h)));
            frame_cap.copyTo(sbs_frame(cv::Rect(cap_w, 0, cap_w, cap_h)));
            frame_bgr = sbs_frame;
        } else {
            frame_bgr = frame_cap;
        }

        bool ok = true;
        enc.encode(frame_bgr, pts++, [&](const uint8_t *data, size_t sz) {
            if (ok) ok = send_frame(fd, data, sz);
        });
        if (!ok) { std::cout << "Client disconnected\n"; break; }

        if (preview) {
            cv::imshow("Preview", frame_bgr);
            if (cv::waitKey(1) == 27) { g_stop = true; break; }
        }
        t_next += interval;
        std::this_thread::sleep_until(t_next);
    }
    // Signal ros_thread to stop, but don't set g_stop (listen_mode needs to continue)
    session_stop = true;
    if (ros_thread.joinable()) ros_thread.join();
    if (preview) cv::destroyAllWindows();
}
#endif

// ─── Listen mode ────────────────────────────────────────────────────────────
static void listen_mode(const std::string &addr, int cam_dev, int cam_dev2, int cam_dev3,
                        const std::string &ros_topic1, const std::string &ros_topic2,
                        const std::string &ros_topic3,
                        int default_w, int default_h, int default_fps, int default_bitrate,
                        bool preview, bool use_nvenc) {
    auto colon = addr.find(':');
    std::string ip = addr.substr(0, colon);
    int port = std::stoi(addr.substr(colon + 1));

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    inet_pton(AF_INET, ip.c_str(), &sa.sin_addr);
    if (bind(srv, (sockaddr*)&sa, sizeof(sa)) < 0) {
        std::cerr << "Bind failed: " << strerror(errno) << "\n"; close(srv); return;
    }
    listen(srv, 1);
    std::cout << "Listening on " << addr << " — waiting for VR headset...\n";

    while (!g_stop) {
        sockaddr_in ca{};
        socklen_t cl = sizeof(ca);
        int cfd = accept(srv, (sockaddr*)&ca, &cl);
        if (cfd < 0) { if (!g_stop) std::cerr << "Accept error\n"; break; }
        std::cout << "VR connected from " << inet_ntoa(ca.sin_addr) << "\n";

        // Read command from VR
        uint8_t buf[2048];
        ssize_t n = recv(cfd, buf, sizeof(buf), 0);
        CamConfig cfg;
        cfg.width = default_w; cfg.height = default_h;
        cfg.fps = default_fps; cfg.bitrate = default_bitrate;

        if (n > 0 && parse_command(buf, n, cfg)) {
            std::cout << "VR requested: " << cfg.width << "x" << cfg.height
                      << " @" << cfg.fps << "fps → " << cfg.ip << ":" << cfg.port << "\n";
            // Use VR's resolution and connection info, but override fps/bitrate from CLI
            cfg.fps = default_fps;
            cfg.bitrate = default_bitrate;
            std::cout << "Streaming at: " << cfg.width << "x" << cfg.height
                      << " @" << cfg.fps << "fps " << cfg.bitrate/1000 << "kbps\n";
            close(cfd);  // close command channel

            // Connect to VR's specified data port
            int data_fd = socket(AF_INET, SOCK_STREAM, 0);
            sockaddr_in da{};
            da.sin_family = AF_INET;
            da.sin_port = htons(cfg.port);
            inet_pton(AF_INET, cfg.ip.c_str(), &da.sin_addr);
            if (connect(data_fd, (sockaddr*)&da, sizeof(da)) < 0) {
                std::cerr << "Cannot connect to " << cfg.ip << ":" << cfg.port << "\n";
                close(data_fd);
                continue;
            }
            std::cout << "Data channel connected to " << cfg.ip << ":" << cfg.port << "\n";
            if (!ros_topic1.empty()) {
#ifdef USE_ROS2
                stream_to_ros(data_fd, ros_topic1, ros_topic2, ros_topic3, cfg.width, cfg.height, cfg.fps, cfg.bitrate, preview, use_nvenc);
#endif
            } else {
                stream_to(data_fd, cam_dev, cam_dev2, cam_dev3, cfg.width, cfg.height, cfg.fps, cfg.bitrate, preview, use_nvenc);
            }
            close(data_fd);
        } else {
            // No valid command, stream directly to this connection with defaults
            std::cout << "No OPEN_CAMERA command, streaming with defaults\n";
            if (!ros_topic1.empty()) {
#ifdef USE_ROS2
                stream_to_ros(cfd, ros_topic1, ros_topic2, ros_topic3, default_w, default_h, default_fps, default_bitrate, preview, use_nvenc);
#endif
            } else {
                stream_to(cfd, cam_dev, cam_dev2, cam_dev3, default_w, default_h, default_fps, default_bitrate, preview, use_nvenc);
            }
            close(cfd);
        }
        std::cout << "Session ended, waiting for next connection...\n";
    }
    close(srv);
}

// ─── main ───────────────────────────────────────────────────────────────────
int main(int argc, char *argv[]) {
    signal(SIGINT, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    std::string listen_addr, send_ip;
    std::string ros_topic1, ros_topic2, ros_topic3;
    int send_port     = 12345;
    int cam_dev       = 0;
    int cam_dev2      = -1;   // left gripper camera (-1 = disabled)
    int cam_dev3      = -1;   // right gripper camera (-1 = disabled)
    int width         = 1920;
    int height        = 1080;
    int fps           = 30;
    int bitrate       = 4000000;
    bool preview      = false;
    bool use_nvenc    = true;
    bool do_listen    = false;
    bool do_send      = false;
    bool use_ros      = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--listen" && i+1 < argc) { do_listen = true; listen_addr = argv[++i]; }
        else if (a == "--send")                              { do_send = true; }
        else if (a == "--server" && i+1 < argc)              { send_ip = argv[++i]; }
        else if (a == "--port"   && i+1 < argc)              { send_port = std::stoi(argv[++i]); }
        else if (a == "--device" && i+1 < argc)              { cam_dev = std::stoi(argv[++i]); }
        else if (a == "--device2" && i+1 < argc)             { cam_dev2 = std::stoi(argv[++i]); }
        else if (a == "--device3" && i+1 < argc)             { cam_dev3 = std::stoi(argv[++i]); }
        else if (a == "--ros")                               { use_ros = true; }
        else if (a == "--topic"  && i+1 < argc)              { ros_topic1 = argv[++i]; }
        else if (a == "--topic2" && i+1 < argc)              { ros_topic2 = argv[++i]; }
        else if (a == "--topic3" && i+1 < argc)              { ros_topic3 = argv[++i]; }
        else if (a == "--width"  && i+1 < argc)              { width = std::stoi(argv[++i]); }
        else if (a == "--height" && i+1 < argc)              { height = std::stoi(argv[++i]); }
        else if (a == "--fps"    && i+1 < argc)              { fps = std::stoi(argv[++i]); }
        else if (a == "--bitrate"&& i+1 < argc)              { bitrate = std::stoi(argv[++i]); }
        else if (a == "--preview")                           { preview = true; }
        else if (a == "--sw")                                { use_nvenc = false; }
        else if (a == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                "  --listen ADDR    Listen mode (e.g. 0.0.0.0:13579)\n"
                "  --send           Direct send mode\n"
                "  --server IP      VR headset IP (send mode)\n"
                "  --port PORT      VR port (default 12345)\n"
                "  --device N       Main camera device index (default 0)\n"
                "  --device2 N      Left gripper camera device index\n"
                "  --device3 N      Right gripper camera device index\n"
                "  --ros            Use ROS2 image topics instead of V4L2\n"
                "  --topic TOPIC    Main camera ROS2 topic\n"
                "  --topic2 TOPIC   Left gripper ROS2 topic\n"
                "  --topic3 TOPIC   Right gripper ROS2 topic\n"
                "  --width W        Width (default 1280)\n"
                "  --height H       Height (default 720)\n"
                "  --fps F          FPS (default 30)\n"
                "  --bitrate B      Bitrate bps (default 4000000)\n"
                "  --preview        Show local preview window\n"
                "  --sw             Force software encoder (libx264)\n";
            return 0;
        }
    }

    if (!do_listen && !do_send) {
        std::cerr << "Need --listen or --send. Use --help.\n"; return 1;
    }

#ifdef USE_ROS2
    if (use_ros) {
        rclcpp::init(argc, argv);
        if (ros_topic1.empty()) ros_topic1 = "/cam/head/color/image_raw";
        std::cout << "ROS2 mode: topics = " << ros_topic1;
        if (!ros_topic2.empty()) std::cout << ", " << ros_topic2;
        if (!ros_topic3.empty()) std::cout << ", " << ros_topic3;
        std::cout << "\n";
    }
#else
    if (use_ros) {
        std::cerr << "ROS2 support not compiled in. Build with cmake -DUSE_ROS2=ON.\n";
        return 1;
    }
#endif

    if (do_listen) {
        listen_mode(listen_addr, cam_dev, cam_dev2, cam_dev3,
                    use_ros ? ros_topic1 : "", use_ros ? ros_topic2 : "", use_ros ? ros_topic3 : "",
                    width, height, fps, bitrate, preview, use_nvenc);
    } else {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in da{};
        da.sin_family = AF_INET;
        da.sin_port = htons(send_port);
        inet_pton(AF_INET, send_ip.c_str(), &da.sin_addr);
        if (connect(fd, (sockaddr*)&da, sizeof(da)) < 0) {
            std::cerr << "Cannot connect to " << send_ip << ":" << send_port << "\n";
            return 1;
        }
        std::cout << "Connected to " << send_ip << ":" << send_port << "\n";
        if (use_ros) {
#ifdef USE_ROS2
            stream_to_ros(fd, ros_topic1, ros_topic2, ros_topic3, width, height, fps, bitrate, preview, use_nvenc);
#endif
        } else {
            stream_to(fd, cam_dev, cam_dev2, cam_dev3, width, height, fps, bitrate, preview, use_nvenc);
        }
        close(fd);
    }

#ifdef USE_ROS2
    if (use_ros && rclcpp::ok()) rclcpp::shutdown();
#endif
    return 0;
}
