// calibration_ws headcam (2026-10-13), derived from b0_20261007/tools/rs_capture.cpp. Local librealsense2 (2.50, /opt/ros/noetic):
// no ROS, publishes nothing, sets no device option. Streams color 1280x720@30 RGB8 and depth 848x480@30 Z16, skips 30 warm-up
// framesets, then for <seconds> saves every <every>-th frameset with host CLOCK_MONOTONIC and CLOCK_REALTIME stamps beside the
// device timestamps. Writes stream_info.json (resolution, fps, format and intrinsics of the streams actually used, depth scale,
// depth to color extrinsics) next to the frames. Writes no serial number or USB path.
// Build: g++ -O2 -std=c++14 headcam.cpp -I/opt/ros/noetic/include -L/opt/ros/noetic/lib/aarch64-linux-gnu -lrealsense2 -o bin/headcam
// Run:   bin/headcam <out_dir> <seconds> <every>
#include <librealsense2/rs.hpp>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

static double now_s(clockid_t c) { timespec t; clock_gettime(c, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static std::string intr_json(const rs2_intrinsics &i) {
    std::ostringstream o; o << std::setprecision(9);
    o << "{\"width\":" << i.width << ",\"height\":" << i.height << ",\"fx\":" << i.fx << ",\"fy\":" << i.fy << ",\"ppx\":" << i.ppx
      << ",\"ppy\":" << i.ppy << ",\"model\":\"" << rs2_distortion_to_string(i.model) << "\",\"coeffs\":[" << i.coeffs[0] << "," << i.coeffs[1]
      << "," << i.coeffs[2] << "," << i.coeffs[3] << "," << i.coeffs[4] << "]}";
    return o.str();
}
static std::string prof_json(const rs2::video_stream_profile &p) {
    std::ostringstream o;
    o << "{\"stream\":\"" << rs2_stream_to_string(p.stream_type()) << "\",\"width\":" << p.width() << ",\"height\":" << p.height() << ",\"fps\":" << p.fps()
      << ",\"format\":\"" << rs2_format_to_string(p.format()) << "\",\"intrinsics\":" << intr_json(p.get_intrinsics()) << "}";
    return o.str();
}

int main(int argc, char **argv) {
    if (argc < 4) { std::cerr << "usage: headcam <out_dir> <seconds> <every>\n"; return 1; }
    std::string out = argv[1];
    double secs = std::atof(argv[2]);
    int every = std::max(1, std::atoi(argv[3]));
    rs2::context ctx;
    auto devs = ctx.query_devices();
    if (devs.size() == 0) { std::cerr << "NO_DEVICE\n"; return 2; }
    rs2::device dev = devs[0];
    float depth_scale = 0;
    for (auto &s : dev.query_sensors()) if (auto ds = s.as<rs2::depth_sensor>()) depth_scale = ds.get_depth_scale();
    rs2::pipeline pipe(ctx);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    auto prof = pipe.start(cfg);
    auto dprof = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto cprof = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto ex = dprof.get_extrinsics_to(cprof);
    {
        std::ofstream js(out + "/stream_info.json");
        js << std::setprecision(9) << "{\"librealsense_api\":\"" << RS2_API_VERSION_STR << "\",\"device_name\":\"" << dev.get_info(RS2_CAMERA_INFO_NAME)
           << "\",\"firmware\":\"" << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\",\"depth_scale_m\":" << depth_scale
           << ",\"color\":" << prof_json(cprof) << ",\"depth\":" << prof_json(dprof) << ",\"extrinsics_depth_to_color\":{\"rotation_col_major\":[";
        for (int k = 0; k < 9; ++k) js << (k ? "," : "") << ex.rotation[k];
        js << "],\"translation_m\":[" << ex.translation[0] << "," << ex.translation[1] << "," << ex.translation[2] << "]},\"warmup_framesets\":30,"
           << "\"seconds\":" << secs << ",\"save_every\":" << every << "}\n";
    }
    for (int i = 0; i < 30; ++i) pipe.wait_for_frames(5000);
    std::ofstream fj(out + "/frames.jsonl");
    double t0 = now_s(CLOCK_MONOTONIC);
    int n = 0, saved = 0;
    while (now_s(CLOCK_MONOTONIC) - t0 < secs) {
        auto fs = pipe.wait_for_frames(5000);
        double tm = now_s(CLOCK_MONOTONIC), tr = now_s(CLOCK_REALTIME);
        if (n++ % every) continue;
        auto d = fs.get_depth_frame();
        auto c = fs.get_color_frame();
        char name[96];
        std::snprintf(name, sizeof(name), "color_%05d_1280x720.ppm", saved);
        { std::ofstream f(out + "/" + name, std::ios::binary); f << "P6\n1280 720\n255\n"; f.write((const char *)c.get_data(), c.get_data_size()); }
        std::string cname = name;
        std::snprintf(name, sizeof(name), "depth_%05d_z16le_848x480.raw", saved);
        { std::ofstream f(out + "/" + name, std::ios::binary); f.write((const char *)d.get_data(), d.get_data_size()); }
        fj << std::setprecision(17) << "{\"i\":" << saved << ",\"t_mono_ns\":" << (long long)(tm * 1e9) << ",\"t_host\":" << tr << ",\"color_file\":\"" << cname
           << "\",\"depth_file\":\"" << name << "\",\"color_frame_number\":" << c.get_frame_number() << ",\"color_timestamp_ms\":" << c.get_timestamp()
           << ",\"color_timestamp_domain\":\"" << rs2_timestamp_domain_to_string(c.get_frame_timestamp_domain()) << "\",\"depth_frame_number\":"
           << d.get_frame_number() << ",\"depth_timestamp_ms\":" << d.get_timestamp() << "}\n";
        ++saved;
    }
    pipe.stop();
    std::cout << "OK framesets_seen " << n << " saved " << saved << "\n";
    return 0;
}
