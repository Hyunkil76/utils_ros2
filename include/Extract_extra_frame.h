#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

struct BufferedFrame {
    std::uint64_t stamp = 0;
    std::string topic_name;
    bool is_compressed = false;

    sensor_msgs::msg::Image raw_msg;
    sensor_msgs::msg::CompressedImage comp_msg;
};

struct SelectedBufferedFrames {
    std::vector<BufferedFrame> frames;
};

SelectedBufferedFrames selectEvenCenteredFrames(
    const std::vector<BufferedFrame>& frames,
    int num_frames,
    bool preferLeftOnExactCenter = true
);