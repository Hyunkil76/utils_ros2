#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "Trajectory.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include "Utils.h"
#include "Extract_extra_frame.h"
#include <vector>

namespace fs = std::filesystem;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("write_images_from_list");

    // --- 1. Parameter Declaration and Fetching ---
    node->declare_parameter("traj_file", "");
    node->declare_parameter("bag_file", "");
    node->declare_parameter("left_image_topic", "/gopro/image_raw");
    node->declare_parameter("right_image_topic", "");
    node->declare_parameter("image_dir", "");
    node->declare_parameter("config_file", "");
    node->declare_parameter("compressed", true);
    node->declare_parameter("skip_first_line", true);
    node->declare_parameter("stereo", false);
    node->declare_parameter("scale", 1.0);
    node->declare_parameter("prefer_left_on_extract_center", true);
    node->declare_parameter("num_extra", 0);
    node->declare_parameter("make_sub_dir", true);

    std::string traj_file = node->get_parameter("traj_file").as_string();
    std::string bag_file = node->get_parameter("bag_file").as_string();
    std::string left_image_topic = node->get_parameter("left_image_topic").as_string();
    std::string image_dir = node->get_parameter("image_dir").as_string();
    std::string config_file = node->get_parameter("config_file").as_string();
    bool compressed = node->get_parameter("compressed").as_bool();
    bool skip_first_line = node->get_parameter("skip_first_line").as_bool();
    bool stereo = node->get_parameter("stereo").as_bool();
    double scale = node->get_parameter("scale").as_double();
    bool prefer_left_on_extract_center = node->get_parameter("prefer_left_on_extract_center").as_bool();
    int num_extra = node->get_parameter("num_extra").as_int();
    bool make_sub_dir = node->get_parameter("make_sub_dir").as_bool();

    // Validations
    if (traj_file.empty() || bag_file.empty() || left_image_topic.empty() || image_dir.empty() || config_file.empty()) {
        RCLCPP_FATAL(node->get_logger(), "Missing essential parameters. Please check your config.");
        return 1;
    }

    std::string right_image_topic = "";
    if (stereo) {
        right_image_topic = node->get_parameter("right_image_topic").as_string();
        if (right_image_topic.empty()) {
            RCLCPP_FATAL(node->get_logger(), "Right image topic not set for stereo mode.");
            return 1;
        }
    }

    // Append "/compressed" if necessary
    if (compressed) {
        left_image_topic += "/compressed";
        if (stereo) right_image_topic += "/compressed";
    }

    // --- 2. Load Intrinsics ---
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        RCLCPP_FATAL(node->get_logger(), "Wrong path to intrinsics config file.");
        return 1;
    }

    cv::Mat left_distort = cv::Mat::zeros(4, 1, CV_64F), left_K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat right_distort = cv::Mat::zeros(4, 1, CV_64F), right_K = cv::Mat::eye(3, 3, CV_64F);

    cv::FileNode left_node;
    cv::FileNode right_node;

    if (stereo) {
        left_node = fsSettings["left"];
        right_node = fsSettings["right"];
    } else {
        left_node = fsSettings.root();
    }

    if (stereo) {
        Utils::readIntrinsics(right_node, right_K, right_distort);
    }
    Utils::readIntrinsics(left_node, left_K, left_distort);

    // --- 3. Load Trajectory ---
    Trajectory traj(traj_file);
    traj.loadTrajectory(' ', skip_first_line, true);
    std::set<std::uint64_t> kf_stamps = traj.getTimestamps();

    // --- 4. Setup Directories ---
    std::string left_image_dir = stereo ? image_dir + "/left" : image_dir;
    std::string right_image_dir = stereo ? image_dir + "/right" : "";

    if (!fs::exists(left_image_dir)) fs::create_directories(left_image_dir);
    if (stereo && !fs::exists(right_image_dir)) fs::create_directories(right_image_dir);

    std::string extra_left_image_dir = stereo ? image_dir + "/left" + "/extra_frames" : image_dir + "/extra_frames";
    std::string extra_right_image_dir = stereo ? image_dir + "/right" + "/extra_frames" : "";
    if (num_extra > 0) {
        if (!fs::exists(extra_left_image_dir)) {
            fs::create_directories(extra_left_image_dir);
        }
        if (stereo && !fs::exists(extra_right_image_dir)) {
            fs::create_directories(extra_right_image_dir);
        }
    }

    // --- 5. Open Bag and Apply Storage Filter ---
    rosbag2_cpp::Reader reader;
    reader.open(bag_file);
    
    // Optimization: Tell the database to ONLY give us our desired image topics
    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back(left_image_topic);
    if (stereo) filter.topics.push_back(right_image_topic);
    reader.set_filter(filter);

    // --- 6. State Variables for Lazy Initialization ---
    bool maps_initialized = false;
    cv::Mat left_map_x, left_map_y, right_map_x, right_map_y;
    int new_width = 0, new_height = 0;

    size_t kf_images = 0;
    size_t total_images = 0;

    // Reuse message objects to avoid allocations in the loop
    sensor_msgs::msg::Image raw_image_msg;
    sensor_msgs::msg::CompressedImage comp_image_msg;
    rclcpp::Serialization<sensor_msgs::msg::Image> raw_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> comp_serialization;
    
    cv::Mat image;
    cv::Mat undistorted_image;
    cv::Mat undistorted_extra_image;

    std::vector<BufferedFrame> left_non_keyframes;
    std::vector<BufferedFrame> right_non_keyframes;
    bool is__left_first_keyframe_found = false; // For tie-breaking in extra keyframe extraction
    bool is_right_first_keyframe_found = false;
    uint64_t prev_left_stamp = 0;
    uint64_t prev_right_stamp = 0;

    // --- 7. Single Pass Loop ---
    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        total_images++;

        std::uint64_t stamp = 0;
        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

        // Optimization: Deserialize into reused object, get stamp BEFORE heavy CV decoding
        if (compressed) {
            comp_serialization.deserialize_message(&serialized_msg, &comp_image_msg);
            stamp = comp_image_msg.header.stamp.nanosec + (comp_image_msg.header.stamp.sec * 1000000000ULL);
        } else {
            raw_serialization.deserialize_message(&serialized_msg, &raw_image_msg);
            stamp = raw_image_msg.header.stamp.nanosec + (raw_image_msg.header.stamp.sec * 1000000000ULL);
        }
        
        if (num_extra == 0) {
            // Optimization: Skip immediately if not a keyframe
            if (kf_stamps.find(stamp) == kf_stamps.end()) {
                continue; 
            }

            // Now we know we need it, do the heavy OpenCV decoding
            if (compressed) {
                image = cv_bridge::toCvCopy(comp_image_msg, "bgr8")->image;
            } else {
                image = cv_bridge::toCvCopy(raw_image_msg, "bgr8")->image;
            }

            // Optimization: Lazy init maps on the very first matched frame
            if (!maps_initialized) {
                int original_width = image.cols;
                int original_height = image.rows;
                new_width = static_cast<int>(original_width * scale);
                new_height = static_cast<int>(original_height * scale);

                cv::initUndistortRectifyMap(left_K, left_distort, cv::Mat(), left_K, cv::Size(original_width, original_height), CV_32FC1, left_map_x, left_map_y);
                if (stereo) {
                    cv::initUndistortRectifyMap(right_K, right_distort, cv::Mat(), right_K, cv::Size(original_width, original_height), CV_32FC1, right_map_x, right_map_y);
                }
                maps_initialized = true;
                
                RCLCPP_INFO_STREAM(node->get_logger(), "Initialized Maps. Resizing " 
                    << original_width << "x" << original_height << " to " 
                    << new_width << "x" << new_height);
            }

            // Process and Write
            std::string filename;

            if (bag_message->topic_name == left_image_topic) {
                cv::remap(image, undistorted_image, left_map_x, left_map_y, cv::INTER_LINEAR);
                filename = left_image_dir + "/" + std::to_string(stamp) + ".png";
            } else if (stereo && bag_message->topic_name == right_image_topic) {
                cv::remap(image, undistorted_image, right_map_x, right_map_y, cv::INTER_LINEAR);
                filename = right_image_dir + "/" + std::to_string(stamp) + ".png";
            } else {
                continue; 
            }

            if (scale != 1.0) {
                cv::resize(undistorted_image, undistorted_image, cv::Size(new_width, new_height));
            }
            
            cv::imwrite(filename, undistorted_image);
            kf_images++;
        }

        else if (num_extra > 0) {
            if (kf_stamps.find(stamp) == kf_stamps.end()) {
                if (is__left_first_keyframe_found || is_right_first_keyframe_found) {
                    BufferedFrame frame;
                    frame.stamp = stamp;
                    frame.topic_name = bag_message->topic_name;
                    frame.is_compressed = compressed;

                    if (compressed) {
                        frame.comp_msg = comp_image_msg;
                    } else {
                        frame.raw_msg = raw_image_msg;
                    }

                    if (bag_message->topic_name == left_image_topic && is__left_first_keyframe_found) {
                        left_non_keyframes.push_back(frame);
                    } else if (stereo && bag_message->topic_name == right_image_topic && is_right_first_keyframe_found) {
                        right_non_keyframes.push_back(frame);
                    }
                }
                continue; 
            }

            if (compressed) {
                image = cv_bridge::toCvCopy(comp_image_msg, "bgr8")->image;
            } else {
                image = cv_bridge::toCvCopy(raw_image_msg, "bgr8")->image;
            }

            // Optimization: Lazy init maps on the very first matched frame
            if (!maps_initialized) {
                int original_width = image.cols;
                int original_height = image.rows;
                new_width = static_cast<int>(original_width * scale);
                new_height = static_cast<int>(original_height * scale);

                cv::initUndistortRectifyMap(left_K, left_distort, cv::Mat(), left_K, cv::Size(original_width, original_height), CV_32FC1, left_map_x, left_map_y);
                if (stereo) {
                    cv::initUndistortRectifyMap(right_K, right_distort, cv::Mat(), right_K, cv::Size(original_width, original_height), CV_32FC1, right_map_x, right_map_y);
                }
                maps_initialized = true;
                
                RCLCPP_INFO_STREAM(node->get_logger(), "Initialized Maps. Resizing " 
                    << original_width << "x" << original_height << " to " 
                    << new_width << "x" << new_height);
            }

            // Process and Write
            std::string filename;

            if (bag_message->topic_name == left_image_topic) {
                cv::remap(image, undistorted_image, left_map_x, left_map_y, cv::INTER_LINEAR);
                filename = left_image_dir + "/" + std::to_string(stamp) + ".png";
                if (!is__left_first_keyframe_found) {
                    is__left_first_keyframe_found = true;
                }
            } else if (stereo && bag_message->topic_name == right_image_topic) {
                cv::remap(image, undistorted_image, right_map_x, right_map_y, cv::INTER_LINEAR);
                filename = right_image_dir + "/" + std::to_string(stamp) + ".png";
                if (!is_right_first_keyframe_found) {
                    is_right_first_keyframe_found = true;
                }
            } else {
                continue; 
            }

            if (scale != 1.0) {
                cv::resize(undistorted_image, undistorted_image, cv::Size(new_width, new_height));
            }
            
            cv::imwrite(filename, undistorted_image);
            kf_images++;

            if (bag_message->topic_name == left_image_topic)
            {
                // Check if we have enough non-keyframes to extract extra keyframes
                if (!left_non_keyframes.empty()) {
                    auto selected = selectEvenCenteredFrames(left_non_keyframes, num_extra, prefer_left_on_extract_center);
                    
                    std::string extra_image_path = "";

                    if (make_sub_dir) {
                        extra_image_path = extra_left_image_dir + "/" + std::to_string(prev_left_stamp);
                        fs::create_directories(extra_image_path);
                    }

                    else {
                        extra_image_path = extra_left_image_dir;
                    }

                    for (const auto& frame : selected.frames) {
                        cv::Mat extra_image;

                        if (frame.is_compressed) {
                            extra_image = cv_bridge::toCvCopy(frame.comp_msg, "bgr8")->image;
                        } else {
                            extra_image = cv_bridge::toCvCopy(frame.raw_msg, "bgr8")->image;
                        }

                        cv::remap(extra_image, undistorted_extra_image, left_map_x, left_map_y, cv::INTER_LINEAR);

                        if (scale != 1.0) {
                            cv::resize(undistorted_extra_image, undistorted_extra_image, cv::Size(new_width, new_height));
                        }
                        std::string extra_filename = extra_image_path + "/" + std::to_string(frame.stamp) + ".png";
                        cv::imwrite(extra_filename, undistorted_extra_image);
                    }

                    // RCLCPP_INFO_STREAM(node->get_logger(), "keyframes at " << prev_left_stamp << " and " << stamp
                    // << " Expected/Stored Count: " << ((stamp-prev_left_stamp)/1e9)*30-1 << " /  " << left_non_keyframes.size() << ", selected " << selected.frames.size() << " for extra frames.");

                    left_non_keyframes.clear();
                }

                prev_left_stamp = stamp;
            }

            else if (stereo && bag_message->topic_name == right_image_topic)
            {
                if (!right_non_keyframes.empty()) {
                    auto selected = selectEvenCenteredFrames(right_non_keyframes, num_extra, prefer_left_on_extract_center);
                    
                    std::string extra_image_path = "";

                    if (make_sub_dir) {
                        extra_image_path = extra_right_image_dir + "/" + std::to_string(prev_right_stamp);
                        fs::create_directories(extra_image_path);
                    }

                    else {
                        extra_image_path = extra_right_image_dir;
                    }

                    for (const auto& frame : selected.frames) {
                        cv::Mat extra_image;

                        if (frame.is_compressed) {
                            extra_image = cv_bridge::toCvCopy(frame.comp_msg, "bgr8")->image;
                        } else {
                            extra_image = cv_bridge::toCvCopy(frame.raw_msg, "bgr8")->image;
                        }

                        cv::remap(extra_image, undistorted_extra_image, right_map_x, right_map_y, cv::INTER_LINEAR);

                        if (scale != 1.0) {
                            cv::resize(undistorted_extra_image, undistorted_extra_image, cv::Size(new_width, new_height));
                        }
                        std::string extra_filename = extra_image_path + "/" + std::to_string(frame.stamp) + ".png";
                        cv::imwrite(extra_filename, undistorted_extra_image);
                    }

                    right_non_keyframes.clear();
                }

                prev_right_stamp = stamp;
            }
        }
    }

    RCLCPP_INFO(node->get_logger(), "Wrote %zu keyframe images out of %zu total image messages parsed.", kf_images, total_images);
    reader.close();
    rclcpp::shutdown();

    return 0;
}