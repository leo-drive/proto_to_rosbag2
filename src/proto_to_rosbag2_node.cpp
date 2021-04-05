#include "rosbag_converter/proto_to_rosbag2_node.h"
#include "rclcpp/logger.hpp"
#include <vector>
#include <memory>
#include "rosbag_converter/proto_rosbag2.pb.h"
#include <iostream>
#include <fstream>
#include <rcpputils/filesystem_helper.hpp>
#include <cstring>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>

RosbagConverterNode::RosbagConverterNode(const rclcpp::NodeOptions & options)
: Node("rosbag_converter_node", options)
{
  auto path_in = static_cast<std::string>(
    rclcpp::Node::declare_parameter<std::string>(
      "path_in_ros2_serialized_binary"));
  auto path_out = static_cast<std::string>(
    rclcpp::Node::declare_parameter<std::string>(
      "path_out_ros2_bag"));

  std::stringstream ss_input_args;
  ss_input_args << "Input args: " << std::endl;
  ss_input_args << "path_in_ros2_serialized_binary: " << path_in << std::endl;
  ss_input_args << "path_out_ros2_bag: " << path_out << std::endl;
  ss_input_args << std::endl;
  RCLCPP_INFO_STREAM(this->get_logger(), ss_input_args.str());

  GOOGLE_PROTOBUF_VERIFY_VERSION;

  rosbag_converter_proto::ProtoRosBag2 proto_rosbag_2;

  {
    // Read serialized bag file
    std::fstream input(path_in, std::ios::in | std::ios::binary);
    if (!input) {
      RCLCPP_ERROR_STREAM(this->get_logger(), path_in << ": File not found.");
      return;
    } else if (!proto_rosbag_2.ParseFromIstream(&input)) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        path_in << "Failed to parse serialized_data_for_bags.");
      return;
    }
  }


  rcpputils::fs::remove_all(path_out);
  rosbag2_cpp::Writer writer;
  writer.open(path_out);

  for (const auto & proto_message : proto_rosbag_2.messages()) {
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->topic_name = proto_message.topic_name();
    bag_message->time_stamp = proto_message.time_stamp();
    std::vector<uint8_t> bytes_copy(
      proto_message.serialized_data().begin(), proto_message.serialized_data().end());

    rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();

    rcutils_allocator_t default_allocator = rcutils_get_default_allocator();
    rmw_serialized_message_init(&serialized_message, bytes_copy.size(), &default_allocator);
    rmw_serialized_message_resize(&serialized_message, bytes_copy.size());

    std::memcpy(serialized_message.buffer, bytes_copy.data(), bytes_copy.size());
    serialized_message.buffer_length = bytes_copy.size();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      &serialized_message, [](rcutils_uint8_array_t * /* data */) {});

    writer.write(bag_message, bag_message->topic_name, proto_message.topic_type_name());
    rmw_serialized_message_fini(&serialized_message);
  }

}
