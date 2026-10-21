// Copyright 2026 nevlife
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <utility>

#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"

#include "pcl_conversions/pcl_type_adapter.hpp"

using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using CloudAdapter = rclcpp::TypeAdapter<Cloud, sensor_msgs::msg::PointCloud2>;
using BlobAdapter = rclcpp::TypeAdapter<pcl::PCLPointCloud2, sensor_msgs::msg::PointCloud2>;

namespace
{
Cloud cloud()
{
  Cloud result;
  for (int i = 0; i < 5; ++i) {
    pcl::PointXYZI p;
    p.x = i; p.y = 2.0f * i; p.z = 3.0f * i; p.intensity = 10.0f + i;
    result.push_back(p);
  }
  result.header.frame_id = "lidar";
  result.header.stamp = 1234567;
  return result;
}
}  // namespace

TEST(PCLTypeAdapter, PointCloudRoundTrip)
{
  sensor_msgs::msg::PointCloud2 msg;
  CloudAdapter::convert_to_ros_message(cloud(), msg);
  EXPECT_EQ(5u, msg.width * msg.height);
  EXPECT_EQ("lidar", msg.header.frame_id);
  EXPECT_EQ(1, msg.header.stamp.sec);
  EXPECT_EQ(234567000u, msg.header.stamp.nanosec);
  Cloud back;
  CloudAdapter::convert_to_custom(msg, back);
  ASSERT_EQ(5u, back.size());
  EXPECT_FLOAT_EQ(8.0f, back[4].y);
  EXPECT_FLOAT_EQ(13.0f, back[3].intensity);
  EXPECT_EQ(1234567u, back.header.stamp);
}

TEST(PCLTypeAdapter, PCLPointCloud2RoundTrip)
{
  pcl::PCLPointCloud2 blob;
  pcl::toPCLPointCloud2(cloud(), blob);
  sensor_msgs::msg::PointCloud2 msg;
  BlobAdapter::convert_to_ros_message(blob, msg);
  EXPECT_EQ(blob.data.size(), msg.data.size());
  EXPECT_EQ(blob.point_step, msg.point_step);
  pcl::PCLPointCloud2 back;
  BlobAdapter::convert_to_custom(msg, back);
  EXPECT_EQ(blob.data, back.data);
  EXPECT_EQ(blob.fields.size(), back.fields.size());
}

TEST(PCLTypeAdapter, IntraProcessDeliversTheCloud)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>(
      "pcl_type_adapter", rclcpp::NodeOptions().use_intra_process_comms(true));
    std::shared_ptr<const Cloud> received;
    auto sub = node->create_subscription<CloudAdapter>(
      "points", 1, [&](std::shared_ptr<const Cloud> msg) {received = msg;});
    auto pub = node->create_publisher<CloudAdapter>("points", 1);
    auto sent = std::make_unique<Cloud>(cloud());
    const Cloud * address = sent.get();
    pub->publish(std::move(sent));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    for (int i = 0; i < 100 && !received; ++i) {
      executor.spin_some(std::chrono::milliseconds(10));
    }
    ASSERT_NE(nullptr, received);
    EXPECT_EQ(address, received.get());
    EXPECT_FLOAT_EQ(13.0f, (*received)[3].intensity);
  }
  rclcpp::shutdown();
}
