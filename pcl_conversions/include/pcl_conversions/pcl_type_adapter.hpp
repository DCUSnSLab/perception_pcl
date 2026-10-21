// Copyright 2026 nevlife
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PCL_CONVERSIONS__PCL_TYPE_ADAPTER_HPP_
#define PCL_CONVERSIONS__PCL_TYPE_ADAPTER_HPP_

#include <type_traits>

#include "pcl/point_cloud.h"
#include "rclcpp/type_adapter.hpp"

#include "pcl_conversions/pcl_conversions.h"

/** rclcpp type adapters for PCL types, so that nodes publish and subscribe PCL types directly.
 * With intra-process delivery the PCL object reaches subscriptions of the same type in the
 * process without conversion; only readers of the ROS message pay for one.
 *
 *   auto pub = node->create_publisher<
 *     rclcpp::adapt_type<pcl::PointCloud<pcl::PointXYZ>>::as<sensor_msgs::msg::PointCloud2>>(
 *     "points", 10);
 *   pub->publish(std::move(cloud));  // std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>
 */
template<typename PointT>
struct rclcpp::TypeAdapter<pcl::PointCloud<PointT>, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = pcl::PointCloud<PointT>;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    pcl::toROSMsg(source, destination);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    pcl::fromROSMsg(source, destination);
  }
};

template<>
struct rclcpp::TypeAdapter<pcl::PCLPointCloud2, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = pcl::PCLPointCloud2;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    pcl_conversions::fromPCL(source, destination);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    pcl_conversions::toPCL(source, destination);
  }
};

template<>
struct rclcpp::TypeAdapter<pcl::PCLImage, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = pcl::PCLImage;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    pcl_conversions::fromPCL(source, destination);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    pcl_conversions::toPCL(source, destination);
  }
};

template<>
struct rclcpp::TypeAdapter<pcl::PointIndices, pcl_msgs::msg::PointIndices>
{
  using is_specialized = std::true_type;
  using custom_type = pcl::PointIndices;
  using ros_message_type = pcl_msgs::msg::PointIndices;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    pcl_conversions::fromPCL(source, destination);
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    pcl_conversions::toPCL(source, destination);
  }
};

#endif  // PCL_CONVERSIONS__PCL_TYPE_ADAPTER_HPP_
