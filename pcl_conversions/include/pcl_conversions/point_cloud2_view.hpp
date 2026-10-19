// SPDX-License-Identifier: BSD-3-Clause

#ifndef PCL_CONVERSIONS__POINT_CLOUD2_VIEW_HPP_
#define PCL_CONVERSIONS__POINT_CLOUD2_VIEW_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>

#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcl_conversions
{

using PointCloud2XYZMap = Eigen::Map<
  const Eigen::Matrix<float, 3, Eigen::Dynamic>, Eigen::Unaligned, Eigen::OuterStride<>>;

/** Return a const XYZ map without copying or creating a pcl::PointCloud.
 * The message buffer must outlive the map and must not be resized while it is used.
 * Requires consecutive native-endian FLOAT32 XYZ fields and tightly packed rows.
 */
inline PointCloud2XYZMap getXYZMatrixXfMap(const sensor_msgs::msg::PointCloud2 & cloud)
{
  const std::uint16_t native_endian = 1;
  const bool native_bigendian = *reinterpret_cast<const unsigned char *>(&native_endian) == 0;
  if (cloud.is_bigendian != native_bigendian) {
    throw std::invalid_argument("XYZ map requires native byte order");
  }
  std::size_t offset = 0;
  std::size_t index = 0;
  for (const auto * name : {"x", "y", "z"}) {
    const auto field = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [name](const sensor_msgs::msg::PointField & value) {return value.name == name;});
    if (field == cloud.fields.end() ||
      field->datatype != sensor_msgs::msg::PointField::FLOAT32 || field->count != 1)
    {
      throw std::invalid_argument("XYZ map requires scalar FLOAT32 fields");
    }
    if (index == 0) {
      offset = field->offset;
    }
    if (field->offset != offset + index * sizeof(float)) {
      throw std::invalid_argument("XYZ map requires consecutive XYZ fields");
    }
    ++index;
  }
  if (offset % alignof(float) != 0 || cloud.point_step % sizeof(float) != 0 ||
    offset + 3 * sizeof(float) > cloud.point_step)
  {
    throw std::invalid_argument("XYZ map requires float-aligned point offsets and strides");
  }
  if (cloud.row_step != static_cast<std::size_t>(cloud.width) * cloud.point_step) {
    throw std::invalid_argument("XYZ map requires tightly packed rows");
  }
  const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
  if (cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height) {
    throw std::invalid_argument("Point cloud data is smaller than its dimensions");
  }
  const auto * data = count ? reinterpret_cast<const float *>(cloud.data.data() + offset) : nullptr;
  return PointCloud2XYZMap(data, 3, count, Eigen::OuterStride<>(cloud.point_step / sizeof(float)));
}

}  // namespace pcl_conversions

#endif  // PCL_CONVERSIONS__POINT_CLOUD2_VIEW_HPP_
