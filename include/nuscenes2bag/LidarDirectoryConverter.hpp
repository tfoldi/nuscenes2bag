#pragma once

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <filesystem>

std::optional<sensor_msgs::PointCloud2> readLidarFile(std::filesystem::path filePath);