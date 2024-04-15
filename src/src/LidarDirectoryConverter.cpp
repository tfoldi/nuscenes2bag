#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"
#include <exception>

using namespace sensor_msgs;
using namespace std;

namespace nuscenes2bag {

inline void
fillFieldsForPointcloud(std::vector<PointField>& fields)
{
  PointField field;
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 0;
  field.count = 1;
  field.name = std::string("x");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 4;
  field.count = 1;
  field.name = std::string("y");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 8;
  field.count = 1;
  field.name = std::string("z");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 12;
  field.count = 1;
  field.name = std::string("intensity");
  fields.push_back(field);
}

inline void
fillFieldsForPointcloudLabel(std::vector<PointField>& fields)
{
  PointField field;
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 0;
  field.count = 1;
  field.name = std::string("x");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 4;
  field.count = 1;
  field.name = std::string("y");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 8;
  field.count = 1;
  field.name = std::string("z");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::UINT32;
  field.offset = 16;
  field.count = 1;
  field.name = std::string("label");
  fields.push_back(field);
}

// Convert float32 to 4 bytes
union
{
  float value;
  uint8_t byte[4];
} floatToBytes;

union
{
  uint32_t value;
  uint8_t byte[4];
} uint32ToBytes;

inline void
push_back_float32(std::vector<uint8_t>& data, float float_data)
{

  /*
  // dereferencing type-punned pointer will break strict-aliasing rules
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 0) & 0xFF);
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 8) & 0xFF);
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 16) & 0xFF);
  data.push_back((*reinterpret_cast<uint32_t*>(&float_data) >> 24) & 0xFF);
  */

  floatToBytes.value = float_data;
  data.push_back(floatToBytes.byte[0]);
  data.push_back(floatToBytes.byte[1]);
  data.push_back(floatToBytes.byte[2]);
  data.push_back(floatToBytes.byte[3]);
}

inline std::vector<float>
readBinaryPcdFile(std::ifstream& fin)
{
  std::vector<float> fileValues;
  uint8_t skipCounter = 0;
  float f;
  while (fin.read(reinterpret_cast<char*>(&f), sizeof(float))) {
    // skip 5th value of each point
    if (skipCounter < 4) {
      fileValues.push_back(f);
      skipCounter++;
    } else {
      skipCounter = 0;
    }
  }

  return fileValues;
}

inline std::vector<uint8_t>
readBinaryPcdFileLabel(std::ifstream& fin)
{
  std::vector<uint8_t> fileValues;
  uint8_t l;
  while (fin.read(reinterpret_cast<char*>(&l), sizeof(uint8_t))) {
    fileValues.push_back(l);
  }

  return fileValues;
}

boost::optional<sensor_msgs::PointCloud2>
readLidarFile(const fs::path& filePath)
{

  PointCloud2 cloud;
  cloud.header.frame_id = std::string("lidar");
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(float) * 4; // Length of each point in bytes
  cloud.height = 1;

  try {
    std::ifstream fin(filePath.string(), std::ios::binary);
    const std::vector<float> fileValues = readBinaryPcdFile(fin);

    if (fileValues.size() % 4 != 0) {
      throw UnableToParseFileException(filePath.string());
    }
    const size_t pointsNumber = fileValues.size() / 4;
    cloud.width = pointsNumber;

    std::vector<uint8_t> data;
    for (auto float_data : fileValues) {
      push_back_float32(data, float_data);
    }

    fillFieldsForPointcloud(cloud.fields);
    cloud.data = data;
    cloud.row_step = data.size(); // Length of row in bytes

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);

    return boost::none;
  }

  return boost::optional<sensor_msgs::PointCloud2>(cloud);
}

boost::optional<sensor_msgs::PointCloud2>
readLidarFileXYZLabel(const fs::path& filePath, const fs::path& filePathLabel)
{
  PointCloud2 cloud;
  cloud.header.frame_id = std::string("lidar");
  cloud.is_bigendian = false;
  cloud.point_step = 20; // Length of each point in bytes
  cloud.height = 1;

//  printf("readLidarFileXYZLabel: filePathLabel=%s\n",
//         filePathLabel.string().c_str());

  try {
    std::ifstream fin(filePath.string(), std::ios::binary);
    const std::vector<float> fileValues = readBinaryPcdFile(fin);

    if (fileValues.size() % 4 != 0) {
      throw UnableToParseFileException(filePath.string());
    }
    const size_t pointsNumber = fileValues.size() / 4;
    cloud.width = pointsNumber;

    std::vector<uint8_t> data;
    for (auto float_data : fileValues) {
      push_back_float32(data, float_data);
    }

    fillFieldsForPointcloudLabel(cloud.fields);

    // adding label info
    std::ifstream finLabel(filePathLabel.string(), std::ios::binary);
    const std::vector<uint8_t> fileValuesLabel =
      readBinaryPcdFileLabel(finLabel);

    std::vector<uint8_t> dataWithLabel;
    assert(pointsNumber == fileValuesLabel.size());

//    printf("readLidarFileXYZLabel: pointsNumber=%d fileValuesLabel.size()=%d\n",
//           pointsNumber,
//           fileValuesLabel.size());

    dataWithLabel.resize(pointsNumber * 20); // PontXYZLabel
    for (size_t i = 0; i < pointsNumber; i++) {
      for (int j = 0; j < 12; j++) {
        dataWithLabel[i * 20 + j] = data[i * 16 + j];
      }
      dataWithLabel[i * 20 + 12] = (uint8_t)0;
      dataWithLabel[i * 20 + 13] = (uint8_t)0;
      dataWithLabel[i * 20 + 14] = (uint8_t)0;
      dataWithLabel[i * 20 + 15] = (uint8_t)0;
      uint32ToBytes.value = (uint32_t)fileValuesLabel[i];
      dataWithLabel[i * 20 + 16] = uint32ToBytes.byte[0];
      dataWithLabel[i * 20 + 17] = uint32ToBytes.byte[1];
      dataWithLabel[i * 20 + 18] = uint32ToBytes.byte[2];
      dataWithLabel[i * 20 + 19] = uint32ToBytes.byte[3];
    }

    cloud.data = dataWithLabel;
    cloud.row_step = dataWithLabel.size(); // Length of row in bytes

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);

    return boost::none;
  }

  return boost::optional<sensor_msgs::PointCloud2>(cloud);
}

}
