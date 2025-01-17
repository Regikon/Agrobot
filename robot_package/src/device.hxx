#ifndef __DEVICE_HXX__
#define __DEVICE_HXX__

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <ros/ros.h>
#include <array>

class Device {
public:
  std::string _serial_number{}, _cam_number;
  unsigned _height{}, _width{}, _gyro_freq{}, _accel_freq{};
  rs2::pipeline _pipe;
  rs2::frameset _frameset;
  rs2::frame _depth;
  rs2::frame _color;
  ros::NodeHandle _nh;
  rs2::device _dev;
  bool _has_imu;
  std::array<float, 3> _depth_to_color_ext;
  std::array<float, 2> _fov;

  void set_extrinsic(float a[3]){
    for(short i{}; i < 3; i++) this->_depth_to_color_ext[i] = a[i];
  }

  void set_fov(float a[2]){
    for(short i{}; i < 2; i++) this->_fov[i] = a[i];
  }
  

  Device(const std::string &serial_number, const std::string &cam_number,
         const unsigned &height, const unsigned &width,
         const unsigned &gyro_freq, const unsigned &accel_freq,
         ros::NodeHandle &nh, rs2::device &dev, bool has_imu)
      : _serial_number{serial_number},
        _cam_number{cam_number}, _height{height}, _width{width},
        _gyro_freq{gyro_freq}, _accel_freq{accel_freq}, _nh{nh}, _dev{dev}, _has_imu{has_imu} {}
};

#endif