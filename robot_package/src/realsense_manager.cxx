#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cmath>
#include <execution>
#include <future>
#include <iomanip>
#include <iostream>             // for cout
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "agro/Accel.h"
#include "agro/Gyro.h"
#include "agro/IMU.h"
#include "agro/Pose.h"
#include "device.hxx"

// PCL Headers
#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/thread/thread.hpp>

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

class RGBD_Manager {
public:
  rs2::context _ctx;
  rs2::colorizer _colorizer;
  std::vector<Device> _device_list{};
  std::vector<
      std::tuple<rs2::pipeline, Device, cv::Mat, cv::Mat, rs2::colorizer>>
      _pipeline_list{};
  unsigned _width{}, _height{}, _fps{};
  ros::NodeHandle nh;
  std::unordered_map<std::string, std::string> _info_dict{
      std::make_pair("044322072287", "1"), std::make_pair("141722076677", "2"),
      std::make_pair("141722078488", "3"), std::make_pair("147322070391", "4"),
      std::make_pair("141722074582", "5"), std::make_pair("105422060587", "6"),
  };
  rs2::colorizer color_map;
  float alpha = 0.5f; // Transparancy coefficient for alignment
  std::unordered_map<std::string, bool> _imu_devices{
      std::make_pair("1", true), std::make_pair("2", true),
      std::make_pair("3", true), std::make_pair("4", false),
      std::make_pair("5", true), std::make_pair("6", true)};

public:
  RGBD_Manager(unsigned &&width = 640, unsigned &&height = 480,
               unsigned &&fps = 30)
      : _width{width}, _height{height}, _fps{fps} {};
  void getDevices() {
    // Query the list of connected RealSense devices
    for (rs2::device &&dev : _ctx.query_devices()) {
      std::string serial_num{dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)};
      this->_device_list.push_back(Device(
          serial_num, _info_dict.at(serial_num), this->_height, this->_width,
          (_info_dict.at(serial_num) == "1") ? 250U : 200U, 200U, this->nh, dev,
          _imu_devices.at(_info_dict.at(serial_num))));
    }
  }

  void enumerateConnectedDevices() {
    std::for_each(
        std::execution::par, this->_device_list.begin(),
        this->_device_list.end(), [&](Device &n) {
          std::cout << "Cam: " << n._serial_number
                    << " assigned to number: " << n._cam_number << "\n";
          rs2::pipeline _pipe{rs2::pipeline(_ctx)};
          rs2::config _cfg;
          _cfg.enable_device(n._serial_number);
          _cfg.enable_stream(RS2_STREAM_COLOR, n._width, n._height,
                             RS2_FORMAT_BGR8, 30);
          _cfg.enable_stream(RS2_STREAM_DEPTH, n._width, n._height,
                             RS2_FORMAT_Z16, 30);
          if (n._has_imu) {
            _cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
            _cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
          }
          rs2::pipeline_profile _profile{_pipe.start(_cfg)};
          auto color_profile = _profile.get_stream(RS2_STREAM_COLOR)
                                   .as<rs2::video_stream_profile>();
          auto depth_profile = _profile.get_stream(RS2_STREAM_DEPTH)
                                   .as<rs2::video_stream_profile>();

          // Get depth to color extrinsics
          auto color_intrinsic{color_profile.get_intrinsics()};
          auto depth_intrinsic{depth_profile.get_intrinsics()};
          rs2_extrinsics e = depth_profile.get_extrinsics_to(color_profile);
          // Apply extrinsics to the origin
          float origin[3]{0.f, 0.f, 0.f}, target[3];
          rs2_transform_point_to_point(target, &e, origin);
          n.set_extrinsic(target);

          // Get field of view
          auto i{depth_profile.get_intrinsics()};
          float fov[2]; // X, Y fov
          rs2_fov(&i, fov);
          n.set_fov(fov);

          cv::Mat mat_color(color_profile.height(), color_profile.width(),
                            CV_8UC3);
          cv::Mat colored_mat_depth(depth_profile.height(),
                                    depth_profile.width(), CV_8UC3);
          // cv::Mat mat_depth(depth_profile.height(), depth_profile.width(),
          // CV_16U);
          rs2::colorizer color_map;
          this->_pipeline_list.emplace_back(std::make_tuple(
              _pipe, n, mat_color, colored_mat_depth, color_map));
          std::cout << "device: " << n._cam_number << " enabled.\n";
        });
  }

  ~RGBD_Manager() {
    std::for_each(std::execution::par, this->_pipeline_list.begin(),
                  this->_pipeline_list.end(),
                  [this](std::tuple<rs2::pipeline, Device, cv::Mat, cv::Mat,
                                    rs2::colorizer> &pipe_pair) {
                    std::get<0>(pipe_pair).stop();
                    std::cout
                        << "Device: " << std::get<1>(pipe_pair)._cam_number
                        << " -> stopped successfully.\n";
                  });
  }

  void frame_to_mat(const rs2::frame &f, cv::Mat &img) {
    rs2::video_frame vf{f.as<rs2::video_frame>()};
    const int w = vf.get_width();
    const int h = vf.get_height();
    const int size = w * h;
    if (f.get_profile().format() == RS2_FORMAT_BGR8) {
      memcpy(static_cast<void *>(img.ptr<cv::Vec3b>()), f.get_data(), size * 3);
    } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
      cv::Mat tmp(h, w, CV_8UC3, const_cast<void *>(f.get_data()),
                  cv::Mat::AUTO_STEP);
      cv::cvtColor(tmp, img, cv::COLOR_RGB2BGR);
    }
  }

  void getRealSenseFrames() {
    // Drop several frames for auto-exposure
    std::for_each(std::execution::par, this->_pipeline_list.begin(),
                  this->_pipeline_list.end(),
                  [this](std::tuple<rs2::pipeline, Device, cv::Mat, cv::Mat,
                                    rs2::colorizer> &pipe_pair) {
                    auto [pipe, dev, mat_color, colored_mat_depth, color_map] =
                        pipe_pair;
                    for (int i{}; i < 90; ++i)
                      auto frames = pipe.wait_for_frames();
                  });

    std::for_each(
        std::execution::par, this->_pipeline_list.begin(),
        this->_pipeline_list.end(),
        [this](std::tuple<rs2::pipeline, Device, cv::Mat, cv::Mat,
                          rs2::colorizer> &pipe_pair) {
          auto [pipe, dev, mat_color, colored_mat_depth, color_map] = pipe_pair;
          rs2::align align_to_depth(RS2_STREAM_DEPTH);
          rs2::align align_to_color(RS2_STREAM_COLOR);
          // image_transport::ImageTransport it(this->nh);
          ros::Publisher color_pub = nh.advertise<sensor_msgs::Image>(
              "camera/color_" + dev._cam_number, 1);
          ros::Publisher colorized_depth_pub = nh.advertise<sensor_msgs::Image>(
              "camera/colorized_depth_" + dev._cam_number, 1);
          ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>(
              "camera/depth_" + dev._cam_number, 1);
          ros::Publisher pc_pub =
              nh.advertise<point_cloud>("camera/pc_" + dev._cam_number, 1);
          // if(dev._has_imu){
          ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(
              "camera/imu_" + dev._cam_number, 1);
          // }

          ros::Publisher depth_to_color_ext_pub{nh.advertise<agro::Pose>(
              "camera/depth_to_color_ext_" + dev._cam_number, 1)};
          ros::Publisher fov_pub{nh.advertise<agro::Pose>(
              "camera/field_of_view_" + dev._cam_number, 1)};

          ros::Rate loop_rate(this->_fps);

          while (ros::ok()) {
            auto start = std::chrono::steady_clock::now();

            // Get Framset
            rs2::frameset frames;
            rs2::frameset aligned_frame;
            frames = pipe.wait_for_frames();
            aligned_frame = align_to_color.process(frames);

            // Get Depth and convert to Mat
            rs2::depth_frame raw_depth{aligned_frame.get_depth_frame()};
            int depth_width{raw_depth.get_width()};
            int depth_height{raw_depth.get_height()};

            // Colorized Depth
            memcpy(static_cast<void *>(colored_mat_depth.ptr<cv::Vec3b>()),
                   raw_depth.apply_filter(color_map).get_data(),
                   depth_width * depth_height * 3);

            // Get Color Frame and convert to Mat
            rs2::video_frame raw_color{aligned_frame.get_color_frame()};

            int color_width{raw_color.get_width()};
            int color_height{raw_color.get_height()};

            memcpy(static_cast<void *>(mat_color.ptr<cv::Vec3b>()),
                   aligned_frame.get_color_frame().get_data(),
                   color_width * color_height * 3);

            rs2::pointcloud pc;
            rs2::points points;
            pc.map_to(raw_color);
            points = pc.calculate(raw_depth);
            const rs2::vertex *vertices{points.get_vertices()};
            const rs2::texture_coordinate *tex_coords{
                points.get_texture_coordinates()};

            const rs2::vertex *v_begin_ptr{vertices};
            const rs2::vertex *v_end_ptr{vertices + points.size()};

            cloud_pointer cloud{
                boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>()};
            rs2::video_stream_profile sp{
                points.get_profile().as<rs2::video_stream_profile>()};

            cloud->width = static_cast<uint32_t>(sp.width());
            cloud->height = static_cast<uint32_t>(sp.height());
            cloud->is_dense = false;
            cloud->points.resize(points.size());

            auto t_begin_ptr{tex_coords};

            const auto new_texture =
                reinterpret_cast<const uint8_t *>(raw_color.get_data());
            auto color_bytes_per_pixel = raw_color.get_bytes_per_pixel();
            auto color_stride_in_byte = raw_color.get_stride_in_bytes();

            std::transform(
                std::execution::par, v_begin_ptr, v_end_ptr, t_begin_ptr,
                cloud->points.begin(),
                std::move([&color_width, &color_height, &new_texture,
                           &color_bytes_per_pixel, &color_stride_in_byte](
                              rs2::vertex v, rs2::texture_coordinate c) {
                  RGB_Cloud p;
                  p.x = v.x;
                  p.y = v.y;
                  p.z = v.z;
                  int x_value{
                      std::min(std::max(int(c.u * color_width + .5f), 0),
                               color_width - 1)};
                  int y_value{
                      std::min(std::max(int(c.v * color_height + .5f), 0),
                               color_height - 1)};
                  int bytes{x_value * color_bytes_per_pixel};
                  int strides{y_value * color_stride_in_byte};
                  int idx{bytes + strides};
                  p.r = new_texture[idx + 2];
                  p.g = new_texture[idx + 1];
                  p.b = new_texture[idx];

                  return p;
                }));

            sensor_msgs::PointCloud2 ros_pc;
            cloud->header.frame_id = "base_link";
            pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);

            // pc_pub.publish(ros_pc);
            pc_pub.publish(*cloud);

            if (dev._has_imu) {
              rs2::motion_frame accel{aligned_frame[2].as<rs2::motion_frame>()};
              rs2::motion_frame gyro{aligned_frame[3].as<rs2::motion_frame>()};
              sensor_msgs::Imu imu_msg;
              rs2_vector gyro_data{gyro.get_motion_data()};
              rs2_vector accel_data{accel.get_motion_data()};

              imu_msg.header.stamp = ros::Time::now();

              imu_msg.angular_velocity.x = gyro_data.x;
              imu_msg.angular_velocity.y = gyro_data.y;
              imu_msg.angular_velocity.z = gyro_data.z;

              imu_msg.linear_acceleration.x = accel_data.x;
              imu_msg.linear_acceleration.y = accel_data.y + 9.81;
              imu_msg.linear_acceleration.z = accel_data.z;

              imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0,  0.0, 0.0, 0.0};
              // imu_msg.linear_acceleration_covariance = { _linear_accel_cov,
              // 0.0, 0.0, 0.0, _linear_accel_cov, 0.0, 0.0, 0.0,
              // _linear_accel_cov}; imu_msg.angular_velocity_covariance = {
              // _angular_velocity_cov, 0.0, 0.0, 0.0, _angular_velocity_cov,
              // 0.0, 0.0, 0.0, _angular_velocity_cov};

              imu_pub.publish(imu_msg);
            }

            auto color_header{std_msgs::Header()};
            color_header.stamp = ros::Time::now();
            sensor_msgs::ImagePtr msg_color =
                cv_bridge::CvImage(color_header, "bgr8", mat_color)
                    .toImageMsg();
            color_pub.publish(msg_color);

            agro::Pose depth_to_color_ext;
            depth_to_color_ext.stamp = color_header.stamp;
            depth_to_color_ext.x = dev._depth_to_color_ext[0];
            depth_to_color_ext.y = dev._depth_to_color_ext[1];
            depth_to_color_ext.z = dev._depth_to_color_ext[2];
            depth_to_color_ext_pub.publish(depth_to_color_ext);

            agro::Pose fov;
            fov.stamp = color_header.stamp;
            fov.x = dev._fov[0];
            fov.y = dev._fov[1];
            fov.z = 0.0;
            fov_pub.publish(fov);

            auto color_depth_header{std_msgs::Header()};
            color_depth_header.stamp = color_header.stamp;
            sensor_msgs::ImagePtr colored_msg_depth =
                cv_bridge::CvImage(color_depth_header, "bgr8",
                                   colored_mat_depth)
                    .toImageMsg();
            colorized_depth_pub.publish(colored_msg_depth);

            auto depth_header{std_msgs::Header()};
            depth_header.stamp = color_header.stamp;

            cv::Mat tmp(cv::Size(depth_width, depth_height), CV_16UC1,
                        const_cast<void *>(raw_depth.get_data()),
                        cv::Mat::AUTO_STEP);
            sensor_msgs::ImagePtr msg_depth =
                cv_bridge::CvImage(depth_header, "16UC1", tmp).toImageMsg();
            depth_pub.publish(msg_depth);

            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            std::cout << "Cam: " << dev._cam_number
                      << ", FPS: " << (1.0 / elapsed_seconds.count()) << "\n";

            // std::this_thread::sleep_for(std::chrono::milliseconds(10));

            ros::spinOnce();
            loop_rate.sleep();
          }
        });
  }
};

int main(int argc, char *argv[]) try {
  ros::init(argc, argv, "main", ros::init_options::AnonymousName);
  /*Accepted Resolutions:
  -1280x720
  -848x480
  -640x480
  -640x360
  -424x240*/
  RGBD_Manager rgbd_manager(640, 480, 10); /* width, height, fps*/
  rgbd_manager.getDevices();
  rgbd_manager.enumerateConnectedDevices();
  rgbd_manager.getRealSenseFrames();

  return EXIT_SUCCESS;
} catch (const rs2::error &e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
