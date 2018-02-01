/**

\file
\author Stéphane Witzmann (2014), Laurent Malaterre (2017)
\copyright 2014 Institut Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <ros/ros.h>
#include <iostream>
#include <functional>
#include <string>
#include <stdexcept>
#include <map>
#include <tuple>
#include "vbus_sockets/Selector.hpp"
#include "ahrsLandmark/AhrsLandmark.hpp"
#include "ahrsLandmark/AhrsFrame.hpp"
#include <ahrs_landmark/AhrsLmrk.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <sstream>

using namespace std;
using namespace ahrs_landmark;

namespace
{
  ros::Publisher ahrslandmark_pub;
  ros::Publisher imu_data_raw_pub;
  ros::Publisher imu_data_pub;
  ros::Publisher imu_mag_pub;
  ros::Publisher imu_temperature_pub;
  std::string parent_frame_id;
  std::string frame_id;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double magnetic_field_stddev;
  double orientation_stddev;
  double local_gravity;
  bool can_publish(false);

  template<typename... Args>
  std::function<void(void)> thread_bind(const std::string &name, Args &&... args)
  {
    const auto f = std::bind(args...);

    return [=]()
    {
      try
      {
        f();
      }
      catch (const std::exception &e)
      {
        std::clog<< " internal error: "<< e.what() << std::endl;
      }
      catch (...)
      {
        std::clog << name << " internal error." << std::endl;
      }
    };
  }

  typedef std::function<AhrsLmrk(uint8_t *)> Parser;
  typedef std::tuple<size_t, Parser> Tuple;
  std::map<uint8_t, std::tuple<size_t, Parser>> parsers =
  {
    { AhrsLandmark::FULL , Tuple(44, [](uint8_t *src)
      {
        return AhrsLmrkFrameFULL::build(src);
      })
    }
  };

  std::string make_thread_name(const std::string e)
  {
    return (std::string("AhrsLandmark [") + e + "]");
  }

  bool decode_crc(const uint8_t *src, size_t length)
  {
    uint8_t crc = 0;

    for (size_t i = 0; i < length; i++)
    {
      crc += src[i];
    }

    return (crc == 0);
  }

  void process_one(AhrsLmrk data,
                   ros::Time ts,
                   tf::TransformBroadcaster& tfbroadcaster
                  )
  {
    // NOTE ros-academic publication of AHRS Datas

    sensor_msgs::Imu imu_data_raw_msg;
    sensor_msgs::Imu imu_data_msg;
    sensor_msgs::MagneticField imu_magnetic_msg;
    std_msgs::Float64 imu_temperature_msg;

    double linear_acceleration_cov = linear_acceleration_stddev * linear_acceleration_stddev;
    double angular_velocity_cov    = angular_velocity_stddev* angular_velocity_stddev;
    double magnetic_field_cov      = magnetic_field_stddev * magnetic_field_stddev;
    double orientation_cov         = orientation_stddev * orientation_stddev;

    imu_data_raw_msg.linear_acceleration_covariance[0] =
      imu_data_raw_msg.linear_acceleration_covariance[4] =
        imu_data_raw_msg.linear_acceleration_covariance[8] =
          imu_data_msg.linear_acceleration_covariance[0] =
            imu_data_msg.linear_acceleration_covariance[4] =
              imu_data_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;

    imu_data_raw_msg.angular_velocity_covariance[0] =
      imu_data_raw_msg.angular_velocity_covariance[4] =
        imu_data_raw_msg.angular_velocity_covariance[8] =
          imu_data_msg.angular_velocity_covariance[0] =
            imu_data_msg.angular_velocity_covariance[4] =
              imu_data_msg.angular_velocity_covariance[8] = angular_velocity_cov;

    imu_data_msg.orientation_covariance[0] =
      imu_data_msg.orientation_covariance[4] =
        imu_data_msg.orientation_covariance[8] = orientation_cov;

    imu_magnetic_msg.magnetic_field_covariance[0] =
      imu_magnetic_msg.magnetic_field_covariance[4] =
        imu_magnetic_msg.magnetic_field_covariance[8] = magnetic_field_cov;

    // NOTE for linear_acceleration (g to m/s^2)
    static double convertor_g2a  = local_gravity;
    // NOTE for angular_velocity (degree to radian)
    static double convertor_d2r  = M_PI/180.0;
    // NOTE for magnetic_field (milliGauss to Tesla)
    static double convertor_mG2t = 0.0000001;

    double roll, pitch, yaw;
    roll = data.AttRot_y * convertor_d2r;
    pitch = data.AttRot_x * convertor_d2r;
    yaw = data.AttRot_z * convertor_d2r;

    tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

    imu_data_raw_msg.header.stamp =
      imu_data_msg.header.stamp     =
        imu_magnetic_msg.header.stamp = ts;

    imu_data_raw_msg.header.frame_id =
      imu_data_msg.header.frame_id     =
        imu_magnetic_msg.header.frame_id = frame_id;

    // NOTE orientation
    imu_data_msg.orientation.x = orientation[0];
    imu_data_msg.orientation.y = orientation[1];
    imu_data_msg.orientation.z = orientation[2];
    imu_data_msg.orientation.w = orientation[3];

    // NOTE original data used the g unit, convert to m/s^2
    imu_data_raw_msg.linear_acceleration.x =
      imu_data_msg.linear_acceleration.x     =  data.A_x * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.y =
      imu_data_msg.linear_acceleration.y     = data.A_y * convertor_g2a;
    imu_data_raw_msg.linear_acceleration.z =
      imu_data_msg.linear_acceleration.z     = data.A_z * convertor_g2a;

    // NOTE original data used the degree/s unit, convert to radian/s
    imu_data_raw_msg.angular_velocity.x =
      imu_data_msg.angular_velocity.x     =  data.G_x * convertor_d2r;
    imu_data_raw_msg.angular_velocity.y =
      imu_data_msg.angular_velocity.y     = data.G_y * convertor_d2r;
    imu_data_raw_msg.angular_velocity.z =
      imu_data_msg.angular_velocity.z     = data.G_z * convertor_d2r;

    // NOTE original data used the milliGauss unit, convert to Tesla
    imu_magnetic_msg.magnetic_field.x = data.Mgn_x / convertor_mG2t;
    imu_magnetic_msg.magnetic_field.y = data.Mgn_y / convertor_mG2t;
    imu_magnetic_msg.magnetic_field.z = data.Mgn_z / convertor_mG2t;

    // original data used the celsius unit
    imu_temperature_msg.data = data.Temp;

    if(can_publish)
    {
      // publish the IMU data
      imu_data_raw_pub.publish(imu_data_raw_msg);
      imu_data_pub.publish(imu_data_msg);
      imu_mag_pub.publish(imu_magnetic_msg);
      imu_temperature_pub.publish(imu_temperature_msg);

      // NOTE publish tf
      tfbroadcaster.sendTransform(tf::StampedTransform(tf::Transform(orientation/*tf::Transform(tf::createQuaternionFromRPY(roll, pitch, yaw)*/,
                                  tf::Vector3(0.0, 0.0, 0.0)),
                                  ts, parent_frame_id, frame_id));

      // NOTE Institut Pascal publication of sensor full AHRS Datas
      data.header.stamp = ts;
      data.header.frame_id = "/imu/ahrs_full_output";
      ahrslandmark_pub.publish(data);
    }
  }

  void process_input_buffer(uint8_t *buffer,
                            size_t &buf_length,
                            ros::Time old_ts,
                            ros::Time new_ts,
                            tf::TransformBroadcaster& tfbroadcaster
                           )
  {
    size_t start = 0;

    for (bool first = true; true; first = false)
    {
      const Tuple *t = nullptr;

      while (start < buf_length)
      {

        const auto i = parsers.find(buffer[start]);
        if (i != parsers.cend())
        {
          t = &((*i).second);
          break;
        }
        ++start;
      }

      if (start == buf_length)
      {
        break;
      }
      const size_t frame_length = std::get<0>(*t);

      if((start + frame_length)>buf_length)
        break;

      if(!decode_crc(buffer + start, frame_length))
      {
        ++start;
      }
      else
      {
        const Parser &parser = std::get<1>(*t);
        process_one(parser(buffer + start), first ? old_ts : new_ts, tfbroadcaster/*, msg_output, msg_odom*/);
        start+=frame_length;
      }
    }

    // NOTE We finished parsing the full buffer, remove the bits
    // we won't need next time
    memmove(buffer, buffer + start, buf_length - start);
    buf_length -= start;

  }
}

AhrsLandmark::AhrsLandmark(const vbus_sockets::Descriptor &fd, const std::string &dev, ros::NodeHandle& nh):
  fd_(fd),
  syncpair_(),
  nh_(nh),
  thread_(thread_bind(make_thread_name(dev), &AhrsLandmark::thread_main, this))
{
  // NOTE default frame id
  nh_.param("frame_id", frame_id, std::string("ahrs_link"));
  // NOTE for broadcasting the tf
  nh_.param("parent_frame_id_", parent_frame_id, std::string("base_link"));
  // NOTE defaults obtained experimentally/documentally from device
  nh_.param("linear_acceleration_stddev", linear_acceleration_stddev, 0.);
  nh_.param("angular_velocity_stddev", angular_velocity_stddev, 0.);
  nh_.param("magnetic_field_stddev", magnetic_field_stddev, 0.);
  nh_.param("orientation_stddev", orientation_stddev, 0.);
  nh_.param("local_gravity", local_gravity, 9.81);

  // NOTE publisher for streaming
  ahrslandmark_pub   = nh_.advertise<AhrsLmrk>("imu/ahrs_raw_output", 1);
  imu_data_raw_pub   = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  imu_data_pub       = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
  imu_mag_pub        = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
  imu_temperature_pub= nh_.advertise<std_msgs::Float64>("imu/temperature", 1);
  can_publish = true;
}


AhrsLandmark::~AhrsLandmark()
{
  syncpair_.signal();
  thread_.join();
}

void AhrsLandmark::thread_main()
{
  constexpr size_t buf_size = 80;
  uint8_t buffer[buf_size];
  size_t cur_length = 0;
  ros::Time old_ts(0);

  while (true)
  {
    vbus_sockets::Selector s;
    s.add_reader(syncpair_);
    s.add_reader(fd_);
    s();

    if (s.is_readable(syncpair_))
      return;

    if (s.is_readable(fd_))
    {
      // NOTE: a nonempty buffer here means it contains a partial
      // frame, so we need to keep the associated timestamp.
      // All the next frames will use a current timestamp.
      const bool was_empty = (cur_length == 0);

      cur_length += fd_.read(buffer + cur_length, buf_size - cur_length);

      ros::Time new_ts = ros::Time::now();
      if (was_empty)
        old_ts = new_ts;

      process_input_buffer(buffer, cur_length, old_ts, new_ts, tfbroadcaster_);

      // Handle worst-case scenario: the frame is larger than our buffer
      if (cur_length == buf_size)
        throw std::runtime_error("AHRS frame is larger than our buffer");
    }
  }
}







