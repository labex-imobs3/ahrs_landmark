/**

\file
\author Laurent Malaterre (2017)
\copyright 2017 Institut Pascal

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
#include <memory>
#include "ahrsLandmark/SerialAhrs.hpp"

using namespace ahrs_landmark;

class ahrsNode
{
public:
  ahrsNode(std::string sd , ros::NodeHandle& nh)
  {
    serial_dev_ = sd;
    nh_ = nh;
  }
  ;

  bool on_start()
  {

    serial_settings_ = std::make_unique<SerialLink::Settings>(
                         SerialLink::Settings::Speed::B_115200,
                         SerialLink::Settings::StopBits::Two,
                         SerialLink::Settings::Parity::Even);

    ahrs = std::make_unique<SerialAhrs>( serial_dev_, *serial_settings_, nh_);

    ROS_INFO("AHRS opened");
    return true;
  }

private:
  std::unique_ptr<SerialAhrs> ahrs;
  std::string serial_dev_;
  unsigned int  baudrate_;
  std::unique_ptr<SerialLink::Settings> serial_settings_;
  ros::NodeHandle nh_;

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ahrs_landmark_node");
  ros::NodeHandle nh(""), nh_param("~");

  std::string serial_dev;
  nh_param.param<std::string>("serial_dev", serial_dev, "/dev/ttyUSB0");
  ROS_INFO("GET param serial device %s", serial_dev.c_str());

  // NOTE We choosed to pass NodeHandle in private domain. Feel free to change.
  ahrsNode ahrs_node(serial_dev, nh_param);
  
  if (!ahrs_node.on_start())
  {
    ROS_ERROR("%s\n", "<ahrsNode> on_start() returns false, please check your devices.");
    return -1;
  }
  else
  {
    ROS_INFO("Initialization OK!\n");
  }

  ros::spin();

}
