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

#ifndef IP_ROS_SENSORS_AHRS_LMRK_SERIALAHRS_HPP
#define IP_ROS_SENSORS_AHRS_LMRK_SERIALAHRS_HPP

#include <string>
#include <ros/ros.h>
#include "serial/SerialLink.hpp"
#include "ahrsLandmark/AhrsLandmark.hpp"

namespace ahrs_landmark
{

  class SerialAhrs
  {
  public:
    SerialAhrs(const std::string &, const SerialLink::Settings &, ros::NodeHandle&);

  private:
    const SerialLink fd_;
    const AhrsLandmark ahrslmrk_;
  };

}
#endif
