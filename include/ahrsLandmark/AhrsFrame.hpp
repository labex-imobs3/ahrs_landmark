/**

\file
\author Laurent Malaterre (2014)
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

#ifndef IP_ROS_SENSORS_IMU_AHRS_LMRK_AHRSLMRKFRAME_HPP
#define IP_ROS_SENSORS_IMU_AHRS_LMRK_AHRSLMRKFRAME_HPP

#include <chrono>
#include <iostream>
#include "ahrsLandmark/AhrsLandmark.hpp"
#include <ahrs_landmark/AhrsLmrk.h>

namespace ahrs_landmark
{

  struct AhrsLmrkFrame
  {
  public:

    enum class Type
    {
      FULL,
      IMU, // NOTE Not implemented : no needs no code...
      Other
    };

    explicit constexpr AhrsLmrkFrame(Type t = Type::Other): type(t) {}

    Type type;
  };

  struct AhrsLmrkFrameFULL: public AhrsLmrkFrame
  {
  private :
    AhrsLmrkFrameFULL(const AhrsLandmark::stFullMsg_LMRK &);

  public:
    AhrsLmrkFrameFULL();
    static AhrsLmrk build(const uint8_t *);

    double G_x;
    double G_y;
    double G_z;
    double A_x;
    double A_y;
    double A_z;
    double Temp;
    double Mgn_x;
    double Mgn_y;
    double Mgn_z;
    double Pressure;
    double AttRot_x;
    double AttRot_y;
    double AttRot_z;
    double Vx;
    double Vy;
    double Vz;
    double BaroAlt;
    double Temperature;
    double AirSpeed;
  };

}

#endif
