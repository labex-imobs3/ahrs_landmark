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

#ifndef IP_ROS_SENSORS_AHRS_LMRK_HPP
#define IP_ROS_SENSORS_AHRS_LMRK_HPP

#include <string>
#include <memory>
#include <thread>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "vbus_sockets/SyncPair.hpp"
#include "vbus_sockets/Descriptor.hpp"

namespace ahrs_landmark
{

  class AhrsLandmark
  {
  public:

    enum ModeLmrk
    {
      FULL=35,
      IMU=43  // NOTE Not implemented : no needs no code...
    } mode_lmrk;

    typedef struct __attribute__((packed))
    {
      uint8_t uSync;
      uint8_t uCount;
      int16_t iGyrX;
      int16_t iGyrY;
      int16_t iGyrZ;
      int16_t iAccX;
      int16_t iAccY;
      int16_t iAccZ;
      int16_t iTemp;
      int16_t iMagX;
      int16_t iMagY;
      int16_t iMagZ;
      uint16_t uPressure;
      int16_t iRoll;
      int16_t iPitch;
      uint16_t uYaw;
      int16_t iVx;
      int16_t iVy;
      int16_t iVz;
      int16_t iBaroAlt;
      int16_t iTemperature;
      int16_t iAirSpeed;
      uint8_t uStatus;
      uint8_t uChecksum;
    } stFullMsg_LMRK;

    typedef struct __attribute__((packed))
    {
      uint8_t uSync;
      uint8_t uCount;
      int16_t iGyrX;
      int16_t iGyrY;
      int16_t iGyrZ;
      int16_t iAccX;
      int16_t iAccY;
      int16_t iAccZ;
      int16_t iTemp;
      uint8_t uStatus;
      uint8_t uChecksum;
    } stImuMsg_LMRK;

    typedef union
    {
      float f;
      uint32_t u;
    } uDat_LMRK;

    typedef struct
    {
      uint8_t sync;
      uint8_t cmd1;
      uint8_t cmd2;
      uint8_t cksum;
      uDat_LMRK dat;
    } stTxMsg_LMRK;

    typedef union
    {
      uint8_t u[sizeof(stTxMsg_LMRK)];
      stTxMsg_LMRK m;
    } uTxMsg_LMRK;

    AhrsLandmark(const vbus_sockets::Descriptor &, const std::string &, ros::NodeHandle& nh);
    ~AhrsLandmark();

  private:
    void thread_main();

    const vbus_sockets::Descriptor &fd_;
    const vbus_sockets::SyncPair syncpair_;
    std::thread thread_;
    ros::NodeHandle nh_;
    tf::TransformBroadcaster tfbroadcaster_;
  };

}

#endif
