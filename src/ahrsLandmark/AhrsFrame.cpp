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

#include <stdexcept>
#include <functional>
#include <map>
#include <vector>
#include <cstring>
#include "ahrsLandmark/AhrsFrame.hpp"

using namespace std;
using namespace ahrs_landmark;

namespace
{
  /* See doc/LandMark_VG_AHRS_GPS_SOFTWARE_INTERFACE_DOCUMENT_Rev 02-16-2pdf p8*/

  constexpr double gyroToDegS=(0.01);
#if 0 // unused
  constexpr double gyroToRadS=(1.745329e-4);
#endif
  constexpr double acclToG=(0.0005);
#if 0 // unused
  constexpr double acclToMS2=(9.81*acclToG);
#endif
  constexpr double tempToDegC=(0.01);
  constexpr double angleToDeg=(0.01);
  constexpr double velToMS=(1.0);
  constexpr double airspdToMS=(0.01);
  constexpr double altToM=(1.0);
  constexpr double altTempToDegC=(0.05);
  constexpr double magToMilligauss=(1.0);
  constexpr double pressureToPa=(2.0);
}

AhrsLmrk AhrsLmrkFrameFULL::build(const uint8_t *buffer)
{
  AhrsLmrk msg;

  AhrsLmrkFrameFULL*  const p = new AhrsLmrkFrameFULL(*reinterpret_cast<const AhrsLandmark::stFullMsg_LMRK *>(buffer));

  msg.G_x = p->G_x;
  msg.G_y = p->G_y;
  msg.G_z = p->G_z;
  msg.A_x = p->A_x;
  msg.A_y = p->A_y;
  msg.A_z = p->A_z;
  msg.Temp = p->Temp;
  msg.Mgn_x = p->Mgn_x;
  msg.Mgn_y = p->Mgn_y;
  msg.Mgn_z = p->Mgn_z;
  msg.Pressure = p->Pressure;
  msg.AttRot_x = p->AttRot_x;
  msg.AttRot_y = p->AttRot_y;
  msg.AttRot_z = p->AttRot_z;
  msg.Vx = p->Vx;
  msg.Vy = p->Vy;
  msg.Vz = p->Vz;
  msg.BaroAlt = p->BaroAlt;
  msg.Temperature = p->Temperature;
  msg.AirSpeed = p->AirSpeed;

  delete p;
  return msg;
}

AhrsLmrkFrameFULL::AhrsLmrkFrameFULL():
  AhrsLmrkFrame(Type::FULL)
{
}

AhrsLmrkFrameFULL::AhrsLmrkFrameFULL(const AhrsLandmark::stFullMsg_LMRK &f):
  AhrsLmrkFrame(Type::FULL)
{
  /* See doc/LandMark_VG_AHRS_GPS_SOFTWARE_INTERFACE_DOCUMENT_Rev 02-16-2pdf p13*/
  // IMU translation values are the less modified possible in despite of the non academic and indirect frame orientation used by the manufacturer.
  // So we leave academic final translation to the application.

  G_x=double(f.iGyrX)*gyroToDegS;
  G_y=double(f.iGyrY)*gyroToDegS;
  G_z=double(f.iGyrZ)*gyroToDegS;
  A_x=double(f.iAccX)*acclToG;
  A_y=double(f.iAccY)*acclToG;
  A_z=double(f.iAccZ)*acclToG;
  Temp=double(f.iTemp)*tempToDegC;
  Mgn_x=double(f.iMagX)*magToMilligauss;
  Mgn_y=double(f.iMagY)*magToMilligauss;
  Mgn_z=double(f.iMagZ)*magToMilligauss;
  Pressure=double(f.uPressure)*pressureToPa;
  AttRot_x=double(f.iRoll)*angleToDeg; // Roll is according to manufacturer's frame orientation (see doc LandMark10_20_40_AHRS_USERGUIDE_USB_Rev_2012-11-20.pdf p.13)
  AttRot_y=double(f.iPitch)*angleToDeg; // Pitch is according to manufacturer's frame orientation (see doc LandMark10_20_40_AHRS_USERGUIDE_USB_Rev_2012-11-20.pdf p.13)
  AttRot_z=double(f.uYaw)*angleToDeg;
  Vx=double(f.iVx)*velToMS;
  Vy=double(f.iVy)*velToMS;
  Vz=double(f.iVz)*velToMS;
  BaroAlt=double(f.iBaroAlt)*altToM;
  Temperature=double(f.iTemperature)*altTempToDegC;
  AirSpeed=double(f.iAirSpeed)*airspdToMS;
}


