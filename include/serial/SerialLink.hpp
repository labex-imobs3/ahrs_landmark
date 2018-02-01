/**

\file
\author Stéphane Witzmann (2014)
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

#ifndef IP_ROS_SENSORS_SERIAL_SERIALLINK_HPP
#define IP_ROS_SENSORS_SERIAL_SERIALLINK_HPP

#include <string>
#include "vbus_sockets/Descriptor.hpp"

namespace ahrs_landmark
{
  
class SerialLink: public vbus_sockets::Descriptor
{
public:
  struct Settings
  {
    enum class Speed
    {
      B_9600,
      B_115200
    } speed;

    enum class StopBits
    {
      One,
      Two
    } stop_bits;

    enum class Parity
    {
      Ignore,
      None,
      Even,
      Odd
    } parity;

    enum class RtsCts
    {
      Off,
      On
    } rtscts;

    Settings(Speed = Speed::B_115200, StopBits = StopBits::One, Parity = Parity::Ignore, RtsCts = RtsCts::Off);
  };

  SerialLink(const std::string &, const Settings &, Mode);

  void flush_input();
};

}
#endif
