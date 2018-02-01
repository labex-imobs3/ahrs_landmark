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

#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <cassert>
#include <iostream>

#include "serial/SerialLink.hpp"

using  namespace ahrs_landmark;

namespace
{
  void configure(int fd, const SerialLink::Settings &settings)
  {
    termios param;
    memset(&param, 0, sizeof(termios));

    param.c_cflag = CS8 | CREAD;
    param.c_iflag = 0;
    param.c_oflag = 0;
    param.c_lflag = 0; // raw
    param.c_cc[VTIME] = 0;
    param.c_cc[VMIN] = 1;

    if (settings.rtscts == SerialLink::Settings::RtsCts::On)
      param.c_cflag |= CRTSCTS;

    switch (settings.speed)
    {
    case SerialLink::Settings::Speed::B_9600:
      param.c_cflag |= B9600;
      break;

    case SerialLink::Settings::Speed::B_115200:
      param.c_cflag |= B115200;
      break;

    default:  // should never happen
      throw std::runtime_error("Unsupported baud rate");
    }

    switch (settings.stop_bits)
    {
    case SerialLink::Settings::StopBits::One:
      break;

    case SerialLink::Settings::StopBits::Two:
      param.c_cflag |= CSTOPB;
      break;

    default:  // should never happen
      throw std::runtime_error("Bad stop bit");
    }

    switch (settings.parity)
    {
    case SerialLink::Settings::Parity::Ignore:
      param.c_iflag |= IGNPAR;
      break;

    case SerialLink::Settings::Parity::None:
      break;

    case SerialLink::Settings::Parity::Even:
      param.c_cflag |= PARENB;
      break;

    case SerialLink::Settings::Parity::Odd:
      param.c_cflag |= PARENB | PARODD;
      break;

    default:  // should never happen
      throw std::runtime_error("Bad parity");
    }

    // Apply
    if (tcsetattr(fd, TCSANOW, &param) != 0)
      throw std::runtime_error("tcsetattr() failed");

    // Also set low-latency flag
    serial_struct serial;
    if (ioctl(fd, TIOCGSERIAL, &serial) != 0)
    {
      std::clog <<  "ioctl(TIOCSSERIAL) failed" << std::endl;
      return;
    }

    serial.flags |= ASYNC_LOW_LATENCY;

    if (ioctl(fd, TIOCSSERIAL, &serial) != 0)
    {
      std::clog <<  "ioctl(TIOCSSERIAL) failed" << std::endl;
      return;
    }
  }
}

SerialLink::Settings::Settings(Speed sp, StopBits stop, Parity par, RtsCts rc):
  speed(sp),
  stop_bits(stop),
  parity(par),
  rtscts(rc)
{
}

SerialLink::SerialLink(const std::string &device, const SerialLink::Settings &settings, Mode mode):
  Descriptor(device, mode)
{
  configure(fd_, settings);
  flush_input();
}

void SerialLink::flush_input()
{
  if (tcflush(fd_, TCIFLUSH) != 0)
    throw std::runtime_error("Cannot flush input");
}

