/*****************************************************************************
 ********************************** LICENSE **********************************
 *****************************************************************************
 * Copyright 2014 Mars Society India                                         *
 *                                                                           *
 * Licensed under the Apache License, Version 2.0 (the "License");           *
 * you may not use this file except in compliance with the License.          *
 * You may obtain a copy of the License at                                   *
 *                                                                           *
 *     http://www.apache.org/licenses/LICENSE-2.0                            *
 *                                                                           *
 * Unless required by applicable law or agreed to in writing, software       *
 * distributed under the License is distributed on an "AS IS" BASIS,         *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
 * See the License for the specific language governing permissions and       *
 * limitations under the License.                                            *
 *                                                                           *
 * File		: uart.cpp                                                   *
 * Author	: Kamal Galrani                                              *
 * Description	: Reads data serial port.                                    *
 *****************************************************************************
 ******************************* DOCUMENTATION *******************************
 *****************************************************************************
 * Member Functions                                                          *
 *                                                                           *
 * connect(const std::string PORT, const unsigned long IBAUD)                *
 *   Connects to serial port PORT at baudrate IBAUD                          *
 *                                                                           *
 * disconnect()                                                              *
 *   Disconnetcs from serial port. This funtion does not destroy the instance*
 *                                                                           *
 * fetch(std::string &str, const unsigned short len)                         *
 *   Fetches next len characters and saves them in string str.               *
 *                                                                           *
 * fetchUntil(std::string &str, const unsigned char needle)                  *
 *   Fetches string until it finds the needle (including the needle)         *
 *   and stores it in string str.                                            *
 *                                                                           *
 * put(const std::string &str)                                               *
 *   Puts the string as character array on the serial port.                  *
 *****************************************************************************/

#include <communication/uart.h>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#include <errno.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iterator>
#include <time.h>

Serial::Serial() {
      memset(&tio,0,sizeof(tio));
      tio.c_iflag     = 0;
      tio.c_oflag     = 0;
      tio.c_cflag     = CS8 | CREAD | CLOCAL;           // 8n1, see termios.h for more information
      tio.c_lflag     = 0;
      tio.c_cc[VMIN]  = 1;
      tio.c_cc[VTIME] = 5;
      _PORT = "/dev/ttyUSB0";
      _BAUD = B115200;
      peeped = false;
}
Serial::~Serial() {
      close(tty_fd);
}
int Serial::disconnect() {
      close(tty_fd);
      return -errno;
}

int Serial::connect(const std::string PORT, const unsigned int BAUD) {
      _PORT = PORT; _BAUD = BAUD;
      tty_fd = open(PORT.c_str(), O_RDWR | O_NONBLOCK);
      if (tty_fd == -1) return -errno;

      cfsetospeed(&tio, BAUD);
      cfsetispeed(&tio, BAUD);
      tcsetattr(tty_fd, TCSANOW,&tio);
      peeped = false;
      return 0;
}

uint8_t Serial::peep(void) {
  if (!peeped) {
    int s = read(tty_fd,&peepchar,1);
    if ( s > 0 ) peeped = true;
    else if (( s == 0 ) || (s == -1 && errno == 9)) {disconnect(); if (connect(_PORT, _BAUD) != 0) return 0;}
  }
  return peepchar;
}

int Serial::fetch(std::string &str, const unsigned short len) {
      std::ostringstream oss;
      unsigned char c;
      int s = 0;
      time_t start;

      time(&start);
      if (peeped) { oss << peepchar; peeped = false; }
      while(1) {
        if (oss.str().length() == len) break;

        if (difftime(time(NULL),start) > len * SERIAL_DEVICE_CHAR_TIMEOUT) return -ETIMEDOUT;

        s = read(tty_fd,&c,1);
        if ( s > 0 ) oss << c;
        else if (( s == 0 ) || (s == -1 && errno == 9)) {disconnect(); if (connect(_PORT, _BAUD) != 0) return -errno;}
      }
      str = oss.str();
      return str.length();
}

int Serial::fetchUntil(std::string &str, const unsigned char needle) {
      std::ostringstream oss;
      unsigned char c;
      int s = 0;
      time_t start;

      time(&start);
      if (peeped) { oss << peepchar; peeped = false; }
      while(1) {
        if (difftime(time(NULL),start) > (oss.str().length()+1) * SERIAL_DEVICE_CHAR_TIMEOUT) return -ETIMEDOUT;

        s = read(tty_fd,&c,1);
        if ( s == 1 ) { oss << c; if (c==needle) break;}
        else if (( s == 0 ) || (s == -1 && errno == 9)) {disconnect(); if (connect(_PORT, _BAUD) != 0) return -errno;}
      }
      str = oss.str();
      return str.length();
    }

int Serial::put(const char* buffer, const unsigned short size) {
      write(tty_fd, buffer, size);
      return -errno;
}
