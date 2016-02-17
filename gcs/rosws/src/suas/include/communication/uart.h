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
 * File		: Serial.h                                                   *
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
 * put(const unsigned char* buffer, const unsigned short size)               *
 *   Puts the string as character array on the serial port.                  *
 *****************************************************************************/

#ifndef SERIAL_DEVICE_CHAR_TIMEOUT
 #define SERIAL_DEVICE_CHAR_TIMEOUT 0.005
#endif

#include <termios.h>
#include <string>
#include <stdint.h>

class Serial {
  private:
    struct termios tio;
    int tty_fd;
    std::string _PORT;
    unsigned int _BAUD;
    uint8_t peepchar;
    bool peeped;

  public:
    Serial();
    ~Serial();

    int disconnect();
    int connect(const std::string PORT, const unsigned int BAUD);

    uint8_t peep(void);
    int fetch(std::string &str, const unsigned short len);
    int fetchUntil(std::string &str, const unsigned char needle);

    int put(const char* buffer, const unsigned short size);
};
