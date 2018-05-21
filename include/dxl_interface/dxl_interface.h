/*******************************************************************************
* Copyright (c) 2018, RoboTICan, LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of RoboTICan nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
/* Author: Elhay Rauper*/



#ifndef ARMADILLO2_HW_ARM_INTERFACE_H
#define ARMADILLO2_HW_ARM_INTERFACE_H

#include <iostream>
#include <stdint.h>
#include <cmath>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dxl_interface/protocol.h>
#include <dxl_interface/math.h>
#include <dxl_interface/motor.h>

namespace dxl
{
    class DxlInterface
    {

    private:
        dynamixel::PacketHandler *pkt_handler_;
        dynamixel::PortHandler *port_handler_;
        float protocol_;

        bool loadProtocol(uint16_t protocol);

    public:

        enum PortState
        {
            PORT_FAIL,
            BAUDRATE_FAIL,
            INVALID_PROTOCOL,
            SUCCESS
        };

        DxlInterface();
        ~DxlInterface();
        PortState openPort(std::string port_name,
                           unsigned int baudrate,
                           float protocol);
        bool ping (Motor & motor);
        bool setTorque(Motor &motor, bool flag);
        bool bulkWriteVelocity(std::vector<Motor> & motors);
        bool bulkWritePosition(std::vector<Motor> & motors);
        bool bulkWriteGoalTorque(std::vector<Motor> &motors);
        bool readMotorsPos(std::vector<Motor> & motors);
        bool readMotorsVel(std::vector<Motor> & motors);
        bool readMotorsLoad(std::vector<Motor> &motors);
        bool readMotorsError(std::vector<Motor> & motors);
        bool reboot(const Motor &motor);
        bool broadcastPing(std::vector<uint8_t> result_vec, uint16_t protocol);
    };

}

#endif //ARMADILLO2_HW_ARM_INTERFACE_H
